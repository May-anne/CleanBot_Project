# roomba_coppeliasim_test.py
# Requisitos: pip install coppeliasim-zmqremoteapi-client
# Inicie o CoppeliaSim com o serviço ZMQ habilitado (menu: Remote API → "Enable remote API server").
# Ajuste os nomes dos objetos da sua cena abaixo.

import time
import math
import random
from enum import Enum, auto
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import matplotlib.pyplot as plt
plt.ion()   # modo interativo ON

# =========================
# CONFIGURE AQUI (nomes/parametros)
# =========================
NOME_BASE                = '/pure_base'             # objeto raiz do robô
NOME_JUNTA_RL            = '/lr_wheel_joint'        # roda esquerda traseira
NOME_JUNTA_RR            = '/rr_wheel_joint'       # roda direita traseira
NOME_JUNTA_FL            = '/lf_wheel_joint'       # se não tiver 4WD, deixe None
NOME_JUNTA_FR            = '/rf_wheel_joint'      # se não tiver 4WD, deixe None
NOME_JUNTA_SERVO         = '/support_joint'              # junta que gira o sensor
NOME_SENSOR_PROX         = '/proximitySensor'             # sensor de proximidade (frontal no servo)

RAIO_RODA                = 0.0327      # m
ENTRE_EIXOS              = 0.14      # distância entre rodas esquerda/direita (track) em m
D_STOP                   = 0.15      # m: “bateu”
MAX_STUCK_TIME           = 1.5       # s
FORWARD_SPEED            = 0.15      # m/s
CONTROL_DT               = 0.05      # s

# =========================
# ESTADOS
# =========================
class State(Enum):
    SPIRAL      = auto()
    WALL_FOLLOW = auto()
    RANDOM_WALK = auto()
    ESCAPE      = auto()

class WallSide(Enum):
    LEFT  = auto()
    RIGHT = auto()

# =========================
# HARDWARE (CoppeliaSim)
# =========================
class SimHardware:
    def __init__(self, scene_file: str | None = None, start_paused: bool = True):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        # Carrega cena externamente se quiser (ou carregue por fora e só conecte)
        # Se já estiver com a cena aberta, comente este bloco:
        if scene_file:
            # Para recarregar limpo:
            self.sim.stopSimulation()
            while self.sim.getSimulationState() != self.sim.simulation_stopped:
                time.sleep(0.05)
            self.sim.closeScene()
            self.sim.loadScene(scene_file)

        # Start/pause sim
        if start_paused:
            self.sim.startSimulation()
            self.sim.pauseSimulation()
        else:
            self.sim.startSimulation()

        # Pega handles
        self.h_base   = self.sim.getObject(NOME_BASE)
        # --- Graph ao vivo (trajetória XY) ---
        self.graph = None
        try:
            self.graph = self.sim.getObject('/traj_graph')
            self.xStream = self.sim.addGraphStream(self.graph, 'x', 'm', 0)
            self.yStream = self.sim.addGraphStream(self.graph, 'y', 'm', 0)
        except Exception as e:
            print("Graph não encontrado na cena (ignorado).")
            self.graph = None
        self.h_wl_r   = self.sim.getObject(NOME_JUNTA_RL)
        self.h_wr_r   = self.sim.getObject(NOME_JUNTA_RR)
        self.h_wl_f   = self.sim.getObject(NOME_JUNTA_FL) if NOME_JUNTA_FL else None
        self.h_wr_f   = self.sim.getObject(NOME_JUNTA_FR) if NOME_JUNTA_FR else None
        self.h_servo  = self.sim.getObject(NOME_JUNTA_SERVO)
        self.h_prox   = self.sim.getObject(NOME_SENSOR_PROX)

        # Coloca juntas de rodas em modo velocidade
        for h in [self.h_wl_r, self.h_wr_r, self.h_wl_f, self.h_wr_f]:
            if h is not None:
                self.sim.setObjectFloatParam(h, self.sim.jointfloatparam_maxvel, 9999.0)
                self.sim.setObjectInt32Param(h, self.sim.jointintparam_dynctrlmode, self.sim.jointdynctrl_velocity)

        # Zera servo (frente = 0 rad)
        self.set_servo_angle(0.0)

        # Estados internos
        self._last_encoder_wl = self.sim.getJointPosition(self.h_wl_r)
        self._last_encoder_wr = self.sim.getJointPosition(self.h_wr_r)
        self._cum_dist_wl     = 0.0
        self._cum_dist_wr     = 0.0

        # Para stuck detection
        self._last_progress_time = time.time()
        self._last_mean_dist     = 0.0

        # Resume sim
        if start_paused:
            self.sim.resumeSimulation()
    
    def push_graph_xy(self):
        """Envia posição atual (x,y) para o Graph do Coppelia."""
        if self.graph is None:
            return
        p = self.sim.getObjectPosition(self.h_base, -1)
        self.sim.setGraphStreamValue(self.graph, self.xStream, p[0])
        self.sim.setGraphStreamValue(self.graph, self.yStream, p[1])

    def get_pose(self):
        """Retorna (x, y, yaw) no mundo."""
        p = self.sim.getObjectPosition(self.h_base, -1)
        e = self.sim.getObjectOrientation(self.h_base, -1)
        return p[0], p[1], e[2]


    def calibrate_encoder_signs(self):
        # mede ângulos atuais
        wl0 = self.sim.getJointPosition(self.h_wl_r)
        wr0 = self.sim.getJointPosition(self.h_wr_r)
        # dá um toque pra FRENTE
        v_test = 0.05
        self.set_wheel_speeds(v_test, v_test)
        time.sleep(0.2)
        self.stop()
        wl1 = self.sim.getJointPosition(self.h_wl_r)
        wr1 = self.sim.getJointPosition(self.h_wr_r)
        self.enc_sign_L = 1.0 if (wl1 - wl0) > 0 else -1.0
        self.enc_sign_R = 1.0 if (wr1 - wr0) > 0 else -1.0

    def ensure_encoder_signs(self):
        if not hasattr(self, "enc_sign_L") or not hasattr(self, "enc_sign_R"):
            self.calibrate_encoder_signs()


    # --- Sensores ---
    def read_proximity_front(self) -> float:
        """
        Lê o sensor de proximidade acoplado ao servo.
        Retorna distância em metros (ou um valor grande caso nada detectado).
        """
        self.sim.handleProximitySensor(self.h_prox)
        state, distance, _, _, _ = self.sim.readProximitySensor(self.h_prox)
        if state:  # detectou algo
            return distance
        else:
            return 10.0  # "muito longe"

    def read_yaw(self) -> float:
        """
        "Giroscópio": usa orientação do corpo no mundo e retorna yaw (rad) ~ eixo Z.
        """
        # Euler (alpha,beta,gamma) em rad para orientação relativa ao mundo
        euler = self.sim.getObjectOrientation(self.h_base, -1)
        yaw = euler[2]
        return yaw

    def read_wheel_encoders(self) -> tuple[float, float]:
        """
        Retorna distâncias lineares acumuladas (m) das rodas traseiras,
        integrando a variação angular (rad) * RAIO_RODA.
        """
        # Posição da junta é o ângulo (rad). Integramos delta.
        wl = self.sim.getJointPosition(self.h_wl_r)
        wr = self.sim.getJointPosition(self.h_wr_r)

        d_wl = (wl - self._last_encoder_wl) * RAIO_RODA
        d_wr = (wr - self._last_encoder_wr) * RAIO_RODA

        self._cum_dist_wl += d_wl
        self._cum_dist_wr += d_wr

        self._last_encoder_wl = wl
        self._last_encoder_wr = wr
        return (self._cum_dist_wl, self._cum_dist_wr)

    # --- Atuadores ---
    def set_servo_angle(self, angle_rad: float):
        self.sim.setJointTargetPosition(self.h_servo, angle_rad)

    def set_wheel_speeds(self, v_left: float, v_right: float):
        """
        Define velocidade linear de cada lado (m/s). Converte para rad/s nas juntas.
        """
        wL = v_left  / RAIO_RODA
        wR = v_right / RAIO_RODA
        # Traseiras (encoders estão nelas)
        self.sim.setJointTargetVelocity(self.h_wl_r, wL)
        self.sim.setJointTargetVelocity(self.h_wr_r, wR)
        # Dianteiras (se existirem, espelhe)
        if self.h_wl_f is not None:
            self.sim.setJointTargetVelocity(self.h_wl_f, wL)
        if self.h_wr_f is not None:
            self.sim.setJointTargetVelocity(self.h_wr_f, wR)

    def stop(self):
        self.set_wheel_speeds(0.0, 0.0)

    def step_sleep(self):
        # Em ZMQ Remote API, a sim roda no tempo dela; um sleep curto ajuda o loop.
        time.sleep(CONTROL_DT)

    # --- Utilidades ---
    def is_running(self) -> bool:
        st = self.sim.getSimulationState()
        return st != self.sim.simulation_stopped

# =========================
# CONTROLADOR (Roomba-like)
# =========================
class RoombaController:
    def __init__(self, hw: SimHardware):
        self.hw = hw
        self.state = State.SPIRAL
        self.wall_side = WallSide.RIGHT

        self.last_progress_time = time.time()
        self.last_encoder_mean  = 0.0

        self.path_x = []
        self.path_y = []

        # janela do gráfico ao vivo
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-', linewidth=2)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_title("Trajetória ao vivo")
        self.ax.set_aspect("equal", adjustable="box")

    # ------- CORE -------
    def step(self):
        d_front = self.hw.read_proximity_front()
        self._update_stuck_detector()

        if self.state == State.SPIRAL:
            print("STATE: Spiral")
            self._state_spiral(d_front)

        elif self.state == State.WALL_FOLLOW:
            print("STATE: Wall Follow")
            self._state_wall_follow(d_front)

        elif self.state == State.RANDOM_WALK:
            print("STATE: Random Walk")
            self._state_random_walk(d_front)

        elif self.state == State.ESCAPE:
            print("STATE: Escape")
            self._state_escape()

        self.hw.push_graph_xy()

        # registra ponto
        x, y, _ = self.hw.get_pose()
        self.path_x.append(x)
        self.path_y.append(y)

        # atualiza o gráfico ao vivo
        self.line.set_xdata(self.path_x)
        self.line.set_ydata(self.path_y)
        self.ax.relim()
        self.ax.autoscale_view()

        plt.pause(0.001)

        self.hw.step_sleep()

    # ------- Estados -------
    def _state_spiral(self, d_front: float):
        self.hw.set_servo_angle(0)
        # parâmetros da espiral (os mesmos do exemplo)
        v0        = 0.10        # m/s
        w_initial = 3.0         # rad/s (default 3.0)
        w_final   = 0.50        # rad/s (default 0.5)
        beta      = 0.18        # taxa de abertura
        WHEEL_MAX = 25.0        # rad/s limite
        L         = ENTRE_EIXOS # bitola
        r         = RAIO_RODA   # raio da roda
        
        # inicializa tempo da espiral
        if not hasattr(self, "_spiral_t0") or self._spiral_t0 is None:
            self._spiral_t0 = time.time()

        # tempo decorrido
        t = time.time() - self._spiral_t0

        # equação da espiral — igual ao teu exemplo
        w = w_final + (w_initial - w_final) * math.exp(-beta * t)
        v = v0 * (1.0 + 0.4 * (t / 20.0))   # opcional e suave

        # velocidades das rodas
        w_left  = (v - 0.5 * w * L) / r
        w_right = (v + 0.5 * w * L) / r

        # clamp
        w_left  = max(-WHEEL_MAX, min(WHEEL_MAX, w_left))
        w_right = max(-WHEEL_MAX, min(WHEEL_MAX, w_right))

        # aplica às rodas do simulador via wrapper
        self.hw.set_wheel_speeds(v_left=v - (w * L / 2),
                                v_right=v + (w * L / 2))

        # --- verificações de transição de estado ---
        # obstáculo frontal detectado
        if d_front < D_STOP:
            self.hw.stop()
            self._spiral_t0 = None

            print("backup")
            self._backup()

            self.wall_side = self._choose_wall_side()
            print("follow wall ", self.wall_side)
            self.state = State.WALL_FOLLOW
            return

        # se ficar preso
        if self._is_stuck():
            self._spiral_t0 = None
            self.state = State.ESCAPE
            return
        
    def _set_vw_direct(self, v, w):
        L = ENTRE_EIXOS
        r = RAIO_RODA

        wl = (v - 0.5*w*L) / r
        wr = (v + 0.5*w*L) / r

        # aplica às rodas
        self.hw.set_wheel_speeds(v - 0.5*w*L, v + 0.5*w*L)


    def _state_wall_follow(self, d_front: float):

        # --- 1) Ângulo fixo do sensor (+60° esquerda, -60° direita) ---
        side = 1 if self.wall_side == WallSide.LEFT else -1
        sensor_angle = math.radians(60.0 * side)
        self.hw.set_servo_angle(sensor_angle)
        time.sleep(0.05)

        # --- 2) Leitura filtrada ---
        raw = self.hw.read_proximity_front()
        if raw >= 9.0:   # nada visto
            raw = None

        if not hasattr(self, "_fw_dist_filt"):
            self._fw_dist_filt = raw if raw is not None else 0.25
            self._fw_prev_dist = self._fw_dist_filt
            self._fw_prev_t = time.time()

        if raw is not None:
            # Filtro exponencial
            alpha = 0.30
            self._fw_dist_filt = alpha*raw + (1-alpha)*self._fw_dist_filt

            # Erro
            d_set = 0.20
            e = d_set - self._fw_dist_filt

            # Derivada
            t = time.time()
            dt = max(1e-3, t - self._fw_prev_t)
            de_dt = (e - (d_set - self._fw_prev_dist)) / dt

            self._fw_prev_dist = self._fw_dist_filt
            self._fw_prev_t = t

            # Deadband
            deadband = 0.001
            if abs(e) < deadband:
                w_cmd = 0.0
            else:
                kP = 1.7
                kD = 0.6
                w_cmd = side*(kP*e + kD*de_dt)

            # Limite
            w_max = 0.8
            w_cmd = max(-w_max, min(w_max, w_cmd))

            # --- 3) Escape se muito perto ---
            if self._fw_dist_filt < 0.15:
                self._backup_escape()
                return

            # --- 4) Avança seguindo a parede ---
            self._set_vw_direct(0.12, w_cmd)

            if self._is_stuck():
                self.state = State.ESCAPE
                return
        else:
            # Perdeu a parede
            self.state = State.RANDOM_WALK
            return
        


    def _state_random_walk(self, d_front: float):
        self.hw.set_wheel_speeds(FORWARD_SPEED, FORWARD_SPEED)
        if d_front < D_STOP:
            self.hw.stop()
            self._backup()
            ang = math.radians(60) + random.random() * math.radians(60)
            if random.random() < 0.5:
                ang = -ang
            self._rotate_relative(ang)

        if self._is_stuck():
            self.state = State.ESCAPE

    def _state_escape(self):
        self.hw.stop()
        self._backup()
        best_dir = self._scan_best_dir()  # rad
        self._rotate_relative(best_dir)
        self.state = State.RANDOM_WALK
        # reset stuck
        self.last_progress_time = time.time()
        self.last_encoder_mean  = self._encoder_mean()

    # ------- Primitivas de controle -------
    def _rotate_relative(self, d_ang: float):
        """
        Gira d_ang (rad) usando apenas odometria de encoder.
        Sem depender de yaw/Euler do Coppelia.
        """
        self.hw.ensure_encoder_signs()

        # alvo e direção
        target = abs(d_ang)
        direction = 1.0 if d_ang >= 0.0 else -1.0  # +CCW / -CW

        # zera integração por encoder
        dL_prev, dR_prev = self.hw.read_wheel_encoders()
        theta = 0.0

        # parâmetros de rotação
        L = ENTRE_EIXOS
        OMEGA_MAX = 1.2   # rad/s no grosso
        OMEGA_MIN = 0.25  # rad/s p/ vencer atrito perto do fim
        TOL = math.radians(1.5)  # tolerância de parada
        T_TIMEOUT = 6.0          # trava de segurança
        t0 = time.time()

        # rampa inicial curta (tira do estático)
        for _ in range(3):
            omega = direction * OMEGA_MIN
            vL = -omega * (L/2.0)
            vR =  omega * (L/2.0)
            self.hw.set_wheel_speeds(vL, vR)
            time.sleep(CONTROL_DT)

        # laço principal
        while True:
            # progresso por encoder
            dL, dR = self.hw.read_wheel_encoders()
            incL = self.hw.enc_sign_L * (dL - dL_prev)
            incR = self.hw.enc_sign_R * (dR - dR_prev)
            dtheta = (incR - incL) / L
            theta += dtheta
            dL_prev, dR_prev = dL, dR

            remaining = target - abs(theta)

            # condição de saída
            if remaining <= TOL:
                break
            if time.time() - t0 > T_TIMEOUT:
                # safety break para não travar se a física não deixar girar
                print("[rotate_relative] timeout; breaking")
                break

            # lei de controle simples: omega proporcional ao restante + piso
            omega_mag = min(OMEGA_MAX, max(OMEGA_MIN, 2.0 * remaining))  # 2.0 = ganho
            omega = direction * omega_mag

            vL = -omega * (L/2.0)
            vR =  omega * (L/2.0)
            self.hw.set_wheel_speeds(vL, vR)
            time.sleep(CONTROL_DT)

        self.hw.stop()


    def _backup(self):
        # 1. parar um pouco
        self.hw.stop()
        time.sleep(0.05)

        # 2. ré curta
        self.hw.set_wheel_speeds(-0.12, -0.12)
        time.sleep(0.45)

        self.hw.stop()

    def _backup_escape(self):
        print("escape wall")

        # 1. parar um pouco
        self.hw.stop()
        time.sleep(0.05)

        # 2. ré curta
        self.hw.set_wheel_speeds(-0.12, -0.12)
        time.sleep(0.45)

        # 3. girar para longe da parede
        side = 1 if self.wall_side == WallSide.LEFT else -1
        self.hw.set_wheel_speeds(0.0, -0.25*side)
        time.sleep(0.25)


    # ------- Varreduras -------
    def _scan_best_dir(self) -> float:
        """
        Varre [-60°, 60°] e retorna ângulo (rad) relativo com maior distância livre.
        """
        best_ang = 0.0
        best_d = -1.0
        for deg in range(-60, 61, 15):
            self.hw.set_servo_angle(math.radians(deg))
            time.sleep(0.06)
            d = self.hw.read_proximity_front()
            if d > best_d:
                best_d = d
                best_ang = math.radians(deg)
        # volta servo pro zero
        self.hw.set_servo_angle(0.0)
        return best_ang

    def _choose_wall_side(self) -> WallSide:
        # mede +/-90° e escolhe lado com obstáculo mais próximo (há “parede”)
        for deg in (-90, 90):
            self.hw.set_servo_angle(math.radians(deg))
            time.sleep(0.06)
        self.hw.set_servo_angle(math.radians(-90))
        dL = self.hw.read_proximity_front()
        self.hw.set_servo_angle(math.radians(90))
        dR = self.hw.read_proximity_front()
        self.hw.set_servo_angle(0.0)

        if dL < dR:
            return WallSide.LEFT
        else:
            return WallSide.RIGHT

    # ------- Stuck detector -------
    def _encoder_mean(self) -> float:
        dL, dR = self.hw.read_wheel_encoders()
        return 0.5 * (dL + dR)

    def _update_stuck_detector(self):
        cur = self._encoder_mean()
        if abs(cur - self.last_encoder_mean) > 0.01:
            self.last_progress_time = time.time()
            self.last_encoder_mean  = cur

    def _is_stuck(self) -> bool:
        return (time.time() - self.last_progress_time) > MAX_STUCK_TIME


# =========================
# MAIN
# =========================
def main():
    # Se quiser que este script carregue sua cena, passe o caminho:
    # Ex.: scene_file = '/mnt/data/sua_cena.ttt'  (ou .ttt/.ttm conforme versão)
    scene_file = 'C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/scenes/CleanBot.ttt'  # ou um caminho para a sua cena

    hw = SimHardware(scene_file=scene_file, start_paused=False)
    ctrl = RoombaController(hw)
    print("Scene loaded: starting simulation")

    try:
        while hw.is_running():
            ctrl.step()
        hw.stop()
    except KeyboardInterrupt:
        hw.stop()
        # opcional: hw.sim.stopSimulation()

if __name__ == '__main__':
    main()
