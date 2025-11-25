# roomba_coppeliasim_mapmem_zigzag_5x5.py
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
NOME_JUNTA_RR            = '/rr_wheel_joint'        # roda direita traseira
NOME_JUNTA_FL            = '/lf_wheel_joint'        # se não tiver 4WD, deixe None
NOME_JUNTA_FR            = '/rf_wheel_joint'        # se não tiver 4WD, deixe None
NOME_JUNTA_SERVO         = '/support_joint'         # junta que gira o sensor
NOME_SENSOR_PROX         = '/proximitySensor'       # sensor de proximidade (frontal no servo)

RAIO_RODA                = 0.0327      # m
ENTRE_EIXOS              = 0.14        # distância entre rodas esquerda/direita (track) em m
D_STOP                   = 0.15        # m: “bateu”
MAX_STUCK_TIME           = 1.5         # s
FORWARD_SPEED            = 0.15        # m/s
CONTROL_DT               = 0.05        # s

# =========================
# PARÂMETROS DO MAPA EM GRID 2D
# Ambiente ≈ 5m x 5m → 50 x 50 células de 0.1m
# =========================
CELL_SIZE     = 0.10    # m por célula
GRID_WIDTH    = 50      # número de células em X
GRID_HEIGHT   = 50      # número de células em Y
# Origem será calculada dinamicamente em torno da posição inicial do robô

# =========================
# ESTADOS
# =========================
class State(Enum):
    SPIRAL      = auto()     # não usado no fluxo principal (ficou de reserva)
    WALL_FOLLOW = auto()
    RANDOM_WALK = auto()     # agora se comporta como ZIGZAG
    ESCAPE      = auto()

class WallSide(Enum):
    LEFT  = auto()
    RIGHT = auto()

# =========================
# HARDWARE (CoppeliaSim)
# =========================
class SimHardware:
    def __init__(self, scene_file: str = None, start_paused: bool = True):
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')

        # Carrega cena se indicado
        if scene_file:
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
        except Exception:
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
        wl0 = self.sim.getJointPosition(self.h_wl_r)
        wr0 = self.sim.getJointPosition(self.h_wr_r)
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
        euler = self.sim.getObjectOrientation(self.h_base, -1)
        yaw = euler[2]
        return yaw

    def read_wheel_encoders(self) -> tuple[float, float]:
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
        self.sim.setJointTargetVelocity(self.h_wl_r, wL)
        self.sim.setJointTargetVelocity(self.h_wr_r, wR)
        if self.h_wl_f is not None:
            self.sim.setJointTargetVelocity(self.h_wl_f, wL)
        if self.h_wr_f is not None:
            self.sim.setJointTargetVelocity(self.h_wr_f, wR)

    def stop(self):
        self.set_wheel_speeds(0.0, 0.0)

    def step_sleep(self):
        time.sleep(CONTROL_DT)

    def is_running(self) -> bool:
        st = self.sim.getSimulationState()
        return st != self.sim.simulation_stopped

# =========================
# CONTROLADOR (Roomba-like)
# =========================
class RoombaController:
    def __init__(self, hw: SimHardware):
        self.hw = hw
        # Começa direto no ZIGZAG
        self.state = State.RANDOM_WALK
        self.wall_side = WallSide.RIGHT

        self.last_progress_time = time.time()
        self.last_encoder_mean  = 0.0

        self.path_x = []
        self.path_y = []

        # -----------------------
        # MEMÓRIA: MAPA EM GRADE
        # -----------------------
        self.cell_size   = CELL_SIZE
        self.grid_width  = GRID_WIDTH
        self.grid_height = GRID_HEIGHT

        # Centraliza o grid na posição inicial do robô
        x0, y0, _ = self.hw.get_pose()
        self.map_origin_x = x0 - (self.grid_width  * self.cell_size) / 2.0
        self.map_origin_y = y0 - (self.grid_height * self.cell_size) / 2.0

        self.visited = [
            [0 for _ in range(self.grid_height)]
            for _ in range(self.grid_width)
        ]
        self.obstacles = [
            [False for _ in range(self.grid_height)]
            for _ in range(self.grid_width)
        ]

        # -----------------------
        # FIGURA 1: TRAJETÓRIA XY
        # -----------------------
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-', linewidth=2)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_title("Trajetória ao vivo")
        self.ax.set_aspect("equal", adjustable="box")

        # -----------------------
        # FIGURA 2: HEATMAP VISITADO
        # -----------------------
        self.fig_map, self.ax_map = plt.subplots()
        self.ax_map.set_title("Mapa de visitas (heatmap)")
        self.ax_map.set_xlabel("célula X")
        self.ax_map.set_ylabel("célula Y")

        self.heatmap = self.ax_map.imshow(
            self.visited,
            origin="lower",
            cmap="viridis",
            vmin=0,
            vmax=1
        )
        self.fig_map.colorbar(self.heatmap, ax=self.ax_map, label="Número de visitas")
        self._heatmap_update_counter = 0

        # -----------------------
        # PARÂMETROS DO ZIGZAG
        # -----------------------
        self.zigzag_initialized = False  # se já alinhou ou não
        self.zigzag_dir = 1             # +1 = indo para +X, -1 = indo para -X

    # ------- Funções auxiliares de mapa ------

    def _cell_in_bounds(self, cx: int, cy: int) -> bool:
        return (0 <= cx < self.grid_width) and (0 <= cy < self.grid_height)

    def _world_to_cell(self, x: float, y: float):
        """
        Converte posição contínua (x,y) no mundo
        para índices de célula (cx,cy). Retorna None se fora do mapa.
        """
        cx = int((x - self.map_origin_x) / self.cell_size)
        cy = int((y - self.map_origin_y) / self.cell_size)
        if not self._cell_in_bounds(cx, cy):
            return None
        return cx, cy

    def _update_visit_map(self):
        x, y, _ = self.hw.get_pose()
        idx = self._world_to_cell(x, y)
        if idx is not None:
            cx, cy = idx
            self.visited[cx][cy] += 1

    def _update_obstacle_from_front_sensor(self, d_front: float):
        if d_front >= D_STOP:
            return

        x, y, yaw = self.hw.get_pose()
        front_x = x + self.cell_size * math.cos(yaw)
        front_y = y + self.cell_size * math.sin(yaw)

        idx = self._world_to_cell(front_x, front_y)
        if idx is not None:
            cx, cy = idx
            self.obstacles[cx][cy] = True

    def _wrap_angle(self, ang: float) -> float:
        while ang > math.pi:
            ang -= 2.0 * math.pi
        while ang < -math.pi:
            ang += 2.0 * math.pi
        return ang

    def _update_heatmap(self):
        self.heatmap.set_data(self.visited)
        max_visits = max(max(row) for row in self.visited) if self.visited else 1
        if max_visits < 1:
            max_visits = 1
        self.heatmap.set_clim(0, max_visits)
        self.ax_map.set_title(f"Mapa de visitas (max={max_visits})")
        self.fig_map.canvas.draw_idle()

    # ------- Funções auxiliares do ZIGZAG -------

    def _drive_forward_for(self, distance, v=FORWARD_SPEED):
        """
        Anda para frente uma certa distância aproximada,
        usando velocidade constante e tempo = dist / v.
        """
        t = distance / max(1e-3, abs(v))
        self.hw.set_wheel_speeds(v, v)
        t0 = time.time()
        while time.time() - t0 < t:
            self.hw.step_sleep()
        self.hw.stop()

    def _init_zigzag_if_needed(self):
        """
        Na primeira vez que entrar no estado RANDOM_WALK,
        alinha o robô para 0 ou pi (eixo X) e define a direção inicial.
        """
        if self.zigzag_initialized:
            return

        _, _, yaw = self.hw.get_pose()
        yaw0 = 0.0
        yaw1 = math.pi
        err0 = abs(self._wrap_angle(yaw0 - yaw))
        err1 = abs(self._wrap_angle(yaw1 - yaw))
        target = yaw0 if err0 < err1 else yaw1

        # gira até o alvo
        self._rotate_relative(self._wrap_angle(target - yaw))

        # direção do zigzag (+X ou -X)
        self.zigzag_dir = 1 if target == 0.0 else -1
        self.zigzag_initialized = True

    # ------- CORE -------
    def step(self):
        d_front = self.hw.read_proximity_front()
        self._update_stuck_detector()

        # Atualiza memória
        self._update_visit_map()
        self._update_obstacle_from_front_sensor(d_front)

        # Máquina de estados
        if self.state == State.SPIRAL:
            # alias: se algum dia cair aqui, força ir para o zigzag
            print("STATE: Spiral (alias → Random Walk/zigzag)")
            self.state = State.RANDOM_WALK

        if self.state == State.WALL_FOLLOW:
            print("STATE: Wall Follow")
            self._state_wall_follow(d_front)
        elif self.state == State.RANDOM_WALK:
            print("STATE: Random Walk (zigzag)")
            self._state_random_walk(d_front)
        elif self.state == State.ESCAPE:
            print("STATE: Escape")
            self._state_escape()

        # Atualiza trajetória no Coppelia
        self.hw.push_graph_xy()

        # Atualiza gráfico de trajetória (fig 1)
        x, y, _ = self.hw.get_pose()
        self.path_x.append(x)
        self.path_y.append(y)
        self.line.set_xdata(self.path_x)
        self.line.set_ydata(self.path_y)
        self.ax.relim()
        self.ax.autoscale_view()

        # Atualiza heatmap periodicamente
        self._heatmap_update_counter += 1
        if self._heatmap_update_counter % 3 == 0:
            self._update_heatmap()

        plt.pause(0.001)
        self.hw.step_sleep()

    # ------- Estados (SPIRAL ficou só de exemplo, não é usado) -------
    def _state_spiral(self, d_front: float):
        """
        Mantido apenas como referência; o fluxo normal não usa SPIRAL.
        """
        self.hw.set_servo_angle(0)
        v0        = 0.10
        w_initial = 3.0
        w_final   = 0.50
        beta      = 0.18
        WHEEL_MAX = 25.0
        L         = ENTRE_EIXOS
        r         = RAIO_RODA
        
        if not hasattr(self, "_spiral_t0") or self._spiral_t0 is None:
            self._spiral_t0 = time.time()

        t = time.time() - self._spiral_t0
        w = w_final + (w_initial - w_final) * math.exp(-beta * t)
        v = v0 * (1.0 + 0.4 * (t / 20.0))

        w_left  = (v - 0.5 * w * L) / r
        w_right = (v + 0.5 * w * L) / r
        w_left  = max(-WHEEL_MAX, min(WHEEL_MAX, w_left))
        w_right = max(-WHEEL_MAX, min(WHEEL_MAX, w_right))

        self.hw.set_wheel_speeds(
            v_left=v - (w * L / 2),
            v_right=v + (w * L / 2)
        )

        if d_front < D_STOP:
            self.hw.stop()
            self._spiral_t0 = None
            print("backup (fim da espiral, indo para zigzag)")
            self._backup()
            self.zigzag_initialized = False
            self.state = State.RANDOM_WALK
            return

        if self._is_stuck():
            self._spiral_t0 = None
            self.state = State.ESCAPE
            return
        
    def _set_vw_direct(self, v, w):
        L = ENTRE_EIXOS
        vL = v - 0.5 * w * L
        vR = v + 0.5 * w * L
        self.hw.set_wheel_speeds(vL, vR)

    def _state_wall_follow(self, d_front: float):
        # Mantido aqui para compatibilidade; não é usado no fluxo atual.
        side = 1 if self.wall_side == WallSide.LEFT else -1
        sensor_angle = math.radians(60.0 * side)
        self.hw.set_servo_angle(sensor_angle)
        time.sleep(0.05)

        raw = self.hw.read_proximity_front()
        if raw >= 9.0:
            raw = None

        if not hasattr(self, "_fw_dist_filt"):
            self._fw_dist_filt = raw if raw is not None else 0.25
            self._fw_prev_dist = self._fw_dist_filt
            self._fw_prev_t = time.time()

        if raw is not None:
            alpha = 0.30
            self._fw_dist_filt = alpha*raw + (1-alpha)*self._fw_dist_filt

            d_set = 0.20
            e = d_set - self._fw_dist_filt

            t = time.time()
            dt = max(1e-3, t - self._fw_prev_t)
            de_dt = (e - (d_set - self._fw_prev_dist)) / dt

            self._fw_prev_dist = self._fw_dist_filt
            self._fw_prev_t = t

            deadband = 0.001
            if abs(e) < deadband:
                w_cmd = 0.0
            else:
                kP = 1.7
                kD = 0.6
                w_cmd = side*(kP*e + kD*de_dt)

            w_max = 0.8
            w_cmd = max(-w_max, min(w_max, w_cmd))

            if self._fw_dist_filt < 0.15:
                self._backup_escape()
                return

            self._set_vw_direct(0.12, w_cmd)

            if self._is_stuck():
                self.state = State.ESCAPE
                return
        else:
            self.zigzag_initialized = False
            self.state = State.RANDOM_WALK
            return

    def _state_random_walk(self, d_front: float):
        """
        RANDOM_WALK versão ZIGZAG (lawnmower):
        - Alinha o robô ao eixo X.
        - Anda reto até:
            * encontrar obstáculo na frente, ou
            * atingir a borda do grid em X.
        - Depois:
            * dá ré,
            * move 1 célula em Y,
            * inverte a direção em X.
        """
        self._init_zigzag_if_needed()

        x, y, yaw = self.hw.get_pose()
        idx = self._world_to_cell(x, y)
        if idx is None:
            # Se saiu do mapa, comportamento simples de recuperação
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
            return

        cx, cy = idx

        hit_front = d_front < D_STOP
        hit_border = (
            (self.zigzag_dir > 0 and cx >= self.grid_width - 1) or
            (self.zigzag_dir < 0 and cx <= 0)
        )

        if hit_front or hit_border:
            self.hw.stop()
            self._backup()

            x, y, yaw = self.hw.get_pose()

            if self.zigzag_dir > 0:
                # indo para +X: sobe 1 célula em +Y, vira para -X
                self._rotate_relative(self._wrap_angle(math.pi/2 - yaw))
                self._drive_forward_for(self.cell_size)
                _, _, yaw2 = self.hw.get_pose()
                self._rotate_relative(self._wrap_angle(math.pi - yaw2))
                self.zigzag_dir = -1
            else:
                # indo para -X: desce 1 célula em -Y, vira para +X
                self._rotate_relative(self._wrap_angle(-math.pi/2 - yaw))
                self._drive_forward_for(self.cell_size)
                _, _, yaw2 = self.hw.get_pose()
                self._rotate_relative(self._wrap_angle(0.0 - yaw2))
                self.zigzag_dir = 1
        else:
            self.hw.set_wheel_speeds(FORWARD_SPEED, FORWARD_SPEED)

        if self._is_stuck():
            self.state = State.ESCAPE

    def _state_escape(self):
        self.hw.stop()
        self._backup()
        best_dir = self._scan_best_dir()
        self._rotate_relative(best_dir)
        self.zigzag_initialized = False
        self.state = State.RANDOM_WALK
        self.last_progress_time = time.time()
        self.last_encoder_mean  = self._encoder_mean()

    # ------- Primitivas de controle -------
    def _rotate_relative(self, d_ang: float):
        self.hw.ensure_encoder_signs()

        target = abs(d_ang)
        direction = 1.0 if d_ang >= 0.0 else -1.0

        dL_prev, dR_prev = self.hw.read_wheel_encoders()
        theta = 0.0

        L = ENTRE_EIXOS
        OMEGA_MAX = 1.2
        OMEGA_MIN = 0.25
        TOL = math.radians(1.5)
        T_TIMEOUT = 6.0
        t0 = time.time()

        for _ in range(3):
            omega = direction * OMEGA_MIN
            vL = -omega * (L/2.0)
            vR =  omega * (L/2.0)
            self.hw.set_wheel_speeds(vL, vR)
            time.sleep(CONTROL_DT)

        while True:
            dL, dR = self.hw.read_wheel_encoders()
            incL = self.hw.enc_sign_L * (dL - dL_prev)
            incR = self.hw.enc_sign_R * (dR - dR_prev)
            dtheta = (incR - incL) / L
            theta += dtheta
            dL_prev, dR_prev = dL, dR

            remaining = target - abs(theta)

            if remaining <= TOL:
                break
            if time.time() - t0 > T_TIMEOUT:
                print("[rotate_relative] timeout; breaking")
                break

            omega_mag = min(OMEGA_MAX, max(OMEGA_MIN, 2.0 * remaining))
            omega = direction * omega_mag

            vL = -omega * (L/2.0)
            vR =  omega * (L/2.0)
            self.hw.set_wheel_speeds(vL, vR)
            time.sleep(CONTROL_DT)

        self.hw.stop()

    def _backup(self):
        self.hw.stop()
        time.sleep(0.05)
        self.hw.set_wheel_speeds(-0.12, -0.12)
        time.sleep(0.45)
        self.hw.stop()

    def _backup_escape(self):
        print("escape wall")
        self.hw.stop()
        time.sleep(0.05)
        self.hw.set_wheel_speeds(-0.12, -0.12)
        time.sleep(0.45)
        side = 1 if self.wall_side == WallSide.LEFT else -1
        self.hw.set_wheel_speeds(0.0, -0.25*side)
        time.sleep(0.25)

    # ------- Varreduras -------
    def _scan_best_dir(self) -> float:
        best_ang = 0.0
        best_d = -1.0
        for deg in range(-60, 61, 15):
            self.hw.set_servo_angle(math.radians(deg))
            time.sleep(0.06)
            d = self.hw.read_proximity_front()
            if d > best_d:
                best_d = d
                best_ang = math.radians(deg)
        self.hw.set_servo_angle(0.0)
        return best_ang

    def _choose_wall_side(self) -> WallSide:
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
    scene_file = None  # se a cena já estiver aberta no Coppelia, deixe None

    hw = SimHardware(scene_file=scene_file, start_paused=False)
    ctrl = RoombaController(hw)
    print("Scene loaded: starting simulation")

    try:
        while hw.is_running():
            ctrl.step()
        hw.stop()
    except KeyboardInterrupt:
        hw.stop()

if __name__ == '__main__':
    main()