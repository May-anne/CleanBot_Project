# Requisitos: pip install coppeliasim-zmqremoteapi-client
# Inicie o CoppeliaSim com o serviço ZMQ habilitado (menu: Remote API → "Enable remote API server").

import time
import math
import random
import numpy as np
from enum import Enum, auto
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import matplotlib.pyplot as plt
import os
import cv2
plt.ion()

# =========================
# PARÂMETROS GERAIS / PLANEJADOR
# =========================
NOME_BASE                = '/pure_base'
NOME_JUNTA_RL            = '/lr_wheel_joint'
NOME_JUNTA_RR            = '/rr_wheel_joint'
NOME_JUNTA_FL            = '/lf_wheel_joint'
NOME_JUNTA_FR            = '/rf_wheel_joint'
NOME_JUNTA_SERVO         = '/support_joint'          # junta que gira o sensor
NOME_SENSOR_PROX         = '/proximitySensor'        # sensor de proximidade (frontal no servo)

RAIO_RODA                = 0.0327      # m
ENTRE_EIXOS              = 0.14        # m
D_STOP                   = 0.30        # m: “bateu”
MAX_STUCK_TIME           = 1.5         # s
FORWARD_SPEED            = 0.15        # m/s
CONTROL_DT               = 0.05        # s

# --- Constantes do Mapeamento ---
MAP_FILE_NPZ = "map.npz"
MAP_FILE_PNG = "map.png"
MAP_EXPLORATION_TIME_S = 2 * 3600 # 2 horas

FREE_LOGODDS, OCC_LOGODDS = -0.4, +0.85
CLAMP_MIN,   CLAMP_MAX   = -4.0, +4.0

WALL_DISTANCE = 0.10
SWEEP_CELL_SIZE = 0.25
LOG_ODDS_FREE_THRESHOLD = -1.0

# =========================
# ESTADOS
# =========================

class MacroState(Enum):
    MAP_EXPLORATION = auto()
    COVERAGE_PLANNING = auto()
    COVERAGE_EXECUTION = auto()

class State(Enum):
    SPIRAL  = auto()
    WALL_FOLLOW     = auto()
    RANDOM_WALK     = auto()
    ESCAPE          = auto()

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

        if scene_file:
            self.sim.stopSimulation()
            while self.sim.getSimulationState() != self.sim.simulation_stopped:
                time.sleep(0.05)
            self.sim.closeScene()
            self.sim.loadScene(scene_file)

        if start_paused:
            self.sim.startSimulation()
            self.sim.pauseSimulation()
        else:
            self.sim.startSimulation()

        self.h_base   = self.sim.getObject(NOME_BASE)

        self.h_wl_r   = self.sim.getObject(NOME_JUNTA_RL)
        self.h_wr_r   = self.sim.getObject(NOME_JUNTA_RR)
        self.h_wl_f   = self.sim.getObject(NOME_JUNTA_FL) if NOME_JUNTA_FL else None
        self.h_wr_f   = self.sim.getObject(NOME_JUNTA_FR) if NOME_JUNTA_FR else None
        self.h_servo  = self.sim.getObject(NOME_JUNTA_SERVO)
        self.h_prox   = self.sim.getObject(NOME_SENSOR_PROX)

        # Juntas em modo velocidade
        for h in [self.h_wl_r, self.h_wr_r, self.h_wl_f, self.h_wr_f]:
            if h is not None:
                self.sim.setObjectFloatParam(h, self.sim.jointfloatparam_maxvel, 9999.0)
                self.sim.setObjectInt32Param(h, self.sim.jointintparam_dynctrlmode, self.sim.jointdynctrl_velocity)

        self.set_servo_angle(0.0)

        self._last_encoder_wl = self.sim.getJointPosition(self.h_wl_r)
        self._last_encoder_wr = self.sim.getJointPosition(self.h_wr_r)
        self._cum_dist_wl     = 0.0
        self._cum_dist_wr     = 0.0

        self._last_progress_time = time.time()
        self._last_mean_dist     = 0.0

        if start_paused:
            self.sim.resumeSimulation()

    def get_pose(self):
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
    def read_proximity_front(self) -> tuple[bool, float, tuple[float, float, float]]:
        """
        Retorna (hit, distance, p_hit_world).
        - p_hit_world SEMPRE em coordenadas do mundo.
        """
        sim = self.sim
        sim.handleProximitySensor(self.h_prox)
        state, distance, point_local, _, _ = sim.readProximitySensor(self.h_prox)

        # matriz do sensor no mundo
        M = sim.getObjectMatrix(self.h_prox, -1)  # 12 floats (3x4)

        def local_to_world(px, py, pz):
            # multiplica ponto local pela matriz 3x4 do Coppelia (sem escala)
            x = M[0]*px + M[1]*py + M[2]*pz + M[3]
            y = M[4]*px + M[5]*py + M[6]*pz + M[7]
            z = M[8]*px + M[9]*py + M[10]*pz + M[11]
            return (x, y, z)

        if state:
            px, py, pz = point_local
            p_hit_world = local_to_world(px, py, pz)
            return True, float(distance), p_hit_world
        else:
            # sem hit: projeta eixo-x do sensor no mundo
            max_range = 1.0
            # ponto 1m à frente do sensor no frame do sensor
            p_local = (max_range, 0.0, 0.0)
            p_hit_world = local_to_world(*p_local)
            return False, max_range, p_hit_world

    def read_yaw(self) -> float:
        euler = self.sim.getObjectOrientation(self.h_base, -1)
        return euler[2]

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
# MAPA (grid de ocupação)
# =========================
class OccupancyGrid:
    def __init__(self, res=0.02, size_x=600, size_y=600, origin_world=(-6.0, -6.0)):
        self.res, self.size_x, self.size_y = res, size_x, size_y
        self.ox, self.oy = origin_world
        self.grid = np.zeros((size_y, size_x), dtype=np.float32)

    def world_to_grid(self, x, y):
        gx = int((x - self.ox) / self.res)
        gy = int((y - self.oy) / self.res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = self.ox + (gx + 0.5)*self.res
        y = self.oy + (gy + 0.5)*self.res
        return x, y

    def bresenham(self, x0, y0, x1, y1):
        x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
        dx = abs(x1 - x0); sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            yield (x0, y0)
            if x0 == x1 and y0 == y1: break
            e2 = 2 * err
            if e2 >= dy: err += dy; x0 += sx
            if e2 <= dx: err += dx; y0 += sy

    def update_ray(self, x_sens, y_sens, x_hit, y_hit, hit=True):
        gx0, gy0 = self.world_to_grid(x_sens, y_sens)
        gx1, gy1 = self.world_to_grid(x_hit,  y_hit)
        cells = list(self.bresenham(gx0, gy0, gx1, gy1))
        if not cells: return
        free_cells = cells[:-1] if hit else cells
        for gx, gy in free_cells:
            if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
                self.grid[gy, gx] = np.clip(self.grid[gy, gx] + FREE_LOGODDS, CLAMP_MIN, CLAMP_MAX)
        if hit:
            gx, gy = cells[-1]
            if 0 <= gx < self.size_x and 0 <= gy < self.size_y:
                self.grid[gy, gx] = np.clip(self.grid[gy, gx] + OCC_LOGODDS, CLAMP_MIN, CLAMP_MAX)

    def to_binary(self, thr=0.0):
        return (self.grid < thr).astype(np.uint8)  # <0 = livre

    def save(self, path_npz="map.npz", path_png=None):
        np.savez_compressed(path_npz, grid=self.grid, res=self.res, ox=self.ox, oy=self.oy)
        if path_png:
            plt.figure(figsize=(8,6))
            prob_map = 1.0 / (1.0 + np.exp(-self.grid))
            plt.imshow(prob_map, cmap='gray', origin='lower', vmin=0.0, vmax=1.0)
            plt.colorbar(label='Probabilidade de Ocupação')
            plt.tight_layout()
            plt.savefig(path_png, dpi=180)
            plt.close()

    @staticmethod
    def load(path_npz="map.npz"):
        if not os.path.exists(path_npz):
            return None
        try:
            z = np.load(path_npz)
            M = OccupancyGrid(res=float(z["res"]),
                              size_x=z["grid"].shape[1], size_y=z["grid"].shape[0],
                              origin_world=(float(z["ox"]), float(z["oy"])))
            M.grid = z["grid"]
            print(f"Mapa carregado de {path_npz}")
            return M
        except Exception as e:
            print(f"Erro ao carregar mapa: {e}")
            return None

# =========================
# CONTROLADOR (Roomba-like)
# =========================
class RoombaController:
    def __init__(self, hw: SimHardware):
        self.hw = hw
        # Estado inicial depende da existência do mapa
        self.occupancy_grid = OccupancyGrid.load(MAP_FILE_NPZ)
        if self.occupancy_grid is None:
            self.occupancy_grid = OccupancyGrid()
            print("Nenhum mapa encontrado. Iniciando fase de Exploração (2h)")
            self.MacroState = MacroState.MAP_EXPLORATION
            self.state = State.SPIRAL
            self._exploration_t0 = time.time()
            self._map_scan_angle = 0
        else:
            print("Mapa encontrado. Iniciando fase de Planejamento de Cobertura.")
            self.MacroState = MacroState.COVERAGE_PLANNING

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
        hit, d_front, p_hit_world = self.hw.read_proximity_front()

        self._update_stuck_detector()
        if self.MacroState == MacroState.MAP_EXPLORATION:
            # Obtém a pose do sensor
            h_sensor = self.hw.sim.getObject(NOME_SENSOR_PROX)
            p_sensor = self.hw.sim.getObjectPosition(h_sensor, -1)
            
            # Atualiza o grid: p_sensor é a origem do raio, p_hit_world é o ponto de hit (ou alcance máximo)
            self.occupancy_grid.update_ray(p_sensor[0], p_sensor[1], p_hit_world[0], p_hit_world[1], hit=hit)


        if self.MacroState == MacroState.MAP_EXPLORATION:
            print(f"STATE: Map Exploration ({int(time.time() - self._exploration_t0)}s/{MAP_EXPLORATION_TIME_S}s)")
            self._state_map_exploration(d_front)
            
        elif self.MacroState == MacroState.COVERAGE_PLANNING:
            print("STATE: Coverage Planning (PLACEHOLDER)")
            self._state_coverage_planning()
        elif self.MacroState == MacroState.COVERAGE_EXECUTION:
            print(f"STATE: Coverage Execution (Waypoint {self.current_waypoint_idx}/{len(self.coverage_waypoints)})")
            self._state_coverage_execution()

        # gráfico + registro
        x, y, _ = self.hw.get_pose()
        self.path_x.append(x); self.path_y.append(y)
        self.line.set_xdata(self.path_x); self.line.set_ydata(self.path_y)
        self.ax.relim(); self.ax.autoscale_view()
        plt.pause(0.001)
        self.hw.step_sleep()


    def _state_map_exploration(self, d_front: float):
        """
        Exploração aleatória com foco em evitar obstáculos e mapear.
        Usa o algoritmo Random Walk ou Espiral/Wall Follow simplificado.
        """
        # Verifica se o tempo limite foi atingido (2 horas)
        if time.time() - self._exploration_t0 >= MAP_EXPLORATION_TIME_S:
            print("Tempo de exploração de mapa esgotado. Salvando mapa e planejando rota.")
            self.occupancy_grid.save(MAP_FILE_NPZ, MAP_FILE_PNG)
            self.MacroState = MacroState.COVERAGE_PLANNING
            return
            
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

    def _state_coverage_planning(self):
        """
        1. Prepara o mapa binário.
        2. Identifica a área livre conhecida (Free Space).
        3. Gera uma rota de varredura (Boustrophedon) para cobrir essa área.
        """
        self.hw.stop()

        print("Iniciando Planejamento de Cobertura...")

        # 1. Obter o mapa binário da área livre
        # O mapa binário (1 = ocupado / 0 = livre/desconhecido)
        log_odds_free_space = self.occupancy_grid.grid < LOG_ODDS_FREE_THRESHOLD
        
        # 2. Expansão (Erosion/Dilation) para garantir margem de segurança
        free_space_grid = log_odds_free_space.astype(np.uint8) * 255 
        kernel = np.ones((5,5),np.uint8)
        eroded_free_space = cv2.erode(free_space_grid, kernel, iterations = 1)
        
        # 3. Gerar Waypoints usando Varredura Simples (Boustrophedon Simplificado)
        waypoints = []
        
        step_x = int(SWEEP_CELL_SIZE / self.occupancy_grid.res)
        start_y_dir = True

        for gx in range(0, self.occupancy_grid.size_x, step_x):
            

            col_free = eroded_free_space[:, gx]

            y_indices = np.where(col_free > 0)[0]
            
            if y_indices.size > 0:
                # Varredura Boustrophedon: Alterne a direção Y a cada coluna X
                if start_y_dir:
                    y_coords = y_indices # Varre Y crescente
                else:
                    y_coords = y_indices[::-1] # Varre Y decrescente

                for gy in y_coords:
                    # Converte coordenada de grid (gx, gy) para coordenada do mundo (x, y)
                    x = self.occupancy_grid.ox + (gx + 0.5) * self.occupancy_grid.res
                    y = self.occupancy_grid.oy + (gy + 0.5) * self.occupancy_grid.res
                    waypoints.append((x, y))
            
                start_y_dir = not start_y_dir # Alterna a direção para a próxima coluna

        if not waypoints:
            print("Área livre insuficiente para planejamento. Fim da simulação.")
            return

        print(f"Planejamento concluído! {len(waypoints)} waypoints gerados.")

        self._draw_path_as_strip(waypoints, size=3)
        self._save_planned_path_png(waypoints)
        
        # Salva os waypoints e transiciona
        self.coverage_waypoints = waypoints
        self.current_waypoint_idx = 0
        self.MacroState = MacroState.COVERAGE_EXECUTION

    def _state_coverage_execution(self):
        """
        Executa a rota de waypoints gerada pelo _state_coverage_planning.
        Usa lógica Go-to-Goal simples.
        """
        if self.current_waypoint_idx >= len(self.coverage_waypoints):
            print("Execução da cobertura concluída! Finalizando simulação.")
            self.hw.stop()
            self.hw.sim.pauseSimulation()
            return

        # Ponto alvo (Goal)
        target_x, target_y = self.coverage_waypoints[self.current_waypoint_idx]
        current_x, current_y, current_yaw = self.hw.get_pose()
        
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        TOLERANCE = 0.05 # m

        if distance < TOLERANCE:
            print(f"Waypoint {self.current_waypoint_idx} alcançado.")
            self.current_waypoint_idx += 1
            return 
        
        # 1. Controle Angular (Girar para o alvo)
        # Ângulo desejado no mundo
        target_angle_world = math.atan2(dy, dx)
        
        # Erro angular (diferença entre o ângulo desejado e o ângulo atual)
        angle_error = target_angle_world - current_yaw
        
        # Normaliza o erro angular para o intervalo [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # 2. PID/Ganho Simples para Velocidade Angular (w)
        Kp_w = 2.5 # Ganho Proporcional para o giro
        w = Kp_w * angle_error
        
        # 3. Velocidade Linear (v)
        # Reduz a velocidade linear se o erro angular for grande
        Kp_v = 0.15 # Velocidade máxima
        v = Kp_v * math.cos(angle_error) # V = V_max * cos(erro) para suavizar
        
        # Limita v e w
        v_final = max(0.0, min(v, FORWARD_SPEED))
        w_final = max(-2.0, min(w, 2.0))
        
        # Aplica V e W ao robô (o self._set_vw_direct já faz a conversão)
        self._set_vw_direct(v_final, w_final)
        
        # DEBUG: O robô deve parar de se mover se ficar preso durante a execução
        if self._is_stuck():
            print("Preso durante a execução de cobertura!")
            self.hw.stop()
            self.hw.sim.pauseSimulation()
    
    def _draw_path_as_strip(self, waypoints, size=3):
        sim = self.hw.sim
        flags = sim.drawing_linestrip
        handle = sim.addDrawingObject(flags, float(size), 0.0, -1, 0)
        z = 0.01
        pts = []
        for x, y in waypoints:
            pts.extend([float(x), float(y), float(z)])
        sim.addDrawingObjectItem(handle, pts)

    def _save_planned_path_png(self, waypoints, filename="planned_path.png"):
        """
        Gera um PNG mostrando:
        - mapa de ocupação (probabilidade)
        - caminho planejado (linha azul)
        """
        print(f"Gerando PNG do caminho planejado: {filename}")

        grid = self.occupancy_grid.grid
        prob = 1.0 / (1.0 + np.exp(-grid))  # log-odds -> probabilidade

        plt.figure(figsize=(7, 7))
        plt.imshow(prob, cmap="gray", origin="lower")

        # Converte waypoints (x,y) para coordenadas de grid
        xs = []
        ys = []
        for (x, y) in waypoints:
            gx, gy = self.occupancy_grid.world_to_grid(x, y)
            xs.append(gx)
            ys.append(gy)

        # Desenha linha em azul
        plt.plot(xs, ys, color="blue", linewidth=1.0)

        plt.title("Caminho planejado (Coverage Path)")
        plt.xlabel("Grid X")
        plt.ylabel("Grid Y")
        plt.tight_layout()
        plt.savefig(filename, dpi=200)
        plt.close()


    # ------- Estados de locomoção -------
    def _state_spiral(self, d_front: float):
        print("STATE: Explore (Spiral)")
        self.hw.set_servo_angle(0)
        v0, w_initial, w_final, beta = 0.10, 3.0, 0.50, 0.18
        L, r = ENTRE_EIXOS, RAIO_RODA

        if not hasattr(self, "_spiral_t0") or self._spiral_t0 is None:
            self._spiral_t0 = time.time()
        t = time.time() - self._spiral_t0
        w = w_final + (w_initial - w_final) * math.exp(-beta * t)
        v = v0 * (1.0 + 0.4 * (t / 20.0))
        self.hw.set_wheel_speeds(v - (w * L / 2), v + (w * L / 2))

        # colisão? troca para follow-wall
        if d_front < D_STOP:
            self.hw.stop()
            self._spiral_t0 = None
            self._backup()
            self.wall_side = self._choose_wall_side()
            self.state = State.WALL_FOLLOW
            return
        if self._is_stuck():
            self._spiral_t0 = None
            self.state = State.ESCAPE
            return

    def _set_vw_direct(self, v, w):
        L = ENTRE_EIXOS
        self.hw.set_wheel_speeds(v - 0.5*w*L, v + 0.5*w*L)

    def _state_wall_follow(self, d_front: float):
        print("STATE: Wall Follow")
        side = 1 if self.wall_side == WallSide.LEFT else -1
        sensor_angle = math.radians(60.0 * side)
        self.hw.set_servo_angle(sensor_angle)
        time.sleep(0.05)

        _,raw,_ = self.hw.read_proximity_front()
        if raw >= 1.0: raw = None
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
            if abs(e) < deadband: w_cmd = 0.0
            else:
                kP, kD = 1.7, 0.6
                w_cmd = side*(kP*e + kD*de_dt)

            w_cmd = max(-0.8, min(0.8, w_cmd))
            if self._fw_dist_filt < 0.15:
                self._backup_escape(); return
            self._set_vw_direct(0.12, w_cmd)
            if self._is_stuck():
                self.state = State.ESCAPE; return
        else:
            self.state = State.RANDOM_WALK; return

    def _state_random_walk(self, d_front: float):
        print("STATE: Random Walk")
        self.hw.set_wheel_speeds(FORWARD_SPEED, FORWARD_SPEED)
        if d_front < D_STOP:
            self.hw.stop()
            self._backup()
            ang = math.radians(60) + random.random() * math.radians(60)
            if random.random() < 0.5: ang = -ang
            self._rotate_relative(ang)
        if self._is_stuck():
            self.state = State.ESCAPE

    def _state_escape(self):
        print("STATE: Escape")
        self.hw.stop()
        self._backup()
        best_dir = self._scan_best_dir()
        self._rotate_relative(best_dir)
        self.state = State.RANDOM_WALK
        self.last_progress_time = time.time()
        self.last_encoder_mean  = self._encoder_mean()


    def _goto_waypoint(self, tx, ty, v_nom=0.12, tol=0.06):
        x, y, yaw = self.hw.get_pose()
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        if dist < tol:
            self.hw.stop(); return True
        # pure pursuit simplificado
        ang = math.atan2(dy, dx)
        heading_err = (ang - yaw + math.pi) % (2*math.pi) - math.pi
        w = 2.0 * heading_err
        v = max(0.05, min(v_nom, 0.10 + 0.3*dist))
        self._set_vw_direct(v, w)
        # pequena evasão se obstáculo frontal
        _, dist, _ = self.hw.read_proximity_front()
        if  dist < D_STOP*0.9:
            self.hw.stop(); self._backup(); self._rotate_relative(math.radians(45))
        return False

    # ------- Primitivas de controle -------
    def _rotate_relative(self, d_ang: float):
        self.hw.ensure_encoder_signs()
        target = abs(d_ang)
        direction = 1.0 if d_ang >= 0.0 else -1.0
        dL_prev, dR_prev = self.hw.read_wheel_encoders()
        theta = 0.0
        L = ENTRE_EIXOS
        OMEGA_MAX, OMEGA_MIN = 1.2, 0.25
        TOL = math.radians(1.5)
        T_TIMEOUT = 3.0
        t0 = time.time()
        for _ in range(3):
            omega = direction * OMEGA_MIN
            vL = -omega * (L/2.0); vR =  omega * (L/2.0)
            self.hw.set_wheel_speeds(vL, vR); time.sleep(CONTROL_DT)
        while True:
            dL, dR = self.hw.read_wheel_encoders()
            incL = self.hw.enc_sign_L * (dL - dL_prev)
            incR = self.hw.enc_sign_R * (dR - dR_prev)
            dtheta = (incR - incL) / L
            theta += dtheta
            dL_prev, dR_prev = dL, dR
            remaining = target - abs(theta)
            if remaining <= TOL or (time.time()-t0) > T_TIMEOUT: break
            omega_mag = min(OMEGA_MAX, max(OMEGA_MIN, 2.0 * remaining))
            omega = direction * omega_mag
            vL = -omega * (L/2.0); vR =  omega * (L/2.0)
            self.hw.set_wheel_speeds(vL, vR); time.sleep(CONTROL_DT)
        self.hw.stop()

    def _backup(self):
        self.hw.stop(); time.sleep(0.05)
        self.hw.set_wheel_speeds(-0.12, -0.12); time.sleep(0.45)
        self.hw.stop()

    def _backup_escape(self):
        print("escape wall")
        self.hw.stop(); time.sleep(0.05)
        self.hw.set_wheel_speeds(-0.12, -0.12); time.sleep(0.45)
        side = 1 if self.wall_side == WallSide.LEFT else -1
        self.hw.set_wheel_speeds(0.0, -0.25*side); time.sleep(0.25)

    # ------- Varreduras -------
    def _scan_best_dir(self) -> float:
        best_ang, best_d = 0.0, -1.0
        for deg in range(-60, 61, 15):
            self.hw.set_servo_angle(math.radians(deg))
            time.sleep(0.06)
            _, d, _ = self.hw.read_proximity_front()
            if d > best_d:
                best_d, best_ang = d, math.radians(deg)
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
        return WallSide.LEFT if dL < dR else WallSide.RIGHT

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
    #scene_file = 'C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/scenes/CleanBot.ttt'
    scene_file = 'C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/scenes/HouseSimulationCleanBot.ttt'

    hw = SimHardware(scene_file=scene_file, start_paused=False)
    ctrl = RoombaController(hw)
    print("Scene loaded: starting simulation")

    hw.set_servo_angle(0.0)

    try:
        while hw.is_running():
            ctrl.step()
        hw.stop()
    except KeyboardInterrupt:
        hw.stop()

    # Salva o mapa final ao sair
    if ctrl.occupancy_grid is not None:
        print("Salvando mapa final ao sair.")
        ctrl.occupancy_grid.save(MAP_FILE_NPZ, MAP_FILE_PNG)

if __name__ == '__main__':
    main()
