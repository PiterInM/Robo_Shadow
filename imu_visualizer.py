"""
imu_visualizer.py — Visualizador 3D em tempo real do MPU-6050 (Robo Shadow)
Recebe leituras de acelerômetro e giroscópio do ESP32 via UDP.

Mapeamento de eixos físicos no peito do robô:
    +X → em direção à cabeça (cima)
    +Y → em direção ao braço direito (direita)
    +Z → perpendicular para fora do peito (frente)

Filtro Complementar:
    Combina a resposta rápida do giroscópio com a referência gravitacional
    estática do acelerômetro, eliminando drift de Roll e Pitch em tempo real.

Porta UDP de escuta : 4212 (ESP32 → PC)
Porta UDP de comando: 4210 (PC → ESP32)
"""

import socket
import threading
import time
import math
import numpy as np
import matplotlib
for _b in ['QtAgg', 'qtagg', 'Qt5Agg', 'TkAgg']:
    try:
        matplotlib.use(_b)
        break
    except Exception:
        pass
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.proj3d import proj_transform
from collections import deque
from matplotlib.widgets import Button
from window_icon import setup_matplotlib_icon

# ── Configuração ─────────────────────────────────────────────
IMU_RECV_PORT = 4212
ESP32_IP      = '192.168.4.1'
ESP32_PORT    = 4210
HISTORY_LEN   = 200
UPDATE_MS     = 40  # ~25 FPS na interface gráfica

ARROW_SCALE_ACCEL = 0.5

# Cores tema Dark / Cyberpunk
BG_COLOR    = '#0d1117'
PANEL_COLOR = '#161b22'
GRID_COLOR  = '#30363d'
TEXT_COLOR  = '#e6edf3'
TEXT_MUTED  = '#8b949e'
ACCENT_X    = '#58a6ff'  # Ciano   (Cabeça / X)
ACCENT_Y    = '#3fb950'  # Verde   (Braço D / Y)
ACCENT_Z    = '#ff7b72'  # Coral   (Peito / Z)
GYRO1       = '#d2a8ff'  # Roxo
GYRO2       = '#ffa657'  # Laranja
GYRO3       = '#79c0ff'  # Azul claro
ACCEL_VEC   = '#f0e040'  # Amarelo ouro (vetor resultante)


# ── Estado do IMU e Fusão Sensorial ──────────────────────────
class IMUState:
    def __init__(self):
        self.lock = threading.Lock()
        self.ax = self.ay = self.az = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        # Offsets de referência (postura em pé)
        self.roll_ref = self.pitch_ref = self.yaw_ref = 0.0
        self.ref_set = False  # True após o primeiro "Em Pé" ser definido
        self._last_t = None
        self.connected = False
        self.pkt_count = 0
        self.last_pkt_time = 0.0
        self.hz = 0.0
        self._hz_counter = 0
        self._hz_timer = time.monotonic()

        self.t_hist  = deque(maxlen=HISTORY_LEN)
        self.ax_hist = deque(maxlen=HISTORY_LEN)
        self.ay_hist = deque(maxlen=HISTORY_LEN)
        self.az_hist = deque(maxlen=HISTORY_LEN)
        self.gx_hist = deque(maxlen=HISTORY_LEN)
        self.gy_hist = deque(maxlen=HISTORY_LEN)
        self.gz_hist = deque(maxlen=HISTORY_LEN)
        self._t0 = time.monotonic()

    def update(self, ax, ay, az, gx, gy, gz):
        now = time.monotonic()
        with self.lock:
            self.ax, self.ay, self.az = ax, ay, az
            self.gx, self.gy, self.gz = gx, gy, gz
            self.connected = True
            self.pkt_count += 1
            self.last_pkt_time = now

            # Cálculo de taxa Hz
            self._hz_counter += 1
            if now - self._hz_timer >= 1.0:
                self.hz = self._hz_counter / (now - self._hz_timer)
                self._hz_counter = 0
                self._hz_timer = now

            # Filtro Complementar
            if self._last_t is not None:
                dt = now - self._last_t
                if dt <= 0.0 or dt > 0.5:
                    dt = 0.02

                norm_a = math.sqrt(ax**2 + ay**2 + az**2)
                # Quando a aceleração total está perto de 1g (sem choques bruscos)
                if 0.65 < norm_a < 1.35:
                    # Inclinação frontal (Pitch): eixo Y é a dobradiça (X=cabeça, Z=peito)
                    pitch_acc = math.degrees(math.atan2(-az, ax))
                    # Inclinação lateral (Roll): eixo Z é a dobradiça (X=cabeça, Y=braço D)
                    roll_acc  = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))

                    alpha = 0.95
                    self.pitch = alpha * (self.pitch + gy * dt) + (1.0 - alpha) * pitch_acc
                    self.roll  = alpha * (self.roll  + gz * dt) + (1.0 - alpha) * roll_acc
                else:
                    self.pitch += gy * dt
                    self.roll  += gz * dt

                # Giro vertical do tronco (Yaw): rotação em torno do eixo X (cabeça)
                self.yaw += gx * dt
            else:
                # Inicialização a partir do vetor gravidade
                norm_a = math.sqrt(ax**2 + ay**2 + az**2)
                if norm_a > 0.2:
                    self.pitch = math.degrees(math.atan2(-az, ax))
                    self.roll  = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
                self.yaw = 0.0

            self._last_t = now
            t_rel = now - self._t0
            self.t_hist.append(t_rel)
            self.ax_hist.append(ax); self.ay_hist.append(ay); self.az_hist.append(az)
            self.gx_hist.append(gx); self.gy_hist.append(gy); self.gz_hist.append(gz)

    def snapshot(self):
        with self.lock:
            # Verifica timeout de conexão (> 1.5s sem pacote)
            is_alive = self.connected and (time.monotonic() - self.last_pkt_time < 1.5)
            return {
                'ax': self.ax, 'ay': self.ay, 'az': self.az,
                'gx': self.gx, 'gy': self.gy, 'gz': self.gz,
                # Ângulos corrigidos pelo offset de referência (postura em pé = 0°)
                'roll':  self.roll  - self.roll_ref,
                'pitch': self.pitch - self.pitch_ref,
                'yaw':   self.yaw   - self.yaw_ref,
                'ref_set': self.ref_set,
                'connected': is_alive, 'pkt_count': self.pkt_count, 'hz': self.hz,
                't':    list(self.t_hist),
                'ax_h': list(self.ax_hist), 'ay_h': list(self.ay_hist), 'az_h': list(self.az_hist),
                'gx_h': list(self.gx_hist), 'gy_h': list(self.gy_hist), 'gz_h': list(self.gz_hist),
            }

    def reset_angles(self):
        with self.lock:
            norm_a = math.sqrt(self.ax**2 + self.ay**2 + self.az**2)
            if norm_a > 0.2:
                self.pitch = math.degrees(math.atan2(-self.az, self.ax))
                self.roll  = math.degrees(math.atan2(self.ay, math.sqrt(self.ax**2 + self.az**2)))
            else:
                self.pitch = 0.0
                self.roll  = 0.0
            self.yaw = 0.0

    def set_upright_reference(self):
        """Salva a orientação atual como referência de 'em pé' (zero relativo)."""
        with self.lock:
            self.roll_ref  = self.roll
            self.pitch_ref = self.pitch
            self.yaw_ref   = self.yaw
            self.ref_set   = True
        print('[IMU] Referência "em pé" definida: '
              f'roll={self.roll_ref:.1f}°  pitch={self.pitch_ref:.1f}°  yaw={self.yaw_ref:.1f}°')


imu = IMUState()


# ── Thread UDP ────────────────────────────────────────────────
def send_udp_cmd(cmd_str):
    """Envia um comando UDP para o ESP32."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.sendto(cmd_str.encode('utf-8'), (ESP32_IP, ESP32_PORT))
        s.close()
    except Exception as e:
        print(f"[UDP] Falha ao enviar '{cmd_str}': {e}")


def udp_recv_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    except Exception:
        pass
    sock.bind(('', IMU_RECV_PORT))
    sock.settimeout(0.5)
    print(f"[UDP] Escutando telemetria na porta {IMU_RECV_PORT}...")

    # Notifica o ESP32 da presença deste PC
    send_udp_cmd("IMU:PING")
    last_ping = time.monotonic()

    while True:
        try:
            data, addr = sock.recvfrom(128)
            txt = data.decode('utf-8', errors='ignore').strip()
            parts = {}
            for token in txt.split(','):
                if ':' in token:
                    k, v = token.split(':', 1)
                    try:
                        parts[k.strip()] = float(v.strip())
                    except ValueError:
                        pass
            if all(k in parts for k in ('ax','ay','az','gx','gy','gz')):
                imu.update(
                    parts['ax'], parts['ay'], parts['az'],
                    parts['gx'], parts['gy'], parts['gz']
                )
        except socket.timeout:
            # Reenvia ping se ainda não conectado a cada 2s
            s = imu.snapshot()
            if not s['connected'] and (time.monotonic() - last_ping > 2.0):
                send_udp_cmd("IMU:PING")
                last_ping = time.monotonic()
        except Exception as e:
            print(f"[UDP] Erro de recepção: {e}")


# ── Renderização 3D ───────────────────────────────────────────
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return np.min(zs)


def add_arrow3d(ax3d, origin, vec, color, lw=2.2):
    x0, y0, z0 = origin
    dx, dy, dz = vec
    arr = Arrow3D(
        [x0, x0 + dx], [y0, y0 + dy], [z0, z0 + dz],
        mutation_scale=13, lw=lw, arrowstyle='-|>', color=color
    )
    ax3d.add_artist(arr)
    return arr


def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    """
    Matriz de rotação 3D correspondente aos ângulos de Euler (Z-Y-X):
    roll (inclinação lateral), pitch (inclinação frontal), yaw (giro vertical).
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y),  math.cos(y), 0],
                   [0,            0,            1]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [0,            1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])
    Rx = np.array([[1, 0,           0            ],
                   [0, math.cos(r), -math.sin(r) ],
                   [0, math.sin(r),  math.cos(r) ]])
    return Rz @ Ry @ Rx


def draw_wire_box(ax3d, center, size, R, color, lw=1.2, alpha=0.6):
    """Desenha uma caixa wireframe 3D rotacionada."""
    dx, dy, dz = size[0] / 2, size[1] / 2, size[2] / 2
    pts = np.array([
        [-dx, -dy, -dz], [ dx, -dy, -dz], [ dx,  dy, -dz], [-dx,  dy, -dz],
        [-dx, -dy,  dz], [ dx, -dy,  dz], [ dx,  dy,  dz], [-dx,  dy,  dz],
    ])
    rot_pts = (R @ pts.T).T + (R @ np.array(center))
    edges = [
        (0,1), (1,2), (2,3), (3,0),
        (4,5), (5,6), (6,7), (7,4),
        (0,4), (1,5), (2,6), (3,7)
    ]
    for i, j in edges:
        ax3d.plot([rot_pts[i,0], rot_pts[j,0]],
                  [rot_pts[i,1], rot_pts[j,1]],
                  [rot_pts[i,2], rot_pts[j,2]],
                  color=color, lw=lw, alpha=alpha)


# ── Configuração dos Gráficos ──────────────────────────────────
def setup_figure():
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(16, 9), facecolor=BG_COLOR)
    try:
        fig.canvas.manager.set_window_title('Robo Shadow — IMU 3D Realtime (MPU-6050)')
    except Exception:
        pass
    setup_matplotlib_icon(fig)

    gs = gridspec.GridSpec(3, 3, figure=fig,
                           left=0.04, right=0.98,
                           top=0.92, bottom=0.06,
                           wspace=0.34, hspace=0.55)

    # Subplot 3D (ocupa toda a coluna esquerda)
    ax3d = fig.add_subplot(gs[:, 0], projection='3d')
    ax3d.set_facecolor(PANEL_COLOR)
    ax3d.view_init(elev=20, azim=45)

    # 3 subplots para Acelerômetro (coluna central)
    ax_acx = fig.add_subplot(gs[0, 1])
    ax_acy = fig.add_subplot(gs[1, 1])
    ax_acz = fig.add_subplot(gs[2, 1])

    # 3 subplots para Giroscópio (coluna direita)
    ax_gyx = fig.add_subplot(gs[0, 2])
    ax_gyy = fig.add_subplot(gs[1, 2])
    ax_gyz = fig.add_subplot(gs[2, 2])

    def style_ax(ax, title, color, ylabel, ylim):
        ax.set_facecolor(PANEL_COLOR)
        ax.set_title(title, color=color, fontsize=9, fontweight='bold', pad=4)
        ax.set_ylabel(ylabel, color=TEXT_MUTED, fontsize=7)
        ax.tick_params(colors=TEXT_MUTED, labelsize=7)
        ax.spines[:].set_color(GRID_COLOR)
        ax.grid(color=GRID_COLOR, alpha=0.45, linewidth=0.5)
        ax.set_xlim(0, HISTORY_LEN)
        ax.set_ylim(*ylim)
        ax.axhline(0, color=GRID_COLOR, linewidth=0.7, linestyle='--')

    style_ax(ax_acx, 'Accel X  (→ Cabeça)', ACCENT_X, 'g',   (-2.5, 2.5))
    style_ax(ax_acy, 'Accel Y  (→ Braço D)', ACCENT_Y, 'g',   (-2.5, 2.5))
    style_ax(ax_acz, 'Accel Z  (→ Peito)',   ACCENT_Z, 'g',   (-2.5, 2.5))
    style_ax(ax_gyx, 'Gyro X  (Yaw / Giro)', GYRO1,    '°/s', (-250, 250))
    style_ax(ax_gyy, 'Gyro Y  (Pitch / Frent)', GYRO2, '°/s', (-250, 250))
    style_ax(ax_gyz, 'Gyro Z  (Roll / Lado)',   GYRO3, '°/s', (-250, 250))

    # Título do topo
    fig.text(0.5, 0.975, 'ROBO SHADOW — TELEMETRIA IMU 3D  |  MPU-6050 (0x68)',
             ha='center', va='top', color=TEXT_COLOR, fontsize=12, fontweight='bold')
    status_txt = fig.text(0.5, 0.945, '● Conectando ao ESP32...',
                          ha='center', va='top', color='#888', fontsize=9)

    # Caixas HUD no painel 3D
    angles_txt = fig.text(0.05, 0.91,
                          'Roll:   0.0°\nPitch:  0.0°\nYaw:    0.0°',
                          va='top', color=TEXT_COLOR, fontsize=9, fontfamily='monospace',
                          bbox=dict(boxstyle='round,pad=0.5', facecolor=PANEL_COLOR, edgecolor=GRID_COLOR))

    accel_txt = fig.text(0.05, 0.12,
                         'Ax:  +0.000 g\nAy:  +0.000 g\nAz:  +0.000 g\n|a|:  0.000 g',
                         va='bottom', color=TEXT_COLOR, fontsize=8.5, fontfamily='monospace',
                         bbox=dict(boxstyle='round,pad=0.5', facecolor=PANEL_COLOR, edgecolor=GRID_COLOR))

    fig.text(0.70, 0.015,
             '[U] Em Pé (referência)   |   [R] Zerar ângulos   |   [C] Re-calibrar MPU   |   [Q] Sair',
             ha='center', color=TEXT_MUTED, fontsize=8)

    # Botão "Em Pé" — define referência de postura ereta
    btn_ax = fig.add_axes([0.04, 0.005, 0.13, 0.038])
    btn_upright = Button(
        btn_ax, '🤖  Em Pé',
        color=PANEL_COLOR, hovercolor='#1f6feb'
    )
    btn_upright.label.set_color(TEXT_COLOR)
    btn_upright.label.set_fontsize(9)
    btn_upright.label.set_fontweight('bold')
    for spine in btn_ax.spines.values():
        spine.set_edgecolor('#1f6feb')
        spine.set_linewidth(1.5)

    accel_axes = [ax_acx, ax_acy, ax_acz]
    gyro_axes  = [ax_gyx, ax_gyy, ax_gyz]

    return fig, ax3d, accel_axes, gyro_axes, status_txt, angles_txt, accel_txt, btn_upright


# ── Loop Principal da Interface Gráfica ───────────────────────
def run_visualizer():
    fig, ax3d, accel_axes, gyro_axes, status_txt, angles_txt, accel_txt, btn_upright = setup_figure()
    ax_acx, ax_acy, ax_acz = accel_axes
    ax_gyx, ax_gyy, ax_gyz = gyro_axes

    # Estado de flash de confirmação do botão
    _ref_flash = {'until': 0.0}

    def do_set_upright(_event=None):
        imu.set_upright_reference()
        _ref_flash['until'] = time.monotonic() + 2.5

    btn_upright.on_clicked(do_set_upright)

    line_ax, = ax_acx.plot([], [], color=ACCENT_X, lw=1.3)
    line_ay, = ax_acy.plot([], [], color=ACCENT_Y, lw=1.3)
    line_az, = ax_acz.plot([], [], color=ACCENT_Z, lw=1.3)
    line_gx, = ax_gyx.plot([], [], color=GYRO1,    lw=1.3)
    line_gy, = ax_gyy.plot([], [], color=GYRO2,    lw=1.3)
    line_gz, = ax_gyz.plot([], [], color=GYRO3,    lw=1.3)

    def on_key(event):
        if event.key in ('u', 'U'):
            do_set_upright()
        elif event.key in ('r', 'R'):
            imu.reset_angles()
            print('[IMU] Ângulos de referência zerados.')
        elif event.key in ('c', 'C'):
            print('[IMU] Enviando comando MODE:CAL ao ESP32... Mantenha o robô parado!')
            send_udp_cmd("MODE:CAL")
        elif event.key in ('q', 'Q'):
            plt.close('all')

    fig.canvas.mpl_connect('key_press_event', on_key)

    _closed = {'flag': False}

    def on_close(_event):
        # Janela fechada pelo usuário (botão X): trava o timer para não redesenhar
        # uma figura destruída e encerra o programa normalmente.
        _closed['flag'] = True
        timer.stop()

    fig.canvas.mpl_connect('close_event', on_close)

    def update(_):
        if _closed['flag']:
            return
        s = imu.snapshot()

        # Flash de confirmação da referência
        if time.monotonic() < _ref_flash['until']:
            status_txt.set_text('✔ Referência "Em Pé" definida! Ângulos zerados na postura atual.')
            status_txt.set_color('#ffa657')
            fig.canvas.draw_idle()
            return

        # Atualiza Status de Conexão
        if s['connected']:
            norm_a = math.sqrt(s["ax"]**2 + s["ay"]**2 + s["az"]**2)
            ref_indicator = '  |  📐 Ref: OK' if s['ref_set'] else ''
            status_txt.set_text(f'● CONECTADO — {s["hz"]:.0f} Hz ({s["pkt_count"]} pacotes)  |  Gravidade: {norm_a:.2f}g{ref_indicator}')
            status_txt.set_color('#3fb950')
        else:
            try:
                _s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                _s.connect(('192.168.4.1', 4210))
                _lip = _s.getsockname()[0]
                _s.close()
                if not _lip.startswith('192.168.4.'):
                    status_txt.set_text(f'⚠ PC EM OUTRA REDE ({_lip}) — Conecte ao Wi-Fi "RoboShadow" (senha: shadow123)')
                    status_txt.set_color('#ffa657')
                else:
                    status_txt.set_text(f'● Wi-Fi RoboShadow OK (IP: {_lip}) — Aguardando dados UDP na porta 4212...')
                    status_txt.set_color('#ff7b72')
            except Exception:
                status_txt.set_text('⚠ Wi-Fi desconectado! Conecte o PC ao Wi-Fi "RoboShadow" (senha: shadow123)')
                status_txt.set_color('#ffa657')

        # HUD Digital
        angles_txt.set_text(
            f"Roll:  {s['roll']:+6.1f}° (lado)\n"
            f"Pitch: {s['pitch']:+6.1f}° (frente)\n"
            f"Yaw:   {s['yaw']:+6.1f}° (giro)"
        )
        norm_a = math.sqrt(s["ax"]**2 + s["ay"]**2 + s["az"]**2)
        accel_txt.set_text(
            f"Ax: {s['ax']:+6.3f} g (cabeça)\n"
            f"Ay: {s['ay']:+6.3f} g (braço D)\n"
            f"Az: {s['az']:+6.3f} g (peito)\n"
            f"|a|: {norm_a:5.3f} g"
        )

        # ── Renderização 3D ──
        # Câmera fixa numa perspectiva frontal — o modelo gira via matriz R
        ax3d.cla()
        ax3d.view_init(elev=20, azim=-60)
        ax3d.set_facecolor(PANEL_COLOR)
        ax3d.set_xlim(-1.2, 1.2); ax3d.set_ylim(-1.2, 1.2); ax3d.set_zlim(-1.2, 1.2)
        ax3d.set_xlabel('Z → Peito',   color=ACCENT_Z, labelpad=2, fontsize=8)
        ax3d.set_ylabel('Y → Braço D', color=ACCENT_Y, labelpad=2, fontsize=8)
        ax3d.set_zlabel('X → Cabeça',  color=ACCENT_X, labelpad=2, fontsize=8)
        ax3d.tick_params(colors=GRID_COLOR, labelsize=6)
        ax3d.set_title('Orientação 3D do Tronco', color=TEXT_COLOR, fontsize=10, pad=4)

        for pane in (ax3d.xaxis.pane, ax3d.yaxis.pane, ax3d.zaxis.pane):
            pane.fill = False
            pane.set_edgecolor(GRID_COLOR)

        R = rotation_matrix(s['roll'], s['pitch'], s['yaw'])
        # Permutação: robot X (cabeça) → plot Z (vertical no mpl)
        #             robot Y (braço D) → plot Y
        #             robot Z (peito)  → plot X (profundidade)
        P = np.array([[0, 0, 1],
                      [0, 1, 0],
                      [1, 0, 0]], dtype=float)
        RP = P @ R
        O = [0, 0, 0]

        # Modelo Esquemático do Tronco do Robô — centros e tamanhos em coordenadas
        # do robô (espaço local). A matriz RP=P@R aplica a permutação automaticamente.
        # Torso: altura em X (cabeça), largura em Y (braços), profundidade em Z (peito)
        draw_wire_box(ax3d, [0, 0, 0],        [0.55, 0.42, 0.22], RP, color='#58a6ff', lw=1.2, alpha=0.7)
        # Cabeça no topo (+X robot)
        draw_wire_box(ax3d, [0.38, 0, 0],     [0.20, 0.20, 0.20], RP, color='#79c0ff', lw=1.0, alpha=0.6)
        # Ombro / Braço Direito (+Y robot)
        draw_wire_box(ax3d, [0.15, 0.28, 0],  [0.12, 0.16, 0.14], RP, color=ACCENT_Y, lw=1.0, alpha=0.6)
        # Ombro / Braço Esquerdo (-Y robot)
        draw_wire_box(ax3d, [0.15, -0.28, 0], [0.12, 0.16, 0.14], RP, color='#a371f7', lw=1.0, alpha=0.6)

        # Eixos do Sensor (permutados para plot space)
        ex = RP @ np.array([1, 0, 0])   # cabeça → aponta para cima (+Z plot)
        ey = RP @ np.array([0, 1, 0])   # braço D → aponta para +Y
        ez = RP @ np.array([0, 0, 1])   # peito → aponta para +X

        add_arrow3d(ax3d, O, ex * 0.85, ACCENT_X, lw=2.5)
        add_arrow3d(ax3d, O, ey * 0.85, ACCENT_Y, lw=2.5)
        add_arrow3d(ax3d, O, ez * 0.85, ACCENT_Z, lw=2.5)

        ax3d.text(*(ex * 0.98), '+X\nCABEÇA', color=ACCENT_X, fontsize=7.5, fontweight='bold', ha='center')
        ax3d.text(*(ey * 0.98), '+Y\nBRAÇO D', color=ACCENT_Y, fontsize=7.5, fontweight='bold', ha='center')
        ax3d.text(*(ez * 0.98), '+Z\nPEITO',  color=ACCENT_Z, fontsize=7.5, fontweight='bold', ha='center')

        # Vetor de Aceleração Resultante (permutado para plot space)
        if norm_a > 0.05:
            av_robot = np.array([s['ax'], s['ay'], s['az']]) * ARROW_SCALE_ACCEL
            av = P @ av_robot
            add_arrow3d(ax3d, O, av, ACCEL_VEC, lw=2.8)
            tip = av * 1.08
            ax3d.text(*tip, f'{norm_a:.2f}g', color=ACCEL_VEC, fontsize=8, fontweight='bold')

        # Cubo delimitador sutil
        for v in [-1, 1]:
            ax3d.plot([-1, 1], [v, v], [v, v], color=GRID_COLOR, alpha=0.15, lw=0.5)
            ax3d.plot([v, v], [-1, 1], [v, v], color=GRID_COLOR, alpha=0.15, lw=0.5)
            ax3d.plot([v, v], [v, v], [-1, 1], color=GRID_COLOR, alpha=0.15, lw=0.5)

        # ── Atualização dos Gráficos Históricos ──
        n = len(s['ax_h'])
        xs = list(range(n))
        x_min = max(0, n - HISTORY_LEN)
        x_max = n if n > HISTORY_LEN else HISTORY_LEN

        # Acelerômetro
        for line, data, ax_s in zip([line_ax, line_ay, line_az],
                                    [s['ax_h'], s['ay_h'], s['az_h']],
                                    accel_axes):
            line.set_data(xs, data)
            ax_s.set_xlim(x_min, x_max)

        # Giroscópio (auto-escala adaptativa)
        all_gyro = s['gx_h'] + s['gy_h'] + s['gz_h']
        max_g = max((max(abs(v) for v in all_gyro) if all_gyro else 30), 30)
        for line, data, ax_s in zip([line_gx, line_gy, line_gz],
                                    [s['gx_h'], s['gy_h'], s['gz_h']],
                                    gyro_axes):
            line.set_data(xs, data)
            ax_s.set_xlim(x_min, x_max)
            ax_s.set_ylim(-max_g * 1.2, max_g * 1.2)

        # Força redesenho de todos os subplots a cada tick
        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=UPDATE_MS)
    timer.add_callback(update, None)
    timer.start()

    plt.show()


def main():
    t = threading.Thread(target=udp_recv_thread, daemon=True)
    t.start()
    print("=" * 60)
    print("  🤖 ROBO SHADOW — IMU 3D Visualizer (MPU-6050)")
    print("=" * 60)
    print(f"  Recepção UDP       : porta {IMU_RECV_PORT}")
    print(f"  Comandos para ESP32: {ESP32_IP}:{ESP32_PORT}")
    print()
    print("  Atalhos de teclado:")
    print("    [R] Zerar ângulos de referência")
    print("    [C] Enviar comando de calibração ao ESP32")
    print("    [Q] Sair")
    print("=" * 60)
    run_visualizer()


if __name__ == '__main__':
    main()
