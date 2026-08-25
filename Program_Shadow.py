# Importação de bibliotecas OpenCV, MediaPipe, Math, NumPy e PySerial
import cv2
import mediapipe as mp
import math
import numpy as np
import serial

# ============================================================
# Configurações e constantes
# ============================================================

# Declaração porta do ESP32
porta = '/dev/ttyUSB0'  # Coloque entre as aspas simples a porta serial do Esp32
velocComunc = 115200  # Coloque aqui a velocidade da comunicação serial

# Definindo cores
corPontos = (0, 0, 255)
corTexto = (0, 0, 0)
corHudTexto = (255, 255, 255)
corHudBorda = (0, 0, 0)
amarelo = (200, 200, 0)
magenta = (200, 0, 200)
ciano = (0, 200, 200)

# Configuração da MediaPipe Pose
# model_complexity: 0=lite, 1=full, 2=heavy. Heavy tem Z bem mais estável, custa CPU.
MODEL_COMPLEXITY = 2

# Gate de visibilidade: landmarks com visibility abaixo disso são ignorados,
# e o último valor de servo é reusado.
VIS_THRESHOLD = 0.5

# Parâmetros do One Euro Filter (Casiez, Roussel, Vogel 2012)
# min_cutoff: cutoff base (Hz) — menor = mais suave parado
# beta: sensibilidade à velocidade — maior = menos lag em movimento rápido
# d_cutoff: cutoff da derivada (Hz)
ONE_EURO_MIN_CUTOFF = 1.0
ONE_EURO_BETA = 0.007
ONE_EURO_DCUTOFF = 1.0

# Mapeamento de cada eixo do robô para servo (mantém ranges numéricos do firmware).
# zero_deg: valor de servo na pose neutra (vem de PosPadrao() no .ino)
# scale: multiplicador do ângulo em graus (normalmente 1.0)
# invert: True inverte o sentido do movimento (flipar se o servo mover ao contrário no teste)
# lo/hi: clamp do valor final enviado ao servo
SERVO_MAP = {
    'OD': {'zero_deg': 175, 'scale': 1.0, 'invert': True,  'lo': 0, 'hi': 180},  # Ombro Direito (abd)
    'OE': {'zero_deg':  15, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Ombro Esquerdo (abd)
    'Ca': {'zero_deg':  90, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Cabeça (yaw)
    'CE': {'zero_deg':  90, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Frontal Braço Esquerdo (flex)
    'CD': {'zero_deg':  90, 'scale': 1.0, 'invert': True,  'lo': 0, 'hi': 180},  # Frontal Braço Direito (flex)
    'AE': {'zero_deg': 110, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Antebraço Esquerdo (cotovelo)
    'AD': {'zero_deg':  70, 'scale': 1.0, 'invert': True,  'lo': 0, 'hi': 180},  # Antebraço Direito (cotovelo)
    'LE': {'zero_deg':  95, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Lateral Perna Esquerdo (abd)
    'LD': {'zero_deg':  85, 'scale': 1.0, 'invert': True,  'lo': 0, 'hi': 180},  # Lateral Perna Direito (abd)
    'FE': {'zero_deg':  70, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Frontal Perna Esquerdo (flex)
    'FD': {'zero_deg': 110, 'scale': 1.0, 'invert': True,  'lo': 0, 'hi': 180},  # Frontal Perna Direito (flex)
    'JE': {'zero_deg':  70, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Joelho Esquerdo
    'JD': {'zero_deg': 120, 'scale': 1.0, 'invert': True,  'lo': 0, 'hi': 180},  # Joelho Direito
}

# ============================================================
# One Euro Filter
# ============================================================

class OneEuroFilter:
    """
    Filtro passa-baixa adaptativo: cutoff aumenta com |derivada|,
    reduzindo tremor parado sem introduzir lag em movimento rápido.
    """
    def __init__(self, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        if self.t_prev is None:
            self.t_prev = t
            self.x_prev = x
            return x
        dt = t - self.t_prev
        if dt <= 0:
            return self.x_prev
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat


# Dict de filtros: uma instância por (nome_landmark, eixo)
_filters = {}

def filter_landmark(name, xyz, t):
    """Aplica One Euro Filter em cada componente xyz de um landmark."""
    out = np.empty(3, dtype=np.float64)
    for i, axis in enumerate('xyz'):
        key = (name, axis)
        if key not in _filters:
            _filters[key] = OneEuroFilter(ONE_EURO_MIN_CUTOFF, ONE_EURO_BETA, ONE_EURO_DCUTOFF)
        out[i] = _filters[key](xyz[i], t)
    return out


# ============================================================
# Geometria: body frame e ângulos
# ============================================================

def _normalize(v):
    n = np.linalg.norm(v)
    if n < 1e-9:
        return v
    return v / n

def body_frame(W):
    """
    Constrói frame torso-centrado a partir de LS, RS, LH, RH.
    Retorna (hip_mid, R) onde R é 3x3, colunas [x_body, y_body, z_body].
    """
    sh_mid = 0.5 * (W['SHOULDER_L'] + W['SHOULDER_R'])
    hip_mid = 0.5 * (W['HIP_L'] + W['HIP_R'])
    y_body = _normalize(sh_mid - hip_mid)               # para cima (coluna)
    x_raw = W['SHOULDER_L'] - W['SHOULDER_R']           # lado esquerdo do sujeito
    x_body = _normalize(x_raw - np.dot(x_raw, y_body) * y_body)
    z_body = np.cross(x_body, y_body)                   # para frente (peito)
    R = np.column_stack([x_body, y_body, z_body])
    return hip_mid, R

def to_body(R, v):
    """Converte vetor do frame de mundo para o frame do corpo."""
    return R.T @ v

def angle_shoulder_abd(W, R, side):
    """Abdução de ombro: 0=braço p/ baixo, +=abdução."""
    s, e = f'SHOULDER_{side}', f'ELBOW_{side}'
    v = to_body(R, W[e] - W[s])
    return math.atan2(v[0], -v[1])

def angle_shoulder_flex(W, R, side):
    """Flexão de ombro: 0=braço p/ baixo, +=braço p/ frente."""
    s, e = f'SHOULDER_{side}', f'ELBOW_{side}'
    v = to_body(R, W[e] - W[s])
    return math.atan2(v[2], -v[1])

def angle_elbow(W, R, side):
    """Flexão de cotovelo: 0=braço reto, pi/2=dobrado 90°."""
    s, e, wr = f'SHOULDER_{side}', f'ELBOW_{side}', f'WRIST_{side}'
    u = to_body(R, W[s] - W[e])
    w = to_body(R, W[wr] - W[e])
    nu, nw = np.linalg.norm(u), np.linalg.norm(w)
    if nu < 1e-9 or nw < 1e-9:
        return 0.0
    c = np.clip(np.dot(u, w) / (nu * nw), -1.0, 1.0)
    return math.pi - math.acos(c)

def angle_hip_abd(W, R, side):
    """Abdução de quadril: 0=perna p/ baixo, +=perna p/ lado."""
    h, k = f'HIP_{side}', f'KNEE_{side}'
    v = to_body(R, W[k] - W[h])
    return math.atan2(v[0], -v[1])

def angle_hip_flex(W, R, side):
    """Flexão de quadril: 0=perna p/ baixo, +=perna p/ frente."""
    h, k = f'HIP_{side}', f'KNEE_{side}'
    v = to_body(R, W[k] - W[h])
    return math.atan2(v[2], -v[1])

def angle_knee(W, R, side):
    """Flexão de joelho: 0=perna reta, pi/2=dobrada 90°."""
    h, k, hl = f'HIP_{side}', f'KNEE_{side}', f'HEEL_{side}'
    u = to_body(R, W[h] - W[k])
    w = to_body(R, W[hl] - W[k])
    nu, nw = np.linalg.norm(u), np.linalg.norm(w)
    if nu < 1e-9 or nw < 1e-9:
        return 0.0
    c = np.clip(np.dot(u, w) / (nu * nw), -1.0, 1.0)
    return math.pi - math.acos(c)

def angle_head_yaw(W, R):
    """Yaw da cabeça: 0=olhando p/ frente, +=virou p/ direita."""
    sh_mid_world = 0.5 * (W['SHOULDER_L'] + W['SHOULDER_R'])
    head = to_body(R, W['NOSE'] - sh_mid_world)
    return math.atan2(head[0], head[2])


def map_angle_to_servo(theta_rad, spec):
    """Mapeia ângulo em radianos para valor inteiro de servo, respeitando zero e sentido."""
    deg = math.degrees(theta_rad) * spec['scale']
    if spec['invert']:
        deg = -deg
    val = int(round(spec['zero_deg'] + deg))
    return max(spec['lo'], min(spec['hi'], val))


# ============================================================
# Landmarks 3D e visibilidade
# ============================================================

# Mapa nome_local -> PoseLandmark
_LM_MAP = None

def _init_lm_map(pose_mod):
    global _LM_MAP
    if _LM_MAP is None:
        _LM_MAP = {
            'NOSE':       pose_mod.PoseLandmark.NOSE,
            'SHOULDER_L': pose_mod.PoseLandmark.LEFT_SHOULDER,
            'SHOULDER_R': pose_mod.PoseLandmark.RIGHT_SHOULDER,
            'HIP_L':      pose_mod.PoseLandmark.LEFT_HIP,
            'HIP_R':      pose_mod.PoseLandmark.RIGHT_HIP,
            'ELBOW_L':    pose_mod.PoseLandmark.LEFT_ELBOW,
            'ELBOW_R':    pose_mod.PoseLandmark.RIGHT_ELBOW,
            'WRIST_L':    pose_mod.PoseLandmark.LEFT_WRIST,
            'WRIST_R':    pose_mod.PoseLandmark.RIGHT_WRIST,
            'KNEE_L':     pose_mod.PoseLandmark.LEFT_KNEE,
            'KNEE_R':     pose_mod.PoseLandmark.RIGHT_KNEE,
            'HEEL_L':     pose_mod.PoseLandmark.LEFT_HEEL,
            'HEEL_R':     pose_mod.PoseLandmark.RIGHT_HEEL,
        }
    return _LM_MAP

def extract_world_landmarks(world_landmarks, visibilities, t):
    """Puxa landmarks 3D, aplica One Euro Filter, e retorna dict W + dict Vis."""
    W, Vis = {}, {}
    for name, lm_id in _LM_MAP.items():
        lm = world_landmarks.landmark[lm_id]
        xyz = np.array([lm.x, lm.y, lm.z], dtype=np.float64)
        W[name] = filter_landmark(name, xyz, t)
        Vis[name] = visibilities.landmark[lm_id].visibility
    return W, Vis

def visible(Vis, names):
    return all(Vis[n] >= VIS_THRESHOLD for n in names)


# ============================================================
# HUD
# ============================================================

def put_hud(img, text, xy, offset=(8, -4)):
    """Desenha texto pequeno com borda preta para contraste."""
    x, y = int(xy[0]) + offset[0], int(xy[1]) + offset[1]
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, corHudBorda, 2, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, corHudTexto, 1, cv2.LINE_AA)


# ============================================================
# Menu de conexão com ESP
# ============================================================

while True:
    conecEsp = str(input('Deseja conectar com esp?\n'
                         '\033[34m[ 1 ]\033[m - \033[34mSIM\033[m\n'
                         '\033[34m[ 2 ]\033[m - \033[34mNÂO\033[m\n'
                         '-> '))
    if conecEsp in '1 2':
        break
    else:
        print('\033[35mINVÁLIDO!\033[m Insira 1 ou 2\n')

if conecEsp == '1':
    while True:
        try:
            esp = serial.Serial(porta, velocComunc)
            print('Esp Conectado')
            break
        except:
            pass


# ============================================================
# Init câmera + MediaPipe
# ============================================================

video = cv2.VideoCapture(0)
pose = mp.solutions.pose
Pose = pose.Pose(min_tracking_confidence=0.75,
                 min_detection_confidence=0.75,
                 model_complexity=MODEL_COMPLEXITY)
draw = mp.solutions.drawing_utils
_init_lm_map(pose)

# Estado: último valor de servo (para reusar quando visibilidade cai)
last_servo = {k: v['zero_deg'] for k, v in SERVO_MAP.items()}


# ============================================================
# Loop principal
# ============================================================

while True:
    conectado, vid = video.read()
    if not conectado:
        continue
    vid = cv2.resize(vid, (0, 0), fx=1.7, fy=1.7)
    results = Pose.process(vid)
    points = results.pose_landmarks
    world_points = results.pose_world_landmarks

    h, w, _ = vid.shape

    if points and world_points:
        # ---- Overlay 2D: círculos e posições dos pontos ----
        nariz = points.landmark[pose.PoseLandmark.NOSE]
        cv2.circle(vid, (int(nariz.x * w), int(nariz.y * h)), 2, corPontos, 2)

        ombroE = points.landmark[pose.PoseLandmark.LEFT_SHOULDER]
        cv2.circle(vid, (int(ombroE.x * w), int(ombroE.y * h)), 2, corPontos, 2)

        ombroD = points.landmark[pose.PoseLandmark.RIGHT_SHOULDER]
        cv2.circle(vid, (int(ombroD.x * w), int(ombroD.y * h)), 2, corPontos, 2)

        quadrilE = points.landmark[pose.PoseLandmark.LEFT_HIP]
        cv2.circle(vid, (int(quadrilE.x * w), int(quadrilE.y * h)), 2, corPontos, 2)

        quadrilD = points.landmark[pose.PoseLandmark.RIGHT_HIP]
        cv2.circle(vid, (int(quadrilD.x * w), int(quadrilD.y * h)), 2, corPontos, 2)

        cotoveloE = points.landmark[pose.PoseLandmark.LEFT_ELBOW]
        cv2.circle(vid, (int(cotoveloE.x * w), int(cotoveloE.y * h)), 2, corPontos, 2)

        cotoveloD = points.landmark[pose.PoseLandmark.RIGHT_ELBOW]
        cv2.circle(vid, (int(cotoveloD.x * w), int(cotoveloD.y * h)), 2, corPontos, 2)

        joelhoE = points.landmark[pose.PoseLandmark.LEFT_KNEE]
        cv2.circle(vid, (int(joelhoE.x * w), int(joelhoE.y * h)), 2, corPontos, 2)

        joelhoD = points.landmark[pose.PoseLandmark.RIGHT_KNEE]
        cv2.circle(vid, (int(joelhoD.x * w), int(joelhoD.y * h)), 2, corPontos, 2)

        calcanharE = points.landmark[pose.PoseLandmark.LEFT_HEEL]
        cv2.circle(vid, (int(calcanharE.x * w), int(calcanharE.y * h)), 2, corPontos, 2)

        calcanharD = points.landmark[pose.PoseLandmark.RIGHT_HEEL]
        cv2.circle(vid, (int(calcanharD.x * w), int(calcanharD.y * h)), 2, corPontos, 2)

        maoE = points.landmark[pose.PoseLandmark.LEFT_WRIST]
        cv2.circle(vid, (int(maoE.x * w), int(maoE.y * h)), 2, corPontos, 2)

        maoD = points.landmark[pose.PoseLandmark.RIGHT_WRIST]
        cv2.circle(vid, (int(maoD.x * w), int(maoD.y * h)), 2, corPontos, 2)

        peE = points.landmark[pose.PoseLandmark.LEFT_FOOT_INDEX]
        cv2.circle(vid, (int(peE.x * w), int(peE.y * h)), 2, corPontos, 2)

        peD = points.landmark[pose.PoseLandmark.RIGHT_FOOT_INDEX]
        cv2.circle(vid, (int(peD.x * w), int(peD.y * h)), 2, corPontos, 2)

        # ---- Extrai world landmarks 3D com filtro temporal ----
        t_now = cv2.getTickCount() / cv2.getTickFrequency()
        W, Vis = extract_world_landmarks(world_points, points, t_now)

        # ---- Body frame ----
        torso_ok = visible(Vis, ['SHOULDER_L', 'SHOULDER_R', 'HIP_L', 'HIP_R'])
        if torso_ok:
            _, Rmat = body_frame(W)
        else:
            Rmat = None

        # ---- Cálculo dos 13 ângulos ----
        # Nomes de variável preservados do código original para o bloco de envio.

        def _compute(key, need, fn):
            if Rmat is not None and visible(Vis, need):
                val = map_angle_to_servo(fn(), SERVO_MAP[key])
                last_servo[key] = val
            return last_servo[key]

        # Ombro (abdução)
        angBd = _compute('OD', ['SHOULDER_R', 'ELBOW_R'], lambda: angle_shoulder_abd(W, Rmat, 'R'))
        angBe = _compute('OE', ['SHOULDER_L', 'ELBOW_L'], lambda: angle_shoulder_abd(W, Rmat, 'L'))
        # Cabeça (yaw)
        angC  = _compute('Ca', ['NOSE', 'SHOULDER_L', 'SHOULDER_R'], lambda: angle_head_yaw(W, Rmat))
        # Ombro (flexão frontal)
        angBraE = _compute('CE', ['SHOULDER_L', 'ELBOW_L'], lambda: angle_shoulder_flex(W, Rmat, 'L'))
        angBraD = _compute('CD', ['SHOULDER_R', 'ELBOW_R'], lambda: angle_shoulder_flex(W, Rmat, 'R'))
        # Cotovelo
        angCotE = _compute('AE', ['SHOULDER_L', 'ELBOW_L', 'WRIST_L'], lambda: angle_elbow(W, Rmat, 'L'))
        angCotD = _compute('AD', ['SHOULDER_R', 'ELBOW_R', 'WRIST_R'], lambda: angle_elbow(W, Rmat, 'R'))
        # Perna (abdução lateral)
        angPe = _compute('LE', ['HIP_L', 'KNEE_L'], lambda: angle_hip_abd(W, Rmat, 'L'))
        angPd = _compute('LD', ['HIP_R', 'KNEE_R'], lambda: angle_hip_abd(W, Rmat, 'R'))
        # Perna (flexão frontal)
        angCoxE = _compute('FE', ['HIP_L', 'KNEE_L'], lambda: angle_hip_flex(W, Rmat, 'L'))
        angCoxD = _compute('FD', ['HIP_R', 'KNEE_R'], lambda: angle_hip_flex(W, Rmat, 'R'))
        # Joelho
        angJe = _compute('JE', ['HIP_L', 'KNEE_L', 'HEEL_L'], lambda: angle_knee(W, Rmat, 'L'))
        angJd = _compute('JD', ['HIP_R', 'KNEE_R', 'HEEL_R'], lambda: angle_knee(W, Rmat, 'R'))

        # ---- Comunicação com esp (protocolo idêntico ao original) ----
        if conecEsp == '1':
            esp.write(str(angBd).encode())
            esp.write('q'.encode())
            esp.write(str(angBe).encode())
            esp.write('w'.encode())
            esp.write(str(angC).encode())
            esp.write('e'.encode())
            esp.write(str(angBraE).encode())
            esp.write('r'.encode())
            esp.write(str(angBraD).encode())
            esp.write('t'.encode())
            esp.write(str(angCotE).encode())
            esp.write('y'.encode())
            esp.write(str(angCotD).encode())
            esp.write('u'.encode())
            esp.write(str(angPe).encode())
            esp.write('i'.encode())
            esp.write(str(angPd).encode())
            esp.write('o'.encode())
            esp.write(str(angCoxE).encode())
            esp.write('p'.encode())
            esp.write(str(angCoxD).encode())
            esp.write('a'.encode())
            esp.write(str(angJe).encode())
            esp.write('s'.encode())
            esp.write(str(angJd).encode())
            esp.write('d'.encode())
            esp.flush()

        # ---- HUD: ângulo de cada servo ao lado do ponto correspondente ----
        put_hud(vid, f'Ca:{angC}',              (nariz.x * w,     nariz.y * h))
        put_hud(vid, f'OD:{angBd}',             (ombroD.x * w,    ombroD.y * h))
        put_hud(vid, f'CD:{angBraD}',           (ombroD.x * w,    ombroD.y * h), offset=(8, 8))
        put_hud(vid, f'OE:{angBe}',             (ombroE.x * w,    ombroE.y * h))
        put_hud(vid, f'CE:{angBraE}',           (ombroE.x * w,    ombroE.y * h), offset=(8, 8))
        put_hud(vid, f'AD:{angCotD}',           (cotoveloD.x * w, cotoveloD.y * h))
        put_hud(vid, f'AE:{angCotE}',           (cotoveloE.x * w, cotoveloE.y * h))
        put_hud(vid, f'LD:{angPd}',             (quadrilD.x * w,  quadrilD.y * h))
        put_hud(vid, f'FD:{angCoxD}',           (quadrilD.x * w,  quadrilD.y * h), offset=(8, 8))
        put_hud(vid, f'LE:{angPe}',             (quadrilE.x * w,  quadrilE.y * h))
        put_hud(vid, f'FE:{angCoxE}',           (quadrilE.x * w,  quadrilE.y * h), offset=(8, 8))
        put_hud(vid, f'JD:{angJd}',             (joelhoD.x * w,   joelhoD.y * h))
        put_hud(vid, f'JE:{angJe}',             (joelhoE.x * w,   joelhoE.y * h))

    cv2.imshow('video', vid)
    vid = cv2.flip(vid, 1)

    if cv2.waitKey(1) == ord('q'):
        break

video.release()
cv2.destroyAllWindows()
