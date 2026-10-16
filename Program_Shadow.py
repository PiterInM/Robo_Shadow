# Importação de bibliotecas OpenCV, MediaPipe, Math, NumPy e Socket UDP
import cv2
import mediapipe as mp
import math
import numpy as np
import socket
import threading

# ============================================================
# Configurações e constantes
# ============================================================

# ---- Configuração UDP ----
ESP32_IP   = '192.168.4.1'   # IP padrão do Access Point do ESP32
ESP32_PORT = 4210             # Porta UDP do ESP32
PC_PORT    = 4211             # Porta local para receber respostas (ex: CAL)

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
    'OD': {'zero_deg': 175, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Ombro Direito (abd)
    'OE': {'zero_deg':  15, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Ombro Esquerdo (abd)
    'Ca': {'zero_deg':  90, 'scale': 1.5, 'invert': False, 'lo': 0, 'hi': 180},  # Cabeça (yaw)
    'CE': {'zero_deg':  90, 'scale': 1.0, 'invert': True, 'lo': 0, 'hi': 180},  # Frontal Braço Esquerdo (flex)
    'CD': {'zero_deg':  90, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Frontal Braço Direito (flex)
    'AE': {'zero_deg': 110, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Antebraço Esquerdo (cotovelo)
    'AD': {'zero_deg':  70, 'scale': 1.0, 'invert': True,  'lo': 0, 'hi': 180},  # Antebraço Direito (cotovelo)
    'LE': {'zero_deg':  95, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Lateral Perna Esquerdo (abd)
    'LD': {'zero_deg':  85, 'scale': 1.0, 'invert': False,  'lo': 0, 'hi': 180},  # Lateral Perna Direito (abd)
    'FE': {'zero_deg':  70, 'scale': 1.0, 'invert': True, 'lo': 0, 'hi': 180},  # Frontal Perna Esquerdo (flex)
    'FD': {'zero_deg': 110, 'scale': 1.0, 'invert': False,  'lo': 0, 'hi': 180},  # Frontal Perna Direito (flex)
    'JE': {'zero_deg':  70, 'scale': 1.0, 'invert': True, 'lo': 0, 'hi': 180},  # Joelho Esquerdo
    'JD': {'zero_deg': 120, 'scale': 1.0, 'invert': False,  'lo': 0, 'hi': 180},  # Joelho Direito
    'Ci': {'zero_deg':  90, 'scale': 1.0, 'invert': True, 'lo': 0, 'hi': 180},  # Cintura (yaw torso)
    'TE': {'zero_deg':  90, 'scale': 1.0, 'invert': False, 'lo': 0, 'hi': 180},  # Tornozelo Esquerdo (flex)
    'TD': {'zero_deg':  90, 'scale': 1.0, 'invert': True,  'lo': 0, 'hi': 180},  # Tornozelo Direito  (flex)
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
    """Abdução de ombro: 0=braço p/ baixo, +=abdução lateral.

    Blend Quártico (x^4): blinda a abdução contra falsos positivos do eixo Z (frente/trás).
    Se a câmera distorcer a profundidade (muito comum no MediaPipe quando o corpo está de lado),
    o blend quártico ignora o Z e força o movimento lateral puro, permitindo passar da linha do ombro.
    """
    s, e = f'SHOULDER_{side}', f'ELBOW_{side}'
    v = to_body(R, W[e] - W[s])
    n = np.linalg.norm(v)
    if n < 1e-9: return 0.0
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    total_hz4 = (vx*vx)**2 + (vz*vz)**2
    if total_hz4 < 1e-12: return 0.0
    
    lat_w = (vx*vx)**2 / total_hz4
    ang_lat = math.atan2(vx, -vy)
    ang_fro = math.asin(np.clip(vx / n, -1.0, 1.0))
    
    return lat_w * ang_lat + (1.0 - lat_w) * ang_fro

def angle_shoulder_flex(W, R, side):
    """Flexão de ombro: 0=braço p/ baixo, +=braço p/ frente.

    Blend Quártico (x^4): força o isolamento da flexão.
    """
    s, e = f'SHOULDER_{side}', f'ELBOW_{side}'
    v = to_body(R, W[e] - W[s])
    n = np.linalg.norm(v)
    if n < 1e-9: return 0.0
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    total_hz4 = (vx*vx)**2 + (vz*vz)**2
    if total_hz4 < 1e-12: return 0.0
    
    fro_w = (vz*vz)**2 / total_hz4
    ang_fro = math.atan2(vz, -vy)
    ang_lat = math.asin(np.clip(vz / n, -1.0, 1.0))
    
    return fro_w * ang_fro + (1.0 - fro_w) * ang_lat

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
    """Abdução de quadril com blend quártico."""
    h, k = f'HIP_{side}', f'KNEE_{side}'
    v = to_body(R, W[k] - W[h])
    n = np.linalg.norm(v)
    if n < 1e-9: return 0.0
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    total_hz4 = (vx*vx)**2 + (vz*vz)**2
    if total_hz4 < 1e-12: return 0.0
    
    lat_w = (vx*vx)**2 / total_hz4
    ang_lat = math.atan2(vx, -vy)
    ang_fro = math.asin(np.clip(vx / n, -1.0, 1.0))
    
    return lat_w * ang_lat + (1.0 - lat_w) * ang_fro

def angle_hip_flex(W, R, side):
    """Flexão de quadril com blend quártico."""
    h, k = f'HIP_{side}', f'KNEE_{side}'
    v = to_body(R, W[k] - W[h])
    n = np.linalg.norm(v)
    if n < 1e-9: return 0.0
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    total_hz4 = (vx*vx)**2 + (vz*vz)**2
    if total_hz4 < 1e-12: return 0.0
    
    fro_w = (vz*vz)**2 / total_hz4
    ang_fro = math.atan2(vz, -vy)
    ang_lat = math.asin(np.clip(vz / n, -1.0, 1.0))
    
    return fro_w * ang_fro + (1.0 - fro_w) * ang_lat

def angle_knee(W, R, side):
    """Flexão de joelho: projetada no plano YZ (sagital) do corpo.
    0=perna reta, +=dobrada para trás.
    Isso ignora completamente ruídos laterais (X) e falsos positivos
    quando a perna é levantada reta para frente.
    """
    h, k, hl = f'HIP_{side}', f'KNEE_{side}', f'ANKLE_{side}'
    
    # Vetores: coxa (Quadril -> Joelho) e canela (Joelho -> Tornozelo)
    thigh = to_body(R, W[k] - W[h])
    shin  = to_body(R, W[hl] - W[k])
    
    # Ângulos no plano de perfil (Z=frente, Y=cima)
    ang_thigh = math.atan2(thigh[2], thigh[1])
    ang_shin  = math.atan2(shin[2], shin[1])
    
    # A dobra do joelho é a diferença (canela dobra para -Z em relação à coxa)
    diff = ang_shin - ang_thigh
    
    # Normalizar para o intervalo [-pi, pi]
    while diff > math.pi: diff -= 2 * math.pi
    while diff < -math.pi: diff += 2 * math.pi
    
    return max(0.0, float(diff))

def angle_ankle(W, R, side):
    """Flexão de tornozelo no plano sagital (YZ do corpo).
    0 = pé alinhado com a canela (plantarflexão extrema).
    Neutro (pé perpendicular à canela) ≈ pi/2 → retorna ~0 mapeado para servo 90.
    Positivo = dorsiflexo (bico do pé levantado).
    Negativo = plantarflexo (bico do pé abaixado).
    """
    k, a, hl = f'KNEE_{side}', f'ANKLE_{side}', f'HEEL_{side}'
    shin = to_body(R, W[a]  - W[k])   # joelho → tornozelo
    foot = to_body(R, W[hl] - W[a])   # tornozelo → calcanhar
    ns, nf = np.linalg.norm(shin), np.linalg.norm(foot)
    if ns < 1e-9 or nf < 1e-9:
        return 0.0
    # Ângulo entre os dois vetores no plano sagital
    ang_shin = math.atan2(shin[2], -shin[1])  # direção da canela no plano YZ
    ang_foot = math.atan2(foot[2], -foot[1])  # direção do pé no plano YZ
    diff = ang_foot - ang_shin
    while diff >  math.pi: diff -= 2 * math.pi
    while diff < -math.pi: diff += 2 * math.pi
    # Subtrai pi/2: quando o pé está perpendicular à canela (neutro), diff ≈ pi/2 → retorna 0
    return float(diff) - math.pi / 2

def angle_head_yaw(W, R):
    """Yaw da cabeça: 0=olhando p/ frente, +=virou p/ direita."""
    sh_mid_world = 0.5 * (W['SHOULDER_L'] + W['SHOULDER_R'])
    head = to_body(R, W['NOSE'] - sh_mid_world)
    return math.atan2(head[0], head[2])

def angle_waist_yaw(W):
    """Yaw da cintura: orientação absoluta dos ombros em relação à câmera.

    Mede o ângulo do vetor ombro-a-ombro no plano horizontal (X, Z).
    Isso permite que o robô vire a cintura quando o usuário gira o corpo todo,
    ignorando o quadril (que giraria junto e anularia a diferença).
    """
    sh_vec = W['SHOULDER_L'] - W['SHOULDER_R']

    # X: eixo horizontal da tela (positivo para a direita da imagem / esquerda da pessoa)
    # Z: profundidade (positivo afastando da câmera)
    sh_x = float(sh_vec[0])
    sh_z = float(sh_vec[2])

    # atan2(z, x): 0 quando paralelo à tela.
    # Girar para um lado dá Z positivo, para o outro Z negativo.
    return math.atan2(sh_z, sh_x)


# Guarda último ângulo bruto da cintura para o HUD de debug
_last_waist_deg = [0.0]
# Offset de calibração da cintura (tecla 'c' redefine o zero)
_waist_zero_rad = [0.0]
# Offset de calibração da cabeça
_head_zero_rad = [0.0]
# Temporizador (frames) para a mensagem de feedback visual de calibração
_calib_msg_frames = [0]


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
            'ANKLE_L':    pose_mod.PoseLandmark.LEFT_ANKLE,
            'ANKLE_R':    pose_mod.PoseLandmark.RIGHT_ANKLE,
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
# Menu gráfico de conexão com ESP
# ============================================================

def menu_conexao_grafico():
    """
    Janela OpenCV para configurar a conexão UDP com o ESP32.
    Instrui o usuário a conectar o PC à rede Wi-Fi 'RoboShadow'
    e testa o socket UDP antes de continuar.
    Retorna (sock_ou_None, '1' se conectado ou '2' se não).
    """
    WIN   = 'Robo Shadow - Conexao ESP32'
    WIN_W = 640
    WIN_H = 360

    COR_BG     = ( 18,  22,  30)
    COR_TITULO = (100, 210, 255)
    COR_SUB    = (160, 160, 170)
    COR_OK     = ( 40, 180,  80)
    COR_ERR    = (180,  50,  50)
    COR_BTN_BG = ( 32,  42,  58)
    COR_BTN_HV = ( 50,  95, 175)
    COR_SKIP_BG= ( 48,  28,  28)
    COR_SKIP_HV= (110,  45,  45)
    COR_SKIP_TX= (210, 110, 110)
    COR_BORDA  = ( 70,  85, 110)
    COR_BRANCO = (230, 230, 230)

    state = {
        'hover':    -1,
        'msg':      f'Conecte o PC à rede Wi-Fi "RoboShadow" (senha: shadow123)',
        'msg_cor':  COR_SUB,
        'done':     False,
        'sock':     None,
        'conecEsp': '2',
    }

    BTN_CON  = (30,  220, 610, 275, '__CONNECT__')
    BTN_SKIP = (30,  285, 610, 335, '__SKIP__')
    RECTS = [BTN_CON, BTN_SKIP]

    def _draw(hover, msg, msg_cor):
        img = np.full((WIN_H, WIN_W, 3), COR_BG, dtype=np.uint8)

        cv2.putText(img, 'Robo Shadow', (WIN_W // 2 - 108, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.05, COR_TITULO, 2, cv2.LINE_AA)
        cv2.putText(img, 'Comunicacao sem fio via Wi-Fi UDP',
                    (WIN_W // 2 - 165, 72),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, COR_SUB, 1, cv2.LINE_AA)
        cv2.line(img, (30, 86), (WIN_W - 30, 86), (50, 60, 80), 1)

        # Instruções
        instrs = [
            f'1. No ESP32: grave o firmware e ligue o robô.',
            f'2. No PC: conecte à rede Wi-Fi  "RoboShadow"  (senha: shadow123).',
            f'3. Clique em "Conectar" para testar a conexão UDP.',
            f'   ESP32 IP: {ESP32_IP}   Porta: {ESP32_PORT}',
        ]
        for i, txt in enumerate(instrs):
            cv2.putText(img, txt, (36, 115 + i * 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, COR_BRANCO, 1, cv2.LINE_AA)

        # Botão Conectar
        x1, y1, x2, y2, _ = BTN_CON
        bg = COR_BTN_HV if hover == 0 else COR_BTN_BG
        cv2.rectangle(img, (x1, y1), (x2, y2), bg, -1)
        cv2.rectangle(img, (x1, y1), (x2, y2), COR_BORDA, 1)
        cv2.putText(img, 'Conectar ao ESP32', (x1 + 14, y1 + 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, COR_TITULO, 1, cv2.LINE_AA)

        # Botão Sem Wi-Fi
        x1, y1, x2, y2, _ = BTN_SKIP
        bg = COR_SKIP_HV if hover == 1 else COR_SKIP_BG
        cv2.rectangle(img, (x1, y1), (x2, y2), bg, -1)
        cv2.rectangle(img, (x1, y1), (x2, y2), (90, 55, 55), 1)
        cv2.putText(img, 'Continuar sem ESP32', (x1 + 14, y1 + 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, COR_SKIP_TX, 1, cv2.LINE_AA)

        cv2.putText(img, msg, (30, WIN_H - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, msg_cor, 1, cv2.LINE_AA)
        return img

    def mouse_cb(event, mx, my, flags, param):
        hover = -1
        for i, (x1, y1, x2, y2, _) in enumerate(RECTS):
            if x1 <= mx <= x2 and y1 <= my <= y2:
                hover = i
                break
        state['hover'] = hover

        if event != cv2.EVENT_LBUTTONDOWN or hover < 0:
            return

        _, _, _, _, tag = RECTS[hover]

        if tag == '__SKIP__':
            state['conecEsp'] = '2'
            state['done']     = True

        elif tag == '__CONNECT__':
            state['connecting'] = True
            state['msg']     = f'Testando UDP em {ESP32_IP}:{ESP32_PORT}...'
            state['msg_cor'] = COR_SUB

    img0 = _draw(-1, state['msg'], state['msg_cor'])
    cv2.imshow(WIN, img0)
    cv2.waitKey(1)
    cv2.setMouseCallback(WIN, mouse_cb)

    while not state['done']:
        if state.get('connecting'):
            state.pop('connecting')
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(2.0)
                sock.bind(('', PC_PORT))
                # Envia um ping vazio para ver se o ESP32 responde (opcional)
                sock.sendto(b'PING', (ESP32_IP, ESP32_PORT))
                state['sock']     = sock
                state['conecEsp'] = '1'
                state['msg']      = f'UDP pronto! Enviando para {ESP32_IP}:{ESP32_PORT}'
                state['msg_cor']  = COR_OK
                img = _draw(state['hover'], state['msg'], state['msg_cor'])
                cv2.imshow(WIN, img)
                cv2.waitKey(900)
                state['done'] = True
            except Exception as e:
                state['msg']     = f'Erro: {str(e)[:70]}'
                state['msg_cor'] = COR_ERR

        img = _draw(state['hover'], state['msg'], state['msg_cor'])
        cv2.imshow(WIN, img)
        if cv2.waitKey(50) == 27:
            break

    cv2.destroyAllWindows()
    return state['sock'], state['conecEsp']


esp, conecEsp = menu_conexao_grafico()

# Thread para receber respostas UDP do ESP32 (ex: CAL)
_udp_inbox = []  # lista de strings recebidas
_udp_lock  = threading.Lock()

def _udp_recv_thread(sock):
    """Fica escutando respostas UDP do ESP32 em background."""
    sock.settimeout(0.5)
    while True:
        try:
            data, _ = sock.recvfrom(64)
            msg = data.decode('utf-8', errors='ignore').strip()
            if msg:
                with _udp_lock:
                    _udp_inbox.append(msg)
        except socket.timeout:
            pass
        except Exception:
            break

if conecEsp == '1' and esp is not None:
    t = threading.Thread(target=_udp_recv_thread, args=(esp,), daemon=True)
    t.start()


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
    _raw_waist = [0.0]  # reset a cada frame; atualizado em _waist_fn se pose detectada
    _raw_head = [0.0]
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

        # ---- Ambos os braços ativos (OD/CD/AD + OE/CE/AE) ----
        # Ombro (abdução)
        angBd  = _compute('OD', ['SHOULDER_R', 'ELBOW_R'], lambda: angle_shoulder_abd(W, Rmat, 'R'))
        angBe  = _compute('OE', ['SHOULDER_L', 'ELBOW_L'], lambda: angle_shoulder_abd(W, Rmat, 'L'))
        # Cabeça (yaw)
        _raw_head = [0.0]
        def _head_fn():
            v = angle_head_yaw(W, Rmat) - _head_zero_rad[0]
            _raw_head[0] = math.degrees(v)
            return v
        angC   = _compute('Ca', ['NOSE', 'SHOULDER_L', 'SHOULDER_R'], _head_fn)
        # Ombro (flexão frontal)
        angBraE = _compute('CE', ['SHOULDER_L', 'ELBOW_L'], lambda: angle_shoulder_flex(W, Rmat, 'L'))
        angBraD = _compute('CD', ['SHOULDER_R', 'ELBOW_R'], lambda: angle_shoulder_flex(W, Rmat, 'R'))
        # Cotovelo
        angCotE = _compute('AE', ['SHOULDER_L', 'ELBOW_L', 'WRIST_L'], lambda: angle_elbow(W, Rmat, 'L'))
        angCotD = _compute('AD', ['SHOULDER_R', 'ELBOW_R', 'WRIST_R'], lambda: angle_elbow(W, Rmat, 'R'))
        # Cintura (yaw torso) — subtrai offset de calibração; 'c' redefine o zero
        _raw_waist = [0.0]
        def _waist_fn():
            v = angle_waist_yaw(W) - _waist_zero_rad[0]
            _raw_waist[0] = math.degrees(v)
            return v
        angCi   = _compute('Ci', ['SHOULDER_L', 'SHOULDER_R', 'HIP_L', 'HIP_R'], _waist_fn)
        # Perna (abdução lateral)
        angPe = _compute('LE', ['HIP_L', 'KNEE_L'], lambda: angle_hip_abd(W, Rmat, 'L'))
        angPd = _compute('LD', ['HIP_R', 'KNEE_R'], lambda: angle_hip_abd(W, Rmat, 'R'))
        # Perna (flexão frontal)
        angCoxE = _compute('FE', ['HIP_L', 'KNEE_L'], lambda: angle_hip_flex(W, Rmat, 'L'))
        angCoxD = _compute('FD', ['HIP_R', 'KNEE_R'], lambda: angle_hip_flex(W, Rmat, 'R'))
        # Joelho
        angJe = _compute('JE', ['HIP_L', 'KNEE_L', 'ANKLE_L'], lambda: angle_knee(W, Rmat, 'L'))
        angJd = _compute('JD', ['HIP_R', 'KNEE_R', 'ANKLE_R'], lambda: angle_knee(W, Rmat, 'R'))
        # Tornozelo (dorsiflexo/plantarflexo)
        angTe = _compute('TE', ['KNEE_L', 'ANKLE_L', 'HEEL_L'], lambda: angle_ankle(W, Rmat, 'L'))
        angTd = _compute('TD', ['KNEE_R', 'ANKLE_R', 'HEEL_R'], lambda: angle_ankle(W, Rmat, 'R'))

        # ---- Comunicação com esp (protocolo idêntico ao original) ----
        if conecEsp == '1':
            # Checa se o ESP32 enviou algum comando via UDP (ex: calibração pelo controle)
            with _udp_lock:
                while _udp_inbox:
                    linha = _udp_inbox.pop(0)
                    if "CAL" in linha:
                        _waist_zero_rad[0] += _raw_waist[0] * (math.pi / 180.0)
                        _head_zero_rad[0] += _raw_head[0] * (math.pi / 180.0)
                        _calib_msg_frames[0] = 30
                        print(f'Calibrado! Cintura zero={math.degrees(_waist_zero_rad[0]):.1f}, Cabeça zero={math.degrees(_head_zero_rad[0]):.1f}')

            # Envia todos os 16 ângulos via UDP em um único datagrama
            pacote = (
                f"{angBd}q{angBe}w{angC}e{angBraE}r{angBraD}t"
                f"{angCotE}y{angCotD}u{angPe}i{angPd}o"
                f"{angCoxE}p{angCoxD}a{angJe}s{angJd}d{angCi}f"
                f"{angTe}g{angTd}h"
            )
            esp.sendto(pacote.encode(), (ESP32_IP, ESP32_PORT))

        # ---- HUD: ângulo de cada servo ao lado do ponto correspondente ----
        put_hud(vid, f'Ca:{angC}',              (nariz.x * w,     nariz.y * h))
        put_hud(vid, f'({_raw_head[0]:.1f}g)',  (nariz.x * w,     nariz.y * h), offset=(8, 12))
        put_hud(vid, f'OD:{angBd}',             (ombroD.x * w,    ombroD.y * h))
        put_hud(vid, f'CD:{angBraD}',           (ombroD.x * w,    ombroD.y * h), offset=(8, 8))
        put_hud(vid, f'OE:{angBe}',             (ombroE.x * w,    ombroE.y * h))
        put_hud(vid, f'CE:{angBraE}',           (ombroE.x * w,    ombroE.y * h), offset=(8, 8))
        put_hud(vid, f'AD:{angCotD}',           (cotoveloD.x * w, cotoveloD.y * h))
        put_hud(vid, f'AE:{angCotE}',           (cotoveloE.x * w, cotoveloE.y * h))
        cintX = (quadrilD.x + quadrilE.x) * 0.5 * w
        cintY = (quadrilD.y + quadrilE.y) * 0.5 * h
        put_hud(vid, f'Ci:{angCi}',             (cintX, cintY))
        put_hud(vid, f'({_raw_waist[0]:.1f}g)', (cintX, cintY), offset=(8, 12))
        # Dica de calibração no canto superior esquerdo
        put_hud(vid, '[C] calibrar eixos de giro', (4, 4), offset=(0, 12))

        # Feedback visual temporário de calibração
        if _calib_msg_frames[0] > 0:
            cv2.putText(vid, "CALIBRADO!", (int(w/2) - 100, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
            _calib_msg_frames[0] -= 1
        put_hud(vid, f'LD:{angPd}',             (quadrilD.x * w,  quadrilD.y * h))
        put_hud(vid, f'FD:{angCoxD}',           (quadrilD.x * w,  quadrilD.y * h), offset=(8, 8))
        put_hud(vid, f'LE:{angPe}',             (quadrilE.x * w,  quadrilE.y * h))
        put_hud(vid, f'FE:{angCoxE}',           (quadrilE.x * w,  quadrilE.y * h), offset=(8, 8))
        put_hud(vid, f'JD:{angJd}',             (joelhoD.x * w,   joelhoD.y * h))
        put_hud(vid, f'JE:{angJe}',             (joelhoE.x * w,   joelhoE.y * h))

        # Tornozelos: usa posição 2D do landmark de tornozelo
        tornozeloE = points.landmark[pose.PoseLandmark.LEFT_ANKLE]
        tornozeloD = points.landmark[pose.PoseLandmark.RIGHT_ANKLE]
        put_hud(vid, f'TE:{angTe}', (tornozeloE.x * w, tornozeloE.y * h))
        put_hud(vid, f'TD:{angTd}', (tornozeloD.x * w, tornozeloD.y * h))

    cv2.imshow('video', vid)
    vid = cv2.flip(vid, 1)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break
    elif key == ord('c'):
        # Calibrar zero da cintura e cabeça: acumula o ângulo atual como novo offset
        _waist_zero_rad[0] += _raw_waist[0] * (math.pi / 180.0)
        _head_zero_rad[0] += _raw_head[0] * (math.pi / 180.0)
        _calib_msg_frames[0] = 30
        print(f'Calibrado! Cintura zero={math.degrees(_waist_zero_rad[0]):.1f}, Cabeça zero={math.degrees(_head_zero_rad[0]):.1f}')


video.release()
cv2.destroyAllWindows()
