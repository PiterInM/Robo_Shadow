"""
main.py — Menu Gráfico de Seleção de Modos do Robo Shadow
Interface visual desenvolvida em OpenCV com o mesmo design de Program_Shadow.py.
Suporta cliques de mouse, efeitos hover e atalhos de teclado (1, 2, 3, Q, ESC).
"""

import os
import sys
import subprocess
import socket

import numpy as np
import cv2


WIN_NAME = 'Robo Shadow - Menu Principal'
WIN_W = 680
WIN_H = 450

# Paleta de cores idêntica ao Program_Shadow.py
COR_BG       = ( 18,  22,  30)
COR_TITULO   = (100, 210, 255)
COR_SUB      = (160, 160, 170)
COR_BORDA    = ( 70,  85, 110)
COR_BORDA_HV = (110, 165, 230)
COR_BRANCO   = (230, 230, 230)
COR_MUTED    = (130, 135, 145)

# Definição dos botões:
# (x1, y1, x2, y2, tag, titulo, descricao, cor_titulo, bg_normal, bg_hover)
BUTTONS = [
    {
        'tag': '1',
        'rect': (40, 105, 640, 175),
        'titulo': '[1]  Modo Shadow  (Pose -> Servos)',
        'desc': 'Controle por visao computacional (Camera + MediaPipe)',
        'cor_txt': (100, 210, 255),
        'bg_norm': ( 28,  36,  50),
        'bg_hov':  ( 42,  68, 105),
    },
    {
        'tag': '2',
        'rect': (40, 185, 640, 255),
        'titulo': '[2]  Visualizador IMU 3D  (MPU-6050)',
        'desc': 'Telemetria do peito do robo em tempo real (grafico 3D)',
        'cor_txt': ( 80, 220, 120),
        'bg_norm': ( 24,  42,  32),
        'bg_hov':  ( 36,  78,  54),
    },
    {
        'tag': '3',
        'rect': (40, 265, 640, 335),
        'titulo': '[3]  Ambos Simultaneos  (Dois Processos)',
        'desc': 'Program_Shadow + Visualizador IMU operando em paralelo',
        'cor_txt': (240, 170,  80),
        'bg_norm': ( 45,  35,  25),
        'bg_hov':  ( 85,  62,  38),
    },
    {
        'tag': 'q',
        'rect': (40, 350, 640, 410),
        'titulo': '[Q]  Sair',
        'desc': 'Encerrar o sistema',
        'cor_txt': (210, 120, 120),
        'bg_norm': ( 42,  26,  26),
        'bg_hov':  ( 80,  40,  40),
    },
]


def run_shadow():
    print("\n▶  Iniciando Program_Shadow.py ...\n")
    subprocess.run([sys.executable, 'Program_Shadow.py'])


def run_imu():
    print("\n▶  Iniciando imu_visualizer.py ...\n")
    subprocess.run([sys.executable, 'imu_visualizer.py'])


def run_both():
    print("\n▶  Iniciando ambos em paralelo ...")
    print("    Program_Shadow.py  +  imu_visualizer.py\n")
    p1 = subprocess.Popen([sys.executable, 'Program_Shadow.py'])
    p2 = subprocess.Popen([sys.executable, 'imu_visualizer.py'])
    print("  Pressione Ctrl+C no terminal para encerrar ambos.\n")
    try:
        p1.wait()
        p2.wait()
    except KeyboardInterrupt:
        print("\n  Encerrando processos...")
        p1.terminate()
        p2.terminate()
        try:
            p1.wait(timeout=2)
            p2.wait(timeout=2)
        except Exception:
            p1.kill()
            p2.kill()
        print("  Pronto.")


def get_wifi_status():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('192.168.4.1', 4210))
        ip = s.getsockname()[0]
        s.close()
        if ip.startswith('192.168.4.'):
            return True, f"Wi-Fi RoboShadow: CONECTADO ({ip})"
        return False, f"Wi-Fi: {ip} (Desconectado do RoboShadow)"
    except Exception:
        return False, "Wi-Fi: Desconectado do RoboShadow"


def draw_menu(hover_idx):
    img = np.full((WIN_H, WIN_W, 3), COR_BG, dtype=np.uint8)

    # Cabeçalho
    cv2.putText(img, 'ROBO SHADOW', (40, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 1.05, COR_TITULO, 2, cv2.LINE_AA)
    cv2.putText(img, 'Central de Modos de Operacao', (40, 72),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, COR_SUB, 1, cv2.LINE_AA)

    # Status de rede no topo direito
    is_ok, wifi_txt = get_wifi_status()
    wifi_col = (80, 220, 100) if is_ok else (80, 170, 250)
    cv2.putText(img, wifi_txt, (WIN_W - 400, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, wifi_col, 1, cv2.LINE_AA)

    cv2.line(img, (40, 88), (WIN_W - 40, 88), (50, 60, 80), 1)

    # Renderiza botões
    for i, btn in enumerate(BUTTONS):
        x1, y1, x2, y2 = btn['rect']
        is_hover = (hover_idx == i)
        bg_col = btn['bg_hov'] if is_hover else btn['bg_norm']
        border_col = COR_BORDA_HV if is_hover else COR_BORDA

        # Fundo e borda
        cv2.rectangle(img, (x1, y1), (x2, y2), bg_col, -1)
        cv2.rectangle(img, (x1, y1), (x2, y2), border_col, 1)

        # Indicador lateral de seleção no hover
        if is_hover:
            cv2.rectangle(img, (x1, y1), (x1 + 6, y2), btn['cor_txt'], -1)

        # Textos do botão
        txt_offset_x = 22 if is_hover else 16
        cv2.putText(img, btn['titulo'], (x1 + txt_offset_x, y1 + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, btn['cor_txt'], 1, cv2.LINE_AA)
        cv2.putText(img, btn['desc'], (x1 + txt_offset_x, y1 + 54),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, COR_SUB, 1, cv2.LINE_AA)

    # Rodapé de instruções
    cv2.putText(img, 'Dica: clique com o mouse ou pressione [1], [2], [3] ou [Q / ESC]',
                (42, WIN_H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.38, COR_MUTED, 1, cv2.LINE_AA)

    return img


def graphical_menu():
    state = {
        'hover': -1,
        'selected': None,
        'done': False
    }

    def mouse_cb(event, mx, my, flags, param):
        hover = -1
        for i, btn in enumerate(BUTTONS):
            x1, y1, x2, y2 = btn['rect']
            if x1 <= mx <= x2 and y1 <= my <= y2:
                hover = i
                break
        state['hover'] = hover

        if event == cv2.EVENT_LBUTTONDOWN and hover >= 0:
            state['selected'] = BUTTONS[hover]['tag']
            state['done'] = True

    try:
        cv2.namedWindow(WIN_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WIN_NAME, mouse_cb)

        while not state['done']:
            frame = draw_menu(state['hover'])
            cv2.imshow(WIN_NAME, frame)
            key = cv2.waitKey(25) & 0xFF

            if key in (ord('1'), ord('2'), ord('3')):
                state['selected'] = chr(key)
                state['done'] = True
            elif key in (ord('q'), ord('Q'), 27):  # 'q' ou ESC
                state['selected'] = 'q'
                state['done'] = True

        cv2.destroyAllWindows()
        return state['selected']
    except Exception as e:
        print(f"[Menu] Não foi possível abrir janela gráfica ({e}). Usando modo terminal.")
        return None


def terminal_menu():
    menu_txt = """
╔══════════════════════════════════════════════════════╗
║             🤖  ROBO SHADOW — Menu Principal         ║
╠══════════════════════════════════════════════════════╣
║                                                      ║
║  [1]  Modo Shadow  (câmera + MediaPipe → servos)     ║
║       Program_Shadow.py                              ║
║                                                      ║
║  [2]  Visualizador IMU MPU-6050  (gráfico 3D)        ║
║       imu_visualizer.py                              ║
║                                                      ║
║  [3]  Ambos simultâneos  (dois processos paralelos)  ║
║                                                      ║
║  [Q]  Sair                                           ║
║                                                      ║
╚══════════════════════════════════════════════════════╝
"""
    print(menu_txt)
    try:
        return input("  Escolha [1/2/3/Q]: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        return 'q'


def main():
    # Garante a execução com o interpretador do ambiente virtual (.venv) se presente
    _venv_python = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.venv', 'bin', 'python')
    if os.path.isfile(_venv_python) and sys.executable != _venv_python:
        os.execv(_venv_python, [_venv_python] + sys.argv)

    choice = graphical_menu()
    if choice is None:
        choice = terminal_menu()

    if choice == '1':
        run_shadow()
    elif choice == '2':
        run_imu()
    elif choice == '3':
        run_both()
    elif choice in ('q', 'exit', 'sair'):
        print("Saindo.")
        sys.exit(0)


if __name__ == '__main__':
    main()
