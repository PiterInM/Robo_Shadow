"""
window_icon.py — Utilitário para configurar ícone e classe da janela no Linux/X11/Wayland.

Garante que as janelas do OpenCV e Matplotlib:
1. Exibam o ícone do robô (_NET_WM_ICON) na barra de títulos, Alt+Tab e painel.
2. Definam a classe X11 (WM_CLASS) como 'robo-shadow', permitindo que o GNOME/KDE
   vinculem a janela aberta ao atalho 'robo-shadow.desktop' no menu e na dock.
"""

import os
import sys
import time
import threading
import ctypes
import subprocess
from typing import Optional, List

_ICON_DATA_CACHE = None
_DEFAULT_ICON_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'icon.png')


def _get_icon_cardinals(icon_path: str):
    """Carrega icon.png e monta os dados em formato _NET_WM_ICON (ARGB)."""
    global _ICON_DATA_CACHE
    if _ICON_DATA_CACHE is not None:
        return _ICON_DATA_CACHE

    if not os.path.isfile(icon_path):
        return None

    try:
        from PIL import Image
        import numpy as np

        im = Image.open(icon_path).convert('RGBA')
        icon_data = []

        # Múltiplas resoluções para nitidez em qualquer escala da interface
        for size in (16, 24, 32, 48, 64, 128, 256):
            resized = im.resize((size, size), Image.Resampling.LANCZOS)
            arr = np.array(resized)
            icon_data.append(size)
            icon_data.append(size)
            r = arr[:, :, 0].astype(np.uint64)
            g = arr[:, :, 1].astype(np.uint64)
            b = arr[:, :, 2].astype(np.uint64)
            a = arr[:, :, 3].astype(np.uint64)
            argb = (a << 24) | (r << 16) | (g << 8) | b
            icon_data.extend(argb.flatten().tolist())

        cardinals = (ctypes.c_ulong * len(icon_data))(*icon_data)
        _ICON_DATA_CACHE = (cardinals, len(icon_data))
        return _ICON_DATA_CACHE
    except Exception as e:
        return None


def _find_window_ids(win_title: Optional[str] = None) -> List[int]:
    """Localiza o(s) ID(s) X11 das janelas pertencentes ao processo atual."""
    wids = []
    my_pid = str(os.getpid())

    # Método 1: wmctrl (rápido e preciso via PID)
    try:
        out = subprocess.check_output(['wmctrl', '-l', '-p'], timeout=0.8).decode('utf-8', errors='ignore')
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 3 and parts[2] == my_pid:
                wids.append(int(parts[0], 16))
            elif win_title and win_title in line:
                wids.append(int(parts[0], 16))
    except Exception:
        pass

    if wids:
        return list(set(wids))

    # Método 2: varredura direta via libX11
    try:
        x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
        Display_p = ctypes.c_void_p
        Window = ctypes.c_ulong

        x11.XOpenDisplay.restype = Display_p
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XDefaultRootWindow.restype = Window
        x11.XDefaultRootWindow.argtypes = [Display_p]
        x11.XQueryTree.restype = ctypes.c_int
        x11.XQueryTree.argtypes = [
            Display_p, Window,
            ctypes.POINTER(Window), ctypes.POINTER(Window),
            ctypes.POINTER(ctypes.POINTER(Window)), ctypes.POINTER(ctypes.c_uint)
        ]
        x11.XFetchName.restype = ctypes.c_int
        x11.XFetchName.argtypes = [Display_p, Window, ctypes.POINTER(ctypes.c_char_p)]
        x11.XFree.restype = ctypes.c_int
        x11.XCloseDisplay.restype = ctypes.c_int
        x11.XCloseDisplay.argtypes = [Display_p]

        dpy = x11.XOpenDisplay(None)
        if dpy:
            root = x11.XDefaultRootWindow(dpy)
            root_ret = Window()
            parent_ret = Window()
            children = ctypes.POINTER(Window)()
            nchildren = ctypes.c_uint()

            if x11.XQueryTree(dpy, root, ctypes.byref(root_ret), ctypes.byref(parent_ret), ctypes.byref(children), ctypes.byref(nchildren)):
                for i in range(nchildren.value):
                    wid = children[i]
                    name_p = ctypes.c_char_p()
                    if x11.XFetchName(dpy, wid, ctypes.byref(name_p)):
                        if name_p.value:
                            title_str = name_p.value.decode('utf-8', errors='ignore')
                            if win_title and win_title in title_str:
                                wids.append(wid)
                        x11.XFree(name_p)
                if children:
                    x11.XFree(children)
            x11.XCloseDisplay(dpy)
    except Exception:
        pass

    return list(set(wids))


def apply_window_icon(win_title: Optional[str] = None,
                      icon_path: Optional[str] = None,
                      wm_class: str = 'robo-shadow') -> bool:
    """
    Aplica o ícone e WM_CLASS às janelas do processo atual.
    Retorna True em caso de sucesso.
    """
    if icon_path is None:
        icon_path = _DEFAULT_ICON_PATH

    icon_tuple = _get_icon_cardinals(icon_path)
    if not icon_tuple:
        return False
    cardinals, total_cardinals = icon_tuple

    target_wids = _find_window_ids(win_title)
    if not target_wids:
        return False

    try:
        x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
        Display_p = ctypes.c_void_p
        Window = ctypes.c_ulong
        Atom = ctypes.c_ulong

        x11.XOpenDisplay.restype = Display_p
        x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        x11.XInternAtom.restype = Atom
        x11.XInternAtom.argtypes = [Display_p, ctypes.c_char_p, ctypes.c_int]
        x11.XChangeProperty.restype = ctypes.c_int
        x11.XChangeProperty.argtypes = [
            Display_p, Window, Atom, Atom, ctypes.c_int,
            ctypes.c_int, ctypes.c_char_p, ctypes.c_int
        ]
        x11.XFlush.restype = ctypes.c_int
        x11.XFlush.argtypes = [Display_p]
        x11.XCloseDisplay.restype = ctypes.c_int
        x11.XCloseDisplay.argtypes = [Display_p]

        class XClassHint(ctypes.Structure):
            _fields_ = [('res_name', ctypes.c_char_p), ('res_class', ctypes.c_char_p)]

        x11.XSetClassHint.restype = ctypes.c_int
        x11.XSetClassHint.argtypes = [Display_p, Window, ctypes.POINTER(XClassHint)]

        dpy = x11.XOpenDisplay(None)
        if not dpy:
            return False

        net_wm_icon = x11.XInternAtom(dpy, b'_NET_WM_ICON', False)
        cardinal = x11.XInternAtom(dpy, b'CARDINAL', False)
        class_b = wm_class.encode('utf-8')
        hint = XClassHint(class_b, class_b)

        for wid in target_wids:
            x11.XSetClassHint(dpy, wid, ctypes.byref(hint))
            x11.XChangeProperty(
                dpy, wid, net_wm_icon, cardinal, 32,
                0,  # PropModeReplace
                ctypes.cast(cardinals, ctypes.c_char_p),
                total_cardinals
            )
        x11.XFlush(dpy)
        x11.XCloseDisplay(dpy)
        return True
    except Exception:
        return False


def setup_window_icon_async(win_title: Optional[str] = None,
                            icon_path: Optional[str] = None,
                            wm_class: str = 'robo-shadow',
                            max_retries: int = 100,
                            interval: float = 0.1) -> None:
    """
    Inicia uma thread leve em segundo plano para aguardar a janela ser mapeada
    pelo X11/OpenCV e aplicar o ícone e WM_CLASS automaticamente.
    """
    def _worker():
        for _ in range(max_retries):
            time.sleep(interval)
            if apply_window_icon(win_title=win_title, icon_path=icon_path, wm_class=wm_class):
                break

    t = threading.Thread(target=_worker, daemon=True)
    t.start()


def setup_matplotlib_icon(fig, icon_path: Optional[str] = None, wm_class: str = 'robo-shadow') -> None:
    """Configura o ícone em figuras Matplotlib (PyQt ou Tkinter)."""
    if icon_path is None:
        icon_path = _DEFAULT_ICON_PATH

    # 1. Configuração direta via PyQt caso disponível
    try:
        from PyQt6.QtGui import QIcon
        from PyQt6.QtCore import QCoreApplication
        from PyQt6.QtGui import QGuiApplication
        QGuiApplication.setDesktopFileName(wm_class)
        manager = fig.canvas.manager
        if hasattr(manager, 'window'):
            manager.window.setWindowIcon(QIcon(icon_path))
    except Exception:
        pass

    # 2. Configuração X11
    win_title = None
    try:
        win_title = fig.canvas.manager.get_window_title()
    except Exception:
        pass
    setup_window_icon_async(win_title=win_title, icon_path=icon_path, wm_class=wm_class)
