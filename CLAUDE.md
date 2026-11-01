# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Keeping This File Up To Date

**This file must always be updated whenever a code change makes any part of it outdated.** Treat it as part of the deliverable, not an afterthought — if a change alters the architecture, data flow, wireless protocol, pin/channel assignments, dependencies, entry points, run commands, or file layout described below, update the corresponding section in the same change.

Likewise, keep `.gitignore` current: any change that introduces generated files, build output, or local environment directories should add them there.

## Project Overview

Robô Shadow is a humanoid robot that mimics a person's movements in real time. A Python script uses a webcam and MediaPipe pose detection to calculate joint angles, then transmits them wirelessly (Wi-Fi UDP) to an ESP32 microcontroller that drives 16 servo motors through a PCA9685 I2C PWM driver. The same ESP32 also streams MPU-6050 IMU telemetry back over UDP for a separate 3D visualizer.

## Running the Python Application

Install dependencies:
```bash
pip install -r requirements.txt
```

Run main menu:
```bash
python main.py
```
Or run directly:
```bash
python Program_Shadow.py   # Modo Shadow
python imu_visualizer.py   # Visualizador IMU 3D
```

Desktop integration:
- `robo-shadow.desktop` installed at `~/.local/share/applications/robo-shadow.desktop` with `StartupWMClass=robo-shadow`.
- `window_icon.py` sets `_NET_WM_ICON` and `WM_CLASS` on OpenCV windows and Qt desktop properties on Matplotlib figures to ensure the robot icon appears properly in the window titlebar, taskbar/dock, and app switcher.

**Wi-Fi connection**: the ESP32 runs its own Access Point (SSID `RoboShadow`, password `shadow123`, ESP32 at `192.168.4.1`). `Program_Shadow.py` shows a graphical screen ("Conectar ao ESP32" / "Continuar sem ESP32") that opens a UDP socket to that network; `imu_visualizer.py` just checks/reports the PC's current network status.

**Automatic Wi-Fi switching (`main.py` only)**: when a mode is launched from the graphical menu (options 1–3), `main.py` uses `nmcli` (NetworkManager) to switch the PC to the `RoboShadow` network before starting the subprocess(es), and switches back to whatever network was active before once the mode is closed. This only happens when launching through `main.py`'s menu — running `Program_Shadow.py`/`imu_visualizer.py` directly does not touch the network, and requires Linux with NetworkManager (`nmcli`) available and permitted to manage Wi-Fi for the current user; if switching fails (e.g. robot AP out of range), it logs a warning to the console and continues without blocking. For mode `[3]` (both at once), the switch/restore happens once in `main.py` around both processes, avoiding any race between the two scripts fighting over the network.

## ESP32 Firmware

Current firmware: `Codigos_ESP/shadow/4_shadow_esp32_completo/4_shadow_esp32_completo.ino` (`Codigos_ESP/shadow/1_...` through `3_...` are earlier iterations kept for reference). A companion sketch, `Codigos_ESP/controle/controle_fisico_esp_now.ino` (a separate XIAO ESP32C3 with 4 buttons), sends `acao` codes `0`–`4` via ESP-NOW to trigger the same preset actions (`PosPadrao`/`Shadow`/`Andar`/`Acenar`/`Sentar`) independently of the Python/Wi-Fi path.

Dev/bring-up sketches, not part of the running system: `Codigos_ESP/i2c_scanner/i2c_scanner.ino` and `Codigos_Arduino/i2c_scanner_uno/i2c_scanner_uno.ino` scan the I2C bus for device addresses; `Codigos_ESP/teste_mpu6050/teste_mpu6050.ino` reads the MPU-6050 standalone over Serial; `Codigos_ESP/controle/teste_bateria_wifi.ino`, `teste_bateria_botoes_web.ino` and `teste_botoes.ino` are early bring-up tests for the XIAO ESP32C3 remote (battery/Wi-Fi stability, button wiring, button state over a web page) that were superseded by `controle_fisico_esp_now.ino`.

Flash using Arduino IDE or PlatformIO with these libraries installed:
- `Adafruit PWM Servo Driver Library` (`Adafruit_PWMServoDriver`)
- `Adafruit BusIO`
- ESP32 board support package (provides `WiFi`, `WiFiUdp`, `esp_wifi`, `esp_now`, `Wire`, `ArduinoOTA`)

## Architecture

### Data Flow

```
Webcam → MediaPipe pose landmarks → angle calculations → Wi-Fi UDP → ESP32 → PCA9685 (I2C) → servos
ESP32 MPU-6050 → Wi-Fi UDP → imu_visualizer.py (3D telemetry)
```

A separate ESP-NOW physical remote (`controle_fisico_esp_now.ino`) can trigger the same preset actions on the ESP32 independently of the Python/Wi-Fi stream.

### Python Script (`Program_Shadow.py`)

Single script, no modules besides `window_icon.py`. Key sections:
- Constants: `ESP32_IP`/`ESP32_PORT` (UDP target), `SERVO_MAP` (16 servo entries: zero position, scale, inversion, clamp range)
- `OneEuroFilter` / `filter_landmark`: adaptive low-pass filtering of each 3D landmark to reduce jitter without adding lag
- `body_frame`, `to_body`, `angle_*` functions: build a torso-centered coordinate frame from shoulders/hips and compute each joint angle (shoulder abduction/flexion, elbow, hip abduction/flexion, knee, ankle, head yaw, waist yaw) in that frame
- `extract_world_landmarks` / `visible`: pulls MediaPipe 3D world landmarks, applies the filter, and gates angles on a visibility threshold (reuses the last known servo value when a landmark isn't visible)
- `menu_conexao_grafico()`: OpenCV screen to connect the UDP socket to the ESP32's `RoboShadow` network (or skip and run without the robot)
- Main loop: reads the webcam, runs MediaPipe Pose, computes all 16 angles, sends them as one UDP datagram per frame, and draws the HUD overlay with per-joint debug values

### Python Script (`imu_visualizer.py`)

Single script, no modules besides `window_icon.py`. Key sections:
- Backend selection: tries `QtAgg` / `qtagg` / `Qt5Agg` / `TkAgg` in order via `matplotlib.use()` until one works
- `IMUState`: thread-safe holder for the latest accelerometer/gyroscope readings and fused orientation. `update()` runs a complementary filter (95% gyro integration + 5% accelerometer correction when the total acceleration is near 1g) to get roll/pitch/yaw, keeps a `HISTORY_LEN`-sample rolling history per axis for the time-series plots, and tracks packet rate (`hz`) and connection timeout
- `set_upright_reference()` (button "🤖 Em Pé" / key `U`): zeroes roll/pitch/yaw at the robot's current pose; `reset_angles()` (key `R`): re-seeds pitch/roll from the raw gravity vector instead
- `udp_recv_thread()`: listens on UDP port `4212` for `key:value` telemetry, pinging the ESP32 (`IMU:PING` on port `4210`) every 2s until a packet arrives; `send_udp_cmd()` sends text commands to the ESP32 (`IMU:PING`, and `MODE:CAL` on key `C`)
- `rotation_matrix`, `draw_wire_box`, `Arrow3D`/`add_arrow3d`: build and rotate a schematic wireframe torso plus sensor axes in the 3D panel from the fused roll/pitch/yaw
- `setup_figure()`: dark-themed dashboard — 3D orientation panel, 3 accelerometer + 3 gyroscope scrolling plots, HUD text boxes, and the "🤖 Em Pé" button
- `run_visualizer()`: redraws everything on a `UPDATE_MS`-interval Matplotlib timer; a `close_event` handler stops that timer as soon as the window is closed, so it doesn't keep trying to redraw a destroyed figure

### ESP32 Firmware (`4_shadow_esp32_completo.ino`)

- 16 servos driven via a PCA9685 PWM driver over I2C (`SDA=GPIO2`, `SCL=GPIO4`, address `0x40`), channels 0–15 — see the `ch*` `#define`s in the firmware for the channel-to-joint mapping
- MPU-6050 IMU on the same I2C bus (address `0x68`)
- Wi-Fi Access Point: SSID `RoboShadow`, password `shadow123`, ESP32 IP `192.168.4.1`
- UDP control port `4210` (PC → ESP32), UDP IMU telemetry port `4212` (ESP32 → PC)
- Operating modes selected by the PC via UDP text commands: `MODE:ROBOT` (servo control, default), `MODE:IMU` (streams MPU-6050 readings), `MODE:CAL` (re-calibrates the gyro while stationary)
- In `MODE:ROBOT`, an `acao` (action) selects: `PosPadrao()`, `Shadow()` (live angle control, see below), `Andar()`, `Acenar()`, `Sentar()`, `Calibrar()` — settable either by the Python scripts or by the ESP-NOW physical remote
- OTA updates enabled via `ArduinoOTA`

## Wi-Fi UDP Protocol

**Control (PC → ESP32, port 4210, `Shadow()`)**: one UDP datagram per frame containing all 16 angles, each terminated by a single delimiter character, in order: `q w e r t y u i o p a s d f g h`. The ESP32 accumulates digits until a delimiter, then maps each delimiter to a PCA9685 channel via `escreverServo()`.

**IMU telemetry (ESP32 → PC, port 4212)**: ASCII `key:value` pairs separated by commas (`ax`, `ay`, `az`, `gx`, `gy`, `gz`), broadcast at ~50 Hz and parsed by `imu_visualizer.py`.

## Code Conventions

- All comments and variable names are in Portuguese.
- No linting or formatting tools are configured.
