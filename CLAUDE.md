# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Keeping This File Up To Date

**This file must always be updated whenever a code change makes any part of it outdated.** Treat it as part of the deliverable, not an afterthought — if a change alters the architecture, data flow, serial protocol, pin assignments, dependencies, entry points, run commands, or file layout described below, update the corresponding section in the same change.

Likewise, keep `.gitignore` current: any change that introduces generated files, build output, or local environment directories should add them there.

## Project Overview

Robô Shadow is a humanoid robot that mimics a person's movements in real time. A Python script uses a webcam and MediaPipe pose detection to calculate joint angles, then transmits them via serial to an ESP32 microcontroller that drives 13 servo motors.

## Running the Python Application

Install dependencies:
```bash
pip install -r requirements.txt
```

Run:
```bash
python Program_Shadow.py
```

The script prompts whether to connect to ESP32 via serial. Press `q` in the video window to exit.

**Serial port** is hardcoded at line 8 (`/dev/ttyUSB0`). Change it if your ESP32 is on a different port.

## ESP32 Firmware

Located in `shadow_esp32_controle/shadow_esp32_controle.ino`. Flash using Arduino IDE or PlatformIO with these libraries installed:
- `ESP32_Servo`
- `WiFiManager`
- ESP32 board support package

## Architecture

### Data Flow

```
Webcam → MediaPipe pose landmarks → angle calculations → serial → ESP32 → servos
```

The WiFi HTTP interface (`/shadow`, `/andar`, `/acenar`, `/sentar`, `/parar`) on the ESP32 triggers preset motion sequences independently of the Python serial stream.

### Python Script (`Program_Shadow.py`)

Monolithic script (~350 lines, no modules). Key sections:
- Lines 1–39: imports, serial connection prompt
- Lines 40–43: webcam init, MediaPipe setup (75% confidence thresholds)
- Lines 44–126: landmark extraction (nose, shoulders, elbows, wrists, hips, knees, heels, feet)
- Lines 128–302: angle calculations using `atan`/`degrees` — 12 angles total (shoulders, head, arms, forearms, legs, knees)
- Lines 303–351: serial write loop sending angles with single-character delimiters (`q w e r t y u i o p a s d`)

### ESP32 Firmware (`shadow_esp32_controle.ino`)

- 13 servos on GPIO pins: 2, 4, 12, 14, 18, 21, 22, 23, 25, 26, 27, 32, 33
- Static IP: `192.168.8.222`, HTTP server on port 80
- WiFi AP fallback: SSID `Shadow`, password `12345678`
- Serial baud rate: 115200
- `Shadow()` function reads serial angles and updates all servos in real time
- Preset functions: `PosPadrao()`, `Andar()`, `Acenar()`, `Sentar()`

### Serial Protocol

Python sends 12 angle values, each terminated by a single delimiter character in order: `q`, `w`, `e`, `r`, `t`, `y`, `u`, `i`, `o`, `p`, `a`, `s`. The ESP32 parses incoming bytes, accumulates digits until a delimiter, then maps each delimiter to a specific servo.

## Code Conventions

- All comments and variable names are in Portuguese.
- No linting or formatting tools are configured.
