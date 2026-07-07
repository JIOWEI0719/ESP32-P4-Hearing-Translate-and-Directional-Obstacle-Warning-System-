# Dual-Board Sign Language & Voice Recognition System

A two-board ESP32-P4 system combining **camera-based gesture recognition** with **voice-based interaction**, designed for assistive communication between sign language users and non-signers.

## System Architecture

```
┌─────────────────────────────┐      UART (115200 8N1)      ┌──────────────────────────────┐
│         Board 1             │  GPIO36(TX) ────→ GPIO38(RX) │          Board 0              │
│  (Gesture Recognition)      │                              │  (Voice & Audio Output)       │
│                             │  AA 55 [7 bytes] [CRC8]      │                               │
│  ┌───────────────────────┐  │                              │  ┌─────────────────────────┐ │
│  │ Camera Capture        │  │                              │  │ LVGL v9 Display UI      │ │
│  │  ↓                    │  │                              │  │  ├─ Email button        │ │
│  │ ESP-DL CNN (96×96)    │  │                              │  │  ├─ Mode toggle         │ │
│  │  ↓                    │  │                              │  │  ├─ DOA radar           │ │
│  │ Gesture Classification│  │                              │  │  └─ Speech command      │ │
│  │  ↓                    │  │                              │  ├─────────────────────────┤ │
│  │ UART Frame Builder    │──┤                              │  │ Audio Speech Recognition│ │
│  └───────────────────────┘  │                              │  │  ├─ WakeNet: "Hi 乐鑫"  │ │
│                             │                              │  │  ├─ MultiNet: 5 commands│ │
│  Hardware:                  │                              │  │  └─ Edge Impulse: EI    │ │
│  • ESP32-P4                │                              │  ├─────────────────────────┤ │
│  • Camera (DVP/MIPI)       │                              │  │ MP3 Playback            │ │
│  • Ethernet                 │                              │  │  ├─ ES8311 codec        │ │
│                             │                              │  │  ├─ Helix MP3 decoder   │ │
└─────────────────────────────┘                              │  │  └─ 7 gesture MP3 files │ │
                                                             │  ├─────────────────────────┤ │
                                                             │  │ UART Gesture Receiver   │ │
                                                             │  │  ├─ AA55 FSM parser     │ │
                                                             │  │  ├─ CRC8 verification   │ │
                                                             │  │  └─ Debounce filter     │ │
                                                             │  └─────────────────────────┘ │
                                                             │                               │
                                                             │  Hardware:                    │
                                                             │  • ESP32-P4 Function EV Board │
                                                             │  • ES8311 Audio Codec         │
                                                             │  • Dual INMP441 Mic Array     │
                                                             │  • LCD Display + Touch        │
                                                             │  • Speaker                    │
                                                             └──────────────────────────────┘
```

## Board 0 — Voice & Audio Output (`board0/`)

The receiver board. Handles voice recognition, audio playback, and the user interface.

### Features

| Module | Description |
|--------|-------------|
| **Voice Recognition** | WakeNet ("Hi 乐鑫") + MultiNet (Chinese commands) |
| **Environment Sound Detection** | Edge Impulse CNN — classifies door_bell / honk with DOA localization |
| **DOA (Direction of Arrival)** | Cross-correlation TDOA on dual INMP441 mic array |
| **MP3 Playback** | Helix MP3 → ES8311 codec → speaker |
| **UART Gesture Receiver** | AA55 protocol parser with CRC8, debounce, and mode gating |
| **Mode Switching** | Voice Mode (SR active) ↔ Sign Mode (UART→MP3 playback) |
| **Email Alert** | SMTP via smtp.qq.com:465 TLS |
| **LVGL UI** | DOA radar display, command text, EI alert labels, mode button |

### Component Structure

```
board0/
├── main/
│   ├── main.c              # App entry, init sequence, mode switch, gesture callback
│   ├── smtp_client.c       # SMTP email sender (TLS)
│   └── smtp_client.h
├── components/
│   ├── audio_sr/           # ESP-SR: AFE + WakeNet + MultiNet + DOA + EI classification
│   ├── music_player/       # MP3 audio player (Helix + ES8311)
│   ├── uart_gesture/       # UART protocol parser (AA55 FSM + CRC8)
│   ├── ui_speech/          # LVGL radar UI + speech status display
│   └── email_ui/           # LVGL email send button + status
└── spiff/                  # SPIFFS image with MP3 files
```

### Key Config (menuconfig)

```
CONFIG_BSP_I2S_NUM=0                    # BSP uses I2S_NUM_0
CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC=y     # mbedTLS allocates from PSRAM
CONFIG_MBEDTLS_SSL_IN_CONTENT_LEN=8192  # Smaller TLS input buffer
CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=65536  # 64KB reserved for internal DRAM
```

---

## Board 1 — Camera Gesture Recognition (`board1/`)

The gesture recognition board. Captures hand gestures via camera and sends results over UART.

### Features

| Module | Description |
|--------|-------------|
| **Camera Capture** | DVP/MIPI camera → RGB → grayscale → 96×96 |
| **CNN Inference** | ESP-DL model (`gesture_cnn_96_gray_esp32p4.espdl`) — 7 gesture classes |
| **UART Transmitter** | AA55 protocol frame builder with CRC8 |
| **Ethernet** | Wired network for RTSP preview |
| **RTSP Preview** | Optional video streaming (`ENABLE_RTSP_PREVIEW`) |

### Gesture Recognition Pipeline

```
Camera → Capture (RGB) → Preprocess (96×96 grayscale) → CNN Inference → 
Confidence Filter → Temporal Smoothing → UART Transmit
```

---

## Build & Flash

### Prerequisites

- ESP-IDF v5.5.4
- ESP32-P4 toolchain

### Board 0

```bash
cd board0
idf.py build
idf.py flash
idf.py monitor
```

### Board 1

```bash
cd board1
idf.py build
idf.py flash
idf.py monitor
```

### Wiring (UART)

```
Board 1 GPIO36 (TX)  ────→  Board 0 GPIO38 (RX)
Board 1 GND           ────→  Board 0 GND
```

---

## Usage Flow

1. Power on both boards
2. Board 0 boots into **Voice Mode** (default) — wake word "Hi 乐鑫" activates command listening
3. Press the mode button to switch to **Sign Mode** — Board 0 listens for UART gesture frames from Board 1
4. Board 1 continuously captures and classifies hand gestures, sending results over UART
5. In Sign Mode, recognized gestures trigger MP3 playback on Board 0's speaker
6. "Send Email" button transmits an alert email via QQ SMTP

---

## Project Status

| Feature | Status |
|---------|--------|
| Voice wake word + command recognition | ✅ Working |
| Environment sound detection (door_bell, honk) | ✅ Working |
| DOA sound localization | ✅ Working |
| Gesture → MP3 via UART | ✅ Working |
| Mode switching (Voice ↔ Sign) | ✅ Working |
| Email alert (SMTP) | ⚠️ Requires menuconfig adjustments |
| Watchdog stability | ✅ Optimized (time-based yield) |
| LVGL radar UI | ✅ Working |

---

## Credits

- ESP-SR: Espressif Speech Recognition framework
- ESP-DL: Espressif Deep Learning library
- Edge Impulse: Embedded ML platform for environment sound classification
- LVGL: Light and Versatile Graphics Library
- Helix MP3: Fixed-point MP3 decoder
