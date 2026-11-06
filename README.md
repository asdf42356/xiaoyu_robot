# Zhìbàn Xiǎoguài — MCP-based Multimodal Desktop Companion Robot

([中文](README_zh.md))

> A project for the National University Student IoT Design Competition · Guilin Institute of Information Technology · Team "方寸之间"

## Overview

Zhìbàn Xiǎoguài is a desktop companion robot for children aged 3–12, housed in a self-designed 3D-printed puppy-style enclosure. An ESP32-S3 runs the firmware, connects to a cloud LLM over a WebSocket, and delivers voice dialogue, action sequences, animated expressions and environmental sensing.

The key idea is to expose every on-device capability as an **MCP (Model Context Protocol) tool**: servos, motors, sensors, memory and reminders are all registered as `self.*` tools, which the cloud LLM can discover and invoke on its own — closing the loop "voice interaction → LLM understanding → MCP tool call → multimodal response".

## Features

- **Voice dialogue**: WebSocket connection + OPUS codec, ESP-SR offline wake word, multi-turn Q&A and storytelling
- **Biomimetic motion**: 5 × SG90 servos (front-left / front-right / rear-left / rear-right / tail) with canned actions — wave, dance, walk, hop, bow and tail wag
- **Wheel-leg dual mode**: quadruped walking and N20 DC-motor wheeled driving in one transformable chassis, switching in ~3.2 s
- **Environment sensing**: AHT10 temperature / humidity sampling with dual-threshold hysteresis alarms, reported to the cloud over MCP and answered with a voice care message
- **Animated face**: 240×280 TFT display rendering expressions in real time with LVGL, switching emotion with dialogue state
- **Memory & OTA**: NVS-persisted nickname and preferences, cloud-side conversation history, OTA firmware upgrade

## Architecture

| Item | Implementation |
| --- | --- |
| MCU | ESP32-S3, 16 MB flash partition table |
| SDK | ESP-IDF v5.5 + FreeRTOS |
| Audio | I2S full duplex, 16 kHz capture / 24 kHz playback, OPUS codec, ESP-SR wake word |
| Network | Wi-Fi + WebSocket (MQTT + UDP also supported) |
| Display | ST7789 SPI TFT + LVGL, emoji assets stored in the assets partition |
| Motion | LEDC-PWM servos (50 Hz) + N20 gear motors (L298N driver) |
| Sensing | AHT10 temperature / humidity over I²C: SDA = GPIO9, SCL = GPIO8 |

### Device-side MCP tools

| Tool | Description |
| --- | --- |
| `self.servo.set_angle` / `self.servo.center_all` | Control a single servo or center all of them |
| `self.action.wave` / `dance` / `walk_forward` / `walk_backward` / `hop` / `stick_butt` / `wag_tail` / `transform` | Preset action sequences |
| `self.motors.move` / `self.motors.demo` | Wheeled movement and demo run |
| `self.sensor.get_temperature` / `get_humidity` / `last_alert` | Read sensors and the latest alert |
| `self.location.get` / `set`, `self.time.now`, `self.reminder.*` | Location, time and reminders |
| `self.emotion.record_keyword` / `self.emotion.report` | Record and report emotion keywords |

## Repository layout

```
main/
├── boards/tangdou/    Board implementation: servo, motor and AHT10 drivers, board MCP tools
├── boards/common/     Shared display, audio, button and power drivers
├── audio/             Audio channels, wake word and audio service
├── display/           LVGL UI and expression rendering
├── protocols/         WebSocket / MQTT protocols
├── assets/            Fonts and emoji assets
├── application.cc     Main application flow and state machine
└── mcp_server.cc      MCP server implementation
partitions/v2/         Partition tables (16 MB by default)
scripts/               Asset, language and packaging tools
docs/                  Protocol and development docs
```

## Build and flash

Requires [ESP-IDF v5.5](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/get-started/):

```bash
idf.py set-target esp32s3
idf.py menuconfig      # Xiaozhi Assistant → Board Type → Tangdou (糖豆)
idf.py build
idf.py -p <PORT> flash monitor
```

On first boot the device enters provisioning mode; see [docs/blufi.md](docs/blufi.md) for Wi-Fi provisioning.

## Credits and license

The firmware framework, board support layer and communication protocols are derived from the open-source project [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32). The board configuration, servo and motor control, temperature / humidity sensing, expression rendering and MCP tool set are implemented by this project.

Released under the [MIT License](LICENSE).
