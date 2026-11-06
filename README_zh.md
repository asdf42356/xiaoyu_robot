# 智伴小怪 — MCP 多模态桌面陪伴机器人

（[English](README.md)）

> 全国大学生物联网设计竞赛作品 · 桂林信息科技学院 · 团队「方寸之间」

## 项目简介

「智伴小怪」是一款面向 3–12 岁儿童的桌面陪伴机器人，整机外形为自主 3D 建模的童趣风格机器犬。设备以 ESP32-S3 为主控，通过 Wi-Fi 与云端 AI 大模型建立 WebSocket 长连接，完成语音对话、动作编排、情绪表情与环境感知。

项目的核心思路是把设备端硬件能力统一抽象为 **MCP（Model Context Protocol）工具接口**：舵机、电机、温湿度传感器、记忆与提醒等能力都以 `self.*` 工具的形式注册，云端大模型可以自主发现并调用，形成「语音交互 → 大模型语义理解 → MCP 工具调用 → 多模态响应」的完整闭环。

## 主要功能

- **语音对话**：WebSocket 长连接 + OPUS 编解码，ESP-SR 离线唤醒，支持多轮问答与故事讲述
- **仿生运动**：5 路 SG90 舵机（左前 / 右前 / 左后 / 右后 / 尾巴），内置招手、跳舞、走路、跳跃、俯身摆尾等动作组
- **轮足双模**：四足步行与 N20 直流电机轮式移动二合一，形态切换约 3.2 秒
- **环境感知**：AHT10 温湿度采集，双阈值滞回告警，超限数据经 MCP 上报并联动语音关怀
- **动态表情**：240×280 TFT 彩屏 + LVGL 实时渲染，随对话状态自动切换情绪
- **记忆与升级**：NVS 持久化昵称与偏好，云端保存对话历史，支持 OTA 远程升级

## 技术架构

| 项目 | 实现 |
| --- | --- |
| 主控 | ESP32-S3，16 MB Flash 分区表 |
| 软件平台 | ESP-IDF v5.5 + FreeRTOS |
| 音频 | I2S 全双工，16 kHz 采集 / 24 kHz 播放，OPUS 编解码，ESP-SR 唤醒词 |
| 通信 | Wi-Fi + WebSocket（同时支持 MQTT + UDP） |
| 显示 | ST7789 SPI TFT + LVGL，表情资源存放于 assets 分区 |
| 运动 | LEDC-PWM 舵机（50 Hz）+ N20 减速电机（L298N 驱动） |
| 感知 | AHT10 温湿度，I²C：SDA = GPIO9，SCL = GPIO8 |

### 设备端 MCP 工具

| 工具 | 说明 |
| --- | --- |
| `self.servo.set_angle` / `self.servo.center_all` | 单路 / 全部舵机角度控制 |
| `self.action.wave` / `dance` / `walk_forward` / `walk_backward` / `hop` / `stick_butt` / `wag_tail` / `transform` | 预设动作组 |
| `self.motors.move` / `self.motors.demo` | 轮式移动与演示 |
| `self.sensor.get_temperature` / `get_humidity` / `last_alert` | 温湿度读取与告警查询 |
| `self.location.get` / `set`、`self.time.now`、`self.reminder.*` | 位置、时间与提醒管理 |
| `self.emotion.record_keyword` / `self.emotion.report` | 情绪关键词记录与上报 |

## 目录结构

```
main/
├── boards/tangdou/    本项目的板级实现：舵机、电机、AHT10 驱动与板级 MCP 工具注册
├── boards/common/     各板共用的显示、音频、按键、电源等驱动
├── audio/             音频通道、唤醒词与服务
├── display/           LVGL 界面与表情显示
├── protocols/         WebSocket / MQTT 通信协议
├── assets/            字体与表情资源
├── application.cc     应用主流程与状态机
└── mcp_server.cc      MCP 服务端实现
partitions/v2/         分区表（默认 16 MB）
scripts/               资源生成、语言生成、打包等工具
docs/                  协议与二次开发文档
```

## 构建与烧录

需要 [ESP-IDF v5.5](https://docs.espressif.com/projects/esp-idf/en/v5.5/esp32s3/get-started/) 环境：

```bash
idf.py set-target esp32s3
idf.py menuconfig      # Xiaozhi Assistant → Board Type → Tangdou (糖豆)
idf.py build
idf.py -p <串口> flash monitor
```

首次上电后设备进入配网模式，按提示连接 Wi-Fi；配网与网络配置说明见 [docs/blufi_zh.md](docs/blufi_zh.md)。

## 致谢与许可

本项目的固件框架、板级支持与通信协议基于开源项目 [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) 二次开发，「智伴小怪」在其之上实现了新的板级配置、舵机与电机控制、温湿度感知、表情显示及 MCP 工具集合。

代码以 [MIT License](LICENSE) 发布。
