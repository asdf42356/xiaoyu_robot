#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "button.h"
#include "config.h"
#include "mcp_server.h"
// #include "lamp_controller.h"
#include "led/single_led.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <ctime>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <nvs_flash.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

// ========== 引入舵机驱动 ==========
#include "duoji.h"

// ========== 引入 AHT10 温湿度传感器驱动 ==========
#include "aht10.h"

// ========== 引入直流电机驱动 ==========
#include "motors.h"

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif

#define TAG "CompactWifiBoardLCD"

class CompactWifiBoardLCD : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay* display_;

    // ========== 外设 ==========
    static constexpr int kMaxReminders = 5;

    void LoadReminders(time_t* times, char messages[][128], int* count) {
        *count = 0;
        nvs_handle_t nvs;
        if (nvs_open("reminders", NVS_READONLY, &nvs) != ESP_OK) return;
        for (int i = 0; i < kMaxReminders; i++) {
            char key[8];
            snprintf(key, sizeof(key), "r%d", i);
            size_t len = sizeof(messages[i]);
            if (nvs_get_str(nvs, key, messages[i], &len) != ESP_OK) break;
            // Parse "timestamp|message"
            char* sep = strchr(messages[i], '|');
            if (sep) {
                *sep = '\0';
                times[i] = atoll(messages[i]);
                memmove(messages[i], sep + 1, strlen(sep + 1) + 1);
            }
            (*count)++;
        }
        nvs_close(nvs);
    }

    void SaveReminders(time_t* times, char messages[][128], int count) {
        nvs_handle_t nvs;
        if (nvs_open("reminders", NVS_READWRITE, &nvs) != ESP_OK) return;
        for (int i = 0; i < kMaxReminders; i++) {
            char key[8];
            snprintf(key, sizeof(key), "r%d", i);
            if (i < count) {
                // 手动拼接 "timestamp|message"（避免 %lld，newlib nano 不支持）
                char buf[200];
                int pos = 0;
                long long t = times[i];
                if (t < 0) { buf[pos++] = '-'; t = -t; }
                char tstr[32]; int tp = 0;
                do { tstr[tp++] = '0' + (t % 10); t /= 10; } while (t > 0);
                while (tp > 0) buf[pos++] = tstr[--tp];
                buf[pos++] = '|';
                strcpy(buf + pos, messages[i]);
                nvs_set_str(nvs, key, buf);
            } else {
                nvs_erase_key(nvs, key);
            }
        }
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    void CheckReminders() {
        time_t times[kMaxReminders];
        char messages[kMaxReminders][128];
        int count;
        xSemaphoreTake(reminder_mutex_, portMAX_DELAY);
        LoadReminders(times, messages, &count);
        if (count == 0) {
            xSemaphoreGive(reminder_mutex_);
            return;
        }

        time_t now = time(nullptr);
        bool changed = false;
        for (int i = 0; i < count; i++) {
            // 跳过已标记的提醒
            if (strncmp(messages[i], "[触发] ", 7) == 0) {
                // 已触发超过5分钟，清理掉
                if (now - times[i] > 300) {
                    for (int j = i; j < count - 1; j++) {
                        times[j] = times[j + 1];
                        strcpy(messages[j], messages[j + 1]);
                    }
                    count--;
                    i--;
                    changed = true;
                }
                continue;
            }
            if (now >= times[i]) {
                ESP_LOGI(TAG, "Reminder fired: %s", messages[i]);
                // 定时重试直到设备空闲，然后唤醒（先保存原始消息再加标记）
                {
                    struct RetryCtx { esp_timer_handle_t timer; char msg[128]; };
                    auto ctx = new RetryCtx{nullptr, {}};
                    strncpy(ctx->msg, messages[i], 127);
                    ctx->msg[127] = '\0';
                    esp_timer_create_args_t args = {
                        .callback = [](void* arg) {
                            auto ctx = static_cast<RetryCtx*>(arg);
                            auto& app = Application::GetInstance();
                            if (app.GetDeviceState() == kDeviceStateIdle) {
                                                                // 截断长消息
                                char short_msg[32];
                                int len = strlen(ctx->msg);
                                if (len > 18) len = 18;
                                while (len > 0 && (ctx->msg[len] & 0xC0) == 0x80) len--;
                                memcpy(short_msg, ctx->msg, len);
                                short_msg[len] = '\0';
                                app.WakeWordInvoke(short_msg);
                                esp_timer_stop(ctx->timer);
                                esp_timer_delete(ctx->timer);
                                delete ctx;
                            } else {
                                esp_timer_start_once(ctx->timer, 1000000);
                            }
                        },
                        .arg = ctx,
                        .dispatch_method = ESP_TIMER_TASK,
                        .name = "reminder_wake"
                    };
                    esp_timer_create(&args, &ctx->timer);
                    esp_timer_start_once(ctx->timer, 3000000);
                }
                // 加标记，不清时间
                char marked[128];
                snprintf(marked, sizeof(marked), "[触发] %s", messages[i]);
                strcpy(messages[i], marked);
                changed = true;
                break;  // 一次只触发一条，避免多条同时混乱
            }
        }
        if (changed) SaveReminders(times, messages, count);
        xSemaphoreGive(reminder_mutex_);
    }

    // ========== 温湿度告警（等 idle 后唤醒，合用一个计时器） ==========
    esp_timer_handle_t alert_retry_timer_ = nullptr;

    void EnsureAlertRetry() {
        auto& app = Application::GetInstance();

        // 先尝试立即说话（idle 直接唤醒）
        if (app.GetDeviceState() == kDeviceStateIdle) {
            char msg[32];
            if (has_alert_temp_ && has_alert_hum_) {
                snprintf(msg, sizeof(msg), "热%.0f潮%.0f", last_alert_temp_, last_alert_hum_);
            } else if (has_alert_temp_) {
                snprintf(msg, sizeof(msg), "热%.0f", last_alert_temp_);
            } else {
                snprintf(msg, sizeof(msg), "潮%.0f", last_alert_hum_);
            }
            app.WakeWordInvoke(msg);
            has_alert_temp_ = false;
            has_alert_hum_ = false;
            return;
        }

        // 非 idle：打断当前对话，启动重试等 idle 后唤醒
        app.WakeWordInvoke("alert");
        if (alert_retry_timer_) return;  // 已有计时器在跑
        esp_timer_create_args_t args = {
            .callback = [](void* arg) {
                auto self = static_cast<CompactWifiBoardLCD*>(arg);
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateIdle) {
                    char msg[32];
                    if (self->has_alert_temp_ && self->has_alert_hum_) {
                        snprintf(msg, sizeof(msg), "热%.0f潮%.0f", self->last_alert_temp_, self->last_alert_hum_);
                    } else if (self->has_alert_temp_) {
                        snprintf(msg, sizeof(msg), "热%.0f", self->last_alert_temp_);
                    } else {
                        snprintf(msg, sizeof(msg), "潮%.0f", self->last_alert_hum_);
                    }
                    app.WakeWordInvoke(msg);
                    self->has_alert_temp_ = false;
                    self->has_alert_hum_ = false;
                } else {
                    esp_timer_start_once(self->alert_retry_timer_, 1000000);
                }
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "alert_wake"
        };
        esp_timer_create(&args, &alert_retry_timer_);
        esp_timer_start_once(alert_retry_timer_, 1000000);
    }

    // ========== 情绪追踪 ==========
    void GetEmotionKey(char* key, size_t size) {
        time_t now = time(nullptr);
        struct tm* ti = localtime(&now);
        snprintf(key, size, "em_%04d%02d%02d",
                 (int16_t)(ti->tm_year + 1900), (int8_t)(ti->tm_mon + 1), (int8_t)ti->tm_mday);
    }

    void LoadEmotionReport(char* buf, size_t buf_size) {
        char key[16]; GetEmotionKey(key, sizeof(key));
        nvs_handle_t nvs;
        if (nvs_open("emotions", NVS_READONLY, &nvs) != ESP_OK) { buf[0] = '\0'; return; }
        size_t len = buf_size;
        if (nvs_get_str(nvs, key, buf, &len) != ESP_OK) buf[0] = '\0';
        nvs_close(nvs);
    }

    void SaveEmotionReport(const char* buf) {
        char key[16]; GetEmotionKey(key, sizeof(key));
        nvs_handle_t nvs;
        if (nvs_open("emotions", NVS_READWRITE, &nvs) != ESP_OK) return;
        nvs_set_str(nvs, key, buf);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    void RecordKeyword(const char* keyword) {
        if (!keyword || !*keyword) return;
        // 格式：kw1:c1,kw2:c2
        char buf[384];
        LoadEmotionReport(buf, sizeof(buf));

        // 解析并增加计数
        bool found = false;
        char new_buf[384] = "";
        char* token = strtok(buf, ",");
        while (token) {
            char kw[64]; int cnt = 0;
            if (sscanf(token, "%63[^:]:%d", kw, &cnt) == 2) {
                if (strcmp(kw, keyword) == 0) { cnt++; found = true; }
                char tmp[96];
                snprintf(tmp, sizeof(tmp), "%s%s:%d", new_buf[0] ? "," : "", kw, cnt);
                strncat(new_buf, tmp, sizeof(new_buf) - strlen(new_buf) - 1);
            }
            token = strtok(nullptr, ",");
        }
        if (!found) {
            char tmp[96];
            snprintf(tmp, sizeof(tmp), "%s%s:1", new_buf[0] ? "," : "", keyword);
            strncat(new_buf, tmp, sizeof(new_buf) - strlen(new_buf) - 1);
        }

        SaveEmotionReport(new_buf);
    }

    Motors motors_;
    DuoJi robot_;
    esp_timer_handle_t wave_timer_ = nullptr;
    int wave_step_ = 0;
    esp_timer_handle_t dance_timer_ = nullptr;
    int dance_step_ = 0;
    esp_timer_handle_t walk_timer_ = nullptr;
    int walk_step_ = 0;
    bool walk_forward_ = true;
    esp_timer_handle_t wag_tail_timer_ = nullptr;
    int wag_tail_step_ = 0;
    int64_t last_wag_tail_us_ = 0;  // 摇尾巴防抖时间戳（microseconds）
    esp_timer_handle_t stick_butt_timer_ = nullptr;
    int stick_butt_step_ = 0;
    bool transformed_ = false;  // 变身标志：变身（折叠）后才能用轮子跑

    static void WaveTimerCb(void* arg) {
        auto* self = static_cast<CompactWifiBoardLCD*>(arg);
        auto& r = self->robot_;
        switch (self->wave_step_++) {
            case 0: r.SetAngle(DuoJi::LEFT_FRONT, 80);  break;  // up
            case 1: r.SetAngle(DuoJi::LEFT_FRONT, -10); break;  // down
            case 2: r.SetAngle(DuoJi::LEFT_FRONT, 80);  break;  // up
            case 3: r.SetAngle(DuoJi::LEFT_FRONT, 0);            // center, done
                    esp_timer_stop(self->wave_timer_); return;
        }
        esp_timer_start_once(self->wave_timer_, self->wave_step_ == 2 ? 300000 : 400000);
    }

    void StartWave() {
        if (wave_timer_ == nullptr) {
            esp_timer_create_args_t args = {
                .callback = WaveTimerCb, .arg = this,
                .dispatch_method = ESP_TIMER_TASK, .name = "wave"
            };
            esp_timer_create(&args, &wave_timer_);
        } else {
            esp_timer_stop(wave_timer_);  // cancel any running animation
        }
        wave_step_ = 0;
        WaveTimerCb(this);
    }

    static void DanceTimerCb(void* arg) {
        auto* self = static_cast<CompactWifiBoardLCD*>(arg);
        auto& r = self->robot_;
        int angle = (self->dance_step_ % 2 == 0) ? 40 : -20;
        r.SetAngle(DuoJi::LEFT_FRONT,  angle);
        r.SetAngle(DuoJi::RIGHT_FRONT, angle);
        r.SetAngle(DuoJi::LEFT_REAR,   angle);
        r.SetAngle(DuoJi::RIGHT_REAR,  angle);
        self->dance_step_++;
        if (self->dance_step_ >= 5) {
            self->dance_step_ = 0;
            r.CenterAll();
            esp_timer_stop(self->dance_timer_);
            return;
        }
        esp_timer_start_once(self->dance_timer_, 500000);
    }

    void StartDance() {
        if (dance_timer_ == nullptr) {
            esp_timer_create_args_t args = {
                .callback = DanceTimerCb, .arg = this,
                .dispatch_method = ESP_TIMER_TASK, .name = "dance"
            };
            esp_timer_create(&args, &dance_timer_);
        } else {
            esp_timer_stop(dance_timer_);
        }
        dance_step_ = 0;
        DanceTimerCb(this);
    }

    static void WalkTimerCb(void* arg) {
        auto* self = static_cast<CompactWifiBoardLCD*>(arg);
        auto& r = self->robot_;
        int sign = self->walk_forward_ ? 1 : -1;
        switch (self->walk_step_ % 4) {
            case 0: r.SetAngle(DuoJi::LEFT_FRONT,   60 * sign); r.SetAngle(DuoJi::LEFT_REAR,    60 * sign); break;
            case 1: r.SetAngle(DuoJi::RIGHT_FRONT,  60 * sign); r.SetAngle(DuoJi::RIGHT_REAR,   60 * sign); break;
            case 2: r.SetAngle(DuoJi::LEFT_FRONT,  -60 * sign); r.SetAngle(DuoJi::LEFT_REAR,   -60 * sign); break;
            case 3: r.SetAngle(DuoJi::RIGHT_FRONT, -60 * sign); r.SetAngle(DuoJi::RIGHT_REAR,  -60 * sign); break;
        }
        self->walk_step_++;
        if (self->walk_step_ >= 6) {
            self->walk_step_ = 0;
            r.CenterAll();
            esp_timer_stop(self->walk_timer_);
            return;
        }
        esp_timer_start_once(self->walk_timer_, 400000);
    }

    void StartWalk(bool forward) {
        if (walk_timer_ == nullptr) {
            esp_timer_create_args_t args = {
                .callback = WalkTimerCb, .arg = this,
                .dispatch_method = ESP_TIMER_TASK, .name = "walk"
            };
            esp_timer_create(&args, &walk_timer_);
        } else {
            esp_timer_stop(walk_timer_);
        }
        walk_forward_ = forward;
        walk_step_ = 0;
        WalkTimerCb(this);
    }

    // ---- Simple preset actions ----
    static void StickButtTimerCb(void* arg) {
        auto* self = static_cast<CompactWifiBoardLCD*>(arg);
        auto& r = self->robot_;
        switch (self->stick_butt_step_++) {
            case 0: r.SetAngle(DuoJi::LEFT_FRONT,   60); esp_timer_start_once(self->stick_butt_timer_,  50000); return;
            case 1: r.SetAngle(DuoJi::RIGHT_FRONT,  60); esp_timer_start_once(self->stick_butt_timer_,  50000); return;
            case 2: r.SetAngle(DuoJi::LEFT_REAR,    45); esp_timer_start_once(self->stick_butt_timer_,  50000); return;
            case 3: r.SetAngle(DuoJi::RIGHT_REAR,   45); esp_timer_start_once(self->stick_butt_timer_,  50000); return;
            case 4: r.SetAngle(DuoJi::TAIL,  30);        esp_timer_start_once(self->stick_butt_timer_, 100000); return;
            case 5: r.SetAngle(DuoJi::TAIL, -30);        esp_timer_start_once(self->stick_butt_timer_, 100000); return;
            case 6: r.SetAngle(DuoJi::TAIL,  30);        esp_timer_start_once(self->stick_butt_timer_, 100000); return;
            case 7: r.SetAngle(DuoJi::TAIL, -30);        esp_timer_start_once(self->stick_butt_timer_, 100000); return;
            case 8: r.SetAngle(DuoJi::TAIL,  30);        esp_timer_start_once(self->stick_butt_timer_, 100000); return;
            case 9: r.SetAngle(DuoJi::TAIL, -30);        esp_timer_start_once(self->stick_butt_timer_, 100000); return;
            case 10: r.SetAngle(DuoJi::TAIL,  30);       esp_timer_start_once(self->stick_butt_timer_, 100000); return;
            case 11: r.SetAngle(DuoJi::TAIL, -30);       esp_timer_start_once(self->stick_butt_timer_, 100000); return;
            case 12: r.SetAngle(DuoJi::TAIL, 0);
                     esp_timer_stop(self->stick_butt_timer_); return;
        }
    }
    void StickButt() {
        if (stick_butt_timer_ == nullptr) {
            esp_timer_create_args_t args = {
                .callback = StickButtTimerCb, .arg = this,
                .dispatch_method = ESP_TIMER_TASK, .name = "stick_butt"
            };
            esp_timer_create(&args, &stick_butt_timer_);
        } else {
            esp_timer_stop(stick_butt_timer_);
        }
        stick_butt_step_ = 0;
        StickButtTimerCb(this);
    }

    void StartTransform() {
        // 变身：四脚折叠放平到 -90°
        robot_.CenterAll();
        vTaskDelay(pdMS_TO_TICKS(200));
        robot_.SetAngle(DuoJi::LEFT_REAR,   -90); vTaskDelay(pdMS_TO_TICKS(100));
        robot_.SetAngle(DuoJi::RIGHT_REAR,  -90); vTaskDelay(pdMS_TO_TICKS(100));
        robot_.SetAngle(DuoJi::LEFT_FRONT,  -90); vTaskDelay(pdMS_TO_TICKS(100));
        robot_.SetAngle(DuoJi::RIGHT_FRONT, -90);
        transformed_ = true;
    }

    static void WagTailTimerCb(void* arg) {
        auto* self = static_cast<CompactWifiBoardLCD*>(arg);
        auto& r = self->robot_;
        int cycle = self->wag_tail_step_ / 2;
        if (cycle >= 4) {
            r.SetAngle(DuoJi::TAIL, 0);
            esp_timer_stop(self->wag_tail_timer_);
            return;
        }
        int angle = (self->wag_tail_step_ % 2 == 0) ? 30 : -30;
        r.SetAngle(DuoJi::TAIL, angle);
        self->wag_tail_step_++;
        esp_timer_start_once(self->wag_tail_timer_, 100000);
    }

    void StartWagTail() {
        if (wag_tail_timer_ == nullptr) {
            esp_timer_create_args_t args = {
                .callback = WagTailTimerCb, .arg = this,
                .dispatch_method = ESP_TIMER_TASK, .name = "wag_tail"
            };
            esp_timer_create(&args, &wag_tail_timer_);
        } else {
            esp_timer_stop(wag_tail_timer_);
        }
        wag_tail_step_ = 0;
        WagTailTimerCb(this);
    }

    esp_timer_handle_t hop_timer_ = nullptr;
    int hop_step_ = 0;
    static void HopTimerCb(void* arg) {
        auto* self = static_cast<CompactWifiBoardLCD*>(arg);
        auto& r = self->robot_;
        switch (self->hop_step_++) {
            case 0: r.SetAngle(DuoJi::LEFT_FRONT,   60); esp_timer_start_once(self->hop_timer_, 200000); return;
            case 1: r.SetAngle(DuoJi::RIGHT_FRONT,  60); esp_timer_start_once(self->hop_timer_, 200000); return;
            case 2: r.SetAngle(DuoJi::LEFT_REAR,    60); esp_timer_start_once(self->hop_timer_, 200000); return;
            case 3: r.SetAngle(DuoJi::RIGHT_REAR,   60); esp_timer_start_once(self->hop_timer_, 200000); return;
            case 4: r.CenterAll(); esp_timer_stop(self->hop_timer_); return;
        }
    }
    void StartHop() {
        if (hop_timer_ == nullptr) {
            esp_timer_create_args_t args = {
                .callback = HopTimerCb, .arg = this,
                .dispatch_method = ESP_TIMER_TASK, .name = "hop"
            };
            esp_timer_create(&args, &hop_timer_);
        } else {
            esp_timer_stop(hop_timer_);  // cancel any running animation
        }
        hop_step_ = 0;
        HopTimerCb(this);
    }

    // ---- 惊讶：四条腿同时前伸45度，身体后仰 ----
    esp_timer_handle_t surprise_timer_ = nullptr;
    int surprise_step_ = 0;
    static void SurpriseTimerCb(void* arg) {
        auto* self = static_cast<CompactWifiBoardLCD*>(arg);
        auto& r = self->robot_;
        switch (self->surprise_step_++) {
            case 0:  // 快速后仰
                r.SetAngle(DuoJi::LEFT_FRONT,   45);
                r.SetAngle(DuoJi::RIGHT_FRONT,  45);
                r.SetAngle(DuoJi::LEFT_REAR,    45);
                r.SetAngle(DuoJi::RIGHT_REAR,   45);
                esp_timer_start_once(self->surprise_timer_, 800000);
                return;
            case 1:  // 回中
                r.CenterAll();
                esp_timer_stop(self->surprise_timer_);
                return;
        }
    }
    void StartSurprise() {
        if (surprise_timer_ == nullptr) {
            esp_timer_create_args_t args = {
                .callback = SurpriseTimerCb, .arg = this,
                .dispatch_method = ESP_TIMER_TASK, .name = "surprise"
            };
            esp_timer_create(&args, &surprise_timer_);
        } else {
            esp_timer_stop(surprise_timer_);
        }
        surprise_step_ = 0;
        SurpriseTimerCb(this);
    }

    i2c_master_bus_handle_t aht10_i2c_bus_;
    AHT10* aht10_sensor_;
    float current_temperature_;
    float current_humidity_;
    bool aht10_initialized_;
    esp_timer_handle_t proactive_timer_ = nullptr;
    bool temp_alerted_ = false;
    bool hum_alerted_ = false;
    float last_alert_temp_ = 0;
    float last_alert_hum_ = 0;
    bool has_alert_temp_ = false;
    bool has_alert_hum_ = false;
    SemaphoreHandle_t reminder_mutex_ = nullptr;

    void InitializeI2C() {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AHT10_SDA_PIN,
            .scl_io_num = AHT10_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags = { .enable_internal_pullup = 1 },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &aht10_i2c_bus_));
        aht10_sensor_ = new AHT10(aht10_i2c_bus_);
        if (aht10_sensor_->Init()) {
            aht10_initialized_ = true;
            ESP_LOGI(TAG, "AHT10 sensor initialized successfully");
            aht10_sensor_->ReadSensor();
            current_temperature_ = aht10_sensor_->GetTemperature();
            current_humidity_ = aht10_sensor_->GetHumidity();
        } else {
            aht10_initialized_ = false;
            ESP_LOGE(TAG, "Failed to initialize AHT10 sensor");
        }
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 20 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();

        // 舵机控制 -- 物理硬件，每次用户要求动作时必须调用这些工具，不能只回复文字
        mcp.AddTool("self.servo.set_angle",
            "PHYSICAL servo motor control for legs/tail. Use this for '走'/'walk' (leg-based). "
            "For '跑'/'run'/'前进'/'后退' use self.motors.move (wheel motors) instead!\n"
            "Servo IDs: 0=left_front_leg, 1=left_rear_leg, 2=tail, 3=right_front_leg, 4=right_rear_leg.\n"
            "Angle range: -90 to 90 (0=center).\n"
            "IMPORTANT: never set 4+ servos at once. Move 2 at a time with ~100ms gap.\n"
            "Common actions:\n"
            "- Wave hello: servo 0 to 80, -10, 80, 0 -- spaced ~400ms\n"
            "- Wag tail (摇尾巴): use self.action.wag_tail preset, NOT manual set_angle\n"
            "- Walk (走): prefer self.action.walk_forward / self.action.walk_backward\n"
            "- Spin (转圈, servos): 0,1到45; 3,4到-45; repeat; then center_all\n"
            "- Bow: 0,1,3,4 to 60 (spread) or -30 (crouch)",
            PropertyList({
                Property("servo_id", kPropertyTypeInteger, 0, 4),
                Property("angle", kPropertyTypeInteger, -90, 90)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int id = properties["servo_id"].value<int>();
                int angle = properties["angle"].value<int>();
                robot_.SetAngle(static_cast<DuoJi::ServoID>(id), angle);
                return true;
            });

        mcp.AddTool("self.servo.center_all",
            "PHYSICAL action: center all 5 servo motors to 0 degrees (standing posture). "
            "Call this when user says '变回来', 'untransform', 'stand up', 'reset', 'center', or after movements.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                robot_.CenterAll();
                transformed_ = false;
                return true;
            });

        // 预设动作
        mcp.AddTool("self.action.wave",
            "PHYSICAL preset: wave hello (打招呼). Raises front leg and wags it.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                StartWave(); return true;
            });
        mcp.AddTool("self.action.stick_butt",
            "PHYSICAL preset: stick butt up (撅屁股). Front legs down, rear legs up, tail up.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                StickButt(); return true;
            });
        mcp.AddTool("self.action.hop",
            "PHYSICAL preset: little hop (跳一下). Crouch, jump up, land, stand. Takes about 1 second.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                StartHop(); return true;
            });
        mcp.AddTool("self.action.dance",
            "PHYSICAL preset: dance (跳舞). Rocks all 4 legs back and forth 5 times (~2.5s), then centers.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                StartDance(); return true;
            });
        mcp.AddTool("self.action.walk_forward",
            "PHYSICAL preset: servo-based walk forward (向前走, NOT 跑). For '跑'/'run' use self.motors.move instead. Trots forward 4 steps (~2s) then centers.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                StartWalk(true); return true;
            });
        mcp.AddTool("self.action.walk_backward",
            "PHYSICAL preset: servo-based walk backward (向后退, NOT 跑). For '跑'/'run' use self.motors.move instead. Trots backward 4 steps (~2s) then centers.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                StartWalk(false); return true;
            });

        mcp.AddTool("self.action.wag_tail",
            "PHYSICAL preset: wag tail (摇尾巴). Wags tail 4 times between -30° and 30°, then centers at 0°.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                StartWagTail(); return true;
            });

        mcp.AddTool("self.action.transform",
            "PHYSICAL preset: transform/fold legs (变身/变形). Centers all servos, then folds all 4 legs flat to -90°. "
            "After transform, the robot is in wheel-driving mode. Use self.motors.move to run/spin.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                StartTransform(); return true;
            });

        // 电机控制 -- 行走/转弯
        mcp.AddTool("self.motors.move",
            "PHYSICAL wheel motor control. Use ONLY after calling self.action.transform first! "
            "The robot MUST be in wheel-driving mode (legs folded).\n"
            "If user says '跑'/'前进'/'后退'/'转弯' and robot is NOT transformed -- call self.action.transform FIRST, then call this.\n"
            "direction: 'forward' / 'backward' / 'left' / 'right' / 'spin_left' / 'spin_right' / 'stop'.\n"
            "speed: 1-100 (default 60 for straight, 40 for turns). Use low speed (20-30) for '慢慢走', higher (80-100) for '快点'.\n"
            "IMPORTANT: When user says '停'/'stop'/'别跑了' you MUST call this tool with direction='stop'. "
            "Just saying '好的我停了' in text does NOT stop the motors -- you have to call the tool!\n"
            "angle: optional turning degrees (uses timed turn then auto-stop).",
            PropertyList({
                Property("direction", kPropertyTypeString),
                Property("speed", kPropertyTypeInteger, 1, 100),
                Property("angle", kPropertyTypeInteger, 0, 360)
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                auto dir = properties["direction"].value<std::string>();
                // 统一转小写，避免 "Stop"/"STOP" 等大小写变体匹配不上
                std::string d;
                d.reserve(dir.size());
                for (char c : dir) {
                    d.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
                }
                auto& m = motors_;
                // 停止：兼容多种说法；同时清掉 demo 定时器，防止演示模式停后又自动重启
                if (d == "stop" || d == "停" || d == "停下" || d == "停车" ||
                    d == "halt" || d == "brake" || d == "pause") {
                    m.StopDemo();  // 内部会调用 Stop()
                    return true;
                }
                if (!transformed_) {
                    return std::string("请先变身（折叠四肢）再跑！调用 self.action.transform 后重试");
                }
                int speed = properties["speed"].value<int>();
                int angle = properties["angle"].value<int>();
                if (d == "forward") m.Forward(speed > 0 ? speed : 60);
                else if (d == "backward") m.Backward(speed > 0 ? speed : 60);
                else if (d == "left") {
                    int s = speed > 0 ? speed : 40;
                    if (angle > 0) m.TurnLeftMs(angle * Motors::MS_PER_DEGREE, s);
                    else m.TurnLeft(s);
                }
                else if (d == "right") {
                    int s = speed > 0 ? speed : 40;
                    if (angle > 0) m.TurnRightMs(angle * Motors::MS_PER_DEGREE, s);
                    else m.TurnRight(s);
                }
                else if (d == "spin_left") {
                    m.SpinLeft(speed > 0 ? speed : 60);
                }
                else if (d == "spin_right") {
                    m.SpinRight(speed > 0 ? speed : 60);
                }
                return std::string("未知方向: " + dir + "。支持 forward/backward/left/right/spin_left/spin_right/stop");
            });

        mcp.AddTool("self.motors.demo",
            "Run a full motion demo sequence: forward 15s -> turn left 15s -> turn right 15s "
            "-> spin left 15s -> spin right 15s -> stop. Call this when user says '跑起来' or "
            "'表演一下' or '展示一下'.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (!transformed_) return std::string("请先变身再跑！调用 self.action.transform 后重试。");
                motors_.DemoRun();
                return true;
            });

        // 温湿度传感器
        mcp.AddTool("self.sensor.get_temperature",
            "Get the current INDOOR temperature from the AHT10 sensor (refreshed every 30s). "
            "Use this for checking if the room is too hot/cold (e.g. '开空调', '好热', '好冷'). "
            "For '我要出门了'/'出门': you MUST call self.location.get -> get_weather -> give clothing advice.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                UpdateSensorData();
                if (aht10_initialized_) {
                    return std::to_string(current_temperature_);
                }
                return std::string("AHT10 sensor not initialized");
            });

        mcp.AddTool("self.sensor.get_humidity",
            "Get the current INDOOR humidity from the AHT10 sensor (refreshed every 30s). "
            "If humidity >70% suggest dehumidifier, <30% suggest humidifier. "
            "Use with get_temperature for indoor comfort advice.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                UpdateSensorData();
                if (aht10_initialized_) {
                    return std::to_string(current_humidity_);
                }
                return std::string("AHT10 sensor not initialized");
            });

        mcp.AddTool("self.sensor.last_alert",
            "Get the most recent environment alert values at the moment the alert was triggered. "
            "Call this when you receive '太热了' or '太潮了' — the current sensor value may have "
            "changed since the alert fired. Returns temp and/or humidity readings at alert time.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (!has_alert_temp_ && !has_alert_hum_) return std::string("无历史告警");
                char buf[64];
                int pos = 0;
                if (has_alert_temp_) pos += snprintf(buf + pos, sizeof(buf) - pos, "温度%.1f°C", last_alert_temp_);
                if (has_alert_hum_) pos += snprintf(buf + pos, sizeof(buf) - pos, "%s湿度%.1f%%", pos ? " " : "", last_alert_hum_);
                return std::string(buf);
            });

        // 定位 — NVS 持久存储，断电不丢
        mcp.AddTool("self.location.get",
            "Get the user's saved city. "
            "For '我要出门'/'出门'/'天气怎么': call this then call get_weather & give clothing advice. "
            "If returns empty: MUST ask '你在哪个城市?' first!",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                nvs_handle_t nvs;
                if (nvs_open("location", NVS_READONLY, &nvs) != ESP_OK) return std::string("");
                size_t len = 0;
                nvs_get_str(nvs, "city", nullptr, &len);
                if (len == 0) { nvs_close(nvs); return std::string(""); }
                char* buf = new char[len];
                nvs_get_str(nvs, "city", buf, &len);
                nvs_close(nvs);
                std::string city(buf);
                delete[] buf;
                return city;
            });

        mcp.AddTool("self.location.set",
            "Save the user's city to persistent storage. Call this when user says '我在XX' or tells you their city. "
            "The saved city survives power-off and will be used by location.get next time.",
            PropertyList({
                Property("city", kPropertyTypeString),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                auto city = properties["city"].value<std::string>();
                nvs_handle_t nvs;
                if (nvs_open("location", NVS_READWRITE, &nvs) != ESP_OK) return false;
                nvs_set_str(nvs, "city", city.c_str());
                nvs_commit(nvs);
                nvs_close(nvs);
                return true;
            });

        // 当前时间（AI 用这个算时间戳）
        mcp.AddTool("self.time.now",
            "Get the current Unix timestamp and local time. Call this before setting reminders "
            "so you can calculate absolute timestamps (e.g. '下午3点' -> today 15:00) "
            "or relative ones (e.g. '5分钟后' -> now + 300).",
            PropertyList(),
            [](const PropertyList&) -> ReturnValue {
                time_t now = time(nullptr);
                struct tm* ti = localtime(&now);
                char buf[128];
                // 手动拼接时间戳（避免 %lld，newlib nano 不支持）
                char ts_str[32];
                long long ts = (long long)now;
                int tp = 0;
                if (ts < 0) { ts_str[tp++] = '-'; ts = -ts; }
                char digs[32]; int dp = 0;
                do { digs[dp++] = '0' + (ts % 10); ts /= 10; } while (ts > 0);
                while (dp > 0) ts_str[tp++] = digs[--dp];
                ts_str[tp] = '\0';
                snprintf(buf, sizeof(buf), "%s (%04d-%02d-%02d %02d:%02d:%02d)",
                         ts_str,
                         ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday,
                         ti->tm_hour, ti->tm_min, ti->tm_sec);
                return std::string(buf);
            });

        // 日程提醒
        mcp.AddTool("self.reminder.set",
            "Set a reminder/schedule. The AI MUST convert the user's time to a Unix timestamp.\n"
            "Example: user says '明天早上8点提醒我开会' -> calculate Unix timestamp for 8:00 AM tomorrow -> call this tool.\n"
            "The reminder persists across reboots.",
            PropertyList({
                Property("time_unix", kPropertyTypeInteger, 0, 2147483647),
                Property("message", kPropertyTypeString),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                auto ts = (time_t)properties["time_unix"].value<int>();
                auto msg = properties["message"].value<std::string>();

                time_t times[kMaxReminders];
                char msgs[kMaxReminders][128];
                int count;
                xSemaphoreTake(reminder_mutex_, portMAX_DELAY);
                LoadReminders(times, msgs, &count);
                if (count >= kMaxReminders) {
                    xSemaphoreGive(reminder_mutex_);
                    return std::string("日程已满（最多N条）");
                }

                times[count] = (time_t)ts;
                strncpy(msgs[count], msg.c_str(), 127);
                msgs[count][127] = '\0';
                count++;
                SaveReminders(times, msgs, count);
                xSemaphoreGive(reminder_mutex_);

                char buf[256];
                snprintf(buf, sizeof(buf), "已设 %d 条日程", count);
                return std::string(buf);
            });

        mcp.AddTool("self.reminder.list",
            "List all active reminders/schedules. Returns time and message for each.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                time_t times[kMaxReminders];
                char msgs[kMaxReminders][128];
                int count;
                xSemaphoreTake(reminder_mutex_, portMAX_DELAY);
                LoadReminders(times, msgs, &count);
                if (count == 0) {
                    xSemaphoreGive(reminder_mutex_);
                    return std::string("暂无日程");
                }

                std::string result;
                for (int i = 0; i < count; i++) {
                    char buf[256];
                    if (strncmp(msgs[i], "[触发] ", 7) == 0) {
                        snprintf(buf, sizeof(buf), "[%d] 刚刚触发 — %s", i, msgs[i] + 7);
                    } else {
                        struct tm* ti = localtime(&times[i]);
                        snprintf(buf, sizeof(buf), "[%d] %02d-%02d %02d:%02d  %.200s\n",
                                 i, ti->tm_mon + 1, ti->tm_mday, ti->tm_hour, ti->tm_min, msgs[i]);
                    }
                    result += buf;
                }
                xSemaphoreGive(reminder_mutex_);
                return result;
            });

        mcp.AddTool("self.reminder.delete",
            "Delete a reminder by its index (0-based, from reminder.list).",
            PropertyList({
                Property("index", kPropertyTypeInteger, 0, kMaxReminders - 1),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int idx = properties["index"].value<int>();
                time_t times[kMaxReminders];
                char msgs[kMaxReminders][128];
                int count;
                xSemaphoreTake(reminder_mutex_, portMAX_DELAY);
                LoadReminders(times, msgs, &count);
                if (idx < 0 || idx >= count) {
                    xSemaphoreGive(reminder_mutex_);
                    return std::string("无效的序号");
                }

                for (int i = idx; i < count - 1; i++) {
                    times[i] = times[i + 1];
                    strcpy(msgs[i], msgs[i + 1]);
                }
                count--;
                SaveReminders(times, msgs, count);
                xSemaphoreGive(reminder_mutex_);
                return std::string("已删除");
            });

        mcp.AddTool("self.reminder.clear",
            "Delete ALL reminders at once. Call this when user says '清除所有提醒' or '清空日程'.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                xSemaphoreTake(reminder_mutex_, portMAX_DELAY);
                nvs_handle_t nvs;
                if (nvs_open("reminders", NVS_READWRITE, &nvs) != ESP_OK) {
                    xSemaphoreGive(reminder_mutex_);
                    return std::string("已清除");
                }
                for (int i = 0; i < kMaxReminders; i++) {
                    char key[8];
                    snprintf(key, sizeof(key), "r%d", i);
                    nvs_erase_key(nvs, key);
                }
                nvs_commit(nvs);
                nvs_close(nvs);
                xSemaphoreGive(reminder_mutex_);
                return std::string("已清空所有日程");
            });

        // 情绪追踪
        mcp.AddTool("self.emotion.record_keyword",
            "Record a keyword from the child's conversation for emotional analysis. "
            "Call this when the child mentions something notable like '孤独'、'害怕'、'开心'. "
            "Only record meaningful words, not every word.",
            PropertyList({
                Property("keyword", kPropertyTypeString),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                auto kw = properties["keyword"].value<std::string>();
                RecordKeyword(kw.c_str());
                return true;
            });

        mcp.AddTool("self.emotion.report",
            "Get the child's emotion report for the last N days. The report shows keywords "
            "the child mentioned (like '开心'、'难过'、'害怕') and how many times each appeared. "
            "Use this when parents ask '孩子这周怎么样' or '看看情绪报告'.",
            PropertyList({
                Property("days", kPropertyTypeInteger, 1, 7),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                int days = properties["days"].value<int>();
                if (days < 1) days = 7;

                std::string result;
                time_t now = time(nullptr);
                for (int d = days - 1; d >= 0; d--) {
                    time_t day_ts = now - d * 86400;
                    struct tm* ti = localtime(&day_ts);
                    char key[24];
                    snprintf(key, sizeof(key), "em_%04d%02d%02d",
                             (int16_t)(ti->tm_year + 1900), (int8_t)(ti->tm_mon + 1), (int8_t)ti->tm_mday);

                    nvs_handle_t nvs;
                    char buf[384] = "";
                    if (nvs_open("emotions", NVS_READONLY, &nvs) == ESP_OK) {
                        size_t len = sizeof(buf);
                        nvs_get_str(nvs, key, buf, &len);
                        nvs_close(nvs);
                    }

                    char line[512];
                    if (buf[0]) {
                        snprintf(line, sizeof(line), "%02d-%02d: %s\n",
                                 ti->tm_mon + 1, ti->tm_mday, buf);
                    } else {
                        snprintf(line, sizeof(line), "%02d-%02d: (无)\n",
                                 ti->tm_mon + 1, ti->tm_mday);
                    }
                    result += line;
                }
                if (result.empty()) result = "(无数据)";
                return result;
            });
    }

    void StartProactiveTimer() {
        esp_timer_create_args_t args = {
            .callback = [](void* arg) {
                auto* self = static_cast<CompactWifiBoardLCD*>(arg);
                self->CheckReminders();
                self->UpdateSensorData();

                float temp = self->current_temperature_;
                float hum = self->current_humidity_;

                if (self->aht10_initialized_) {
                    // 温度 > 30°C 提醒开空调（等 idle 后唤醒 AI）
                    if (temp > 30.0f && !self->temp_alerted_) {
                        self->temp_alerted_ = true;
                        self->has_alert_temp_ = true;
                        self->last_alert_temp_ = temp;
                        ESP_LOGW(TAG, "[告警] 温度超标: %.1f°C", temp);
                        self->EnsureAlertRetry();
                    }
                    if (temp <= 28.0f) self->temp_alerted_ = false;

                    // 湿度 > 70% 提醒开除湿（等 idle 后唤醒 AI）
                    if (hum > 70.0f && !self->hum_alerted_) {
                        self->hum_alerted_ = true;
                        self->has_alert_hum_ = true;
                        self->last_alert_hum_ = hum;
                        ESP_LOGW(TAG, "[告警] 湿度超标: %.1f%%", hum);
                        self->EnsureAlertRetry();
                    }
                    if (hum <= 60.0f) self->hum_alerted_ = false;
                }
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "proactive"
        };
        esp_timer_create(&args, &proactive_timer_);
        esp_timer_start_periodic(proactive_timer_, 30000000);  // 30 秒检查一次
        ESP_LOGI(TAG, "Proactive sensor timer started (30s interval)");
    }

    void UpdateSensorData() {
        if (aht10_initialized_) {
            aht10_sensor_->ReadSensor();
            current_temperature_ = aht10_sensor_->GetTemperature();
            current_humidity_ = aht10_sensor_->GetHumidity();
        }
    }

public:
    // 用户开口/打断（唤醒词、按钮打断）时由 Application 回调：兜底停住轮子，
    // 防止模型只回复“停了”却没调用 self.motors.move(stop) 导致电机继续跑
    void OnUserSpeak() override {
        motors_.StopDemo();
    }

    // 设备进入 Idle（对话结束/待机）时由 Application 回调：兜底停住轮子
    void OnDeviceIdle() override {
        motors_.StopDemo();
    }

    CompactWifiBoardLCD() :
        boot_button_(BOOT_BUTTON_GPIO),
        display_(nullptr) {
        reminder_mutex_ = xSemaphoreCreateMutex();
        // 初始化外设
        InitializeI2C();
        // InitializeUltrasonic(); // 超声波已弃用，GPIO 18 给 Motor A IN2
        motors_.InitLedc();       // 在背光之前初始化电机 LEDC，避免时钟干扰
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeTools();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
        StartProactiveTimer();
    }

    virtual bool GetTemperature(float& temperature) override {
        if (aht10_initialized_) {
            temperature = current_temperature_;
            return true;
        }
        return false;
    }

    virtual bool GetHumidity(float& humidity) override {
        if (aht10_initialized_) {
            humidity = current_humidity_;
            return true;
        }
        return false;
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual void OnGreeting() override { StartWave(); }
    // LLM 讲笑话/大笑（播报含笑声词）时自动摇尾巴，无需用户语言指令
    void OnTtsSentence(const char* text) override {
        if (!text || !*text) return;
        static const char* kLaughterWords[] = { "哈哈", "呵呵", "嘻嘻", "咯咯", "嘿嘿" };
        bool laugh = false;
        for (const char* kw : kLaughterWords) {
            if (strstr(text, kw)) { laugh = true; break; }
        }
        if (!laugh) return;
        int64_t now = esp_timer_get_time();
        if (now - last_wag_tail_us_ < 1500000LL) return;  // 1.5 秒内不重复触发
        last_wag_tail_us_ = now;
        StartWagTail();
    }
    // 动作全由 LLM 通过 MCP 工具调用（时序天然正确），emotion 仅控制表情显示
    virtual void OnEmotion(const char* emotion) override { /* 孩子情绪通过 record_keyword 追踪 */ }
    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(CompactWifiBoardLCD);
