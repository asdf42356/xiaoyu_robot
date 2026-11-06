#include "duoji.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "DuoJi";

static const struct ServoPinConfig {
    gpio_num_t pin;
    ledc_channel_t channel;
    int default_angle;
} servo_configs[DuoJi::NUM_SERVOS] = {
    {GPIO_NUM_12, LEDC_CHANNEL_3, 0},   // 左前
    {GPIO_NUM_13, LEDC_CHANNEL_4, 0},   // 左后
    {GPIO_NUM_14, LEDC_CHANNEL_5, 0},   // 尾巴
    {GPIO_NUM_38, LEDC_CHANNEL_6, 0},   // 右前
    {GPIO_NUM_11, LEDC_CHANNEL_7, 0},   // 右后
};
DuoJi::DuoJi() {
    // 初始化 LEDC 定时器（50Hz，14位分辨率）
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = TIMER,
        .freq_hz = FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    vTaskDelay(pdMS_TO_TICKS(500));  // 留时间给直流电机初始化完毕，电源稳定后再动舵机

    // 逐个错开舵机初始化，避免同时上电导致电流浪涌掉电复位
    for (int i = 0; i < NUM_SERVOS; i++) {
        channels_[i] = servo_configs[i].channel;

        ledc_channel_config_t ch_cfg = {
            .gpio_num = servo_configs[i].pin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = servo_configs[i].channel,
            .timer_sel = TIMER,
            .duty = 0,
            .hpoint = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

        int angle = servo_configs[i].default_angle;
        uint32_t duty = (uint32_t)((1.5 - angle * (1.0 / 90.0)) * 16384 / 20.0);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channels_[i], duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channels_[i]));

        vTaskDelay(pdMS_TO_TICKS(200));  // 错开 200ms，避免 5 个舵机同时启动电流叠加
    }
    ESP_LOGI(TAG, "所有舵机初始化完成");
}

void DuoJi::SetAngle(ServoID id, int angle) {
    if (angle < -90) angle = -90;
    if (angle > 90) angle = 90;

    // Right-side servos are mounted mirrored → invert to match left-side direction
    if (id == RIGHT_FRONT || id == RIGHT_REAR) {
        angle = -angle;
    }

    // 舵机 PWM 映射: 0.5ms=90°, 1.0ms=45°, 1.5ms=0°, 2.0ms=-45°, 2.5ms=-90°
    // pulse_ms = 1.5 - angle * (1.0 / 90.0)
    uint32_t duty = (uint32_t)((1.5 - angle * (1.0 / 90.0)) * 16384 / 20.0);
    ledc_channel_t ch = channels_[id];

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, ch));
    ESP_LOGI(TAG, "舵机[%d] → %d°", id, angle);
}

void DuoJi::SetAllAngles(const std::array<int, NUM_SERVOS>& angles) {
    for (int i = 0; i < NUM_SERVOS; i++) {
        SetAngle(static_cast<ServoID>(i), angles[i]);
        vTaskDelay(pdMS_TO_TICKS(50));  // stagger servos to avoid current spike
    }
}

void DuoJi::CenterAll() {
    ESP_LOGI(TAG, "所有舵机归中");
    for (int i = 0; i < NUM_SERVOS; i++) {
        SetAngle(static_cast<ServoID>(i), 0);
        vTaskDelay(pdMS_TO_TICKS(80));  // 错开电流
    }
}