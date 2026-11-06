#pragma once
#include <driver/ledc.h>
#include <esp_log.h>
#include <array>

class DuoJi {
public:
    enum ServoID {
        LEFT_FRONT  = 0,   // 左前
        LEFT_REAR   = 1,   // 左后
        TAIL        = 2,   // 尾巴
        RIGHT_FRONT = 3,   // 右前
        RIGHT_REAR  = 4,   // 右后
        NUM_SERVOS
    };

    DuoJi();
    void SetAngle(ServoID id, int angle);
    void SetAllAngles(const std::array<int, NUM_SERVOS>& angles);
    void CenterAll();

private:
    static constexpr ledc_timer_t TIMER = LEDC_TIMER_3;
    static constexpr uint32_t FREQ_HZ = 50;
    std::array<ledc_channel_t, NUM_SERVOS> channels_;
};