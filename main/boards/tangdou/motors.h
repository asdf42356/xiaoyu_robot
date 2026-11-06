#pragma once
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_pm.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class Motors {
public:
    Motors();
    ~Motors();

    void Forward(uint8_t speed = 100);
    void Backward(uint8_t speed = 100);
    void TurnLeft(uint8_t speed = 100);
    void TurnRight(uint8_t speed = 100);
    void Stop();
    void InitLedc() { EnsureLedc(); }  // 提前初始化 LEDC，避免干扰背光

    void SpinLeft(uint8_t speed = 100);
    void SpinRight(uint8_t speed = 100);
    void TurnLeftMs(int ms, uint8_t speed = 40);
    void TurnRightMs(int ms, uint8_t speed = 40);
    void DemoRun();
    void StopDemo();
    static constexpr int MS_PER_DEGREE = 8;  // ~8ms per degree, calibrate as needed

private:
    static void DemoTimerCallback(void* arg);
    static constexpr ledc_timer_t TIMER = LEDC_TIMER_2;
    static constexpr ledc_mode_t MODE = LEDC_LOW_SPEED_MODE;
    static constexpr uint32_t FREQ_HZ = 1000;
    static constexpr uint32_t MAX_DUTY = 8191;

    void MotorA(int16_t duty);
    void MotorB(int16_t duty);
    void SetPwm(ledc_channel_t ch, uint32_t duty);
    static void TurnTimerCallback(void* arg);
    void StartTurnTimer(int ms);
    void EnsureLedc();
    // 递归互斥锁：保护电机状态与定时器，避免工具调用（主线程）与定时器回调（esp_timer 任务）竞态
    void Lock();
    void Unlock();

    bool running_;
    bool ledc_ready_ = false;
    SemaphoreHandle_t mutex_ = nullptr;
    int16_t last_duty_a_ = 0;
    int16_t last_duty_b_ = 0;
    esp_timer_handle_t turn_timer_ = nullptr;
    esp_timer_handle_t demo_timer_ = nullptr;
    int demo_step_ = 0;
    esp_pm_lock_handle_t pm_lock_ = nullptr;
};
