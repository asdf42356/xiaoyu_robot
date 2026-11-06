#include "motors.h"
#include "config.h"
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "Motors";

// Motor pins — GPIO 1 moved to 17 to avoid UART0 TX conflict
static constexpr gpio_num_t MOTOR_A_IN1 = GPIO_NUM_17;
static constexpr gpio_num_t MOTOR_A_IN2 = GPIO_NUM_18;
static constexpr gpio_num_t MOTOR_B_IN1 = GPIO_NUM_10;
static constexpr gpio_num_t MOTOR_B_IN2 = GPIO_NUM_39;

Motors::Motors() : running_(false), ledc_ready_(false) {
    mutex_ = xSemaphoreCreateRecursiveMutex();
    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "motors", &pm_lock_);
    // 只配置 GPIO 输出 LOW+下拉，不碰 LEDC
    gpio_config_t motor_pre = {
        .pin_bit_mask = (1ULL << MOTOR_A_IN1) | (1ULL << MOTOR_A_IN2)
                      | (1ULL << MOTOR_B_IN1) | (1ULL << MOTOR_B_IN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&motor_pre);
    gpio_set_level(MOTOR_A_IN1, 0);
    gpio_set_level(MOTOR_A_IN2, 0);
    gpio_set_level(MOTOR_B_IN1, 0);
    gpio_set_level(MOTOR_B_IN2, 0);

    // ENA 拉低
    gpio_config_t ena_conf = {
        .pin_bit_mask = (1ULL << MOTOR_ENA_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&ena_conf);
    gpio_set_level(MOTOR_ENA_PIN, 0);

    esp_timer_create_args_t timer_args = {
        .callback = TurnTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "turn_timer"
    };
    esp_timer_create(&timer_args, &turn_timer_);

    gpio_set_level(MOTOR_ENA_PIN, 1);
    ESP_LOGD(TAG, "GPIO only (LEDC deferred) A=%d/%d B=%d/%d ENA=%d",
             MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_ENA_PIN);
}

Motors::~Motors() {
    Stop();
    if (turn_timer_ != nullptr) {
        esp_timer_stop(turn_timer_);
        esp_timer_delete(turn_timer_);
    }
    if (pm_lock_ != nullptr) {
        esp_pm_lock_delete(pm_lock_);
    }
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
    }
}

void Motors::Lock() {
    if (mutex_ != nullptr) {
        xSemaphoreTakeRecursive(mutex_, portMAX_DELAY);
    }
}

void Motors::Unlock() {
    if (mutex_ != nullptr) {
        xSemaphoreGiveRecursive(mutex_);
    }
}

void Motors::EnsureLedc() {
    if (ledc_ready_) return;
    ledc_ready_ = true;

    ledc_timer_config_t timer_cfg = {
        .speed_mode = MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = TIMER,
        .freq_hz = FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    struct { gpio_num_t gpio; ledc_channel_t ch; } pins[] = {
        {MOTOR_A_IN1, LEDC_CHANNEL_1},  // CH1 — CH0 被背光占用
        {MOTOR_B_IN1, LEDC_CHANNEL_2},
    };
    for (auto& p : pins) {
        ledc_channel_config_t ch = {
            .gpio_num = p.gpio,
            .speed_mode = MODE,
            .channel = p.ch,
            .timer_sel = TIMER,
            .duty = 0,
            .hpoint = 0,
            .flags = { .output_invert = 0 },
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
        ledc_update_duty(MODE, p.ch);
    }
    ESP_LOGD(TAG, "LEDC initialized");
}

void Motors::SetPwm(ledc_channel_t ch, uint32_t duty) {
    EnsureLedc();
    ledc_set_duty(MODE, ch, duty);
    ledc_update_duty(MODE, ch);
}

void Motors::MotorA(int16_t duty) {
    duty = (int16_t)((float)duty * MOTOR_LEFT_FACTOR);
    if (duty > (int16_t)MAX_DUTY) duty = (int16_t)MAX_DUTY;
    if (duty < -(int16_t)MAX_DUTY) duty = -(int16_t)MAX_DUTY;
    last_duty_a_ = duty;
    if (duty > 0) {
        // Forward: IN1=PWM, IN2=LOW
        ledc_set_pin(MOTOR_A_IN1, MODE, LEDC_CHANNEL_1);
        gpio_set_direction(MOTOR_A_IN2, GPIO_MODE_OUTPUT);
        gpio_set_level(MOTOR_A_IN2, 0);
        SetPwm(LEDC_CHANNEL_1, duty);
        ESP_LOGD(TAG, "MotorA FWD duty=%d", duty);
    } else if (duty < 0) {
        // Reverse: IN1=LOW, IN2=PWM
        ledc_set_pin(MOTOR_A_IN2, MODE, LEDC_CHANNEL_1);
        gpio_set_direction(MOTOR_A_IN1, GPIO_MODE_OUTPUT);
        gpio_set_level(MOTOR_A_IN1, 0);
        SetPwm(LEDC_CHANNEL_1, -duty);
        ESP_LOGD(TAG, "MotorA REV duty=%d", -duty);
    } else {
        // Stop: both LOW
        SetPwm(LEDC_CHANNEL_1, 0);
        gpio_set_direction(MOTOR_A_IN1, GPIO_MODE_OUTPUT);
        gpio_set_direction(MOTOR_A_IN2, GPIO_MODE_OUTPUT);
        gpio_set_level(MOTOR_A_IN1, 0);
        gpio_set_level(MOTOR_A_IN2, 0);
        ESP_LOGD(TAG, "MotorA STOP");
    }
}

void Motors::MotorB(int16_t duty) {
    duty = (int16_t)((float)duty * MOTOR_RIGHT_FACTOR);
    if (duty > (int16_t)MAX_DUTY) duty = (int16_t)MAX_DUTY;
    if (duty < -(int16_t)MAX_DUTY) duty = -(int16_t)MAX_DUTY;
    last_duty_b_ = duty;
    if (duty > 0) {
        // Forward: IN1=PWM, IN2=LOW
        ledc_set_pin(MOTOR_B_IN1, MODE, LEDC_CHANNEL_2);
        gpio_set_direction(MOTOR_B_IN2, GPIO_MODE_OUTPUT);
        gpio_set_level(MOTOR_B_IN2, 0);
        SetPwm(LEDC_CHANNEL_2, duty);
        ESP_LOGD(TAG, "MotorB FWD duty=%d", duty);
    } else if (duty < 0) {
        // Reverse: IN1=LOW, IN2=PWM
        ledc_set_pin(MOTOR_B_IN2, MODE, LEDC_CHANNEL_2);
        gpio_set_direction(MOTOR_B_IN1, GPIO_MODE_OUTPUT);
        gpio_set_level(MOTOR_B_IN1, 0);
        SetPwm(LEDC_CHANNEL_2, -duty);
        ESP_LOGD(TAG, "MotorB REV duty=%d", -duty);
    } else {
        // Stop: both LOW
        SetPwm(LEDC_CHANNEL_2, 0);
        gpio_set_direction(MOTOR_B_IN1, GPIO_MODE_OUTPUT);
        gpio_set_direction(MOTOR_B_IN2, GPIO_MODE_OUTPUT);
        gpio_set_level(MOTOR_B_IN1, 0);
        gpio_set_level(MOTOR_B_IN2, 0);
        ESP_LOGD(TAG, "MotorB STOP");
    }
}

void Motors::Forward(uint8_t speed) {
    Lock();
    running_ = true;
    esp_pm_lock_acquire(pm_lock_);
    uint32_t d = (uint32_t)speed * MAX_DUTY / 100;
    MotorA(d);
    MotorB(-(int16_t)d);
    ESP_LOGD(TAG, "Forward speed=%u", speed);
    Unlock();
}

void Motors::Backward(uint8_t speed) {
    Lock();
    running_ = true;
    esp_pm_lock_acquire(pm_lock_);
    uint32_t d = (uint32_t)speed * MAX_DUTY / 100;
    MotorA(-(int16_t)d);
    MotorB(d);
    ESP_LOGD(TAG, "Backward speed=%u", speed);
    Unlock();
}

void Motors::TurnLeft(uint8_t speed) {
    Lock();
    running_ = true;
    esp_pm_lock_acquire(pm_lock_);
    uint32_t d = (uint32_t)speed * MAX_DUTY / 100;
    MotorA(d / 2);
    MotorB(-(int16_t)d);
    ESP_LOGD(TAG, "TurnLeft speed=%u", speed);
    Unlock();
}

void Motors::TurnRight(uint8_t speed) {
    Lock();
    running_ = true;
    esp_pm_lock_acquire(pm_lock_);
    uint32_t d = (uint32_t)speed * MAX_DUTY / 100;
    MotorA(d);
    MotorB(-(int16_t)(d / 2));
    ESP_LOGD(TAG, "TurnRight speed=%u", speed);
    Unlock();
}

void Motors::Stop() {
    Lock();
    running_ = false;
    esp_pm_lock_release(pm_lock_);
    MotorA(0);
    MotorB(0);
    if (turn_timer_ != nullptr) {
        esp_timer_stop(turn_timer_);
    }
    ESP_LOGD(TAG, "Stop");
    Unlock();
}

void Motors::TurnTimerCallback(void* arg) {
    Motors* self = static_cast<Motors*>(arg);
    self->Stop();
}

void Motors::StartTurnTimer(int ms) {
    if (turn_timer_ != nullptr) {
        esp_timer_stop(turn_timer_);
        esp_timer_start_once(turn_timer_, ms * 1000);
    }
}

void Motors::TurnLeftMs(int ms, uint8_t speed) {
    Lock();
    TurnLeft(speed);
    StartTurnTimer(ms);
    Unlock();
}

void Motors::TurnRightMs(int ms, uint8_t speed) {
    Lock();
    TurnRight(speed);
    StartTurnTimer(ms);
    Unlock();
}

void Motors::SpinLeft(uint8_t speed) {
    Lock();
    running_ = true;
    esp_pm_lock_acquire(pm_lock_);
    uint32_t d = (uint32_t)speed * MAX_DUTY / 100;
    MotorA(0);
    MotorB(-(int16_t)d);
    ESP_LOGD(TAG, "SpinLeft speed=%u", speed);
    Unlock();
}

void Motors::SpinRight(uint8_t speed) {
    Lock();
    running_ = true;
    esp_pm_lock_acquire(pm_lock_);
    uint32_t d = (uint32_t)speed * MAX_DUTY / 100;
    MotorA(d);
    MotorB(0);
    ESP_LOGD(TAG, "SpinRight speed=%u", speed);
    Unlock();
}

void Motors::DemoTimerCallback(void* arg) {
    Motors* self = static_cast<Motors*>(arg);
    self->Lock();
    // 若 demo 已被 StopDemo/Stop 取消，直接返回，
    // 避免“已经触发、正在等锁”的回调在用户 Stop 之后又把电机跑起来
    if (self->demo_timer_ == nullptr) {
        self->Unlock();
        return;
    }
    self->demo_step_++;
    switch (self->demo_step_) {
        case 1: self->Forward(60);  break;
        case 2: self->TurnLeft(40); break;
        case 3: self->TurnRight(40);break;
        case 4: self->SpinLeft(60); break;
        case 5: self->SpinRight(60);break;
        default: self->StopDemo(); self->Unlock(); return;
    }
    esp_timer_start_once(self->demo_timer_, 15000000);  // 15 seconds
    self->Unlock();
}

void Motors::DemoRun() {
    Lock();
    StopDemo();  // 先停掉之前的
    demo_step_ = 0;
    esp_timer_create_args_t args = {
        .callback = DemoTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "demo_timer"
    };
    esp_timer_create(&args, &demo_timer_);
    ESP_LOGI(TAG, "DemoRun started");
    DemoTimerCallback(this);  // 立即执行第一步
    Unlock();
}

void Motors::StopDemo() {
    Lock();
    if (demo_timer_ != nullptr) {
        esp_timer_stop(demo_timer_);
        esp_timer_delete(demo_timer_);
        demo_timer_ = nullptr;
    }
    demo_step_ = -1;
    Stop();
    Unlock();
}
