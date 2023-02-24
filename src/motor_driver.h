#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include "rcl/time.h"
#include "esp_timer.h"
#include <driver/gpio.h>
#include <driver/ledc.h>

#include "driver/i2c.h"
#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// Constants
typedef struct {
    gpio_num_t LED_BUILTIN;
    gpio_num_t PIN_LEFT_FORWARD;
    gpio_num_t PIN_LEFT_BACKWARD;
    gpio_num_t PIN_RIGHT_FORWARD;
    gpio_num_t PIN_RIGHT_BACKWARD;
    ledc_channel_t PWM_LEFT_FORWARD;
    ledc_channel_t PWM_LEFT_BACKWARD;
    ledc_channel_t PWM_RIGHT_FORWARD;
    ledc_channel_t PWM_RIGHT_BACKWARD;
    uint32_t PWM_FREQUENCY;
    ledc_timer_bit_t PWM_RESOLUTION;
    ledc_timer_t PWM_TIMER;
    ledc_mode_t PWM_MODE;
    int PWM_MOTOR_MIN;
    int PWM_MOTOR_MAX;
} motor_setup_t;

typedef struct {
    float linear;
    float angular;
} motor_speed_t;

/*Function prototypes*/
float fmap(float val, float in_min, float in_max, float out_min, float out_max);
void InitMotorDriver();
//void SetMotorSpeed(float linear, float angular);
void SetMotorSpeed(motor_speed_t motor_speed);

#endif