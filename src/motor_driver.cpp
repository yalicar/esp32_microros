#include "motor_driver.h"

// Global variables
motor_setup_t motor_setup = {
    .LED_BUILTIN = GPIO_NUM_2,
    .PIN_LEFT_FORWARD =  GPIO_NUM_12,
    .PIN_LEFT_BACKWARD = GPIO_NUM_13,
    .PIN_RIGHT_FORWARD = GPIO_NUM_15,
    .PIN_RIGHT_BACKWARD = GPIO_NUM_14,
    .PWM_LEFT_FORWARD = LEDC_CHANNEL_0,
    .PWM_LEFT_BACKWARD = LEDC_CHANNEL_3,
    .PWM_RIGHT_FORWARD = LEDC_CHANNEL_4,
    .PWM_RIGHT_BACKWARD = LEDC_CHANNEL_5,
    .PWM_FREQUENCY = 50,
    .PWM_RESOLUTION = LEDC_TIMER_12_BIT,
    .PWM_TIMER = LEDC_TIMER_0,
    .PWM_MODE = LEDC_HIGH_SPEED_MODE,
    .PWM_MOTOR_MIN = 900,
    .PWM_MOTOR_MAX = 1500,
};
// function to initialize the motor driver
void InitMotorDriver() {
    // initialize the LED pin as an output:
    gpio_pad_select_gpio(motor_setup.LED_BUILTIN);
    //set direction of the GPIO as output
    gpio_set_direction(motor_setup.LED_BUILTIN, GPIO_MODE_OUTPUT);
    gpio_set_level(motor_setup.LED_BUILTIN, 1);

    // configure timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = motor_setup.PWM_MODE,           // timer mode
        .duty_resolution = motor_setup.PWM_RESOLUTION, // resolution of PWM duty
        .timer_num = motor_setup.PWM_TIMER,            // timer index
        .freq_hz = motor_setup.PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,                     // Auto select the source clock
        };
    ledc_timer_config(&ledc_timer);

    // configure 4 pwm channels
    ledc_channel_config_t ledc_channel[4] = {
        {
            .gpio_num = motor_setup.PIN_LEFT_FORWARD,
            .speed_mode = motor_setup.PWM_MODE,
            .channel = motor_setup.PWM_LEFT_FORWARD,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = motor_setup.PWM_TIMER,
            .duty = 0,
            .hpoint = 0
        },
        {
            .gpio_num = motor_setup.PIN_LEFT_BACKWARD,
            .speed_mode = motor_setup.PWM_MODE,
            .channel = motor_setup.PWM_LEFT_BACKWARD,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = motor_setup.PWM_TIMER,
            .duty = 0,
            .hpoint = 0
        },
        {
            .gpio_num = motor_setup.PIN_RIGHT_FORWARD,
            .speed_mode = motor_setup.PWM_MODE,
            .channel = motor_setup.PWM_RIGHT_FORWARD,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = motor_setup.PWM_TIMER,
            .duty = 0,
            .hpoint = 0
        },
        {
            .gpio_num = motor_setup.PIN_RIGHT_BACKWARD,
            .speed_mode = motor_setup.PWM_MODE,
            .channel = motor_setup.PWM_RIGHT_BACKWARD,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = motor_setup.PWM_TIMER,
            .duty = 0,
            .hpoint = 0
        }
    };
    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }


}

// function to set the motor speed
void SetMotorSpeed(motor_speed_t motor_speed) {
    // convert the linear and angular speeds to motor speeds for differential drive
    float left_motor_speed = (motor_speed.linear + motor_speed.angular)/2.0f;
    float right_motor_speed = (motor_speed.linear - motor_speed.angular)/2.0f;

    //map the motor speeds to the pwm duty cycle
    uint16_t left_pwm = (uint16_t) fmap(fabs(left_motor_speed), 0.0, 1.0, motor_setup.PWM_MOTOR_MIN, motor_setup.PWM_MOTOR_MAX);
    uint16_t right_pwm = (uint16_t) fmap(fabs(right_motor_speed), 0.0, 1.0, motor_setup.PWM_MOTOR_MIN, motor_setup.PWM_MOTOR_MAX);
    //set direction and speed
    ledc_set_duty(motor_setup.PWM_MODE, motor_setup.PWM_LEFT_FORWARD, left_pwm * (left_motor_speed > 0)); 
    ledc_set_duty(motor_setup.PWM_MODE, motor_setup.PWM_LEFT_BACKWARD, left_pwm * (left_motor_speed < 0));
    ledc_set_duty(motor_setup.PWM_MODE, motor_setup.PWM_RIGHT_FORWARD, right_pwm * (right_motor_speed > 0));
    ledc_set_duty(motor_setup.PWM_MODE, motor_setup.PWM_RIGHT_BACKWARD, right_pwm * (right_motor_speed < 0));
    //update the pwm channels
    ledc_update_duty(motor_setup.PWM_MODE, motor_setup.PWM_LEFT_FORWARD);
    ledc_update_duty(motor_setup.PWM_MODE, motor_setup.PWM_LEFT_BACKWARD);
    ledc_update_duty(motor_setup.PWM_MODE, motor_setup.PWM_RIGHT_FORWARD);
    ledc_update_duty(motor_setup.PWM_MODE, motor_setup.PWM_RIGHT_BACKWARD);
}

// fmap function
float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}