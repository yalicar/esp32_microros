// Author: Yalmar Cardenas
// Date: 2022-12-15

#ifndef ROS_H
#define ROS_H

//#include "motor_driver.h"
//#include "encoder.h"
//#include "imu.h"

#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>
#include "rcl/time.h"
#include "esp_timer.h"
#include <driver/gpio.h>
#include <driver/ledc.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif
#include <micro_ros_platformio.h>
#include <Arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/transform.h> 
#include <std_msgs/msg/header.h>

// define constants
#define FRAME_TIME 100 // 10ms
//int FRAME_TIME = 100; // 100ms

//---------------------------------------------Macro functions---------------------------------------------
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// Function prototypes , definitions in ros.cpp
void setupRos();
void cmd_vel_callback(const void *msgin);   
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void publish_imu_raw();
void publish_mag_raw();
void PublishWheelOdom();

#endif