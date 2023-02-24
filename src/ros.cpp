// include libraries once and define constants
#include "ros.h"
#include "mpu9250.h"
#include "motor_driver.h"
#include "encoder.h"

rcl_publisher_t publisher; 
std_msgs__msg__Int32 msg_int;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_publisher_t publisher_twist; // Publisher to publish twist message
rcl_publisher_t publisher_imu; // Publisher to publish imu message
rcl_publisher_t publisher_mag; // Publisher to publish magnetometer message
geometry_msgs__msg__Twist msg; // Message type Twist
geometry_msgs__msg__Twist twist_msg; // Message type Twist for publisher twist
nav_msgs__msg__Odometry odom_msg; // Message type Odometry for publisher encoder odometry
sensor_msgs__msg__Imu imu_msg; // Message type Imu for publish imu data
sensor_msgs__msg__MagneticField mag_msg; // Message type MagneticField for publish magnetometer data


extern imu_data_t imu;
extern encoder_position_t encoder_position;
extern encoder_velocity_t encoder_velocity;
extern encoder_count_t encoder_count;
extern encoder_speed_t encoder_speed;

motor_speed_t motor_speed = {
    .linear = 0,
    .angular = 0
};

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void cmd_vel_callback(const void *msgin) {
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) {
    return;
    }

    // save linear and angular velocity in motor_speed struct
    motor_speed.linear = constrain(msg.linear.x, -1, 1);
    motor_speed.angular = constrain(msg.angular.z, -1, 1);
    SetMotorSpeed(motor_speed);
    get_imu_data();
    publish_imu_raw();
    publish_mag_raw();
    PublishWheelOdom();

}

void setupRos() {
  // create init_options
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "esp32_differential_drive", "", &support));

  // create subscriber to subscribe twist message from topic /cmd_vel
  rcl_subscription_t subscriber;
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));
    
 
  // create publisher to /wheel/odometry
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/wheel/odometry"));

  // create publisher to publish imu data
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data_raw"));
  
  // create publisher to publish magnetometer data
  RCCHECK(rclc_publisher_init_default(
    &publisher_mag,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
    "/imu/mag"));

  // create timer,
  const unsigned int timer_timeout = FRAME_TIME;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor,&subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg_int.data = 0;

  // spin
  while (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(timer_timeout)) == RCL_RET_OK) {
    delay(10);
  }
  
}

void publish_imu_raw() {
  imu_msg.header.frame_id.data = const_cast<char*>("imu_link");
  imu_msg.header.stamp.sec = millis() / 1000;
  imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
  imu_msg.angular_velocity.x = imu.gyroscope.x;
  imu_msg.angular_velocity.y = imu.gyroscope.y;
  imu_msg.angular_velocity.z = imu.gyroscope.z;
  imu_msg.linear_acceleration.x = imu.accelerometer.x;
  imu_msg.linear_acceleration.y = imu.accelerometer.y;
  imu_msg.linear_acceleration.z = imu.accelerometer.z;
  // publish imu data
  RCSOFTCHECK(rcl_publish(&publisher_imu, &imu_msg, NULL));

}

void publish_mag_raw() {
  mag_msg.header.frame_id.data = const_cast<char*>("imu_link");
  mag_msg.header.stamp.sec = millis() / 1000;
  mag_msg.header.stamp.nanosec = (millis() % 1000) * 1000000;
  mag_msg.magnetic_field.x = imu.magnetometer.x;
  mag_msg.magnetic_field.y = imu.magnetometer.y;
  mag_msg.magnetic_field.z = imu.magnetometer.z;
  // publish magnetometer data
  RCSOFTCHECK(rcl_publish(&publisher_mag, &mag_msg, NULL));

}

void PublishWheelOdom() {
    // get current time for header timestamp field in microROS message
    odom_msg.header.frame_id.data = const_cast<char*>("odom");
    odom_msg.child_frame_id.data = const_cast<char*>("base_link");
    odom_msg.header.stamp.sec = (uint32_t) (esp_timer_get_time() / 1000000);
    odom_msg.header.stamp.nanosec = (uint32_t) (esp_timer_get_time() % 1000000) * 1000;
    // set the position
    odom_msg.pose.pose.position.x = encoder_position.x; // x position
    odom_msg.pose.pose.position.y = encoder_position.y; // y position
    odom_msg.pose.pose.position.z = encoder_position.theta; // z position
    // set the orientation
    odom_msg.pose.pose.orientation.x = 0.0; // x orientation 
    odom_msg.pose.pose.orientation.y = 0.0; // y orientation
    odom_msg.pose.pose.orientation.z = 0.0; // z orientation
    odom_msg.pose.pose.orientation.w = 1.0; // w orientation
    // set the linear velocity 
    odom_msg.twist.twist.linear.x = encoder_velocity.linear;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    // set the angular velocity 
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = encoder_velocity.angular;


    
    // publish odometry message
    RCSOFTCHECK(rcl_publish(&publisher, (const void*)&odom_msg, NULL));
    // get encoder data
    GetEncoder(); 
}