#include <micro_ros_platformio.h>
#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif
#include "ros.h"
#include "mpu9250.h"
#include "motor_driver.h"
#include "encoder.h"


imu_setup_t imu_setup = {
    .direction = 0x68
};




void setup() {
// put your setup code here, to run once:
IPAddress agent_ip(192, 168, 1, 31);
size_t agent_port = 8888;
char ssid[] = "CLARO1_8D971B";
char psk[]= "0s3WtDtFLq";
set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

// import functions
InitMotorDriver();
InitEncoder();
setup_imu(imu_setup);
setupRos();

}

void loop() {
// put your main code here, to run repeatedly:

}