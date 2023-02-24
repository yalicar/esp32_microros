
#ifndef MPU9250_H
#define MPU9250_H
#include <inttypes.h>
#include <MPU9250_WE.h>


// struct for setup imu
typedef struct {
    char direction;
} imu_setup_t;

typedef struct{
    struct {
        float x;
        float y;
        float z;
    } accelerometer, gyroscope, magnetometer;
    float temperature;
} imu_data_t;

void setup_imu(imu_setup_t imu_setup);
void get_imu_data();

#endif