#ifndef MPU6050_H
#define MPU6050_H


#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


//  Data structs
struct GyroData {
    float rate_roll;
    float rate_pitch;
    float rate_yaw;
};
struct AccData {
    float ax;
    float ay;
    float az;
};


class MPU6050 {
    private:
        //  Protocols, pins and registers variables.
        i2c_inst_t *i2c;
        uint16_t scl_pin;
        uint16_t sda_pin;
        int addr = 0x68;

        //  Calibration object.
        GyroData gcd;

        //  Gyro register and variables.
        uint8_t gyro_reg = 0x43;
        uint8_t gyro_bits[6];
        int16_t gyro_x, gyro_y, gyro_z;
        GyroData gd;

        //  Acc register and variables.
        uint8_t acc_reg = 0x3B;
        uint8_t acc_bits[6];
        int16_t acc_x, acc_y, acc_z;
        AccData ad;

    public:
        //  Calibration variables.
        float rate_calibration_roll;
        float rate_calibration_pitch;
        float rate_calibration_yaw;

        //  General functions.
        MPU6050(i2c_inst_t *i2c_instance, uint16_t scl, uint16_t sda);
        void init();
        void set_gyro_calibration();
        GyroData get_gyro();
        AccData get_acc();
};

#endif
