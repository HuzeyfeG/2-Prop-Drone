#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "MPU6050.h"


MPU6050::MPU6050(i2c_inst_t *i2c_instance, uint16_t scl, uint16_t sda) {
    this->i2c = i2c_instance;
    this->scl_pin = scl;
    this->sda_pin = sda;
}


void MPU6050::init() {
    //  Set frequency and pins.
    i2c_init(i2c, 400 * 1000);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_pull_up(scl_pin);
    gpio_pull_up(sda_pin);

    uint8_t data[2];
    //  Power management.
    data[0] = 0x6B;
    data[1] = 0x00;
    i2c_write_blocking(i2c, addr, data, 2, true);

    //  Configuration settings (low_pass_filter).
    data[0] = 0x1A;
    data[1] = 0x05; // Bandwith 10Hz 
    i2c_write_blocking(i2c, addr, data, 2, true); 

    //  Set the gyro configuration.
    data[0] = 0x1B;
    data[1] = 0x08; // ±500 degrees/second
    i2c_write_blocking(i2c, addr, data, 2, true); 

    //  Set the acc configuration.
    data[0] = 0x1C;
    data[1] = 0x10; // ±8g
    i2c_write_blocking(i2c, addr, data, 2, true);
}

void MPU6050::set_gyro_calibration() {
    for(int i = 0; i < 2000; i++) {
        gcd = get_gyro();
        rate_calibration_roll += gcd.rate_roll;
        rate_calibration_pitch += gcd.rate_pitch;
        rate_calibration_yaw += gcd.rate_yaw;
        sleep_ms(1);
    }

    rate_calibration_roll /= 2000;
    rate_calibration_pitch /= 2000;
    rate_calibration_yaw /= 2000;
}

GyroData MPU6050::get_gyro() {
    //  Write and read the register.
    i2c_write_blocking(i2c, addr, &gyro_reg, 1, true);
    i2c_read_blocking(i2c, addr, gyro_bits, 6, false);

    gyro_y = ((gyro_bits[0]<<8) | gyro_bits[1]);
    gyro_x = ((gyro_bits[2]<<8) | gyro_bits[3]);
    gyro_z = ((gyro_bits[4]<<8) | gyro_bits[5]);

    gd.rate_roll = (float)gyro_x / 65.5;
    gd.rate_pitch = (float)gyro_y / 65.5;
    gd.rate_yaw = (float)gyro_z / 65.5;
    return gd;
}

AccData MPU6050::get_acc() {
    //  Write and read the register.
    i2c_write_blocking(i2c, addr, &acc_reg, 1, true);
    i2c_read_blocking(i2c, addr, acc_bits, 6, false);

    acc_x = ((acc_bits[0]<<8) | acc_bits[1]);
    acc_y = ((acc_bits[2]<<8) | acc_bits[3]);
    acc_z = ((acc_bits[4]<<8) | acc_bits[5]);

    //  Manual calibration.
    ad.ax = (float)acc_x / 4096 - 0.0207;
    ad.ay = (float)acc_y / 4096 - 0.0215;
    ad.az = (float)acc_z / 4096 + 0.01;
    return ad;
}