#include <stdio.h>
#include "pico/stdlib.h"
#include "NRF24L01.h"
#include "MPU6050.h"
#include "BLDC.h"
#include "Servo.h"
#include "math.h"


//  Pins and Ports
#define SPI_PORT spi0
#define CE_PIN 20
#define CSN_PIN 17

#define I2C_PORT i2c0
#define SCL_PIN 1
#define SDA_PIN 0

#define BLDC_L_PIN 2
#define BLDC_R_PIN 4

#define SERVO_L_PIN 6
#define SERVO_R_PIN 8


//  nRF24
NRF24 radio = NRF24(SPI_PORT, CSN_PIN, CE_PIN);
uint8_t address[] = "ykrc1";
uint8_t received_data[4] = {0};
void nrf_setup() {
    radio.config(address, 46, 10);
    radio.modeRX();
}


//  MPU6050
MPU6050 mpu = MPU6050(I2C_PORT, SCL_PIN, SDA_PIN);
AccData acc_values;
GyroData gyro_values;
float angle_roll, angle_pitch;


//  BLDC
BLDC bldc_l(BLDC_L_PIN, 1000, 2000);
BLDC bldc_r(BLDC_R_PIN, 1000, 2000);


//  Servo
Servo servo_l(SERVO_L_PIN, 1000, 2000);
Servo servo_r(SERVO_R_PIN, 1000, 2000);


//  Drone Data
struct DroneData {
    uint16_t m_l = 1000;
    uint16_t m_r = 1000;
    uint16_t s_l = 1500;
    uint16_t s_r = 1500;
};
DroneData drone_data;
int16_t input_throttle = 0;


//	PID constants, variables and equation.
//	Note: Kp, Ki and Kd constants can be different in every systems.
float Kp_roll = 3.0, Ki_roll = 0.01, Kd_roll = 2.0;
float desired_rate_roll = 0.0;
float input_roll, error_rate_roll, prev_error_rate_roll, prev_iterm_rate_roll;
float Kp_pitch = 10.0, Ki_pitch = 0.001, Kd_pitch = 3.5;
float desired_rate_pitch = 0.0;
float input_pitch, error_rate_pitch, prev_error_rate_pitch, prev_iterm_rate_pitch;
float Kp_yaw = 1.0, Ki_yaw = 0.001, Kd_yaw = 0.01;
float desired_rate_yaw = 0.0;
float input_yaw, error_rate_yaw, prev_error_rate_yaw, prev_iterm_rate_yaw;
float pid_return[] = {0, 0, 0};
void pid_equation(float error, float kp, float ki, float kd, float prev_error, float prev_iterm, uint8_t limit_value) {
	//	Calculate the P.
	float p_term = kp * error;

	//	Calculate the I.
	float i_term = prev_iterm + (ki * (error + prev_error) * 0.004 / 2);
	if(i_term > limit_value) {
		i_term = limit_value;
	} else if(i_term < -limit_value) {
		i_term = -limit_value;
	}

	//	Calculate the D.
	float d_term = kd * ((error - prev_error) / 0.004);

	//	P+I+D
	float pid_output = p_term + i_term + d_term;

	if(pid_output > limit_value) {
		pid_output = limit_value;
	} else if(pid_output < -limit_value) {
		pid_output = -limit_value;
	}
	//	Transfer results to variable.
	pid_return[0] = pid_output;
	pid_return[1] = error;
	pid_return[2] = i_term;
}
void reset_pid() {
  prev_error_rate_roll = 0; prev_error_rate_pitch = 0; prev_error_rate_yaw = 0;
  prev_iterm_rate_roll = 0; prev_error_rate_pitch = 0; prev_error_rate_yaw = 0;
}


//  Kalman Filter
float kalman_angle_roll = 0, kalman_uncertainty_angle_roll = 2 * 2;
float kalman_angle_pitch = 0, kalman_uncertainty_angle_pitch =  2 * 2;
float kalman_output[] = {0, 0};
void kalman_filter(float kalman_state, float kalman_uncertainty, float kalman_input, float kalman_measurement) {
    kalman_state = kalman_state + (0.004 * kalman_input);
    kalman_uncertainty = kalman_uncertainty + (0.004 * 0.004 * 4 * 4);
    float kalman_gain = kalman_uncertainty * 1 / ((1 * kalman_uncertainty) + (3 * 3));
    kalman_state = kalman_state + (kalman_gain * (kalman_measurement - kalman_state));
    kalman_uncertainty = (1 - kalman_gain) * kalman_uncertainty;
    
    kalman_output[0] = kalman_state;
    kalman_output[1] = kalman_uncertainty;
}


//  Timer
uint32_t timer_throttle, timer_loop, servo_timer;


int main() {
    stdio_init_all();
    sleep_ms(3000);

    bldc_l.set_power(1000);
    bldc_r.set_power(1000);
    servo_l.set_duty(1500);
    servo_r.set_duty(1500);

    nrf_setup();
    mpu.init();
    sleep_ms(250);

    mpu.set_gyro_calibration();
    
    sleep_ms(3000);
    timer_throttle = time_us_32();
    timer_loop = timer_throttle;
    servo_timer = timer_throttle;
    while (true) {
        if (radio.newMessage()) {
            gyro_values = mpu.get_gyro();
            gyro_values.rate_roll -= mpu.rate_calibration_roll;
            gyro_values.rate_pitch -= mpu.rate_calibration_pitch;
            gyro_values.rate_yaw -= mpu.rate_calibration_yaw;
            acc_values = mpu.get_acc();
            angle_pitch = atan(acc_values.ay / sqrt((acc_values.ax * acc_values.ax) + (acc_values.az * acc_values.az))) * 1 / (3.142 / 180);
            angle_roll = -atan(acc_values.ax / sqrt((acc_values.ay * acc_values.ay) + (acc_values.az * acc_values.az))) * 1 / (3.142 / 180);

            kalman_filter(kalman_angle_roll, kalman_uncertainty_angle_roll, gyro_values.rate_roll, angle_roll);
            kalman_angle_roll = kalman_output[0];
            kalman_uncertainty_angle_roll = kalman_output[1];
            kalman_filter(kalman_angle_pitch, kalman_uncertainty_angle_pitch, gyro_values.rate_pitch, angle_pitch);
            kalman_angle_pitch = kalman_output[0];
            kalman_uncertainty_angle_pitch = kalman_output[1];

            radio.getMessage(received_data);
            if((time_us_32() - timer_throttle) >= 100000) {
                input_throttle += received_data[1] - 50;
                if(input_throttle <= 0) {
                    input_throttle = 0;
                } else if(input_throttle >= 800) {
                    input_throttle = 800;
                }
                timer_throttle = time_us_32();
            }
            desired_rate_roll = (float)(received_data[2] - 20);
            desired_rate_pitch = -(float)(received_data[3] - 20);
            desired_rate_yaw = -(float)(received_data[0] - 20);

            //	PID controller.
            error_rate_roll = desired_rate_roll - kalman_angle_roll;
            pid_equation(error_rate_roll, Kp_roll, Ki_roll, Kd_roll, prev_error_rate_roll, prev_iterm_rate_roll, 200);
            input_roll = pid_return[0];
            prev_error_rate_roll = pid_return[1];
            prev_iterm_rate_roll = pid_return[2];
            error_rate_pitch = desired_rate_pitch - kalman_angle_pitch;
            pid_equation(error_rate_pitch, Kp_pitch, Ki_pitch, Kd_pitch, prev_error_rate_pitch, prev_iterm_rate_pitch, 250);
            input_pitch = pid_return[0];
            prev_error_rate_pitch = pid_return[1];
            prev_iterm_rate_pitch = pid_return[2];
            error_rate_yaw = desired_rate_yaw - gyro_values.rate_yaw;
            pid_equation(error_rate_yaw, Kp_yaw, Ki_yaw, Kd_yaw, prev_error_rate_yaw, prev_iterm_rate_yaw, 250);
            input_yaw = pid_return[0];
            prev_error_rate_yaw = pid_return[1];
            prev_iterm_rate_yaw = pid_return[2];
            
            drone_data.m_l = 1000 + input_throttle + input_roll;
            drone_data.m_r = 1000 + input_throttle - input_roll;
            drone_data.s_l = 1500 - input_pitch - input_yaw;
            drone_data.s_r = 1500 + input_pitch - input_yaw;
                        
            if (drone_data.m_l < 1050 || drone_data.m_r < 1050) {
                drone_data.m_l = 1000;
                drone_data.m_r = 1000;
                drone_data.s_l = 1500;
                drone_data.s_r = 1500;
                reset_pid();
            }

            bldc_l.set_power(drone_data.m_l);
            bldc_r.set_power(drone_data.m_r);
            if((time_us_32() - servo_timer) >= 20000) {
                servo_l.set_duty(drone_data.s_l);
                servo_r.set_duty(drone_data.s_r);
                servo_timer = time_us_32();
            }
        }

        while((time_us_32() - timer_loop) < 4000){
            // wait to finish the loop.
        }
        timer_loop = time_us_32();
    }

    return 0;
}





