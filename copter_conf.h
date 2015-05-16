#ifndef _KOPTERI_KONF_H_
#define _KOPTERI_KONF_H_

#include "mbed.h"
#include "rtos.h"

Serial pc(USBTX, USBRX);

Mutex mtx_shared_rx_data;
Mutex mtx_shared_imu_data;
Mutex mtx_shared_esc_data;

#define ARMED       0x01
#define INFLIGHT    0x02

#define PITCH_RATE_MIN -500
#define PITCH_RATE_MAX  500
#define ROLL_RATE_MIN -500
#define ROLL_RATE_MAX  500
#define YAW_RATE_MIN -300
#define YAW_RATE_MAX  300

//IMU adjustments:
int16_t gyro_adjust[3] = {18.5, 0.5, -7};
float yrp_adjust[3] = {0.119, 0.028, 0.017};

//RX adjustments:
int rx_adjust[4] = {0,0,0,0};

//PID tuning parameters:
//tuning parameters
volatile float pid_multi = 1;
volatile int tuning_value = 0;
volatile int tuning_pid_ar = 0;
volatile float tuning_values[3] = {0,0,0}; //pid
volatile int tuning_values_rx[3] = {0,0,0}; //rx(roll yaw pitch)

//PID parameters
volatile float roll_pid_angle[3] = {0, 0.1, 0.2};
volatile float roll_pid_gyro_rate[3] = {0.3, 0.4, 0.5};

volatile float pitch_pid_angle[3] = {0.6, 0.7, 0.8}; //i 2.9 , 1.4
volatile float pitch_pid_gyro_rate[3] = {0.9, 1.0, 1.1};

volatile float yaw_pid_angle[3] = {1.2,1.3,1.4}; //not in use
volatile float yaw_pid_gyro_rate[3] = {1.5,1.6, 1.7};

//PID biases:
#define PITCH_BIAS    0
#define ROLL_BIAS     0
#define YAW_BIAS      0

//Thread wait times:
#define PIDCTRL_TASK    2 //    500Hz
#define IMU_TASK        4 //    250Hz
#define RX_TASK         20//    50Hz
#define DRIVE_TASK      5//     200Hz

//PID control delta time:
#define PIDCTRL_dT      0.002// 500 Hz



#endif
/*

volatile float roll_pid_angle[3] = {1, 0, 0};
volatile float roll_pid_gyro_rate[3] = {1, 0, 0};

volatile float pitch_pid_angle[3] = {1, 0, 0}; //i 2.9 , 1.4
volatile float pitch_pid_gyro_rate[3] = {1, 0, 0};

volatile float yaw_pid_angle[3] = {0,0,0}; //not in use
volatile float yaw_pid_gyro_rate[3] = {2, 3, 0};


*/