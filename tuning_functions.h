#ifndef _TUNING_H_
#define _TUNING_H_

#include "pid_control_thread.h"
#include "copter_conf.h"

void tunings_pc()
{

    if(pc.readable()) {
        // 1 = roll, 2 = pitch, 3 = yaw, 4 = imu out, 0 = rx calibration values
        switch (pc.getc()) {

            case '1':
                tuning_value = 1;
                pid_multi = 1;
                if(tuning_pid_ar == 0) {
                    tuning_values[0] = roll_pid_angle[0];
                    tuning_values[1] = roll_pid_angle[1];
                    tuning_values[2] = roll_pid_angle[2];
                } else {
                    tuning_values[0] = roll_pid_gyro_rate[0];
                    tuning_values[1] = roll_pid_gyro_rate[1];
                    tuning_values[2] = roll_pid_gyro_rate[2];

                }
                break;

            case '9':
                for(int i = 0; i < 3; i++)
                    tuning_values[i] = roll_pid_gyro_rate[i];
                tuning_pid_ar = 1;
                break;
                
            case '8':
                for(int i = 0; i < 3; i++)
                    tuning_values[i] = roll_pid_angle[i];
                tuning_pid_ar = 0;
                break;               
/*
            case '0':
                tuning_value = 0;
                tuning_values_rx[0] = rx_correction_shared[1]; //roll  [p,l]
                tuning_values_rx[1] = rx_correction_shared[2]; //yaw   [i,k]
                tuning_values_rx[2] = rx_correction_shared[3]; //pitch [d,c]
                pid_multi = 1000;
                break;
     */           
            //pid multiplier
            case 'm':
                pid_multi *=10;
                break;
            case 'n':
                pid_multi*=0.1;
                break;
            case 'b':
                pid_multi*=1;
                break;

            case 'i':
                if( tuning_value == 0)
                    tuning_values_rx[1] +=1; //yaw
                else {
                    tuning_values[1] += (0.001*pid_multi); //i
                }
                break;
                
            case 'k':
                if( tuning_value == 0)
                    tuning_values_rx[1] -=1; //yaw
                else {
                    tuning_values[1] -= (0.001*pid_multi); //k
                }
                break;
                
            case 'p':
                if( tuning_value == 0)
                    tuning_values_rx[0] +=1; //roll
                else {
                    tuning_values[0] += (0.001*pid_multi); //p
                }
                break;
                
            case 'l':
                if( tuning_value == 0)
                    tuning_values_rx[0] -=1; //roll
                else {
                    tuning_values[0] -= (0.001*pid_multi); //p
                }
                break;
                
            case 'd':
                if( tuning_value == 0)
                    tuning_values_rx[2] +=1; //pitch
                else {
                    tuning_values[2] += (0.00001*pid_multi); //d
                }
                break;
                
            case 'c':

                if( tuning_value == 0)
                    tuning_values_rx[2] -=1; //pitch
                else {
                    tuning_values[2] -= (0.00001*pid_multi); //d
                }
                break;
        }
    }

}

#endif