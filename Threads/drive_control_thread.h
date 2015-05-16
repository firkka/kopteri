#ifndef _ESCCONTROL_H_
#define _ESCCONTROL_H_

#include "pid_control_thread.h"


PwmOut ESC4(p24); //rear left
PwmOut ESC2(p21); //rear right

PwmOut ESC3(p23); //front right
PwmOut ESC1(p22); //front left


/* ESC initialization */
void setup_ESCs()
{
    pc.printf("\r\nInitializing BEC's");
    
    //Command frequency: 50Hz, pulsewidth to 1ms (0rpm)
    ESC1.period_ms(20);
    ESC2.period_ms(20);
    ESC1.pulsewidth_us(1000);
    ESC2.pulsewidth_us(1000);
    ESC3.period_ms(20);
    ESC4.period_ms(20);
    ESC3.pulsewidth_us(1000);
    ESC4.pulsewidth_us(1000);

    for(int i = 0 ; i < 6 ; i++) { //print some dots
        wait(0.25);
        pc.printf(".");
    }
    pc.printf("\r\n");
    g_copter_armed = 0; // set copter unarmed
    update_state(); //update copter state

}

void update_ESCs()
{
    mtx_shared_esc_data.lock();
    if(g_copter_state_flag & ARMED) { //if armed
        __disable_irq();

        //FRONT-LEFT ESC COMMAND
        if( g_drive_command[0] < 150 ) { //give minimum rpm cmd to ESC, even if setpoint goes below minimum.
            ESC1.pulsewidth_us(1150);
        } else {
            ESC1.pulsewidth_us(1000 + g_drive_command[0]);
        }
        //REAR-RIGHT ESC COMMAND
        if( g_drive_command[1] < 150 ) { //give minimum rpm cmd to ESC, even if setpoint goes below minimum.
            ESC2.pulsewidth_us(1150);
        } else {
            ESC2.pulsewidth_us(1000 + g_drive_command[1]);
        }

        //FRONT-RIGHT ESC COMMAND
        if( g_drive_command[2] < 150 ) { //give minimum rpm cmd to ESC, even if setpoint goes below minimum.
            ESC3.pulsewidth_us(1150);
        } else {
            ESC3.pulsewidth_us(1000 + g_drive_command[2]);
        }

        //REAR-LEFT ESC COMMAND
        if(g_drive_command[3] < 150) {  //keep minimum rpm on ESC.
            ESC4.pulsewidth_us(1150);
        } else {
            ESC4.pulsewidth_us(1000 + g_drive_command[3]);
        }

        __enable_irq();

    } else { //set all motor commands to 0 (0rpm)
        ESC1.pulsewidth_us(1000);
        ESC2.pulsewidth_us(1000);
        ESC3.pulsewidth_us(1000);
        ESC4.pulsewidth_us(1000);

        // and reset PID
        roll_ctrl_angle.reset();
        pitch_ctrl_angle.reset();
        //yaw_ctrl_angle.reset();
        roll_ctrl_gyro_rate.reset();
        pitch_ctrl_gyro_rate.reset();
        yaw_ctrl_gyro_rate.reset();
    }

    if(g_copter_calib) {
        
        yrp_adj[0] = (-1)*yrp[0]; //reset YAW
        
        // reset all PID
        roll_ctrl_angle.reset();
        pitch_ctrl_angle.reset();
        //yaw_ctrl_angle.reset();
        roll_ctrl_gyro_rate.reset();
        pitch_ctrl_gyro_rate.reset();
        yaw_ctrl_gyro_rate.reset();
    }
    mtx_shared_esc_data.unlock(); //unlock 
}

void ESC_control(void const *argument)
{
    pc.printf("Prop drivers control\r\n");
    wait(0.25);
    while (true) {
        
        update_ESCs();
        Thread::wait(DRIVE_TASK);
    }

}
#endif