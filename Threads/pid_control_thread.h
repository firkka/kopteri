#ifndef _PIDCONTROL_H_
#define _PIDCONTROL_H_

#include "PID.h"
#include "imu_control_thread.h"
#include "tuning_functions.h"
//#include "QEI.h"

volatile float g_throttle = 0;
volatile float g_drive_command[4] = {0,0,0,0}; //FRONT, REAR, RIGHT, LEFT

volatile float g_angle_roll;
volatile float g_angle_pitch;
volatile float g_angle_yaw;

volatile float g_roll_output;
volatile float g_pitch_output;
volatile float g_yaw_output;
volatile float yaw_target = 0;

volatile int print_counter = 0;

PID roll_ctrl_angle(roll_pid_angle[0], roll_pid_angle[1], roll_pid_angle[2], PIDCTRL_dT);
PID pitch_ctrl_angle(pitch_pid_angle[0], pitch_pid_angle[1], pitch_pid_angle[2], PIDCTRL_dT);
//PID yaw_ctrl_angle(0, 0, 0, PIDCTRL_dT);

PID roll_ctrl_gyro_rate(roll_pid_gyro_rate[0], roll_pid_gyro_rate[1], roll_pid_gyro_rate[2], PIDCTRL_dT);
PID pitch_ctrl_gyro_rate(pitch_pid_gyro_rate[0], pitch_pid_gyro_rate[1], pitch_pid_gyro_rate[2], PIDCTRL_dT);
PID yaw_ctrl_gyro_rate( yaw_pid_gyro_rate[0], yaw_pid_gyro_rate[1], yaw_pid_gyro_rate[2], PIDCTRL_dT);

void setup_pid_controllers(void)
{
    //Angle PID
    roll_ctrl_angle.setInputLimits(ROLL_ANGLE_MIN, ROLL_ANGLE_MAX);
    roll_ctrl_angle.setOutputLimits(ROLL_ANGLE_MIN, ROLL_ANGLE_MAX);
    roll_ctrl_angle.setBias(ROLL_BIAS);
    roll_ctrl_angle.setMode(AUTO_MODE);

    pitch_ctrl_angle.setInputLimits(PITCH_ANGLE_MIN, PITCH_ANGLE_MAX);
    pitch_ctrl_angle.setOutputLimits(PITCH_ANGLE_MIN, PITCH_ANGLE_MAX);
    pitch_ctrl_angle.setBias(PITCH_BIAS);
    pitch_ctrl_angle.setMode(AUTO_MODE);
    /*
        yaw_ctrl_angle.setInputLimits(YAW_ANGLE_MIN, YAW_ANGLE_MAX);
        yaw_ctrl_angle.setOutputLimits(YAW_ANGLE_MIN, YAW_ANGLE_MAX);
        yaw_ctrl_angle.setBias(YAW_BIAS);
        yaw_ctrl_angle.setMode(AUTO_MODE);
    */
    //Rate PID
    roll_ctrl_gyro_rate.setInputLimits(ROLL_RATE_MIN, ROLL_RATE_MAX);
    roll_ctrl_gyro_rate.setOutputLimits(ROLL_RATE_MIN, ROLL_RATE_MAX);
    roll_ctrl_gyro_rate.setBias(ROLL_BIAS);
    roll_ctrl_gyro_rate.setMode(AUTO_MODE);

    pitch_ctrl_gyro_rate.setInputLimits(PITCH_RATE_MIN, PITCH_RATE_MAX);
    pitch_ctrl_gyro_rate.setOutputLimits(PITCH_RATE_MIN, PITCH_RATE_MAX);
    pitch_ctrl_gyro_rate.setBias(PITCH_BIAS);
    pitch_ctrl_gyro_rate.setMode(AUTO_MODE);

    yaw_ctrl_gyro_rate.setInputLimits(YAW_RATE_MIN, YAW_RATE_MAX);
    yaw_ctrl_gyro_rate.setOutputLimits(YAW_RATE_MIN, YAW_RATE_MAX);
    yaw_ctrl_gyro_rate.setBias(YAW_BIAS);
    yaw_ctrl_gyro_rate.setMode(AUTO_MODE);
}

void compute_PID()
{
    mtx_shared_rx_data.lock(); //lock RX data (counter part in telemetry thread)

    g_throttle = g_rx_channel[0];
    yaw_target = 0;

    //yaw_ctrl_angle.setSetPoint(yaw_target);
    //yaw_ctrl_angle.setProcessValue(map_yaw(yrp[0]));
    //g_angle_yaw = yaw_ctrl_angle.compute();

    //lock sensor data (counter part in imu_control_thread)
    mtx_shared_imu_data.lock();

    pitch_ctrl_angle.setSetPoint(g_rx_channel[3]);
    pitch_ctrl_angle.setProcessValue(yrp[2]);
    g_angle_pitch = pitch_ctrl_angle.compute();

    roll_ctrl_angle.setSetPoint(g_rx_channel[1]);
    roll_ctrl_angle.setProcessValue(yrp[1]);
    g_angle_roll = roll_ctrl_angle.compute();

    if( abs(g_rx_channel[2]) > 10) { //filter yaw jitter.
        yaw_target = g_rx_channel[2];
    }

    mtx_shared_rx_data.unlock(); //unlock RX data (counter part in telemetry thread)

    roll_ctrl_gyro_rate.setSetPoint(g_angle_roll);
    pitch_ctrl_gyro_rate.setSetPoint(g_angle_pitch);
    yaw_ctrl_gyro_rate.setSetPoint(yaw_target);//

    roll_ctrl_gyro_rate.setProcessValue(gyro[1]); //gy
    pitch_ctrl_gyro_rate.setProcessValue(gyro[0]); //gx
    yaw_ctrl_gyro_rate.setProcessValue(gyro[2]); //gz

    yaw_target = yrp[0]; //update yaw target
    
    mtx_shared_imu_data.unlock(); //unlock sensor data (counter part in imu_control_thread)

    /* compute axle outputs*/
    g_roll_output = roll_ctrl_gyro_rate.compute();
    g_pitch_output = pitch_ctrl_gyro_rate.compute();
    g_yaw_output = yaw_ctrl_gyro_rate.compute();
}

void calculate_motor_cmds()
{
    mtx_shared_esc_data.lock(); // lock mtx (g_drive_command)
    //X configuration
    g_drive_command[0] = g_throttle + g_roll_output + g_pitch_output + g_yaw_output; //front-left
    g_drive_command[3] = g_throttle + g_roll_output - g_pitch_output - g_yaw_output; //rear-left
    g_drive_command[2] = g_throttle - g_roll_output + g_pitch_output - g_yaw_output; //front-right
    g_drive_command[1] = g_throttle - g_roll_output - g_pitch_output + g_yaw_output; //rear-right
    mtx_shared_esc_data.unlock();
}

/*some PID tuning stuff, not relevant*/
void set_PID_tunings()
{
    if(tuning_pid_ar == 0) {
        if(tuning_value == 1) {
            pitch_ctrl_angle.setTunings(tuning_values[0], tuning_values[1], tuning_values[2]);
            roll_ctrl_angle.setTunings(tuning_values[0], tuning_values[1], tuning_values[2]);
            roll_pid_angle[0] = tuning_values[0]; //i
            roll_pid_angle[1] = tuning_values[1]; //d
            roll_pid_angle[2] = tuning_values[2]; //p
            pc.printf("[a]: ");
        }

    } else {
        if(tuning_value == 1) {
            pitch_ctrl_gyro_rate.setTunings(tuning_values[0], tuning_values[1], tuning_values[2]);
            roll_ctrl_gyro_rate.setTunings(tuning_values[0], tuning_values[1], tuning_values[2]);
            roll_pid_gyro_rate[0] = tuning_values[0]; //i
            roll_pid_gyro_rate[1] = tuning_values[1]; //d
            roll_pid_gyro_rate[2] = tuning_values[2]; //p
            pc.printf("[r]: ");
        }

    }
    pc.printf("p:%0.4f[p,l] i:%0.4f[i,k] d:%0.6f[d,c] multi:%0.3f \r\n", tuning_values[0], tuning_values[1], tuning_values[2], pid_multi);

}

void pid_control(void const *argument)
{
    pc.printf("PID control\r\n");
    wait(0.25);
    setup_pid_controllers();

    while (true) {

        compute_PID();
        
        //tuning stuff:
        if(((print_counter % 20) == 0)) {
            tunings_pc();
            if(tuning_value > 0 && tuning_value < 4)
                set_PID_tunings();
            print_counter = 0;
        }
        print_counter++;

        calculate_motor_cmds();
        Thread::wait(PIDCTRL_TASK);
    }
}
#endif