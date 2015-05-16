#ifndef _RXTASK_H_
#define _RXTASK_H_

//channel limits:
#define RX_LOW_CH0       1084
#define RX_HIGH_CH0      1891

#define RX_LOW_CH1       1103
#define RX_HIGH_CH1      1910

#define RX_LOW_CH2       1084
#define RX_HIGH_CH2      1906

#define RX_LOW_CH3       1068
#define RX_HIGH_CH3      1875

#define RX_LOW_CH4       1000
#define RX_HIGH_CH4      2000

//Mapping values:
#define THRO_OUTPUT_MIN  125
#define THRO_OUTPUT_MAX  800

#define PITCH_INPUT_MIN  -45 //degrees (ref. imu_control_thread)
#define PITCH_INPUT_MAX   45
#define ROLL_INPUT_MIN   -45
#define ROLL_INPUT_MAX    45
#define YAW_INPUT_MIN   -160
#define YAW_INPUT_MAX    160

#define PITCH_ANGLE_MIN -250
#define PITCH_ANGLE_MAX  250
#define ROLL_ANGLE_MIN  -250
#define ROLL_ANGLE_MAX   250
#define YAW_ANGLE_MIN   -360
#define YAW_ANGLE_MAX    360

//Update flags
#define ROLL_FLAG 1
#define PITCH_FLAG 2
#define YAW_FLAG 4
#define THRO_FLAG 8
#define ARMED_FLAG 0x10
#define CALIB_FLAG 0x12

//Receiver channel pin assignement
#define PIN_PITCH_IN    p14
#define PIN_ROLL_IN     p15
#define PIN_YAW_IN      p16
#define PIN_THRO_IN     p17
#define CALIB_IN        p11
#define ARM_VEHICLE_IN  p12


#define RX_LOCK   0

//bools
int g_b_thro;
int g_b_roll;
int g_b_pitch;
int g_b_yaw;
int g_bArmed;
int g_b_calibr;

// Update flags
volatile uint8_t b_update_flags_shared;

//Interrupts
InterruptIn roll_in(PIN_ROLL_IN);
InterruptIn pitch_in(PIN_PITCH_IN);
InterruptIn yaw_in(PIN_YAW_IN);
InterruptIn thro_in(PIN_THRO_IN);
InterruptIn arm_in(ARM_VEHICLE_IN);
InterruptIn calib_in(CALIB_IN);

// shared variables (updated by the ISR and read by loop)
volatile uint16_t un_thro_in_shared;
volatile uint16_t un_roll_in_shared;
volatile uint16_t un_pitch_in_shared;
volatile uint16_t un_yaw_in_shared;
volatile uint16_t un_armed_in_shared;
volatile uint16_t un_calib_in_shared;

volatile uint16_t rx_raw_shared[4] = {0,0,0,0};
volatile int rx_correction_shared[4] = {85,0,0,0}; //thro, roll, yaw, pitch {85, 20, 0, -40}; 
volatile uint16_t rx_cmd_shared[4] = {0,0,0,0};
volatile uint16_t rx_armed;
volatile uint16_t rx_calib;

volatile float g_rx_channel[4] = {0,0,0,0};

//Recordings of the rising edge of a pulse in the ISR functions.
uint32_t g_ul_thro_start;
uint32_t g_ul_roll_start;
uint32_t g_ul_pitch_start;
uint32_t g_ul_yaw_start;
uint32_t g_ul_armed_start;
uint32_t g_ul_calibr_start;

Timer timer_rx;

void setup_telemetry()
{
    pc.printf("Waiting for RX");
    for(int i = 0; i < 6; i++) {
        wait(0.25);
        pc.printf(".");
    }
    pc.printf("\r\n");

}

//ISR: arm_ing switch (Gear Channel)
void arming_switch()
{
    if(g_bArmed == 0) {
        g_ul_armed_start = timer_rx.read_us();
        g_bArmed = 1;
    } else {
        un_armed_in_shared = (timer_rx.read_us() - g_ul_armed_start);
        b_update_flags_shared |= ARMED_FLAG;
        g_bArmed = 0;
        timer_rx.reset();
    }
}

//ISR: arm_ing switch (Gear Channel)
void calibration_switch()
{
    if(g_b_calibr == 0) {
        g_ul_calibr_start = timer_rx.read_us();
        g_b_calibr = 1;
    } else {
        un_calib_in_shared = (timer_rx.read_us() - g_ul_calibr_start);
        b_update_flags_shared |= CALIB_FLAG;
        g_b_calibr = 0;
        timer_rx.reset();
    }
}

//ISR: Throttle
void calcThro()
{
    //rising edge
    if(g_b_thro == 0) {
        g_ul_thro_start = timer_rx.read_us(); //Save the beginning of the time interval.
        g_b_thro = 1; //Next time will be falling edge (below).

    }
    //falling edge
    else {
        un_thro_in_shared = (timer_rx.read_us() - g_ul_thro_start); //Calculate interval.
        b_update_flags_shared |= THRO_FLAG; //Update flag (Flag is set when new information about throttle interval is available).
        g_b_thro = 0; //Cycle completed.
        timer_rx.reset(); //Reset timer.
    }
}

//ISR: Roll
void calcRoll()
{
    if(g_b_roll == 0) {
        g_ul_roll_start = timer_rx.read_us();
        g_b_roll = 1;
    } else {
        un_roll_in_shared = (timer_rx.read_us() - g_ul_roll_start);
        b_update_flags_shared |= ROLL_FLAG;
        g_b_roll = 0;
        timer_rx.reset();
    }
}

//ISR: Pitch
void calcPitch()
{
    if(g_b_pitch == 0) {
        g_ul_pitch_start = timer_rx.read_us();
        g_b_pitch = 1;
    } else {
        un_pitch_in_shared = (timer_rx.read_us() - g_ul_pitch_start);
        b_update_flags_shared |= PITCH_FLAG;
        g_b_pitch = 0;
        timer_rx.reset();
    }
}

//ISR: Yaw
void calcYaw()
{
    if(g_b_yaw == 0) {
        g_ul_yaw_start = timer_rx.read_us();
        g_b_yaw = 1;
    } else {
        un_yaw_in_shared = (timer_rx.read_us() - g_ul_yaw_start);
        b_update_flags_shared |= YAW_FLAG;
        g_b_yaw = 0;
        timer_rx.reset();
    }
}

void read_receiver()
{
    //Local copy of update flags
    static uint8_t bUpdateFlags;

    if(b_update_flags_shared) {
        //Turn interrupts off quickly while we take local copies of the shared variables.
        __disable_irq();

        //Take a local copy of which channels were updated.
        bUpdateFlags = b_update_flags_shared;

        if(bUpdateFlags & ARMED_FLAG)
            rx_armed = un_armed_in_shared;

        if(bUpdateFlags & CALIB_FLAG)
            rx_calib = un_calib_in_shared;

        if(bUpdateFlags & THRO_FLAG)
            rx_raw_shared[0] = un_thro_in_shared;

        if(bUpdateFlags & ROLL_FLAG)
            rx_raw_shared[1] = un_roll_in_shared;

        if(bUpdateFlags & YAW_FLAG)
            rx_raw_shared[2] = un_yaw_in_shared;

        if(bUpdateFlags & PITCH_FLAG)
            rx_raw_shared[3] = un_pitch_in_shared;


        __enable_irq();
        // clear shared copy of updated flags as we have already taken the updates
        b_update_flags_shared = 0;
    }
}

void telemetry_task(void const *argument)
{
    pc.printf("Receiver task\r\n");
    wait(0.25);

    timer_rx.start();

    //attach the interrupts used to read the channels
    roll_in.rise(&calcRoll);
    roll_in.fall(&calcRoll);

    pitch_in.rise(&calcPitch);
    pitch_in.fall(&calcPitch);

    yaw_in.rise(&calcYaw);
    yaw_in.fall(&calcYaw);

    thro_in.rise(&calcThro);
    thro_in.fall(&calcThro);

    arm_in.rise(&arming_switch);
    arm_in.fall(&arming_switch);

    calib_in.rise(&calibration_switch);
    calib_in.fall(&calibration_switch);

    while (true) {

        mtx_shared_rx_data.lock(); //lock RX data (counter part in pid_control_thread)
        read_receiver();

        if((rx_calib < 1200) && (rx_calib > 900)) {
            if(g_copter_calib == 1)
                pc.printf("Copter is in Calibration mode\r\n");
            g_copter_calib = 0;
            update_state();

        } else if((rx_calib > 1700) &&(rx_calib < 2100)) {
            if(g_copter_calib == 0)
                pc.printf("Copter is in Calibration mode\r\n");
            g_copter_calib = 1;
            update_state();

        } else {
            //pc.printf("rx_calib[%d]\r\n", rx_calib);
        }

        if((rx_armed < 1200) && (rx_armed > 900)) {
            if(g_copter_armed == 1)
                pc.printf("Copter is now UNARMED\r\n");
            g_copter_armed = 0;
            update_state();

        } else if((rx_armed > 1700) &&(rx_armed < 2100)) {
            if(g_copter_armed == 0)
                pc.printf("Copter is now ARMED\r\n");
            g_copter_armed = 1;
            update_state();
        } else {
           // pc.printf("Arming-switch out of region [%d]\r\n", rx_armed);
        }

        for(int i = 0; i < 4; i++)
            if(rx_raw_shared[i] > 2100 ) { //connection lost
                rx_cmd_shared[0] = 1000;
            } else {

                if(g_copter_armed) {
                    rx_cmd_shared[0] =  rx_raw_shared[0] - rx_correction_shared[0];
                    rx_cmd_shared[1] =  rx_raw_shared[1] - rx_correction_shared[1];
                    rx_cmd_shared[2] =  rx_raw_shared[2] - rx_correction_shared[2];
                    rx_cmd_shared[3] =  rx_raw_shared[3] - rx_correction_shared[3];

                    /* map rx channels */
                    g_rx_channel[0] = map(rx_cmd_shared[0], RX_LOW_CH0, RX_HIGH_CH0, THRO_OUTPUT_MIN, THRO_OUTPUT_MAX);
                    g_rx_channel[1] = map(rx_cmd_shared[1], RX_LOW_CH1, RX_HIGH_CH1, ROLL_INPUT_MIN, ROLL_INPUT_MAX);
                    g_rx_channel[2] = map(rx_cmd_shared[2], RX_LOW_CH2, RX_HIGH_CH2, YAW_INPUT_MIN,  YAW_INPUT_MAX);
                    g_rx_channel[3] = map(rx_cmd_shared[3], RX_LOW_CH3, RX_HIGH_CH3, PITCH_INPUT_MIN, PITCH_INPUT_MAX);
                }

            }
            
        mtx_shared_rx_data.unlock(); //unlock RX data (counter part in pid_control_thread)

        Thread::wait(RX_TASK);
    }
}

#endif