#ifndef _VSTATE_H_
#define _VSTATE_H_

#include "copter_conf.h"

volatile uint8_t g_copter_state_flag = 0x00;
volatile uint8_t g_copter_armed = 0;
volatile uint8_t g_copter_calib = 0;


void update_state()
{
    if(g_copter_calib) {
        g_copter_state_flag |= 0x02;
        g_copter_state_flag &= 0x02;
        pc.printf("Copter is in Calibration MODE and unarmed\r\n");
    } 
    else {
        g_copter_state_flag &= 0xFD;
    }

    if(g_copter_armed)
        g_copter_state_flag |= 0x01;
    else {
        g_copter_state_flag &= 0xFE;
    }
}

#endif