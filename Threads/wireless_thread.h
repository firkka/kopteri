#ifndef _WLESSCONTROL_H_
#define _WLESSCONTROL_H_
/*
Wireless connection

    Sends sensor data and PID parameters wirelessly to ground mbed.

    By request, sends PID or SEN parameters to ground mbed.
    Receives new parameter set for PID parameters and updates copter_conf PID parameters.

*/

#include "RFM12B/RFM12B.h"

#define W_TASK 100 //10Hz

RFM12B rfm12b(p5, p6, p7, p10, p9, LED1); //SDI, SDO, SCK, SEL, IRQ, IRQ_LED);


uint8_t KEY[] = "ABCDABCDABCDABCD";

#define ACK_TIME    1300
#define TOUT_TIME   1000


#define NODE1       10 //Copter
#define NODE2       11 //Ground

#define NETWORD_ID  5

//float to bytes
typedef union _data1 {
    float f;
    char  s[4];
} var_fb;

typedef union _data2 {
    uint16_t ui;
    char  s[2];
} var_ib;

bool requestACK = true;
Timer ackTimer;
Timer wait_timeout;

void initialize_modem()
{
    rfm12b.Initialize(NODE1, RF12_868MHZ, NETWORD_ID); //id = 2, band 866, group 5
    rfm12b.SetEncryptionKey(NULL); //(uint8_t*) KEY
    rfm12b.ReceiveStart();

}

void pAc_current_values(var_fb *param_set); // print and copy current values to

void wless_task(void const *argument)
{
    pc.printf("wireless task\r\n");
    initialize_modem();

    while(true) {

        if (rfm12b.ReceiveComplete()) {
            if (rfm12b.CRC_Pass()) {
                pc.printf("MBED - received a message ");

                int msg_len = rfm12b.GetDataLen();

                //PID request:
                if( msg_len == 5 ) {
                    pc.printf("(PID Request)\r\n");
                    var_fb param_set[18];
                    if (rfm12b.ACKRequested())
                        rfm12b.SendACK();

                    //!!MUTEX
                    for(int i = 0; i < 3; i++)
                        param_set[i].f = roll_pid_angle[i];

                    for(int i = 0; i < 3; i++)
                        param_set[6 + i].f = pitch_pid_angle[i];

                    for(int i = 0; i < 3; i++)
                        param_set[12 + i].f = yaw_pid_angle[i];

                    for(int i = 0; i < 3; i++)
                        param_set[3 + i].f = roll_pid_gyro_rate[i];

                    for(int i = 0; i < 3; i++)
                        param_set[9 + i].f = pitch_pid_gyro_rate[i];

                    for(int i = 0; i < 3; i++)
                        param_set[15 + i].f = yaw_pid_gyro_rate[i];

                    wait(0.1);

                    /* /w Timeout: */
                    int can_send = 1;
                    wait_timeout.start();
                    //rfm12b.ReceiveStart();

                    while(rfm12b.CanSend()) {
                        if(wait_timeout.read_ms() > TOUT_TIME) {
                            can_send = 0;
                            break; // break in timeout and set send flag to 0, otherwise continue to sending phase.
                        }
                    }
                    wait_timeout.stop();
                    wait_timeout.reset();

                    if(can_send) {
                        rfm12b.SendStart(NODE2, param_set, 18*sizeof(float), requestACK);

                        //odota ACK
                        if (requestACK) {
                            ackTimer.start();
                            while (ackTimer.read_ms() <= ACK_TIME)
                                if (rfm12b.ACKReceived(NODE2) == true)
                                    break;

                            ackTimer.stop();
                            ackTimer.reset();
                            rfm12b.ReceiveStart();
                        }
                    } else {
                        pc.printf("can send failed [pid]\r\n");
                    }

                } //message len check (req + parameter set sending)


                //Sensor Values Requested:
                else if( msg_len == 4 ) {
                    pc.printf("(SEN Request)\r\n");

                    if (rfm12b.ACKRequested())
                        rfm12b.SendACK();

                    var_fb ps_sen_angle[3];
                    int len_ps_sen_angle = 3*sizeof(float);

                    var_ib ps_sen_rate[3];
                    int len_ps_sen_rate = 3*sizeof(uint16_t);

                    mtx_shared_imu_data.lock();
                    for(int i = 0; i < 3; i++) {
                        ps_sen_angle[i].f = yrp[i];
                    }
                    //ps_sen_angle[i].f = yrp[i];

                    for(int i = 0; i < 3; i++) {
                        ps_sen_rate[i].ui = i;
                    }
                    //ps_sen_rate[i].ui = gyro[i];
                    mtx_shared_imu_data.unlock();

                    int *combined_data = (int*) malloc(3 * sizeof(float) + 3 * sizeof(uint16_t));

                    memcpy(combined_data, ps_sen_angle, len_ps_sen_angle);
                    memcpy(combined_data + len_ps_sen_angle, ps_sen_rate, len_ps_sen_rate);

                    wait(0.1); //2
                    //while(rfm12b.CanSend());
                    int can_send = 1;
                    wait_timeout.start();
                    //rfm12b.ReceiveStart();

                    while(rfm12b.CanSend()) {
                        if(wait_timeout.read_ms() > TOUT_TIME) {
                            can_send = 0;
                            break; // break in timeout and set send flag to 0, otherwise continue to sending phase.
                        }
                    }
                    wait_timeout.stop();
                    wait_timeout.reset();

                    if(can_send) {
                        rfm12b.SendStart(NODE2, combined_data, (3 * sizeof(float)) + (3 * sizeof(uint16_t)), requestACK);
                        //odota ACK
                        if (requestACK) {
                            ackTimer.start();
                            while (ackTimer.read_ms() <= ACK_TIME)
                                if (rfm12b.ACKReceived(NODE2) == true)
                                    break;

                            ackTimer.stop();
                            ackTimer.reset();
                            rfm12b.ReceiveStart();
                        }
                    } else {
                        pc.printf("can send failed [sen]\r\n");
                    }
                    //free(combined_data);
                }

                //If receiving message is 72 bytes, it is the new parameter set.
                else if( msg_len == (18 * sizeof(float)) ) {
                    pc.printf("(new parameter set [%d])\r\n", msg_len);
                    if (rfm12b.ACKRequested())
                        rfm12b.SendACK();

                    var_fb param_set[18];

                    //Copy new PID values to table param_set:
                    for(int f = 0; f < 18; f++)
                        for(int i = 0; i < 4; i++)
                            param_set[f].s[i] = rfm12b.GetData()[4*f + i];

                    //MUTEX!
                    for(int i = 0; i < 3; i++)
                        roll_pid_angle[i] = param_set[i].f;

                    for(int i = 0; i < 3; i++)
                        roll_pid_gyro_rate[i] = param_set[3 + i].f;

                    for(int i = 0; i < 3; i++)
                        pitch_pid_angle[i] =  param_set[6 + i].f;

                    for(int i = 0; i < 3; i++)
                        pitch_pid_gyro_rate[i] = param_set[9 + i].f;

                    for(int i = 0; i < 3; i++)
                        yaw_pid_angle[i] = param_set[12 + i].f;

                    for(int i = 0; i < 3; i++)
                        yaw_pid_gyro_rate[i] = param_set[15 + i].f;
                    //!MUTEX!
                } else {
                    pc.printf("mslen: %d", msg_len);
                }
            } //CRC pass
        } //message complete
        //rfm12b.ReceiveStart();
        Thread::wait(W_TASK);
    } // while true
}

#endif