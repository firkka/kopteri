#ifndef _MOTIONCONTROL_H_
#define _MOTIONCONTROL_H_


// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//     2012-06-20 - improved FIFO overflow handling and simplified read process
//     2012-06-19 - completely rearranged DMP initialization code and simplification
//     2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//     2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//     2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                - add 3D math helper file to DMP6 example sketch
//                - add Euler output and Yaw/Pitch/Roll output formats
//     2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//     2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//     2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

const float M_PI = 3.14159265;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

//jukan gyrot:

int16_t gx, gy, gz;
int16_t gyro[3] = {0, 0, 0};

float euler[3];         // [psi, theta, phi]    Euler angle container

float yrp[3];         // [yaw, roll, pitch]   yaw/pitch/roll container and gravity vector

//calibration adjustment:
float yrp_adj[3] = {0,0,0};

DigitalOut led1(LED1);
InterruptIn checkpin(p29);

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup();
void loop();

// ================================================================
// ===                     MPU INITIAL SETUP                    ===
// ================================================================

void setup_mpu()
{

    // initialize device
    pc.printf("Initializing I2C devices...\r\n");
    mpu.initialize();
    wait(0.25);
    // verify connection
    pc.printf("Testing device connections...\r\n");
    if (mpu.testConnection()) pc.printf("MPU6050 connection successful\r\n");
    else pc.printf("MPU6050 connection failed\r\n");
    // load and configure the DMP
    pc.printf("Initializing DMP...\r\n");
    devStatus = mpu.dmpInitialize();
    wait(0.5);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        pc.printf("Enabling DMP...\r\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        pc.printf("Enabling interrupt detection (Arduino external interrupt 0)...\r\n");
        checkpin.rise(&dmpDataReady);

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        pc.printf("DMP ready!\r\n");
        dmpReady = true;

        int8_t xgOffsetTC = mpu.getXGyroOffset();
        int8_t ygOffsetTC = mpu.getYGyroOffset();
        int8_t zgOffsetTC = mpu.getZGyroOffset();

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)

        pc.printf("DDMP Initialization failed (code ");
        pc.printf("%d", devStatus);
        pc.printf(")\r\n");
    }

}

// ================================================================
// ===                    MOTION CONTROL LOOP                   ===
// ================================================================

void motion_control(void const *argument)
{
    pc.printf("motion control\r\n");
    wait(0.25);
    while(true) {

        // if programming failed, don't try to do anything
        if (!dmpReady) return;

        // wait for MPU interrupt or extra packet(s) available
        while (!mpuInterrupt && fifoCount < packetSize) {}

        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();

        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();

            // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);

            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;


            //lock sensor data (counter part in pid_control_thread)
            mtx_shared_imu_data.lock();
            //Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);

            //Get imu sample:
            yrp[0] += yrp_adjust[0];
            yrp[1] += yrp_adjust[1];
            yrp[2] += yrp_adjust[2];

            mpu.getRotation(&gx, &gy, &gz);

            gyro[0] = gx + gyro_adjust[0];
            gyro[1] = gy + gyro_adjust[1];
            gyro[2] = gz + gyro_adjust[2];

            //pc.printf("y:%.3lf \tr:%.3lf \tp:%.3lf\t", yrp[0], yrp[1], yrp[2]);
            //pc.printf("\tgx:%d \tgy:%d \tgz:%d \r\n", gyro[0], gyro[1], gyro[2]);

            yrp[0] = yrp[0] * 180/M_PI;
            yrp[1] = (-1)*yrp[1] * 180/M_PI;
            yrp[2] = yrp[2] * 180/M_PI;

            for (int i = 0; i < 3; i++)
                gyro[i] /= 32.8;
            
            //unlock sensor data (counter part in pid_control_thread)
            mtx_shared_imu_data.unlock();

            Thread::wait(IMU_TASK);
        }
    }
}
#endif