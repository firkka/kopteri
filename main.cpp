#include "mbed.h"
#include "rtos.h"

#include <math.h>

#include "helpers.h"
#include "vehicle_state.h"
#include "copter_conf.h"
#include "telemetry_thread.h"
#include "imu_control_thread.h"
#include "pid_control_thread.h"
#include "drive_control_thread.h"
#include "wireless_thread.h"
#include "init_threads.h"

extern "C" void mbed_reset();

int main()
{
  //  print_propeller();
    pc.printf("\r\nStarting copter initialization:\r\n");
    setup_telemetry();
    setup_mpu();
    setup_ESCs();
    create_threads();

    while (true){
        Thread::wait(50);
    }
        
}