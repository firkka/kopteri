#ifndef _INITIALIZE_THREADS_H_
#define _INITIALIZE_THREADS_H_



void create_threads() {

    /* Create threads */
    pc.printf("Creating threads:\r\n");
    Thread thread1(motion_control, NULL, osPriorityHigh, DEFAULT_STACK_SIZE, NULL);
    //Thread thread2(telemetry_task, NULL, osPriorityLow, DEFAULT_STACK_SIZE, NULL);
    //Thread thread3(pid_control, NULL, osPriorityHigh, DEFAULT_STACK_SIZE, NULL);
    //Thread thread4(ESC_control, NULL, osPriorityNormal, DEFAULT_STACK_SIZE, NULL);
    
    //thread langattomaan viestintään:
    Thread thread5(wless_task, NULL, osPriorityHigh, DEFAULT_STACK_SIZE, NULL);

    Thread::wait(osWaitForever);
    
}
#endif