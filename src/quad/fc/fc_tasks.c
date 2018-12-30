
#include "fc_tasks.h"
#include "fc_core.h"
#include "fc_rc.h"
#include "scheduler.h"
#include "fc_core.h"
#include "common.h"
#include "rx.h"
#include "led.h"

//#define TASKS_LEDS_TESTING

#define TASK_PERIOD_HZ(hz)              (1000000 / (hz))            // units in microseconds (us)
#define TASK_PERIOD_MS(ms)              ((ms) * 1000)
#define TASK_PERIOD_US(us)              (us)

void taskUpdateRxMain(timeUs_t currentTimeUs);

/* Tasks initialisation */
cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
//        .desiredPeriod = TASK_PERIOD_HZ(0.05),            // 1000000 / 0.05 = 20000000 us = 20000 ms = 20 s
//        .desiredPeriod = TASK_PERIOD_HZ(0.5),            // 1000000 / 0.5 = 2000000 us = 2000 ms = 2 s
//        .desiredPeriod = TASK_PERIOD_HZ(1),            // 1000000 / 1 = 1000000 us = 1000 ms = 1 s
        .desiredPeriod = TASK_PERIOD_HZ(10),            // 1000000 / 10 = 100000 us = 100 ms
        .staticPriority = TASK_PRIORITY_MEDIUM_HIGH,
    },
    
    [TASK_GYROPID] = {
        .taskName = "PID",
        .subTaskName = "GYRO",
        .taskFunc = taskMainPidLoop,
//        .desiredPeriod = TASK_PERIOD_HZ(50),                // 1000000 / 50 = 20000 us = 20 ms
        .desiredPeriod = TASK_GYROPID_DESIRED_PERIOD,       // desiredPeriod = TASK_GYROPID_DESIRED_PERIOD = 125 us using STM32F4
        .staticPriority = TASK_PRIORITY_REALTIME,           // TASK_PRIORITY_REALTIME = 6
    },

#if 1
    [TASK_RX] = {
        .taskName = "RX",
        .checkFunc = rxUpdateCheck,
        .taskFunc = taskUpdateRxMain,
//        .desiredPeriod = TASK_PERIOD_HZ(1),            // 1000000 / 1 = 1000000 us = 1 s
        .desiredPeriod = TASK_PERIOD_HZ(50),            // 1000000 / 50 = 20000 us = 20 ms
        .staticPriority = TASK_PRIORITY_HIGH,
    },
#endif
    
#ifdef TASKS_LEDS_TESTING
    [TASK_LED3] = {
        .taskName = "LED3",
        .taskFunc = taskLed3,
        .desiredPeriod = TASK_PERIOD_HZ(2),             // 1000000 / 2 (hz) = 500000 us = 500 ms
//        .desiredPeriod = TASK_PERIOD_HZ(5),             // 1000000 / 5 (hz) = 200000 us = 200 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_LED4] = {
        .taskName = "LED4",
        .taskFunc = taskLed4,
        .desiredPeriod = TASK_PERIOD_HZ(1),             // 1000000 / 1 (hz) = 1000000 us = 1000 ms
//        .desiredPeriod = TASK_PERIOD_HZ(2.5),             // 1000000 / 2.5 (hz) = 400000 us = 400 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_LED5] = {
        .taskName = "LED5",
        .taskFunc = taskLed5,
        .desiredPeriod = TASK_PERIOD_HZ(0.67),             // 1000000 / 0.67 (hz) = 1500000 us = 1500 ms
//        .desiredPeriod = TASK_PERIOD_HZ(1.67),             // 1000000 / 1.67 (hz) = 600000 us = 600 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_LED6] = {
        .taskName = "LED6",
        .taskFunc = taskLed6,
        .desiredPeriod = TASK_PERIOD_HZ(0.5),             // 1000000 / 0.5 (hz) = 2000000 us = 2000 ms
//        .desiredPeriod = TASK_PERIOD_HZ(1.25),             // 1000000 / 1.25 (hz) = 800000 us = 800 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif    
};

//static void taskUpdateRxMain(timeUs_t currentTimeUs)			// TODO: make this function static for rtos tasks assignment
void taskUpdateRxMain(timeUs_t currentTimeUs)
{
	/* retrieve RX data */
	processRx(currentTimeUs);
	
	/* new rx data is available */
	isRXDataNew = true;
	
	/* updateRcCommands function sets rcCommand  */
	updateRcCommands();
	
	/* update LEDs */
	updateLEDs();
}

void fcTasksInit(void)
{
    /* Clear RTOS queue and initialise SYSTEM TASK */
    schedulerInit();

    /* Add GYRO, MOTOR, and PID TASK to the queue */
    rescheduleTask(TASK_GYROPID, TASK_PERIOD_HZ(500));       	// 500Hz for standard ESC using PWM signals (on YQ450 quadcopter)
//    rescheduleTask(TASK_GYROPID, TASK_PERIOD_HZ(2000));       	// 2Khz for racing quadcopter using oneshot125, oneshot42 or multishot motor protocols
    setTaskEnabled(TASK_GYROPID, true);

    /* Add Receiver RX TASK to tthe queue */
    setTaskEnabled(TASK_RX, true);
    
#ifdef TASKS_LEDS_TESTING
    /* Add LED3 TASK to the queue (JUST for TESTING) */
    setTaskEnabled(TASK_LED3, true);

    /* Add LED4 TASK to the queue (JUST for TESTING) */
    setTaskEnabled(TASK_LED4, true);

    /* Add LED5 TASK to the queue (JUST for TESTING) */
    setTaskEnabled(TASK_LED5, true);

    /* Add LED6 TASK to the queue (JUST for TESTING) */
    setTaskEnabled(TASK_LED6, true);
#endif    
}
