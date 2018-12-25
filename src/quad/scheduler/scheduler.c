
#include <stdio.h>

#include "utils.h"
#include "scheduler.h"
#include "system.h"
#include "maths.h"
#include "common.h"

static cfTask_t *currentTask = NULL;

static bool calculateTaskStatistics;

static uint32_t totalWaitingTasks;
static uint32_t totalWaitingTasksSamples;

uint16_t averageSystemLoadPercentage = 0;

cfTask_t *taskQueueArray[TASK_COUNT + 1];           // just for testing
//static cfTask_t *taskQueueArray[TASK_COUNT + 1];

static int taskQueuePos = 0;
static int taskQueueSize = 0;

/* No need to implement linked list for the queue as items are always inserted at the front */

#ifndef SKIP_TASK_STATISTICS
#define MOVING_SUM_COUNT            32
timeUs_t checkFuncMaxExecutionTime;
timeUs_t checkFuncTotalExecutionTime;
timeUs_t checkFuncMovingSumExecutionTime;
#endif

void queueClear(void)
{
    memset(taskQueueArray, 0, sizeof(taskQueueArray));
    taskQueuePos = 0;
    taskQueueSize = 0;
}

bool queueContains(cfTask_t *task)
{
    for (int i = 0; i < taskQueueSize; ++i) {
        if (taskQueueArray[i] == task) {
            return true;
        }
    }
    
    return false;
}

bool queueAdd(cfTask_t *task)
{
    if ((taskQueueSize >= TASK_COUNT) || queueContains(task)) {
        return false;
    }
    
    for (int i = 0; i <= taskQueueSize; ++i) {
        if (taskQueueArray[i] == NULL || taskQueueArray[i]->staticPriority < task->staticPriority) {
            memmove(&taskQueueArray[i+1], &taskQueueArray[i], sizeof(task) * (taskQueueSize - i));
            taskQueueArray[i] = task;
            ++taskQueueSize;
            return true;
        }
    }
    
    return false;
}

bool queueRemove(cfTask_t *task)
{
    for (int i = 0; i < taskQueueSize; ++i) {
        if (taskQueueArray[i] == task) {
            memmove(&taskQueueArray[i], &taskQueueArray[i+1], sizeof(task) * (taskQueueSize - i));
            --taskQueueSize;
            return true;
        }
    }
    
    return false;
}

/* Returns the first item in the queue or NULL if queue empty */
cfTask_t *queueFirst(void)
{
    taskQueuePos = 0;
    return taskQueueArray[0];
}

/* Returns the next item in the queue or NULL if at the end of queue */
cfTask_t *queueNext(void)
{
    return taskQueueArray[++taskQueuePos];
}

/* Get the task runtime difference between the current time and last time */
uint32_t getTaskDeltaTime(cfTaskId_e taskId)
{
	if (taskId == TASK_SELF) {
		return currentTask->taskLatestDeltaTime;
	} else if (taskId < TASK_COUNT) {
		return cfTasks[taskId].taskLatestDeltaTime;
	}
	
	return 0;
}

/*
 *  Remove task from queue if enabled is set to false,
 *  otherwise, add task to queue.
 */
void setTaskEnabled(cfTaskId_e taskId, bool enabled)
{
    if (taskId == TASK_SELF || taskId < TASK_COUNT) {
        cfTask_t *task = taskId == TASK_SELF ? currentTask : &cfTasks[taskId];
        if (enabled && task->taskFunc) {
            queueAdd(task);
        } else {
            queueRemove(task);
        }
    }
}

void rescheduleTask(cfTaskId_e taskId, uint32_t newPeriodInMicros)
{
    if (taskId == TASK_SELF) {
        cfTask_t *task = currentTask;
        task->desiredPeriod = MAX(SCHEDULER_DELAY_LIMIT, newPeriodInMicros);    // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    } else if (taskId < TASK_COUNT) {
        cfTask_t *task = &cfTasks[taskId];
        task->desiredPeriod = MAX(SCHEDULER_DELAY_LIMIT, newPeriodInMicros);    // Limit delay to 100us (10 kHz) to prevent scheduler clogging
    }
}

void schedulerInit(void)
{
    calculateTaskStatistics = true;
    queueClear();
    queueAdd(&cfTasks[TASK_SYSTEM]);
}

void scheduler(void)
{
    /* Cache the current time in microseconds */
    const timeUs_t currentTimeUs = micros();
    
    /* Check for realtime tasks */
    timeUs_t timeToNextRealtimeTask = TIMEUS_MAX;
    
    for (const cfTask_t *task = queueFirst(); task != NULL && task->staticPriority >= TASK_PRIORITY_REALTIME; task = queueNext()) {
        const timeUs_t nextExecutedAt = task->lastExecutedAt + task->desiredPeriod;
        
        if ((int32_t)(currentTimeUs - nextExecutedAt) >= 0) {
            /* Missed the next executed time */
            timeToNextRealtimeTask = 0;
        } else {
            const timeUs_t newTimeInterval = nextExecutedAt - currentTimeUs;
            timeToNextRealtimeTask = MIN(timeToNextRealtimeTask, newTimeInterval);
        }
    }
    
//    printf("%u\r\n", timeToNextRealtimeTask);
    const bool outsideRealtimeGuardInterval = (timeToNextRealtimeTask > 0);
    
    /* The task to be executed */
    cfTask_t *selectedTask = NULL;
    uint16_t selectedTaskDynamicPriority = 0;
    
    /* Update task dynamic priority */
    uint16_t waitingTasks = 0;
    
    for (cfTask_t *task = queueFirst(); task != NULL; task = queueNext()) {
        /* Task has checkFunc - event driven case */
        if (task->checkFunc) {
            
            const timeUs_t currentTimeBeforeCheckFuncCall = currentTimeUs;
//            printf("%u\r\n", currentTimeBeforeCheckFuncCall);
            
//            printf("%u, %d\r\n", task->dynamicPriority, __LINE__);
            
            /* Increase priority for event-driven tasks */
            if (task->dynamicPriority > 0) {
//                printf("%u\r\n", task->desiredPeriod);
                task->taskAgeCycles = 1 + ((currentTimeUs - task->lastSignaledAt) / task->desiredPeriod);
//                printf("%u\r\n", task->dynamicPriority);
                task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                waitingTasks++;
            } else if (task->checkFunc(currentTimeBeforeCheckFuncCall, currentTimeBeforeCheckFuncCall - task->lastExecutedAt)) {
                
//                printf("%u\r\n", micros() - currentTimeBeforeCheckFuncCall);
                
#ifndef SKIP_TASK_STATISTICS
                if (calculateTaskStatistics) {
                    const uint32_t checkFuncExecutionTime = micros() - currentTimeBeforeCheckFuncCall;
                    checkFuncMovingSumExecutionTime += checkFuncExecutionTime - checkFuncMovingSumExecutionTime / MOVING_SUM_COUNT;
                    checkFuncTotalExecutionTime += checkFuncExecutionTime;              // time consumed by scheduler and task
                    checkFuncMaxExecutionTime = MAX(checkFuncMaxExecutionTime, checkFuncExecutionTime);
                }
#endif
                task->lastSignaledAt = currentTimeBeforeCheckFuncCall;
                task->taskAgeCycles = 1;
                task->dynamicPriority = 1 + task->staticPriority;
//                printf("%u, %d\r\n", task->dynamicPriority, __LINE__);
                
                waitingTasks++;
            } else {
                task->taskAgeCycles = 0;
            }
        } else {
            /* Time-driven, dynamicPriority is last execution age (measured in desiredPeriods)
             * Task age is calculated from last execution
             */
            task->taskAgeCycles = ((currentTimeUs - task->lastExecutedAt) / task->desiredPeriod);
//            printf("%u\r\n", task->taskAgeCycles);
            if (task->taskAgeCycles > 0) {
                task->dynamicPriority = 1 + task->staticPriority * task->taskAgeCycles;
                waitingTasks++;
            }
        }
        
//        printf("%u, %d\r\n", selectedTaskDynamicPriority, __LINE__);          // selectedTaskDynamicPriority = 0
        
        if (task->dynamicPriority > selectedTaskDynamicPriority) {
//            printf("%d, %d\r\n", outsideRealtimeGuardInterval, __LINE__);               // true
//            printf("%d, %d\r\n", task->taskAgeCycles > 1, __LINE__);      // false
            const bool taskCanBeChosenForScheduling = 
                (outsideRealtimeGuardInterval) ||
                (task->taskAgeCycles > 1) ||
                (task->staticPriority == TASK_PRIORITY_REALTIME);
            
            if (taskCanBeChosenForScheduling) {
//                printf("%u\r\n", task->dynamicPriority);
                selectedTaskDynamicPriority = task->dynamicPriority;
                selectedTask = task;
            }
        }
    }
    
    /* Got a task ready to be executed */
    totalWaitingTasksSamples++;
    totalWaitingTasks += waitingTasks;
    
    /* Update currentTask */
    currentTask = selectedTask;
    
    if (selectedTask) {
        /* Found a task that should be run */
        selectedTask->taskLatestDeltaTime = currentTimeUs - selectedTask->lastExecutedAt;
//        printf("taskName\t taskPeriod\r\n");
//        printf("%s\t\t   %u (us)\r\n", selectedTask->taskName, selectedTask->taskLatestDeltaTime);
        selectedTask->lastExecutedAt = currentTimeUs;
        selectedTask->dynamicPriority = 0;
        
        /* Execute this task */
#ifdef SKIP_TASK_STATISTICS
        selectedTask->taskFunc(currentTimeUs);
#else
        if (calculateTaskStatistics) {
            const timeUs_t currentTimeBeforeTaskCall = micros();
            selectedTask->taskFunc(currentTimeBeforeTaskCall);
            const timeUs_t taskExecutionTime = micros() - currentTimeBeforeTaskCall;
//            printf("%u, %d---\r\n", micros(), __LINE__);
//            printf("%u, %d\r\n", currentTimeBeforeTaskCall, __LINE__);
            selectedTask->movingSumExecutionTime += taskExecutionTime - selectedTask->movingSumExecutionTime / MOVING_SUM_COUNT;
            selectedTask->totalExecutionTime += taskExecutionTime;          // time consumed by scheduler + task
            selectedTask->maxExecutionTime = MAX(selectedTask->maxExecutionTime, taskExecutionTime);
//            printf("%u, %d\r\n", selectedTask->maxExecutionTime, __LINE__);
        } else {
            selectedTask->taskFunc(currentTimeUs);
        }
#endif
    }
}

void taskSystem(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    
    if (totalWaitingTasksSamples > 0) {
//        printf("totalWaitingTasks: %u\r\n", totalWaitingTasks);
//        printf("totalWaitingTasksSamples: %u\r\n", totalWaitingTasksSamples);
        averageSystemLoadPercentage = 100 * totalWaitingTasks / totalWaitingTasksSamples;
//        printf("averageSystemLoadPercentage: %u\r\n", averageSystemLoadPercentage);
        totalWaitingTasksSamples = 0;
        totalWaitingTasks = 0;
    }
    
//    printf("%s, %d\r\n", __FUNCTION__, __LINE__);
}
