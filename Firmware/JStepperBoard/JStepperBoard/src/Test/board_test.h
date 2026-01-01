/*
 * UnitTest.h
 *
 * Created: 06/10/2025 09:41:44
 *  Author: Tan
 */

#ifndef BOARD_TEST_H_
#define BOARD_TEST_H_

#include "FreeRTOS.h" // for FreeRTOS API
#include "asf.h"
#include "can_interface.h"
#include "motion_queue.h"
#include "semphr.h" // for SemaphoreHandle_t, xSemaphoreTake/Give
#include "stepper.h"
#include "task.h" // for xTaskGetTickCount (optional timestamp)
#include "test_config.h"
#include <cstdlib>
#include <math.h>
#include <stdarg.h> // for va_list, va_start, va_end
#include <stdio.h>  // for printf, vprintf

struct testItem {
    Stepper *stepper_ptr;
    uint8_t  driver_num;
};

extern void driverInit(Stepper *stepper);
extern void motionInit_homing(Stepper *stepper);
extern void motionInit_velocityMode(Stepper *stepper);
extern void motionInit_positioningMode(Stepper *stepper);

extern void waitReadback(testItem *item);
extern bool handleStatus_and_proceed(testItem *item);

extern void test_spiComm(void *parameters);
extern void test_moveVelocity(void *parameters);
extern void test_moveRelative(void *parameters);
extern void test_moveRelative_inverseTime(void *parameters);
extern void test_moveVibration(void *parameters);
extern void test_moveHoming(void *parameters);
extern void test_initialze_and_fetchStatus_only(void *parameters);
extern void test_sensorPolling(void *parameters);

extern void test_m1GateTest(void *parameters);

/* ---------------------------------------------------------------------------------- */
extern void test_canTransmit(void *parameters);

/* ---------------------------------------------------------------------------------- */
struct testMotionQueue {
    Stepper     *stepper_ptr[MAX_STEPPERS];
    MotionQueue *motion_queue;
};
extern void test_motionQueue(void *parameters);

#endif /* BOARD_TEST_H_ */