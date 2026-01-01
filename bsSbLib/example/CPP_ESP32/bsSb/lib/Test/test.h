#pragma once

#include "Arduino.h"
#include "bsSb.h"
#include <cstdlib>
#include <functional>
#include <string.h>

#define TEST_DEBUG

#ifdef TEST_DEBUG
#    include <stdio.h>
#    define test_debug(...) printf(__VA_ARGS__)
#else
#    define test_debug(...)
#endif

/* ---------------------------------------------------------------------------------- */
enum DriverType {
    OPEN_LOOP,
    OPEN_LOOP_WITH_STALL,
    CLOSED_LOOP,
};

enum TestType {
    MOVE_RELATIVE,
    MOVE_VELOCITY,
    MOVE_HOMING,
    MOVE_VIBRATION,
    M1_GATE_TEST,
    MOVE_INVERSE_TIME,
    JOGGING,
};

static inline const char *toString_testType(TestType tt) {
    switch (tt) {
    case TestType::MOVE_RELATIVE    : return "Move (Relative)"; break;
    case TestType::MOVE_VELOCITY    : return "Move (Velocity)"; break;
    case TestType::MOVE_HOMING      : return "Homing"; break;
    case TestType::MOVE_VIBRATION   : return "Vibration"; break;
    case TestType::M1_GATE_TEST     : return "M1 Gate"; break;
    case TestType::MOVE_INVERSE_TIME: return "Move (Relative + Inverse Time)"; break;
    case TestType::JOGGING          : return "Jogging"; break;
    default                         : return "Invalid"; break;
    }
}

/* ---------------------------------------------------------------------------------- */
typedef struct {
    bsStepperBoard *bsSb;           // board obj
    uint8_t         driver_num;     // stepper number
    DriverType      driver_type;    // drv config
    int32_t         position_units; // move units
    TestType        test_type;
} test_parameters_t;

#define LED_PIN 2
#define BUTTON  27

extern bool listenButton();

extern void testTask_RTOS(void *parameters);
/* ---------------------------------------------------------------------------------- */

// Driver
constexpr uint8_t INPUT_Z  = 0;
constexpr uint8_t OUTPUT_Z = 1;
constexpr uint8_t INPUT_X  = 4;
// constexpr uint8_t INPUT_X  = 2;
constexpr uint8_t OUTPUT_X = 3;
// constexpr uint8_t CAGE_Z   = 4;
constexpr uint8_t CAGE_Z = 0xF;

// Sensors
constexpr bool    SENSOR_TRIGGERED = 0;
constexpr uint8_t STOP_L           = 0;
constexpr uint8_t STOP_R           = 1;

constexpr uint8_t INPUT_Z_TOP_SENSOR = HomingSensor::STOP_L;
constexpr uint8_t INPUT_Z_BTM_SENSOR = HomingSensor::STOP_R;

constexpr uint8_t OUTPUT_Z_TOP_SENSOR = HomingSensor::STOP_L;
constexpr uint8_t OUTPUT_Z_BTM_SENSOR = HomingSensor::STOP_R;

constexpr uint8_t INPUT_X_BELT_SENSOR = HomingSensor::STOP_L;
constexpr uint8_t INPUT_X_MID_SENSOR  = HomingSensor::STOP_R;

constexpr uint8_t OUTPUT_X_BELT_SENSOR = HomingSensor::STOP_L;
constexpr uint8_t OUTPUT_X_MID_SENSOR  = HomingSensor::STOP_R;

constexpr uint8_t CAGE_Z_TOP_SENSOR = HomingSensor::STOP_L;
constexpr uint8_t CAGE_Z_BTM_SENSOR = HomingSensor::STOP_R;

enum Action {
    INPUT_Z_HOME,
    INPUT_Z_RAISE,
    INPUT_Z_LOWER,

    OUTPUT_Z_HOME,
    OUTPUT_Z_RAISE,
    OUTPUT_Z_LOWER,

    INPUT_X_HOME,
    INPUT_X_TO_BELT,
    INPUT_X_TO_MID,

    OUTPUT_X_HOME,
    OUTPUT_X_TO_BELT,
    OUTPUT_X_TO_MID,

    CAGE_Z_HOME,
    CAGE_Z_RAISE,
    CAGE_Z_LOWER,
};

enum Sequence {
    ALL_HOME,
    SEQUENCE,
};

extern void cageSequence_manualCall(void *parameters);
extern void cageSequence_queue(void *parameters);
extern void cageSequence_preloadedMotion(void *parameters);
