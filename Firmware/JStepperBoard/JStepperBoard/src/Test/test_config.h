/*
 * TestConfig.h
 *
 * Created: 06/10/2025 10:50:39
 *  Author: Tan
 */

#ifndef TESTCONFIG_H_
#define TESTCONFIG_H_

/** Driver settings */
#define PEAK_CURRENT         16  // (1-31) -> Formula: 4.7A * scaler/31
#define HOLD_CURRENT         8   // (1-31) -> Formula: 4.7A * scaler/31
#define STEALTH_THRSHRPM     200 // 400 is Nema17 Max (Japondo)
#define CLOSED_LOOP          false
#define STOP_ON_STALL        false

/** Motion settings */
#define PEAK_RPM             100 // (1-3000)
#define ACCELERATION_TIME    2
#define DECELERATION_TIME    2
#define MOVE_TARGET_POSITION 20 // Target position for move tests (degrees)

/** Utility */
#define DELAY_MS             1000 // Delay between commands
#define RUN_THROUGH_CAN
#define PRINT_STATUS       true

/** Homing */
#define HOMING_PEAK_RPM    2    // (1-3000)
#define HOMING_MOVE_TARGET 3600 // Target position for homing tests (degrees)
#define HOMING_MOVE_OFFSET 3    // Target offset for homing tests (degrees)

/** Available Tests */
// test_moveRelative
// test_moveRelative_inverseTime
// test_moveVelocity
// test_moveVibration
// test_sensorPolling
// test_spiComm
// test_moveHoming
// test_m1GateTest
// test_motionQueue
#define RUN_TEST           test_moveVelocity

/** Run tests on these drivers */
// #define DRV0
// #define DRV1
#define DRV2
// #define DRV3
// #define DRV4

#endif /* TESTCONFIG_H_ */