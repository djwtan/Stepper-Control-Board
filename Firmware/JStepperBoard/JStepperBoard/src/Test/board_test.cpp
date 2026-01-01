/*
 * Test.cpp
 *
 * Created: 06/10/2025 09:42:17
 *  Author: Tan
 */

#include "board_test.h"

/* ====================================== Utils ===================================== */
xQueueHandle PRINT_MUTEX = xSemaphoreCreateMutex();

void safePrintf(const char *fmt, ...) {
    if (PRINT_STATUS == false) return;
    if (PRINT_MUTEX == nullptr) return; // not initialized yet

    xSemaphoreTake(PRINT_MUTEX, portMAX_DELAY);

    // Print timestamp first
    portTickType tick = xTaskGetTickCount();
    printf("[%10lu ms] ", tick * portTICK_RATE_MS);

    // Print the formatted string
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    xSemaphoreGive(PRINT_MUTEX);
}

int randomSign() {
    int random = rand() % 10;

    return (random > 5 ? 1 : -1);
}

/* ============================== Driver Initialization ============================= */
void driverInit(Stepper *stepper) {
    Stepper::DrvSettings drv_s;
    drv_s.mstep_per_fs      = 256;
    drv_s.fs_per_rev        = 200;
    drv_s.mstatus_selection = 0xFB;

    Stepper::CurrentSettings current_s;
    current_s.i_hold       = HOLD_CURRENT;
    current_s.i_hold_delay = 4;
    current_s.i_run        = PEAK_CURRENT; // !(1-31)

    Stepper::StallSettings stall_s;
    stall_s.stop_on_stall    = STOP_ON_STALL;
    stall_s.stall_thresh_rpm = 100;

    Stepper::ClosedLoopSettings cl_s;
    cl_s.enable       = CLOSED_LOOP;
    cl_s.enc_in_res   = 4000;
    cl_s.cl_tolerance = 51200;

    stepper->setDrv(drv_s);
    stepper->setCurrent(current_s);
    stepper->setStopOnStall(stall_s);
    stepper->setClosedLoop(cl_s);
    stepper->setStealthChopThreshold(STEALTH_THRSHRPM); // default // !MAX 180 for Stall Detection (TBD)

    // Quiet + High Torque -> Increase stealthchop
    //                     -> Stop on Stall won't work above 180
    //                     -> Status flags go booboo
    // Quiet + Low Torque  -> Reduce current
    //                     -> Stop on Stall will work
    //                     -> Can be tuned according to needs
    // Quet + High Torque + Stall detection -> Just use closed loop.
}

/* ============================== Motion Initialization ============================= */
void motionInit_homing(Stepper *stepper) {
    uint16_t move_h = (HOMING_MOVE_TARGET >> 16) & 0xFFFF;
    uint16_t move_l = HOMING_MOVE_TARGET & 0xFFFF;

    uint16_t offset_h = (HOMING_MOVE_OFFSET >> 16) & 0xFFFF;
    uint16_t offset_l = HOMING_MOVE_OFFSET & 0xFFFF;

    uint16_t max_rpm_h = ((HOMING_PEAK_RPM * 1000) >> 16) & 0xFFFF;
    uint16_t max_rpm_l = (HOMING_PEAK_RPM * 1000) & 0xFFFF;

    Stepper::HomeSettings home_s;
    home_s.homing_mode       = Stepper::HomingMode::SENSOR;
    home_s.homing_sensor     = Stepper::HomingSensor::STOP_L;
    home_s.sensor_home_value = 0;
    home_s.max_find_h        = move_h;
    home_s.max_find_l        = move_l;
    home_s.max_speed_h       = max_rpm_h;
    home_s.max_speed_l       = max_rpm_l;
    home_s.max_accel_h       = max_rpm_h;
    home_s.max_accel_l       = max_rpm_l;
    home_s.max_decel_h       = 0x00FF; // max
    home_s.max_decel_l       = 0xFFFF;
    home_s.offset_h          = offset_h;
    home_s.offset_l          = offset_l;
    home_s.timeout_ms_h      = 0;
    home_s.timeout_ms_l      = 10000;

    stepper->confHome(home_s);
}

void motionInit_velocityMode(Stepper *stepper) {
    // Calculation
    uint16_t max_rpm_h = ((PEAK_RPM * 1000) >> 16) & 0xFFFF;
    uint16_t max_rpm_l = (PEAK_RPM * 1000) & 0xFFFF;

    float    max_accel   = (float)PEAK_RPM / ACCELERATION_TIME;
    float    max_decel   = (float)PEAK_RPM / DECELERATION_TIME;
    uint16_t max_accel_h = ((static_cast<uint32_t>(max_accel) * 1000) >> 16) & 0xFFFF;
    uint16_t max_accel_l = (static_cast<uint32_t>(max_accel) * 1000) & 0xFFFF;
    uint16_t max_decel_h = ((static_cast<uint32_t>(max_decel) * 1000) >> 16) & 0xFFFF;
    uint16_t max_decel_l = (static_cast<uint32_t>(max_decel) * 1000) & 0xFFFF;

    // Settings
    Stepper::MoveSettings move_s;
    move_s.allow_write_motion_when_busy      = false;
    move_s.reset_motion_conf_after_each_move = false;

    Stepper::PositioningSettings pos_s;
    pos_s.pos_mode     = Stepper::PositioningMode::PM_RELATIVE;
    pos_s.follow_mode  = Stepper::FollowMode::INTERNAL;
    pos_s.unit_per_rev = 360;

    Stepper::RampSettings ramp_s;
    ramp_s.ramp_mode = TMC4361A::RampMode::VELOCITY_MODE;
    ramp_s.ramp_type = TMC4361A::RampType::TRAPEZOIDAL_RAMP;

    Stepper::SpeedSettings speed_s;
    speed_s.max_h = max_rpm_h;
    speed_s.max_l = max_rpm_l; // 200 RPM

    Stepper::AccelerationSettings acel_s;
    acel_s.max_accel_h = max_accel_h;
    acel_s.max_accel_l = max_accel_l;
    acel_s.max_decel_h = max_decel_h;
    acel_s.max_decel_l = max_decel_l;

    Stepper::BowSettings bow_s;
    bow_s.bow1_h = 0x0000;
    bow_s.bow1_l = 0xC350;
    bow_s.bow2_h = 0x0000;
    bow_s.bow2_l = 0xC350;
    bow_s.bow3_h = 0x0000;
    bow_s.bow3_l = 0xC350;
    bow_s.bow4_h = 0x0000;
    bow_s.bow4_l = 0xC350; // 50

    stepper->confMove(move_s);
    stepper->confPositioning(pos_s);
    stepper->confRamp(ramp_s);
    stepper->confSpeed(speed_s);
    stepper->confAccel(acel_s);
    stepper->confBow(bow_s);
}

void motionInit_positioningMode(Stepper *stepper) {
    // Calculation
    uint16_t max_rpm_h = ((PEAK_RPM * 1000) >> 16) & 0xFFFF;
    uint16_t max_rpm_l = (PEAK_RPM * 1000) & 0xFFFF;

    float    max_accel   = fabs((float)PEAK_RPM) / (float)ACCELERATION_TIME;
    float    max_decel   = fabs((float)PEAK_RPM) / (float)DECELERATION_TIME;
    uint16_t max_accel_h = ((static_cast<uint32_t>(max_accel) * 1000) >> 16) & 0xFFFF;
    uint16_t max_accel_l = (static_cast<uint32_t>(max_accel) * 1000) & 0xFFFF;
    uint16_t max_decel_h = ((static_cast<uint32_t>(max_decel) * 1000) >> 16) & 0xFFFF;
    uint16_t max_decel_l = (static_cast<uint32_t>(max_decel) * 1000) & 0xFFFF;

    // Settings
    Stepper::MoveSettings move_s;
    move_s.allow_write_motion_when_busy      = false;
    move_s.reset_motion_conf_after_each_move = false;

    Stepper::PositioningSettings pos_s;
    pos_s.pos_mode     = Stepper::PositioningMode::PM_RELATIVE;
    pos_s.follow_mode  = Stepper::FollowMode::INTERNAL;
    pos_s.unit_per_rev = 360;

    Stepper::RampSettings ramp_s;
    ramp_s.ramp_mode = TMC4361A::RampMode::POSITIONING_MODE;
    ramp_s.ramp_type = TMC4361A::RampType::TRAPEZOIDAL_RAMP;

    Stepper::SpeedSettings speed_s;
    speed_s.max_h = max_rpm_h;
    speed_s.max_l = max_rpm_l;

    Stepper::AccelerationSettings acel_s;
    acel_s.max_accel_h = max_accel_h;
    acel_s.max_accel_l = max_accel_l;
    acel_s.max_decel_h = max_decel_h;
    acel_s.max_decel_l = max_decel_l;

    stepper->confMove(move_s);
    stepper->confPositioning(pos_s);
    stepper->confRamp(ramp_s);
    stepper->confSpeed(speed_s);
    stepper->confAccel(acel_s);
}

/* =================================== Test Utils =================================== */
void waitReadback(testItem *item) {
    uint32_t readback_value    = 0;
    bool     readback_acquired = false;

    while (!readback_acquired) {
        readback_acquired = item->stepper_ptr->getReadback(&readback_value);
        vTaskDelay(10 / portTICK_RATE_MS);
    }

    safePrintf("[Drv%d] (Readback) %d\n", item->driver_num, readback_value);
}

/* ===================================== Status ===================================== */
bool handleStatus_and_proceed(testItem *item) {
    bool proceed = false;

    Stepper::Status status = item->stepper_ptr->getStatus();

    switch (status) {
    case Stepper::Status::READY: {
        safePrintf("[Drv%d] (Status) READY\n", item->driver_num);
        proceed = true;
        break;
    }
    case Stepper::Status::BUSY: {
        uint32_t pos_dev = item->stepper_ptr->getEncoderPosDev();
        safePrintf("[Drv%d] (Status) BUSY | PosDev %d\n", item->driver_num, pos_dev);
        break;
    }
    case Stepper::Status::HOMING: {
        safePrintf("[Drv%d] (Status) HOMING\n", item->driver_num);
        break;
    }
    case Stepper::Status::VIBRATING: {
        safePrintf("[Drv%d] (Status) BRRRR\n", item->driver_num);
        break;
    }
    case Stepper::Status::CTRL_NOT_INIT: {
        safePrintf("[Drv%d] (Status) INIT CONTROLLER\n", item->driver_num);
        Stepper::PutQueueRes put_queue = item->stepper_ptr->initController();
        safePrintf("[Drv%d] (PutQueueRes) %d\n", item->driver_num, put_queue);
        waitReadback(item);
        break;
    }
    case Stepper::Status::MOTION_NOT_INIT: {
        safePrintf("[Drv%d] (Status) INIT MOTION\n", item->driver_num);
        Stepper::PutQueueRes put_queue = item->stepper_ptr->setMotion();
        safePrintf("[Drv%d] (PutQueueRes) %d\n", item->driver_num, put_queue);
        waitReadback(item);
        break;
    }
    case Stepper::Status::STALL: {
        safePrintf("[Drv%d] (Status) STALLED. Reinitializing in 5s...\n", item->driver_num);
        item->stepper_ptr->d_releaseDriver();
        vTaskDelay(5000 / portTICK_RATE_MS);
        Stepper::PutQueueRes put_queue = item->stepper_ptr->initController();
        safePrintf("[Drv%d] (PutQueueRes) %d\n", item->driver_num, put_queue);
        waitReadback(item);
        break;
    }
    case Stepper::Status::POS_ERR: {
        safePrintf("[Drv%d] (Status) POSITION ERR. Reinitializing in 5s...\n", item->driver_num);
        item->stepper_ptr->d_releaseDriver();
        vTaskDelay(5000 / portTICK_RATE_MS);
        Stepper::PutQueueRes put_queue = item->stepper_ptr->initController();
        safePrintf("[Drv%d] (PutQueueRes) %d\n", item->driver_num, put_queue);
        waitReadback(item);
        break;
    }
    case Stepper::Status::COIL_OL: {
        item->stepper_ptr->d_releaseDriver();
        safePrintf("[Drv%d] (Status) COIL OPEN LOAD\n", item->driver_num);
        vTaskDelay(5000 / portTICK_RATE_MS);
        break;
    }
    case Stepper::Status::COIL_SHORT: {
        item->stepper_ptr->d_releaseDriver();
        safePrintf("[Drv%d] (Status) COIL SHORT\n", item->driver_num);
        vTaskDelay(5000 / portTICK_RATE_MS);
        break;
    }
    case Stepper::Status::OVERTEMP: {
        item->stepper_ptr->d_releaseDriver();
        safePrintf("[Drv%d] (Status) OVERTEMPERATURE\n", item->driver_num);
        vTaskDelay(5000 / portTICK_RATE_MS);
        break;
    }
    case Stepper::Status::TMC4361A_COMM_ERR: {
        item->stepper_ptr->d_releaseDriver();
        safePrintf("[Drv%d] (Status) TMC4361A COMM ERR\n", item->driver_num);
        vTaskDelay(5000 / portTICK_RATE_MS);
        break;
    }
    case Stepper::Status::TMC5160_COMM_ERR: {
        item->stepper_ptr->d_releaseDriver();
        safePrintf("[Drv%d] (Status) TMC5160 COMM ERR\n", item->driver_num);
        vTaskDelay(5000 / portTICK_RATE_MS);
        break;
    }
    default: {
        item->stepper_ptr->d_releaseDriver();
        safePrintf("[Drv%d] (Status) UNDEFINED STATUS\n", item->driver_num);
        vTaskDelay(5000 / portTICK_RATE_MS);
        break;
    }
    }
    return proceed;
}

/* ================================================================================== */
/*                                        Tests                                       */
/* ================================================================================== */
void test_spiComm(void *parameters) {
    auto *item = static_cast<testItem *>(parameters);

    // Wait
    vTaskDelay(1000 / portTICK_RATE_MS);

    safePrintf("[Drv%d](SPI Comm Test) Start \n", item->driver_num);
    for (;;) {
        uint8_t res = item->stepper_ptr->test_Comm();
        switch (res) {
        case 0: {
            safePrintf("[Drv%d] (TMC4361A) Write -> PASS\n", item->driver_num);
            safePrintf("[Drv%d] (TMC4361A) Verify -> FAILED\n", item->driver_num);
            safePrintf("[Drv%d] (TMC5160) Write -> NOT TESTED\n", item->driver_num);
            break;
        }
        case 1: {
            safePrintf("[Drv%d] (TMC4361A) Write -> PASS\n", item->driver_num);
            safePrintf("[Drv%d] (TMC4361A) Verify -> FAILED\n", item->driver_num);
            safePrintf("[Drv%d] (TMC5160) Write -> NOT TESTED\n", item->driver_num);
            break;
        }
        case 2: {
            safePrintf("[Drv%d] (TMC4361A) Write -> PASS\n", item->driver_num);
            safePrintf("[Drv%d] (TMC4361A) Verify -> PASS\n", item->driver_num);
            safePrintf("[Drv%d] (TMC5160) Write -> FAILED\n", item->driver_num);
            break;
        }
        case 3: {
            safePrintf("[Drv%d] (TMC4361A) Write -> PASS\n", item->driver_num);
            safePrintf("[Drv%d] (TMC4361A) Verify -> PASS\n", item->driver_num);
            safePrintf("[Drv%d] (TMC5160) Write -> PASS\n", item->driver_num);
            break;
        }
        default: safePrintf("[Drv%d] Code is Broken.\n", item->driver_num);
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}

void test_moveVelocity(void *parameters) {
    auto *item = static_cast<testItem *>(parameters);

    // Wait
    vTaskDelay(1000 / portTICK_RATE_MS);

    uint16_t max_rpm_h = (PEAK_RPM >> 16) & 0xFFFF;
    uint16_t max_rpm_l = PEAK_RPM & 0xFFFF;

    driverInit(item->stepper_ptr);
    motionInit_velocityMode(item->stepper_ptr);
    safePrintf("[Drv%d] (Velocity Mode Test) Start \n", item->driver_num);

    for (;;) {
        bool proceed = handleStatus_and_proceed(item);

        if (proceed) {
            // Motion
            item->stepper_ptr->setTargetUnitsHigh(max_rpm_h);
            item->stepper_ptr->setTargetUnitsLow(max_rpm_l); // 60 rpm

            // Move
            vTaskDelay(DELAY_MS / portTICK_RATE_MS);
            Stepper::PutQueueRes put_queue = item->stepper_ptr->move();
            safePrintf("[Drv%d] (PutQueueRes) %d\n", item->driver_num, put_queue);

            // Readback
            waitReadback(item);
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

/* ---------------------------------------------------------------------------------- */
void test_moveRelative(void *parameters) {
    auto *item = static_cast<testItem *>(parameters);

    // Wait
    vTaskDelay(1000 / portTICK_RATE_MS);

    uint32_t run_iterations = 0;
    int32_t  current_pos    = 0;
    int32_t  encoder_pos    = 0;
    uint16_t move_h         = (MOVE_TARGET_POSITION >> 16) & 0xFFFF;
    uint16_t move_l         = MOVE_TARGET_POSITION & 0xFFFF;

    driverInit(item->stepper_ptr);
    motionInit_positioningMode(item->stepper_ptr);
    safePrintf("[Drv%d] (Positioning Mode Test) Start \n", item->driver_num);

    for (;;) {
        bool proceed = handleStatus_and_proceed(item);

        if (proceed) {
            // ovf
            // if (run_iterations % 20 == 0) {
            //   // bool position_set = item->stepper_ptr->setPosition(INT32_MIN + 300000);
            //   bool position_set = item->stepper_ptr->setPosition(INT32_MAX - 300000);
            //   if (PRINT_STATUS) safePrintf("[Drv%d] Position set -> %d\n", item->driver_num, position_set);
            // }

            // Motion
            item->stepper_ptr->setTargetUnitsHigh(move_h);
            item->stepper_ptr->setTargetUnitsLow(move_l);

            // Move
            vTaskDelay(DELAY_MS / portTICK_RATE_MS);
            Stepper::PutQueueRes put_queue = item->stepper_ptr->move();
            safePrintf("[Drv%d] (PutQueueRes) %d\n", item->driver_num, put_queue);

            // Readback
            waitReadback(item);

            // Position
            run_iterations++;
            current_pos = item->stepper_ptr->getInternalPosition();
            encoder_pos = item->stepper_ptr->getEncoderPosition();
            safePrintf("[Drv%d] %d || Pos -> %.12d | %.12d\n", item->driver_num, run_iterations, current_pos,
                       encoder_pos);
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

/* ---------------------------------------------------------------------------------- */
void test_moveRelative_inverseTime(void *parameters) {
    auto *item = static_cast<testItem *>(parameters);

    // Wait
    vTaskDelay(1000 / portTICK_RATE_MS);

    uint32_t run_iterations = 0;
    int32_t  current_pos    = 0;
    int32_t  encoder_pos    = 0;
    uint16_t move_h         = (MOVE_TARGET_POSITION >> 16) & 0xFFFF;
    uint16_t move_l         = MOVE_TARGET_POSITION & 0xFFFF;

    driverInit(item->stepper_ptr);
    motionInit_positioningMode(item->stepper_ptr);
    safePrintf("[Drv%d] (Positioning Mode [Inverse Time] Test) Start \n", item->driver_num);

    for (;;) {
        bool proceed = handleStatus_and_proceed(item);

        if (proceed) {
            // ovf
            // if (run_iterations % 20 == 0) {
            //   bool position_set = item->stepper_ptr->setPosition(INT32_MAX - 300000);
            //   if (PRINT_STATUS) safePrintf("[Drv%d] Position set -> %d\n", item->driver_num, position_set);
            // }

            // Motion
            item->stepper_ptr->setTargetUnitsHigh(move_h);
            item->stepper_ptr->setTargetUnitsLow(move_l); // 360 units

            item->stepper_ptr->setTimeMsHigh(0);
            item->stepper_ptr->setTimeMsLow(2000); // 2 seconds

            // Move
            vTaskDelay(DELAY_MS / portTICK_RATE_MS);
            Stepper::PutQueueRes put_queue = item->stepper_ptr->moveInverseTime();
            safePrintf("[Drv%d] (PutQueueRes) %d\n", item->driver_num, put_queue);

            // Start time
            portTickType start_tick = xTaskGetTickCount();

            // Readback
            waitReadback(item);

            // End time
            item->stepper_ptr->waitCompleteRun();
            portTickType end_tick   = xTaskGetTickCount();
            portTickType total_time = end_tick - start_tick;
            safePrintf("[Drv%d] (Time Taken) -> %d ms\n", item->driver_num, total_time * portTICK_RATE_MS);

            // Position
            run_iterations++;
            current_pos = item->stepper_ptr->getInternalPosition();
            encoder_pos = item->stepper_ptr->getEncoderPosition();
            safePrintf("[Drv%d]%d || Pos -> %d | %d\n", item->driver_num, run_iterations, current_pos, encoder_pos);
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

/* ---------------------------------------------------------------------------------- */
void test_moveVibration(void *parameters) {
    auto *item = static_cast<testItem *>(parameters);

    // Wait
    vTaskDelay(1000 / portTICK_RATE_MS);

    driverInit(item->stepper_ptr); // open loop only
    motionInit_positioningMode(item->stepper_ptr);
    safePrintf("[Drv%d] (Vibration Test Start \n", item->driver_num);

    for (;;) {
        bool proceed = handleStatus_and_proceed(item);

        if (proceed) {
            item->stepper_ptr->setTargetUnitsHigh(0x0000);
            item->stepper_ptr->setTargetUnitsLow(0x0005);   // 3 units
            item->stepper_ptr->setVibrationIterations(100); // 100 iterations
            item->stepper_ptr->setDiminishingFactor(1000);  // 0.95 dim factor
            item->stepper_ptr->setLoop(true);               // vibrate forever

            Stepper::PutQueueRes put_queue = item->stepper_ptr->moveVibration();
            safePrintf("[Drv%d] (PutQueueRes) %d\n", item->driver_num, put_queue);

            // Readback
            waitReadback(item);
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

/* ---------------------------------------------------------------------------------- */
void test_moveHoming(void *parameters) {
    auto *item = static_cast<testItem *>(parameters);

    bool run_homing = true;

    // Wait
    vTaskDelay(1000 / portTICK_RATE_MS);

    driverInit(item->stepper_ptr); // open loop only
    motionInit_positioningMode(item->stepper_ptr);
    motionInit_homing(item->stepper_ptr);
    safePrintf("[Drv%d] (Homing Test Start \n", item->driver_num);

    for (;;) {
        bool proceed = handleStatus_and_proceed(item);

        if (proceed) {
            vTaskDelay(DELAY_MS / portTICK_RATE_MS);

            if (run_homing) {
                Stepper::PutQueueRes put_queue = item->stepper_ptr->moveHoming();
                safePrintf("[Drv%d] (PutQueueRes) %d\n", item->driver_num, put_queue);

                // Readback
                waitReadback(item);

            } else {
                // Motion
                item->stepper_ptr->setTargetUnitsHigh(0);
                item->stepper_ptr->setTargetUnitsLow(60);

                // Move
                vTaskDelay(DELAY_MS / portTICK_RATE_MS);
                Stepper::PutQueueRes put_queue = item->stepper_ptr->move();
                safePrintf("[Drv%d] (PutQueueRes) %d\n", item->driver_num, put_queue);

                // Readback
                waitReadback(item);
            }
            // run_homing = !run_homing;
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

/* ---------------------------------------------------------------------------------- */
void test_initialze_and_fetchStatus_only(void *parameters) {
    auto *item = static_cast<testItem *>(parameters);

    // Wait
    vTaskDelay(1000 / portTICK_RATE_MS);

    driverInit(item->stepper_ptr);
    motionInit_positioningMode(item->stepper_ptr);
    safePrintf("[Drv%d] (Fetch Status Only Test) Start \n", item->driver_num);

    for (;;) {
        bool proceed = handleStatus_and_proceed(item);
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

/* ---------------------------------------------------------------------------------- */
void test_sensorPolling(void *parameters) {
    auto *item = static_cast<testItem *>(parameters);

    // Wait
    vTaskDelay(1000 / portTICK_RATE_MS);

    safePrintf("[Drv%d] (Sensor Polling Test) Start \n", item->driver_num);

    uint8_t sensor_state = 0;

    for (;;) {
        sensor_state = item->stepper_ptr->getSensorReading();

        safePrintf("[Drv%d] Sensor State -> %d%d\n", item->driver_num, sensor_state >> 0 & 0x01,
                   sensor_state >> 1 & 0x01);
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

/* ---------------------------------------------------------------------------------- */
void test_m1GateTest(void *parameters) {
    auto *item = static_cast<testItem *>(parameters);

    bool          run_homing          = true;
    uint8_t       sensor_status       = 0;
    uint8_t       home_sensor_reading = 0;
    int32_t       target              = 120;
    const uint8_t HOME_VALUE          = 0;
    uint16_t      move_h              = 0;
    uint16_t      move_l              = 0;
    // Wait
    vTaskDelay(1000 / portTICK_RATE_MS);

    driverInit(item->stepper_ptr); // open loop only
    motionInit_positioningMode(item->stepper_ptr);
    motionInit_homing(item->stepper_ptr);
    safePrintf("[Drv%d] (m1 Gate Test) Start \n", item->driver_num);

    for (;;) {
        bool proceed        = handleStatus_and_proceed(item);
        sensor_status       = item->stepper_ptr->getSensorReading();
        home_sensor_reading = (sensor_status >> 0) & 1;

        if (proceed) {
            if (home_sensor_reading != HOME_VALUE) {
                Stepper::PutQueueRes put_queue = item->stepper_ptr->moveHoming();
                safePrintf("[Drv%d] (PutQueueRes) %d\n", item->driver_num, put_queue);

                // Readback
                waitReadback(item);
            } else {
                target = 120 * randomSign();

                move_h = (target >> 16) & 0xFFFF;
                move_l = target & 0xFFFF;

                // Motion
                item->stepper_ptr->setTargetUnitsHigh(move_h);
                item->stepper_ptr->setTargetUnitsLow(move_l);

                // Move
                vTaskDelay(DELAY_MS / portTICK_RATE_MS);
                Stepper::PutQueueRes put_queue = item->stepper_ptr->move();
                safePrintf("[Drv%d] (PutQueueRes) %d\n", item->driver_num, put_queue);

                // Readback
                waitReadback(item);
            }
        }
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

/* ---------------------------------------------------------------------------------- */
void test_canTransmit(void *parameters) {
    auto *can = static_cast<CANInterface *>(parameters);

    printf("Can Transmit Test Start\n");
    for (;;) {

        printf("(CAN) Transmit\n");
        can->transmitMessage(0x12345667, 0);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

/* ---------------------------------------------------------------------------------- */
void test_motionQueue(void *parameters) {
    auto *item = static_cast<testMotionQueue *>(parameters);

    vTaskDelay(1000 / portTICK_RATE_MS);

    safePrintf("(Motion Queue Test) Start \n");
    driverInit(item->stepper_ptr[0]);
    driverInit(item->stepper_ptr[1]);
    driverInit(item->stepper_ptr[2]);
    driverInit(item->stepper_ptr[3]);
    driverInit(item->stepper_ptr[4]);
    motionInit_positioningMode(item->stepper_ptr[0]);
    motionInit_positioningMode(item->stepper_ptr[1]);
    motionInit_positioningMode(item->stepper_ptr[2]);
    motionInit_positioningMode(item->stepper_ptr[3]);
    motionInit_positioningMode(item->stepper_ptr[4]);

    item->stepper_ptr[0]->execInitController();
    item->stepper_ptr[1]->execInitController();
    item->stepper_ptr[2]->execInitController();
    item->stepper_ptr[3]->execInitController();
    item->stepper_ptr[4]->execInitController();
    item->stepper_ptr[0]->execSetMotion();
    item->stepper_ptr[1]->execSetMotion();
    item->stepper_ptr[2]->execSetMotion();
    item->stepper_ptr[3]->execSetMotion();
    item->stepper_ptr[4]->execSetMotion();

    MotionQueue::Queue q1; // queue 1

    MotionQueue::QueueItem q11; // queue 1 item 1
    MotionQueue::QueueItem q12; // queue 1 item 2
    MotionQueue::QueueItem q13; // queue 1 item 3
    MotionQueue::QueueItem q14; // queue 1 item 4
    MotionQueue::QueueItem q15; // queue 1 item 5

    uint16_t move_h      = (MOVE_TARGET_POSITION >> 16) & 0xFFFF;
    uint16_t move_l      = MOVE_TARGET_POSITION & 0xFFFF;
    uint16_t max_rpm_h   = ((PEAK_RPM * 1000) >> 16) & 0xFFFF;
    uint16_t max_rpm_l   = (PEAK_RPM * 1000) & 0xFFFF;
    float    max_accel   = (float)PEAK_RPM / ACCELERATION_TIME;
    float    max_decel   = (float)PEAK_RPM / DECELERATION_TIME;
    uint16_t max_accel_h = ((static_cast<uint32_t>(max_accel) * 1000) >> 16) & 0xFFFF;
    uint16_t max_accel_l = (static_cast<uint32_t>(max_accel) * 1000) & 0xFFFF;
    uint16_t max_decel_h = ((static_cast<uint32_t>(max_decel) * 1000) >> 16) & 0xFFFF;
    uint16_t max_decel_l = (static_cast<uint32_t>(max_decel) * 1000) & 0xFFFF;

    // M0-0
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MOTION_TYPE, 0, 0,
                                        (uint16_t)MotionQueue::MotionType::MOVE_RELATIVE);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::SEQUENCE_NUMBER, 0, 0, 0);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::DRIVER_NUMBER, 0, 0, 0);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::POSITION_H, 0, 0, move_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::POSITION_L, 0, 0, move_l - 180);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::RAMP_TYPE, 0, 0, 1);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_SPEED_H, 0, 0, max_rpm_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_SPEED_L, 0, 0, max_rpm_l);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_ACCEL_H, 0, 0, max_accel_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_ACCEL_L, 0, 0, max_accel_l);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_DECEL_H, 0, 0, max_decel_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_DECEL_L, 0, 0, max_decel_l);

    // M0-1
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MOTION_TYPE, 0, 1,
                                        (uint16_t)MotionQueue::MotionType::MOVE_HOMING);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::SEQUENCE_NUMBER, 0, 1, 0);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::DRIVER_NUMBER, 0, 1, 1);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::POSITION_H, 0, 1, move_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::POSITION_L, 0, 1, move_l - 90);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::RAMP_TYPE, 0, 1, 1);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_SPEED_H, 0, 1, max_rpm_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_SPEED_L, 0, 1, max_rpm_l);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_ACCEL_H, 0, 1, max_accel_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_ACCEL_L, 0, 1, max_accel_l);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_DECEL_H, 0, 1, max_decel_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_DECEL_L, 0, 1, max_decel_l);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::OFFSET_H, 0, 1, 0);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::OFFSET_L, 0, 1, 3);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::HOMING_MODE, 0, 1,
                                        (uint16_t)Stepper::HomingMode::SENSOR);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::HOMING_SENSOR, 0, 1,
                                        (uint16_t)Stepper::HomingSensor::STOP_R);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::SENSOR_HOME_VALUE, 0, 1, 0);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::TIME_H, 0, 1, 0x0000);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::TIME_L, 0, 1, 0x1388);

    // M0-2
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MOTION_TYPE, 0, 2,
                                        (uint16_t)MotionQueue::MotionType::MOVE_RELATIVE);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::SEQUENCE_NUMBER, 0, 2, 2);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::DRIVER_NUMBER, 0, 2, 3);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::POSITION_H, 0, 2, move_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::POSITION_L, 0, 2, move_l);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::RAMP_TYPE, 0, 2, 1);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_SPEED_H, 0, 2, max_rpm_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_SPEED_L, 0, 2, max_rpm_l);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_ACCEL_H, 0, 2, max_accel_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_ACCEL_L, 0, 2, max_accel_l);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_DECEL_H, 0, 2, max_decel_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MAX_DECEL_L, 0, 2, max_decel_l);

    // M0-3
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MOTION_TYPE, 0, 3,
                                        (uint16_t)MotionQueue::MotionType::MOVE_RELATIVE);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::SEQUENCE_NUMBER, 0, 3, 2);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::DRIVER_NUMBER, 0, 3, 4);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::POSITION_H, 0, 3, move_h);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::POSITION_L, 0, 3, move_l);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::USE_INVERSE_TIME, 0, 3, 1);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::TIME_H, 0, 3, 0x0000);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::TIME_L, 0, 3, 0x07D0);

    // M0-4 (Wait)
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MOTION_TYPE, 0, 4,
                                        (uint16_t)MotionQueue::MotionType::WAIT_TIME);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::SEQUENCE_NUMBER, 0, 4, 1);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::TIME_H, 0, 4, 0x0000);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::TIME_L, 0, 4, 0x07D0);

    // M0-5 (Wait Sensor) - Pass 0 for no timeout
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::MOTION_TYPE, 0, 5,
                                        (uint16_t)MotionQueue::MotionType::WAIT_SENSOR);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::SEQUENCE_NUMBER, 0, 5, 3);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::DRIVER_NUMBER, 0, 5, 1);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::HOMING_SENSOR, 0, 5,
                                        (uint16_t)Stepper::HomingSensor::STOP_L);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::SENSOR_HOME_VALUE, 0, 5, 0);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::TIME_H, 0, 5, 0x0000);
    item->motion_queue->writeMotionItem(MotionQueue::QueueItemKey::TIME_L, 0, 5, 0x2710);

    for (;;) {
        item->motion_queue->queueMotion(0);

        bool     has_readback = false;
        uint32_t readback     = 0;

        while (!has_readback) {
            has_readback = item->motion_queue->getReadback(&readback);
            vTaskDelay(25 / portTICK_RATE_MS);
        }

        vTaskDelay(DELAY_MS / portTICK_RATE_MS);
    }
}