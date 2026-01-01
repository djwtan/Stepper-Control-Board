#include "test.h"

/* ================================================================================== */
/*                                     Test Utils                                     */
/* ================================================================================== */
float randomScaler() {
    int r = rand() % 100;

    if (r > 60) return -1.0f;
    if (r > 20) return 1.0f;
    return 0.5f;
}

bsStepperBoard::DriverConfig getTestDriverConfig(DriverType driver_type) {
    bsStepperBoard::DriverConfig drv_config;
    drv_config.holding_current  = 500;
    drv_config.peak_rms_current = 2200;
    drv_config.unit_per_rev     = 360;

    switch (driver_type) {
    /* ------------------------------------------ */
    case DriverType::OPEN_LOOP: {
        drv_config.stealth_chop_thresh = 400;
        break;
    }
    /* ------------------------------------------ */
    case DriverType::OPEN_LOOP_WITH_STALL: {
        drv_config.stop_on_stall_enable = 1;
        drv_config.stop_on_stall_thresh = 200;
        drv_config.stealth_chop_thresh  = 180;
        break;
    }
    /* ------------------------------------------ */
    case DriverType::CLOSED_LOOP: {
        drv_config.cl_enable           = 1;
        drv_config.cl_tolerance        = 2560;
        drv_config.stealth_chop_thresh = 400;
        break;
    }
    /* ------------------------------------------ */
    default: break;
    }
    return drv_config;
}

bsStepperBoard::MotionConfig getTestMotionConfig(TestType test_type) {
    bsStepperBoard::MotionConfig motion_config;

    switch (test_type) {
    /* ------------------------------------------ */
    case TestType::MOVE_RELATIVE: {
        motion_config.max_speed = 100.0f;
        motion_config.max_accel = 100.0f / 1.0f;
        motion_config.max_decel = 100.0f / 1.0f;
        break;
    }
    /* ------------------------------------------ */
    case TestType::MOVE_VELOCITY: {
        motion_config.max_speed = 800.0f;
        motion_config.max_accel = 800.0f / 10.0f;
        motion_config.max_decel = 800.0f / 10.0f;
        motion_config.ramp_mode = RampMode::VELOCITY_MODE;
        break;
    }
    /* ------------------------------------------ */
    case TestType::MOVE_HOMING: {
        break;
    }
    /* ------------------------------------------ */
    case TestType::MOVE_VIBRATION: {
        break;
    }
    /* ------------------------------------------ */
    case TestType::M1_GATE_TEST: {
        motion_config.max_speed = 350.0f;
        motion_config.max_accel = 350.0f / 1.0f;
        motion_config.max_decel = 350.0f / 1.0f;
        motion_config.bow1      = 350.0f / 2.0f;
        motion_config.bow2      = 350.0f / 2.0f;
        motion_config.bow3      = 350.0f / 2.0f;
        motion_config.bow4      = 350.0f / 2.0f;
        motion_config.ramp_mode = RampMode::POSITIONING_MODE;
        motion_config.ramp_type = RampType::S_CURVE;
        break;
    }
    /* ------------------------------------------ */
    case TestType::MOVE_INVERSE_TIME: {
        motion_config.max_speed = 100.0f;
        motion_config.max_accel = 100.0f / 1.0f;
        motion_config.max_decel = 100.0f / 1.0f;
        break;
    }
    /* ------------------------------------------ */
    case TestType::JOGGING: {
        motion_config.max_speed                    = 200.0f;
        motion_config.max_accel                    = 200.0f / 1.0f;
        motion_config.max_decel                    = 200.0f / 1.0f;
        motion_config.allow_write_motion_when_busy = true;
        motion_config.ramp_mode                    = RampMode::POSITIONING_MODE;
    }
    /* ------------------------------------------ */
    default: {
        break;
    }
    }
    return motion_config;
}

bool listenButton() {
    if (digitalRead(BUTTON) == HIGH) {
        digitalWrite(LED_PIN, HIGH);
        return true;
    } else {
        return false;
    }
}

/* ================================================================================== */
/*                                      Test Main                                     */
/* ================================================================================== */

void onReady(test_parameters_t *pr) {
    switch (pr->test_type) {
    /* ------------------------------------------ */
    case TestType::MOVE_RELATIVE: {
        bool moved = pr->bsSb->q_moveStepper(pr->driver_num, pr->position_units);

        test_debug("%d units\n", pr->position_units);

        // Readback
        uint16_t queue_readback;
        moved &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);  // read success
        moved &= pr->bsSb->verifyQueueReadback_execSuccessful(queue_readback); // condition pass
        break;
    }
    /* ------------------------------------------ */
    case TestType::MOVE_VELOCITY: {
        bool moved = pr->bsSb->q_moveStepper(pr->driver_num, pr->position_units);

        test_debug("%d rpm\n", pr->position_units);

        // Readback
        uint16_t queue_readback;
        moved &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);  // read success
        moved &= pr->bsSb->verifyQueueReadback_execSuccessful(queue_readback); // condition pass
        break;
    }
    /* ------------------------------------------ */
    case TestType::MOVE_HOMING: {
        bsStepperBoard::HomingConfig homing_config;
        homing_config.homing_mode           = HomingMode::SENSOR;
        homing_config.homing_sensor         = HomingSensor::STOP_L;
        homing_config.home_sensor_value     = 0;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = 360;
        homing_config.max_speed             = 5.0f;        // rpm
        homing_config.max_accel             = 2.0f / 1.0f; // rpm/s
        homing_config.homing_timeout_ms     = 5000;        // ms

        pr->bsSb->configureHoming(pr->driver_num, homing_config);
        bool moved = pr->bsSb->q_moveStepper_homing(pr->driver_num);

        // Readback
        uint16_t queue_readback;
        moved &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);              // read success
        HomingCode homing_code = pr->bsSb->decodeQueueReadback_homingCode(queue_readback); // retrieve homing code

        test_debug("%s\n", toString_homingCode(homing_code));
        break;
    }
    /* ------------------------------------------ */
    case TestType::MOVE_VIBRATION: {
        bool moved = pr->bsSb->q_moveStepper_vibration(pr->driver_num, 3, 1000, 1.0, false);

        test_debug("%d magnitude\n", pr->position_units);

        break;
    }
    /* ------------------------------------------ */
    case TestType::MOVE_INVERSE_TIME: {
        bool moved = pr->bsSb->q_moveStepper_inverseTime(pr->driver_num, pr->position_units, 5000);

        test_debug("%d units in 5000 ms\n", pr->position_units);

        // Readback
        uint16_t queue_readback;
        moved &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);  // read success
        moved &= pr->bsSb->verifyQueueReadback_execSuccessful(queue_readback); // condition pass
        break;
    }
    /* ------------------------------------------ */
    case TestType::M1_GATE_TEST: {
        uint8_t sensor_status = pr->bsSb->readSensors(pr->driver_num);
        bool    stop_l        = (sensor_status >> 0) & 1;

        if (stop_l != 0) {
            bsStepperBoard::HomingConfig homing_config;
            homing_config.homing_mode           = HomingMode::SENSOR;
            homing_config.homing_sensor         = HomingSensor::STOP_L;
            homing_config.home_sensor_value     = 0;
            homing_config.homing_offset_units   = 3;
            homing_config.homing_distance_units = 360;
            homing_config.max_speed             = 60.0f;        // rpm
            homing_config.max_accel             = 60.0f / 1.0f; // rpm/s

            pr->bsSb->configureHoming(pr->driver_num, homing_config);
            bool moved = pr->bsSb->q_moveStepper_homing(pr->driver_num);

            // Readback
            uint16_t queue_readback;
            moved &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);              // read success
            HomingCode homing_code = pr->bsSb->decodeQueueReadback_homingCode(queue_readback); // retrieve homing code

            test_debug("%s\n", toString_homingCode(homing_code));
        } else {
            float position = 120.0f * randomScaler();
            bool  moved    = pr->bsSb->q_moveStepper(pr->driver_num, static_cast<int32_t>(position));

            // Readback
            uint16_t queue_readback;
            moved &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);  // read success
            moved &= pr->bsSb->verifyQueueReadback_execSuccessful(queue_readback); // condition pass

            test_debug("%s\n", toString_execCode(static_cast<ExecCode>(queue_readback)));
        }
        break;
    }
    /* ------------------------------------------ */
    default: test_debug("?\n"); break;
    }
}

void testTask_RTOS(void *parameters) {
    auto *pr = static_cast<test_parameters_t *>(parameters);

    test_debug("[Drv%d] Running Test -> %s\n", pr->driver_num, toString_testType(pr->test_type));
    bsStepperBoard::DriverConfig drv_config    = getTestDriverConfig(pr->driver_type);
    bsStepperBoard::MotionConfig motion_config = getTestMotionConfig(pr->test_type);

    vTaskDelay(pdMS_TO_TICKS(2000));
    bool button_pressed;

    pr->bsSb->emergencyStop(pr->driver_num); // force reinit

    for (;;) {
        /* ====================================== Read ====================================== */
        DriverStatus status = pr->bsSb->readDriverStatus(pr->driver_num);
        test_debug("[Drv%d] (Status) %s | ", pr->driver_num, toString_stepperStatus(status));

        uint8_t sensor_reading = pr->bsSb->readSensors(pr->driver_num);
        test_debug("(Sensor) %d%d | ", sensor_reading > 1, sensor_reading);

        uint32_t pos_dev = pr->bsSb->readCurrentPosition(pr->driver_num);
        int32_t  speed   = pr->bsSb->readCurrentSpeed(pr->driver_num);
        test_debug("(Pos) %d | (Spd) %d | ", pos_dev, speed);

        /* ---------------------------------------------------------------------------------- */
        button_pressed = listenButton();

        switch (status) {
        /* ------------------------------------------ */
        case DriverStatus::STS_READY: {
            test_debug("(Do) move ->");

            if (pr->test_type == TestType::JOGGING) {
                if (button_pressed) {
                    bool moved = pr->bsSb->q_moveStepper(pr->driver_num, pr->position_units);

                    test_debug("(+%d ->", pr->position_units);
                    // Readback
                    uint16_t queue_readback;
                    moved &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);  // read success
                    moved &= pr->bsSb->verifyQueueReadback_execSuccessful(queue_readback); // condition pass

                    test_debug("%s\n", toString_execCode(static_cast<ExecCode>(queue_readback)));
                } else {
                    test_debug("Waiting button\n");
                }
            } else {
                onReady(pr);
            }

            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_BUSY: {

            if (pr->test_type == TestType::JOGGING) {
                test_debug("(Do) move ->");
                if (button_pressed) {
                    bool moved = pr->bsSb->q_moveStepper(pr->driver_num, pr->position_units);

                    test_debug("(+%d ->", pr->position_units);
                    // Readback
                    uint16_t queue_readback;
                    moved &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);  // read success
                    moved &= pr->bsSb->verifyQueueReadback_execSuccessful(queue_readback); // condition pass

                    test_debug("%s\n", toString_execCode(static_cast<ExecCode>(queue_readback)));
                } else {
                    pr->bsSb->rampStop(pr->driver_num);
                    test_debug("Waiting button\n");
                }
            } else {
                test_debug("(Do) nothing\n");
            }
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_HOMING: {
            test_debug("(Do) nothing\n");
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_VIBRATING: {
            test_debug("(Do) nothing\n");
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_CTRL_NOT_INIT: {
            bool inited = pr->bsSb->q_setAndInitDriver(pr->driver_num, drv_config);
            // Readback
            uint16_t queue_readback;
            inited &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);   // read success
            inited &= pr->bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass
            test_debug("(Do) drv_init -> %d\n", inited);
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_MOTION_NOT_INIT: {
            bool minited = pr->bsSb->q_setMotion(pr->driver_num, motion_config);
            // Readback
            uint16_t queue_readback;
            minited &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);   // read success
            minited &= pr->bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass
            test_debug("(Do) motion_init -> %d\n", minited);
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_STALL: {
            bool inited = pr->bsSb->q_setAndInitDriver(pr->driver_num, drv_config);
            // Readback
            uint16_t queue_readback;
            inited &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);   // read success
            inited &= pr->bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass
            test_debug("(Do) drv_init -> %d\n", inited);
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_POS_ERR: {
            uint32_t pos_dev = pr->bsSb->readEncoderPositionDeviation(pr->driver_num);
            bool     inited  = pr->bsSb->q_setAndInitDriver(pr->driver_num, drv_config);
            // Readback
            uint16_t queue_readback;
            inited &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);   // read success
            inited &= pr->bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass
            test_debug("(Do) drv_init -> %d\n", inited);
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_STOPPED: {
            bool inited = pr->bsSb->q_setAndInitDriver(pr->driver_num, drv_config);
            // Readback
            uint16_t queue_readback;
            inited &= pr->bsSb->getQueueReadback(pr->driver_num, &queue_readback);   // read success
            inited &= pr->bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass
            test_debug("(Do) drv_init -> %d\n", inited);
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_COIL_OL: {
            test_debug("(Do) nothing -> reboot required\n");
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_COIL_SHORT: {
            test_debug("(Do) nothing -> reboot required\n");
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_OVERTEMP: {
            test_debug("(Do) nothing -> reboot required\n");
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_TMC4361A_COMM_ERR: {
            test_debug("(Do) nothing -> reboot required\n");
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_TMC5160_COMM_ERR: {
            test_debug("(Do) nothing -> reboot required\n");
            break;
        }
        /* ------------------------------------------ */
        case DriverStatus::STS_NO_COMM: {
            test_debug("(Do) nothing -> reboot required\n");
            break;
        }
        /* ------------------------------------------ */
        default: {
            test_debug("(Do) nothing -> reboot required\n");
            break;
        }
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

/* ================================================================================== */
/*                        Cage Sequence (Without Motion Queue)                        */
/* ================================================================================== */

bsStepperBoard::HomingConfig getHomingConfig(Action action) {
    bsStepperBoard::HomingConfig homing_config;
    homing_config.homing_mode       = HomingMode::SENSOR;
    homing_config.home_sensor_value = SENSOR_TRIGGERED;
    homing_config.homing_timeout_ms = 15000;

    /* ------------------------------------------ */
    switch (action) {
    case Action::INPUT_Z_HOME: {
        homing_config.homing_sensor         = INPUT_Z_TOP_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = 360;
        homing_config.max_speed             = 20.0f;        // rpm
        homing_config.max_accel             = 20.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::INPUT_Z_RAISE: {
        homing_config.homing_sensor         = INPUT_Z_TOP_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = -360;
        homing_config.max_speed             = 30.0f;        // rpm
        homing_config.max_accel             = 30.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::INPUT_Z_LOWER: {
        homing_config.homing_sensor         = INPUT_Z_BTM_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = 360;
        homing_config.max_speed             = 60.0f;        // rpm
        homing_config.max_accel             = 60.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::OUTPUT_Z_HOME: {
        homing_config.homing_sensor         = OUTPUT_Z_TOP_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = 360;
        homing_config.max_speed             = 20.0f;        // rpm
        homing_config.max_accel             = 20.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::OUTPUT_Z_RAISE: {
        homing_config.homing_sensor         = OUTPUT_Z_TOP_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = -360;
        homing_config.max_speed             = 60.0f;        // rpm
        homing_config.max_accel             = 60.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::OUTPUT_Z_LOWER: {
        homing_config.homing_sensor         = OUTPUT_Z_BTM_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = 360;
        homing_config.max_speed             = 60.0f;        // rpm
        homing_config.max_accel             = 60.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::INPUT_X_HOME: {
        homing_config.homing_sensor         = INPUT_X_BELT_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = 36000;
        homing_config.max_speed             = 20.0f;        // rpm
        homing_config.max_accel             = 20.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::INPUT_X_TO_BELT: {
        homing_config.homing_sensor         = INPUT_X_BELT_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = 36000;
        homing_config.max_speed             = 80.0f;        // rpm
        homing_config.max_accel             = 80.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::INPUT_X_TO_MID: {
        homing_config.homing_sensor         = INPUT_X_MID_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = -36000;
        homing_config.max_speed             = 30.0f;        // rpm
        homing_config.max_accel             = 30.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::OUTPUT_X_HOME: {
        homing_config.homing_sensor         = OUTPUT_X_BELT_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = 36000;
        homing_config.max_speed             = 20.0f;        // rpm
        homing_config.max_accel             = 20.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::OUTPUT_X_TO_BELT: {
        homing_config.homing_sensor         = OUTPUT_X_BELT_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = 36000;
        homing_config.max_speed             = 30.0f;        // rpm
        homing_config.max_accel             = 30.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::OUTPUT_X_TO_MID: {
        homing_config.homing_sensor         = OUTPUT_X_MID_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = -36000;
        homing_config.max_speed             = 80.0f;        // rpm
        homing_config.max_accel             = 80.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::CAGE_Z_HOME: {
        homing_config.homing_sensor         = CAGE_Z_TOP_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = 36000;
        homing_config.max_speed             = 60.0f;        // rpm
        homing_config.max_accel             = 60.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::CAGE_Z_RAISE: {
        homing_config.homing_sensor         = CAGE_Z_TOP_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = 36000;
        homing_config.max_speed             = 60.0f;        // rpm
        homing_config.max_accel             = 60.0f / 1.0f; // rpm/s
        break;
    }
    /* ------------------------------------------ */
    case Action::CAGE_Z_LOWER: {
        homing_config.homing_sensor         = CAGE_Z_BTM_SENSOR;
        homing_config.homing_offset_units   = 3;
        homing_config.homing_distance_units = -36000;
        homing_config.max_speed             = 60.0f;        // rpm
        homing_config.max_accel             = 60.0f / 1.0f; // rpm/s
        break;
    }
    }

    return homing_config;
}

bool allDriverInit(bsStepperBoard *bsSb) {
    test_debug("Init all driver");
    bsStepperBoard::DriverConfig DRIVER_CONFIG = getTestDriverConfig(DriverType::OPEN_LOOP);

    bool success = true;

    // Init driver
    success &= bsSb->q_setAndInitDriver(INPUT_Z, DRIVER_CONFIG);
    success &= bsSb->q_setAndInitDriver(OUTPUT_Z, DRIVER_CONFIG);
    success &= bsSb->q_setAndInitDriver(INPUT_X, DRIVER_CONFIG);
    success &= bsSb->q_setAndInitDriver(OUTPUT_X, DRIVER_CONFIG);
    // success &= bsSb->q_setAndInitDriver(CAGE_Z, DRIVER_CONFIG);

    // Readback
    uint16_t queue_readback;
    success &= bsSb->getQueueReadback(INPUT_Z, &queue_readback);          // read success
    success &= bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass

    success &= bsSb->getQueueReadback(OUTPUT_Z, &queue_readback);         // read success
    success &= bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass

    success &= bsSb->getQueueReadback(INPUT_X, &queue_readback);          // read success
    success &= bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass

    success &= bsSb->getQueueReadback(OUTPUT_X, &queue_readback);         // read success
    success &= bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass

    // success &= bsSb->getQueueReadback(CAGE_Z, &queue_readback);           // read success
    // test_debug("%d", queue_readback);
    // success &= bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass
    test_debug("\n", queue_readback);

    return success;
}

bool run1Axis(bsStepperBoard *bsSb, uint8_t driver, Action action, const char *action_name) {
    test_debug("(%s) -> Start\n", action_name);
    bool success = true;

    success &= bsSb->configureHoming(driver, getHomingConfig(action));

    bsSb->q_moveStepper_homing(driver);

    // Wait 5ms
    vTaskDelay(pdMS_TO_TICKS(100));

    uint16_t rb0;
    success &= bsSb->getQueueReadback(driver, &rb0); // read success

    test_debug("(%d) -> %s\n", driver, toString_homingCode(static_cast<HomingCode>(rb0)));

    success &= rb0 == HomingCode::H_SUCCESS;

    test_debug("(%s) -> Outcome %d\n", action_name, success);

    return success;
}

bool run2Axis(bsStepperBoard *bsSb, uint8_t driver0, uint8_t driver1, Action action0, Action action1,
              const char *action_name) {
    test_debug("(%s) -> Start\n", action_name);
    bool success = true;

    success &= bsSb->configureHoming(driver0, getHomingConfig(action0));
    success &= bsSb->configureHoming(driver1, getHomingConfig(action1));

    bsSb->q_moveStepper_homing(driver0);
    bsSb->q_moveStepper_homing(driver1);

    // Wait 5ms
    vTaskDelay(pdMS_TO_TICKS(100));

    uint16_t rb0;
    uint16_t rb1;
    success &= bsSb->getQueueReadback(driver0, &rb0); // read success
    success &= bsSb->getQueueReadback(driver1, &rb1); // read success

    test_debug("(%d) -> %s\n", driver0, toString_homingCode(static_cast<HomingCode>(rb0)));
    test_debug("(%d) -> %s\n", driver1, toString_homingCode(static_cast<HomingCode>(rb1)));

    success &= rb0 == HomingCode::H_SUCCESS;
    success &= rb1 == HomingCode::H_SUCCESS;

    test_debug("(%s) -> Outcome %d\n", action_name, success);

    return success;
}

bool runSequence(Sequence sequence, bsStepperBoard *bsSb) {
    switch (sequence) {
    case Sequence::ALL_HOME: {
        bool succeed;

        succeed = allDriverInit(bsSb);
        if (!succeed) {
            test_debug("(All Home) configure driver failed.\n");
            return false;
        }

        // Raise Cage Z
        // succeed = run1Axis(bsSb, CAGE_Z, Action::CAGE_Z_HOME, "Home Cage Z");
        // if (!succeed) { return false; }

        // Raise Output Z and Input Z
        succeed = run2Axis(bsSb, OUTPUT_Z, INPUT_Z, Action::OUTPUT_Z_HOME, Action::INPUT_Z_HOME, "Raise Zs");
        if (!succeed) { return false; }

        // Move both forks to belt
        succeed = run2Axis(bsSb, OUTPUT_X, INPUT_X, Action::OUTPUT_X_HOME, Action::INPUT_X_HOME, "Home Xs");
        if (!succeed) { return false; }

        // Lower input fork
        succeed = run1Axis(bsSb, INPUT_Z, Action::INPUT_Z_LOWER, "Home Cage Z");
        if (!succeed) { return false; }

        return true;
    }
    case Sequence::SEQUENCE: {
        bool succeed;

        // Raise Cage Z & Output Z
        // succeed = run2Axis(bsSb, CAGE_Z, OUTPUT_Z, Action::CAGE_Z_RAISE, Action::OUTPUT_Z_RAISE, "Raise Cage Output
        // Zs");
        succeed = run1Axis(bsSb, OUTPUT_Z, Action::OUTPUT_Z_RAISE, "Raise Cage Output Zs");
        if (!succeed) { return false; }

        // Output X to cage
        succeed = run1Axis(bsSb, OUTPUT_X, Action::OUTPUT_X_TO_MID, "Output X to mid");
        if (!succeed) { return false; }

        // Lower Output Z
        succeed = run1Axis(bsSb, OUTPUT_Z, Action::OUTPUT_Z_LOWER, "Output Z lower");
        if (!succeed) { return false; }

        // Input X to mid, output X to belt
        succeed = run2Axis(bsSb, INPUT_X, OUTPUT_X, Action::INPUT_X_TO_MID, Action::OUTPUT_X_TO_BELT, "Pots in");
        if (!succeed) { return false; }

        // Raise input Z
        succeed = run1Axis(bsSb, INPUT_Z, Action::INPUT_Z_RAISE, "Input Z Raise");
        if (!succeed) { return false; }

        // Input X return to belt
        succeed = run1Axis(bsSb, INPUT_X, Action::INPUT_X_TO_BELT, "Input X to belt");
        if (!succeed) { return false; }

        // Input Z + Cage Lower
        // succeed = run2Axis(bsSb, INPUT_Z, CAGE_Z, Action::INPUT_Z_LOWER, Action::CAGE_Z_LOWER, "Z Cage lower");
        succeed = run1Axis(bsSb, INPUT_Z, Action::INPUT_Z_LOWER, "Z Cage lower");
        if (!succeed) { return false; }

        return true;
    }
    }
    return false;
}

void cageSequence_manualCall(void *parameters) {
    auto *bsSb = static_cast<bsStepperBoard *>(parameters);

    // Operation
    Sequence sequence = Sequence::ALL_HOME;
    bool     sequence_succeeded;

    bsSb->emergencyStop(0); // force reinit
    bsSb->emergencyStop(1); // force reinit
    bsSb->emergencyStop(2); // force reinit
    bsSb->emergencyStop(3); // force reinit
    bsSb->emergencyStop(4); // force reinit

    test_debug("Cage Sequence - Manual\n");
    for (;;) {
        sequence_succeeded = runSequence(sequence, bsSb);

        sequence = sequence_succeeded ? Sequence::SEQUENCE : Sequence::ALL_HOME;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ================================================================================== */
/*                           Cage Sequence With Motion Queue                          */
/* ================================================================================== */

void cageSequence_queue(void *parameters) {
    auto *bsSb = static_cast<bsStepperBoard *>(parameters);

    test_debug("Cage Sequence - Queue\n");
    for (;;) {
        if (bsSb->mq_startMotion(0)) {
            vTaskDelay(pdMS_TO_TICKS(100));

            QueueStatus q_sts;
            q_sts.is_executing = 1;

            while (q_sts.is_executing == 1) {
                q_sts = bsSb->getMotionQueueStatus();
                test_debug("E%d M%d S%d\n", q_sts.is_executing, q_sts.motion_num, q_sts.sequence_num);
            }

            MotionQueueRes res;
            bsSb->getMotionQueueReadback(&res);
            test_debug("Readback ->%d", res);

            if (res == MotionQueueRes::MQ_SUCCESS) {
                while (res == 1) {
                    bsSb->mq_startMotion(1);
                    vTaskDelay(pdMS_TO_TICKS(100));

                    q_sts.is_executing = 1;

                    while (q_sts.is_executing == 1) {
                        q_sts = bsSb->getMotionQueueStatus();
                        test_debug("E%d M%d S%d\n", q_sts.is_executing, q_sts.motion_num, q_sts.sequence_num);
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }

                    bsSb->getMotionQueueReadback(&res);
                    test_debug("Readback ->%d\n", res);
                }
            } else {
                test_debug("Failed\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ================================================================================== */
/*                         Cage Sequence with Preloaded Motion                        */
/* ================================================================================== */

bool allDriverInit_preloaded(bsStepperBoard *bsSb) {
    test_debug("Init all driver");

    bool success = true;

    // Init driver only
    success &= bsSb->q_initDriver(INPUT_Z);
    success &= bsSb->q_initDriver(OUTPUT_Z);
    success &= bsSb->q_initDriver(INPUT_X);
    success &= bsSb->q_initDriver(OUTPUT_X);
    // success &= bsSb->q_initDriver(CAGE_Z);

    // Readback
    uint16_t queue_readback;
    success &= bsSb->getQueueReadback(INPUT_Z, &queue_readback);          // read success
    success &= bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass

    success &= bsSb->getQueueReadback(OUTPUT_Z, &queue_readback);         // read success
    success &= bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass

    success &= bsSb->getQueueReadback(INPUT_X, &queue_readback);          // read success
    success &= bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass

    success &= bsSb->getQueueReadback(OUTPUT_X, &queue_readback);         // read success
    success &= bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass

    // success &= bsSb->getQueueReadback(CAGE_Z, &queue_readback);           // read success
    // test_debug("%d", queue_readback);
    // success &= bsSb->verifyQueueReadback_writeSuccessful(queue_readback); // condition pass
    test_debug("\n", queue_readback);

    return success;
}

bool runSequence_preloaded(Sequence sequence, bsStepperBoard *bsSb) {
    uint8_t  drv0;
    uint8_t  drv1;
    uint16_t rb0;
    uint16_t rb1;

    switch (sequence) {
    case Sequence::ALL_HOME: {
        bool succeed;
        test_debug("\n(All Home)\n");
        succeed = allDriverInit_preloaded(bsSb);
        if (!succeed) {
            test_debug("Configure driver failed.\n");
            return false;
        }
        // succeed &= bsSb->q_movePreloaded(0, &drv0);
        // succeed &= bsSb->getQueueReadback(drv0, &rb0);
        // printf("Cage Z Home -> %s\n", toString_homingCode(static_cast<HomingCode>(rb0)));
        // if (!succeed) { return false; }
        /* ---------------------------------------------------------------------------------- */
        succeed &= bsSb->q_movePreloaded(12, &drv0);
        succeed &= bsSb->q_movePreloaded(6, &drv1);
        succeed &= bsSb->getQueueReadback(drv0, &rb0);
        succeed &= bsSb->getQueueReadback(drv1, &rb1);

        printf("Output Z Home -> %s\n", toString_homingCode(static_cast<HomingCode>(rb0)));
        printf("Input Z Home -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));
        succeed &= rb0 == HomingCode::H_SUCCESS;
        succeed &= rb1 == HomingCode::H_SUCCESS;
        if (!succeed) { return false; }
        /* ---------------------------------------------------------------------------------- */
        succeed &= bsSb->q_movePreloaded(9, &drv0);
        succeed &= bsSb->q_movePreloaded(3, &drv1);
        succeed &= bsSb->getQueueReadback(drv0, &rb0);
        succeed &= bsSb->getQueueReadback(drv1, &rb1);

        printf("Output X Home -> %s\n", toString_homingCode(static_cast<HomingCode>(rb0)));
        printf("Input X Home -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));
        succeed &= rb0 == HomingCode::H_SUCCESS;
        succeed &= rb1 == HomingCode::H_SUCCESS;
        if (!succeed) { return false; }
        /* ---------------------------------------------------------------------------------- */
        succeed &= bsSb->q_movePreloaded(7, &drv0);
        succeed &= bsSb->getQueueReadback(drv0, &rb0);

        printf("Input Z Lower -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));
        succeed &= rb0 == HomingCode::H_SUCCESS;
        if (!succeed) { return false; }
        /* ---------------------------------------------------------------------------------- */
        return true;
    }
    case Sequence::SEQUENCE: {
        test_debug("\n(Sequence)\n");
        bool succeed = true;

        // succeed &= bsSb->q_movePreloaded(2, &drv0);
        succeed &= bsSb->q_movePreloaded(14, &drv1);
        // succeed &= bsSb->getQueueReadback(drv0, &rb0);
        succeed &= bsSb->getQueueReadback(drv1, &rb1);

        // printf("Raise Cage Z -> %s\n", toString_homingCode(static_cast<HomingCode>(rb0)));
        printf("Raise Output Z -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));
        // succeed &= rb0 == HomingCode::H_SUCCESS;
        succeed &= rb1 == HomingCode::H_SUCCESS;
        if (!succeed) { return false; }
        /* ---------------------------------------------------------------------------------- */
        succeed &= bsSb->q_movePreloaded(11, &drv0);
        succeed &= bsSb->getQueueReadback(drv0, &rb0);

        printf("Output X to Mid -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));
        succeed &= rb0 == HomingCode::H_SUCCESS;
        if (!succeed) { return false; }
        /* ---------------------------------------------------------------------------------- */
        succeed &= bsSb->q_movePreloaded(13, &drv0);
        succeed &= bsSb->getQueueReadback(drv0, &rb0);

        printf("Output Z Lower -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));
        succeed &= rb0 == HomingCode::H_SUCCESS;
        if (!succeed) { return false; }
        /* ---------------------------------------------------------------------------------- */
        succeed &= bsSb->q_movePreloaded(5, &drv0);
        succeed &= bsSb->q_movePreloaded(10, &drv1);
        succeed &= bsSb->getQueueReadback(drv0, &rb0);
        succeed &= bsSb->getQueueReadback(drv1, &rb1);

        printf("Input X to Mid -> %s\n", toString_homingCode(static_cast<HomingCode>(rb0)));
        printf("Output X to Belt -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));
        succeed &= rb0 == HomingCode::H_SUCCESS;
        succeed &= rb1 == HomingCode::H_SUCCESS;
        if (!succeed) { return false; }
        /* ---------------------------------------------------------------------------------- */
        succeed &= bsSb->q_movePreloaded(8, &drv0);
        succeed &= bsSb->getQueueReadback(drv0, &rb0);

        printf("Raise input Z -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));
        succeed &= rb0 == HomingCode::H_SUCCESS;
        if (!succeed) { return false; }
        /* ---------------------------------------------------------------------------------- */
        succeed &= bsSb->q_movePreloaded(4, &drv0);
        succeed &= bsSb->getQueueReadback(drv0, &rb0);

        printf("Input X to belt -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));
        succeed &= rb0 == HomingCode::H_SUCCESS;
        if (!succeed) { return false; }
        /* ---------------------------------------------------------------------------------- */
        succeed &= bsSb->q_movePreloaded(7, &drv0);
        // succeed &= bsSb->q_movePreloaded(1, &drv1);
        succeed &= bsSb->getQueueReadback(drv0, &rb0);
        // succeed &= bsSb->getQueueReadback(drv1, &rb1);

        printf("Lower Input Z -> %s\n", toString_homingCode(static_cast<HomingCode>(rb0)));
        // printf("Lower Cage Z -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));
        succeed &= rb0 == HomingCode::H_SUCCESS;
        // succeed &= rb1 == HomingCode::H_SUCCESS;
        if (!succeed) { return false; }

        return true;
    }
    }
    return false;
}

void cageSequence_preloadedMotion(void *parameters) {
    auto *bsSb = static_cast<bsStepperBoard *>(parameters);

    Sequence sequence = Sequence::ALL_HOME;
    bool     sequence_succeeded;

    test_debug("Cage Sequence - Preloaded\n");
    for (;;) {
        sequence_succeeded = runSequence_preloaded(sequence, bsSb);

        sequence = sequence_succeeded ? Sequence::SEQUENCE : Sequence::ALL_HOME;

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}