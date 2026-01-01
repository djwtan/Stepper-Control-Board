#include "bsSb.h"

/* ================================================================================== */
/*                                     Board Info                                     */
/* ================================================================================== */

bool bsStepperBoard::readFirmwareVersion(std::string **ver) {
    return fetchString(Reg_Board::FIRMWARE_VER, 0, ver) == bssb_response_t::SUCCESS;
}

/* ================================================================================== */
/*                                       SD Card                                      */
/* ================================================================================== */

bool bsStepperBoard::readDriverConfig(uint8_t driver_num, std::string **file_content) {
    return fetchString(Reg_Board::SD_FETCH_DRV_CONFIG, driver_num, file_content) == bssb_response_t::SUCCESS;
}

bool bsStepperBoard::readMotionConfig(uint8_t motion_index, std::string **file_content) {
    return fetchString(Reg_Board::SD_FETCH_MOTION_CONFIG, motion_index, file_content) == bssb_response_t::SUCCESS;
}

bool bsStepperBoard::readSequenceConfig(uint8_t sequence_index, std::string **file_content) {
    return fetchString(Reg_Board::SD_FETCH_SEQUENCE, sequence_index, file_content) == bssb_response_t::SUCCESS;
}

bool bsStepperBoard::readBuffer(std::string **buf_content) {
    return fetchString(Reg_Board::READ_BUFFER, 0, buf_content) == bssb_response_t::SUCCESS;
}

bool bsStepperBoard::readPath(std::string **path) {
    return fetchString(Reg_Board::READ_CURRENT_PATH, 0, path) == bssb_response_t::SUCCESS;
}

bool bsStepperBoard::readFileAtPath(std::string *path, std::string **file_content) {
    uint8_t rslt = 1;

    rslt *= static_cast<uint8_t>(transferStringToBuffer(path));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::CP_BUFFER_AS_PATH, 0));

    if (rslt != 1) { return false; } // Early return on failure

    rslt *= static_cast<uint8_t>(fetchString(Reg_Board::SD_READ_FILE_AT_PATH, 0, file_content));

    return (rslt == 1);
}

bool bsStepperBoard::deleteFileAtPath(std::string *path) {
    uint8_t rslt = 1;

    rslt *= static_cast<uint8_t>(transferStringToBuffer(path));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::CP_BUFFER_AS_PATH, 0));

    if (rslt != 1) { return false; } // Early return on failure

    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::SD_DELETE_FILE_AT_PATH, 0));

    return (rslt == 1);
}

bool bsStepperBoard::writeToFileAtPath(std::string *path, std::string *content) {
    uint8_t rslt = 1;

    rslt *= static_cast<uint8_t>(transferStringToBuffer(path));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::CP_BUFFER_AS_PATH, 0));

    if (rslt != 1) { return false; } // Early return on failure

    rslt *= static_cast<uint8_t>(transferStringToBuffer(content));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::SD_SAVE_FILE_AT_PATH, 0));

    return (rslt == 1);
}

/* ================================================================================== */
/*                                       Decode                                       */
/* ================================================================================== */

void bsStepperBoard::clearQueuedReadback(uint8_t driver_num) {
    uint16_t dummy;

    readStepper(driver_num, RegS_R::READBACK, &dummy);
}

bool bsStepperBoard::getQueueReadback(uint8_t driver_num, uint16_t *response) {
    uint16_t        res           = Response::READ_FAIL;
    bssb_response_t bssb_response = bssb_response_t::SUCCESS;

    while ((res == Response::READ_FAIL) && (bssb_response == bssb_response_t::SUCCESS)) {
        bssb_response = readStepper(driver_num, RegS_R::READBACK, &res);

        // Sleep (if provided)
        asyncSleep_ms(1);
    }

    bool read_success = bssb_response == bssb_response_t::SUCCESS;
    *response         = res;
    return read_success;
}

HomingCode bsStepperBoard::decodeQueueReadback_homingCode(uint16_t response) {
    return static_cast<HomingCode>(response);
}

bool bsStepperBoard::verifyQueueReadback_execSuccessful(uint16_t response) {
    if (response == Response::READ_FAIL) { return false; }
    return (response == ExecCode::E_SUCCESS);
}

bool bsStepperBoard::verifyQueueReadback_writeSuccessful(uint16_t response) { return (response == Response::SUCCESS); }

/* ================================================================================== */
/*                                    Move Commands                                   */
/* ================================================================================== */

bool bsStepperBoard::q_movePreloaded(uint16_t index, uint8_t *driver_num) {
    uint16_t response;
    uint16_t cmd_res;

    // Write
    bssb_response_t bssb_response = writeBoard(Reg_Board::RUN_PRELOAD, index, nullptr, &response);

    // Expect driver num (2 byte) + response (2 byte)
    *driver_num = static_cast<uint8_t>(response >> 8);
    cmd_res     = response & 0x00FF;
    bssb_debug("%s\n", toString_preloadPutQueueRes(static_cast<PreloadPutQueueRes>(cmd_res)));

    return (bssb_response == bssb_response_t::SUCCESS) && (cmd_res == PreloadPutQueueRes::P_SUCCESS);
}

bool bsStepperBoard::q_moveStepper(uint8_t driver_num, int32_t position_units) {
    uint8_t rslt = 1;

    // Split position into two words
    UInt32Parts position_word = int32ToHighAndLowWords(position_units);

    // Write position
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::MOVE_UNIT_HIGH, position_word.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::MOVE_UNIT_LOW, position_word.low));

    // Start motion
    rslt *= static_cast<uint8_t>(writeIntoQueue(driver_num, ControlWord::MOVE));

    bssb_debug("(b%d-d%d) q_moveStepper -> %d\n", m_board_num, driver_num, rslt);

    return (rslt == 1);
}

bool bsStepperBoard::q_moveStepper_homing(uint8_t driver_num) {
    uint8_t rslt = 1;

    // Start motion
    rslt *= static_cast<uint8_t>(writeIntoQueue(driver_num, ControlWord::MOVE_HOMING));

    bssb_debug("(b%d-d%d) q_moveStepper_homing -> %d\n", m_board_num, driver_num, rslt);

    return (rslt == 1);
}

bool bsStepperBoard::q_moveStepper_inverseTime(uint8_t driver_num, int32_t position_units, uint32_t time_ms) {

    UInt32Parts position_word = int32ToHighAndLowWords(position_units);
    UInt32Parts time_word     = int32ToHighAndLowWords(time_ms);

    uint8_t rslt = 1;
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::MOVE_UNIT_HIGH, position_word.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::MOVE_UNIT_LOW, position_word.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::MOVE_TIME_MS_HIGH, time_word.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::MOVE_TIME_MS_LOW, time_word.low));

    // Start motion
    rslt *= static_cast<uint8_t>(writeIntoQueue(driver_num, ControlWord::MOVE_INVERSE_TIME));

    bssb_debug("(b%d-d%d) q_moveStepper_inverseTime -> %d\n", m_board_num, driver_num, rslt);

    return (rslt == 1);
}

bool bsStepperBoard::q_moveStepper_vibration(uint8_t driver_num, int32_t position_units, uint16_t iterations,
                                             float diminishing_factor, bool loop) {
    // Value check
    if (diminishing_factor > 1.0) { return false; }

    uint16_t    dim_factor_w  = static_cast<uint16_t>(diminishing_factor * 1000);
    UInt32Parts position_word = int32ToHighAndLowWords(position_units);

    uint8_t rslt = 1;
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::MOVE_UNIT_HIGH, position_word.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::MOVE_UNIT_LOW, position_word.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::MOVE_VIB_I, iterations));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::MOVE_VIB_DIM_FACTOR, dim_factor_w));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::MOVE_VIB_LOOP, static_cast<uint16_t>(loop)));

    // Start motion
    rslt *= static_cast<uint8_t>(writeIntoQueue(driver_num, ControlWord::MOVE_VIBRATION));

    bssb_debug("(b%d-d%d) q_moveStepper_inverseTime -> %d\n", m_board_num, driver_num, rslt);

    return (rslt == 1);
}

/* ================================================================================== */
/*                                    Control Word                                    */
/* ================================================================================== */

bool bsStepperBoard::rampStop(uint8_t driver_num) {
    bssb_response_t bssb_response = writeStepper(driver_num, RegS_W::CONTROL_WORD, ControlWord::RAMP_STOP);

    return (bssb_response == bssb_response_t::SUCCESS);
}

bool bsStepperBoard::emergencyStop(uint8_t driver_num) {
    bssb_response_t bssb_response = writeStepper(driver_num, RegS_W::CONTROL_WORD, ControlWord::EMERGENCY_STOP);

    return (bssb_response == bssb_response_t::SUCCESS);
}

bool bsStepperBoard::resetPosition(uint8_t driver_num) {
    bssb_response_t bssb_response = writeStepper(driver_num, RegS_W::CONTROL_WORD, ControlWord::SET_ZERO);

    return (bssb_response == bssb_response_t::SUCCESS);
}

bool bsStepperBoard::enableDriver(uint8_t driver_num) {
    bssb_response_t bssb_response = writeStepper(driver_num, RegS_W::CONTROL_WORD, ControlWord::ENABLE_DRIVER);

    return (bssb_response == bssb_response_t::SUCCESS);
}

bool bsStepperBoard::releaseDriver(uint8_t driver_num) {
    bssb_response_t bssb_response = writeStepper(driver_num, RegS_W::CONTROL_WORD, ControlWord::RELEASE_DRIVER);

    return (bssb_response == bssb_response_t::SUCCESS);
}

/* ================================================================================== */
/*                                    Stepper Read                                    */
/* ================================================================================== */

DriverStatus bsStepperBoard::readDriverStatus(uint8_t driver_num) {
    uint16_t        status_response;
    bssb_response_t bssb_response = readStepper(driver_num, RegS_R::STATUS, &status_response);

    if (bssb_response != bssb_response_t::SUCCESS) { return DriverStatus::STS_NO_COMM; }

    return static_cast<DriverStatus>(status_response);
}

int32_t bsStepperBoard::readCurrentPosition(uint8_t driver_num) {
    uint32_t response;

    readStepper32b(driver_num, RegS_R::INTERNAL_POSITION, &response);

    return static_cast<int32_t>(response);
}

int32_t bsStepperBoard::readEncoderPosition(uint8_t driver_num) {
    uint32_t response;

    readStepper32b(driver_num, RegS_R::ENCODER_POSITION, &response);

    return static_cast<int32_t>(response);
}

int32_t bsStepperBoard::readCurrentSpeed(uint8_t driver_num) {
    uint32_t response;

    readStepper32b(driver_num, RegS_R::CURRENT_SPEED, &response);

    return static_cast<int32_t>(response);
}

uint8_t bsStepperBoard::readSensors(uint8_t driver_num) {
    uint32_t response;

    readStepper32b(driver_num, RegS_R::SENSOR_STATUS, &response);

    return static_cast<int32_t>(response);
}

int32_t bsStepperBoard::readEncoderPositionDeviation(uint8_t driver_num) {
    uint32_t response;

    readStepper32b(driver_num, RegS_R::ENCODER_POS_DEV, &response);

    return static_cast<int32_t>(response);
}

/* ================================================================================== */
/*                                   Motion Planner                                   */
/* ================================================================================== */

QueueStatus bsStepperBoard::getMotionQueueStatus() {
    uint16_t        status_response = 0xFFFF;
    bssb_response_t bssb_response   = readBoard(Reg_Board::QSTATUS, &status_response);

    QueueStatus queue_status;
    queue_status.is_executing = (status_response >> 15) & 0x1;
    queue_status.motion_num   = (status_response >> 11) & 0xF;
    queue_status.sequence_num = status_response & 0x7FF;

    return queue_status;
}

bool bsStepperBoard::mq_startMotion(uint16_t motion_num) {
    bssb_debug("(b%d) mq_startMotion ->", m_board_num);
    if (motion_num > MAX_MOTIONS && motion_num != TEST_MOTION_NUM) {
        bssb_debug("Bad motion_num input!\n");
        return false;
    }

    uint16_t res = PutQueueRes::Q_FULL;

    bssb_response_t bssb_response = writeBoard(Reg_Board::MOTION_CONTROL_WORD, motion_num, nullptr, &res);

    bssb_debug("%s\n", toString_putQueueRes(static_cast<PutQueueRes>(res)));

    return (bssb_response == bssb_response_t::SUCCESS) && (res == PutQueueRes::Q_SUCCESS);
}

bool bsStepperBoard::abortMotion() {
    bssb_response_t bssb_response = writeBoard(Reg_Board::MOTION_CONTROL_WORD, MotionControlWord::ABORT);

    return (bssb_response == bssb_response_t::SUCCESS);
}

void bsStepperBoard::clearMotionQueueReadback() {
    uint16_t dummy;
    readBoard(Reg_Board::MOTION_READBACK, &dummy);
}

bool bsStepperBoard::getMotionQueueReadback(MotionQueueRes *mq_res) {
    uint16_t        res           = Response::READ_FAIL;
    bssb_response_t bssb_response = bssb_response_t::SUCCESS;

    while ((res == Response::READ_FAIL) && (bssb_response == bssb_response_t::SUCCESS)) {
        bssb_response = readBoard(Reg_Board::MOTION_READBACK, &res);

        // Sleep (if provided)
        asyncSleep_ms(1);
    }

    *mq_res = static_cast<MotionQueueRes>(res);

    return (bssb_response == bssb_response_t::SUCCESS);
}

bool bsStepperBoard::writeQueueItem(uint16_t motion_num, uint16_t item_num, uint16_t sequence_number,
                                    QueueItemParameters parameters) {
    bssb_debug("(b%d) writeQueueItem ->", m_board_num);
    // Data check
    if (motion_num > MAX_MOTIONS && motion_num != TEST_MOTION_NUM) {
        bssb_debug("Bad motion_num input!\n");
        return false;
    }
    if (item_num > MAX_ITEMS_PER_MOTION) {
        bssb_debug("Bad item_num input!\n");
        return false;
    }

    // Select motion and queue number
    uint16_t select_word = constructSelectWord(static_cast<uint8_t>(motion_num), static_cast<uint8_t>(item_num));

    uint8_t rslt = 1;

    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::SELECT_MOTION_AND_QUEUE_ITEM, select_word));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_SEQUENCE_NUMBER, sequence_number));

    if (rslt != 1) {
        bssb_debug("Failed before write!\n");
        return false;
    }

    // Write
    return writeQueueItemParameters(parameters);
}

bool bsStepperBoard::mq_startTestQueueItem() { return mq_startMotion(TEST_MOTION_NUM); }

bool bsStepperBoard::writeTestQueueItem(QueueItemParameters parameters) {
    return writeQueueItem(TEST_MOTION_NUM, 0, 0, parameters);
}

/* ================================================================================== */
/*                                       Driver                                       */
/* ================================================================================== */

bool bsStepperBoard::q_setAndInitDriver(uint8_t driver_num, DriverConfig config) {
    uint8_t rslt = 1;

    uint16_t holding_current_scaled  = getCurrentScaler(config.holding_current);
    uint16_t peak_rms_current_scaled = getCurrentScaler(config.peak_rms_current);

    // Write settings
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::DRV_MSTEP_PER_FS, config.mstep_per_fs));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::DRV_FS_PER_REV, config.fs_per_rev));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::POS_UNIT_PER_REV, config.unit_per_rev));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::CURRENT_HOLD, holding_current_scaled));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::CURRENT_RUN, peak_rms_current_scaled));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::STOP_ON_STALL_ENABLE, config.stop_on_stall_enable));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::STOP_ON_STALL_THRESH, config.stop_on_stall_thresh));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::CL_ENABLE, config.cl_enable));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::CL_ENABLE_PID, config.cl_enable_pid));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::CL_ENC_IN_RES, config.cl_enc_in_res));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::CL_TOLERANCE, config.cl_tolerance));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::STEALTH_CHOP_THRESH, config.stealth_chop_thresh));

    // Initialize
    rslt *= static_cast<uint8_t>(writeIntoQueue(driver_num, ControlWord::INIT));

    bssb_debug("(b%d-d%d) Set and init driver -> %d\n", m_board_num, driver_num, rslt);

    return (rslt == 1);
}

bool bsStepperBoard::q_initDriver(uint8_t driver_num) {
    uint8_t rslt = 1;

    // Initialize
    rslt *= static_cast<uint8_t>(writeIntoQueue(driver_num, ControlWord::INIT));

    bssb_debug("(b%d-d%d) Set and init driver -> %d\n", m_board_num, driver_num, rslt);

    return (rslt == 1);
}

/* ================================================================================== */
/*                                       Motion                                       */
/* ================================================================================== */

bool bsStepperBoard::q_setMotion(uint8_t driver_num, MotionConfig config) {
    uint8_t rslt = 1;

    // Write settings
    rslt *= static_cast<uint8_t>(
        writeStepper(driver_num, RegS_W::MOVE_ALLOW_WRITE_MOTION_WHEN_BUSY, config.allow_write_motion_when_busy));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::MOVE_RESET_MOTION_CONF_AFTER_EACH_MOVE,
                                              config.reset_motion_conf_after_each_move));

    // Motion Settings
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::POS_MODE, config.pos_mode));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::POS_FOLLOW_MODE, config.follow_mode));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::RAMP_MODE, config.ramp_mode));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::RAMP_TYPE, config.ramp_type));

    // Motion Profile
    UInt32Parts max_speed   = floatToHighAndLowWords(config.max_speed);
    UInt32Parts start_speed = floatToHighAndLowWords(config.start_speed);
    UInt32Parts stop_speed  = floatToHighAndLowWords(config.stop_speed);
    UInt32Parts break_speed = floatToHighAndLowWords(config.break_speed);
    UInt32Parts max_accel   = floatToHighAndLowWords(config.max_accel);
    UInt32Parts max_decel   = floatToHighAndLowWords(config.max_decel);
    UInt32Parts start_accel = floatToHighAndLowWords(config.start_accel);
    UInt32Parts final_decel = floatToHighAndLowWords(config.final_decel);
    UInt32Parts bow1        = floatToHighAndLowWords(config.bow1);
    UInt32Parts bow2        = floatToHighAndLowWords(config.bow2);
    UInt32Parts bow3        = floatToHighAndLowWords(config.bow3);
    UInt32Parts bow4        = floatToHighAndLowWords(config.bow4);

    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::SPEED_MAX_H, max_speed.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::SPEED_MAX_L, max_speed.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::SPEED_START_H, start_speed.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::SPEED_START_L, start_speed.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::SPEED_STOP_H, stop_speed.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::SPEED_STOP_L, stop_speed.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::SPEED_BREAK_H, break_speed.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::SPEED_BREAK_L, break_speed.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::ACC_MAX_ACCEL_H, max_accel.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::ACC_MAX_ACCEL_L, max_accel.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::ACC_MAX_DECEL_H, max_decel.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::ACC_MAX_DECEL_L, max_decel.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::ACC_START_ACCEL_H, start_accel.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::ACC_START_ACCEL_L, start_accel.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::ACC_FINAL_DECEL_H, final_decel.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::ACC_FINAL_DECEL_L, final_decel.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::BOW1_H, bow1.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::BOW1_L, bow1.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::BOW2_H, bow2.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::BOW2_L, bow2.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::BOW3_H, bow3.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::BOW3_L, bow3.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::BOW4_H, bow4.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::BOW4_L, bow4.low));

    // Initialize
    rslt *= static_cast<uint8_t>(writeIntoQueue(driver_num, ControlWord::SET_MOTION));

    bssb_debug("(b%d-d%d) Set motion -> %d\n", m_board_num, driver_num, rslt);

    return (rslt == 1);
}

/* ================================================================================== */
/*                                       Homing                                       */
/* ================================================================================== */

bool bsStepperBoard::configureHoming(uint8_t driver_num, HomingConfig config) {
    uint8_t rslt = 1;

    // Write settings
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_HOMING_MODE, config.homing_mode));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_HOMING_SENSOR, config.homing_sensor));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_SENSOR_HOME_VALUE, config.home_sensor_value));

    // Homing Profile
    UInt32Parts homing_distance = int32ToHighAndLowWords(config.homing_distance_units);
    UInt32Parts homing_offset   = int32ToHighAndLowWords(config.homing_offset_units);
    UInt32Parts homing_timeout  = int32ToHighAndLowWords(config.homing_timeout_ms);
    UInt32Parts max_speed       = floatToHighAndLowWords(config.max_speed);
    UInt32Parts max_accel       = floatToHighAndLowWords(config.max_accel);
    UInt32Parts max_decel       = floatToHighAndLowWords(config.max_decel);

    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_MAX_FIND_H, homing_distance.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_MAX_FIND_L, homing_distance.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_OFFSET_H, homing_offset.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_OFFSET_L, homing_offset.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_TIMEOUT_MS_H, homing_timeout.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_TIMEOUT_MS_L, homing_timeout.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_MAX_SPEED_H, max_speed.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_MAX_SPEED_L, max_speed.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_MAX_ACCEL_H, max_accel.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_MAX_ACCEL_L, max_accel.low));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_MAX_DECEL_H, max_decel.high));
    rslt *= static_cast<uint8_t>(writeStepper(driver_num, RegS_W::HOME_MAX_DECEL_L, max_decel.low));

    bssb_debug("(b%d-d%d) Configure homing -> %d\n", m_board_num, driver_num, rslt);

    return (rslt == 1);
}

/* ---------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------- */
/* ===================================== Private ==================================== */
/* ---------------------------------------------------------------------------------- */
/* ---------------------------------------------------------------------------------- */

bool bsStepperBoard::writeQueueItemParameters(QueueItemParameters parameters) {
    uint8_t rslt = 1;

    // Motion Settings
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_MOTION_TYPE, parameters.motion_type));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_STEPPER_NUM, parameters.driver_num));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_HOMING_MODE, parameters.homing_mode));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_HOMING_SENSOR, parameters.homing_sensor));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_SENSOR_HOME_VALUE, parameters.sensor_home_value));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_RAMP_TYPE, parameters.ramp_type));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_USE_INVERSE_TIME, parameters.use_inverse_time));

    // Motion Profile
    UInt32Parts max_speed = floatToHighAndLowWords(parameters.max_speed);
    UInt32Parts max_accel = floatToHighAndLowWords(parameters.max_accel);
    UInt32Parts max_decel = floatToHighAndLowWords(parameters.max_decel);
    UInt32Parts bow1      = floatToHighAndLowWords(parameters.bow1);
    UInt32Parts bow2      = floatToHighAndLowWords(parameters.bow2);
    UInt32Parts bow3      = floatToHighAndLowWords(parameters.bow3);
    UInt32Parts bow4      = floatToHighAndLowWords(parameters.bow4);
    UInt32Parts time_ms   = int32ToHighAndLowWords(parameters.time_ms);
    UInt32Parts position  = int32ToHighAndLowWords(parameters.position);
    UInt32Parts offset    = int32ToHighAndLowWords(parameters.offset);

    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_MAX_SPEED_H, max_speed.high));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_MAX_SPEED_L, max_speed.low));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_ACCEL_H, max_accel.high));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_ACCEL_L, max_accel.low));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_DECEL_H, max_decel.high));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_DECEL_L, max_decel.low));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_BOW1_H, bow1.high));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_BOW1_L, bow1.low));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_BOW2_H, bow2.high));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_BOW2_L, bow2.low));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_BOW3_H, bow3.high));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_BOW3_L, bow3.low));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_BOW4_H, bow4.high));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_BOW4_L, bow4.low));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_TIME_H, time_ms.high));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_TIME_L, time_ms.low));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_POSITION_H, position.high));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_POSITION_L, position.low));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_OFFSET_H, offset.high));
    rslt *= static_cast<uint8_t>(writeBoard(Reg_Board::QITEM_OFFSET_L, offset.low));

    bssb_debug("%d\n", rslt);

    return (rslt == 1);
}

bool bsStepperBoard::writeIntoQueue(uint8_t driver_num, uint16_t control_word) {
    uint16_t res;

    bssb_response_t bssb_response = writeStepper(driver_num, RegS_W::CONTROL_WORD, control_word, nullptr, &res);

    bssb_debug("%s\n", toString_putQueueRes(static_cast<PutQueueRes>(res)));

    return (bssb_response == bssb_response_t::SUCCESS) && (res == PutQueueRes::Q_SUCCESS);
}

/* ================================================================================== */
/*                              Driver Response Verifiers                             */
/* ================================================================================== */

bool bsStepperBoard::getReadback_writeSuccessful(uint16_t response) { return (response == Response::SUCCESS); }

bool bsStepperBoard::getReadback_putQueueSuccessful(uint16_t response) { return (response == PutQueueRes::Q_SUCCESS); }

/* ================================================================================== */
/*                                    Communication                                   */
/* ================================================================================== */

bool bsStepperBoard::CANTransfer(uint32_t identifier, uint32_t word, uint32_t *response_buffer, int buffer_size) {
    // Acquire mutex (if provided)
    if (m_can_mutex != nullptr) m_can_mutex(1);

    // Transmit
    m_can_transmit(identifier, word, 4);

    // Receive
    int count = m_can_receive(response_buffer, buffer_size);

    // Release mutex (if provided)
    if (m_can_mutex != nullptr) m_can_mutex(0);

    asyncSleep_ms(1);

    return (count > 0);
}

bool bsStepperBoard::transferStringToBuffer(std::string *content) {
    // Clear buffer
    if (writeBoard(Reg_Board::BUF_CLEAR, 0) != bssb_response_t::SUCCESS) { return false; }

    // Write Start Word
    if (writeBuffer(START_WORD, true) != bssb_response_t::SUCCESS) { return false; }

    // Pack into 32-bit words (little-endian)
    const char *data       = content->c_str();
    size_t      len        = strlen(data);
    uint32_t    word       = 0;
    int         byte_count = 0;

    for (size_t i = 0; i < len; i++) {
        uint8_t c = (uint8_t)data[i];
        word |= ((uint32_t)c) << (8 * byte_count);
        byte_count++;

        // Full 4-byte word → transmit
        if (byte_count == 4) {
            if (writeBuffer(word, false) != bssb_response_t::SUCCESS) { return false; }
            word       = 0;
            byte_count = 0;
        }
    }

    // last partial word
    if (byte_count > 0) {
        writeBuffer(word, false);
        len++;
    }

    // Write End Word
    if (writeBuffer(END_WORD, true) != bssb_response_t::SUCCESS) { return false; }

    // Confirm with buffer length
    uint16_t buf_len = 0;
    if (readBoard(Reg_Board::BUF_LEN, &buf_len) != bssb_response_t::SUCCESS) { return false; }

    // buf_len is multiple of 4
    return (buf_len <= len + 3);
}

bsStepperBoard::bssb_response_t bsStepperBoard::fetchString(uint16_t reg_num, uint16_t data,
                                                            std::string **file_content) {
    // Sanity check
    if (m_can_transmit == nullptr || m_can_receive == nullptr || m_async_sleep == nullptr) {
        assert(false && "Not implemented");
        return bssb_response_t::NOT_INIT;
    }

    // Create CAN word
    uint32_t word = 0;
    word |= (reg_num & 0x0FFF) << 16;
    word |= (data & 0xFFFF);

    // Destination CAN ID
    uint32_t identifier = 0x0300 | (m_board_num & 0xFF);

    if (m_can_mutex != nullptr) m_can_mutex(1);

    uint32_t buffer[256] = {0};
    int      count       = 0;

    // Transmit
    m_can_transmit(identifier, word, 4);

    // Receive
    count = m_can_receive(buffer, 256);

    // Release mutex (if provided)
    if (m_can_mutex != nullptr) m_can_mutex(0);

    if (!buffer || count == 0) { return bssb_response_t::COMM_FAIL; }

    // Expect the first word to be START_WORD
    if (buffer[0] != START_WORD) { return bssb_response_t::COMM_FAIL; }

    // Create result std::string
    std::string *result = new std::string();

    // Process each 32-bit word after START_WORD
    for (size_t i = 1; i < count; i++) {
        uint32_t word = buffer[i];

        // END_WORD → finished
        if (word == END_WORD) {
            *file_content = result;
            return bssb_response_t::SUCCESS;
        }

        // Extract 4 bytes (little-endian)
        for (int b = 0; b < 4; b++) {
            uint8_t c = (word >> (8 * b)) & 0xFF;
            if (c == 0x00) {
                continue; // ignore padding
            }
            result->push_back((char)c);
        }
    }

    // END_WORD not found
    delete result;
    return bssb_response_t::COMM_FAIL;
}

/* ================================================================================== */
/*                                    Mailbox Comm                                    */
/* ================================================================================== */

bsStepperBoard::bssb_response_t bsStepperBoard::writeStepper(uint8_t driver_num, uint16_t reg_num, uint16_t data,
                                                             verifier_t verifier, uint16_t *response) {
    // Sanity check
    if (m_can_transmit == nullptr || m_can_receive == nullptr || m_async_sleep == nullptr) {
        assert(false && "Not implemented");
        return bssb_response_t::NOT_INIT;
    }

    // Create CAN word
    uint32_t word = 0;
    word |= (driver_num & 0x0F) << 28;
    word |= (reg_num & 0x0FFF) << 16;
    word |= (data & 0xFFFF);

    // Destination CAN ID
    uint32_t identifier = constructIdentifier(Mailbox::STEPPER_WRITE, m_board_num);

    // Expect 1 frame response
    uint32_t response_buffer[1];
    bool     msg_received = CANTransfer(identifier, word, response_buffer);
    if (!msg_received) { return bssb_response_t::COMM_FAIL; }

    // Compare bits 16-31
    bool cmd_tar_match = ((response_buffer[0] & 0xFFFF0000) == (word & 0xFFFF0000));
    if (!cmd_tar_match) { return bssb_response_t::BAD_CAN_TIMING; }

    // Process Response
    uint16_t cmd_res = static_cast<uint16_t>(response_buffer[0] & 0xFFFF);

    // Write Response Value
    if (response != nullptr) { *response = cmd_res; }

    // Compute Condition
    if (verifier != nullptr && !verifier(cmd_res)) { return bssb_response_t::CONDITION_FAIL; }

    // All Pass
    return bssb_response_t::SUCCESS;
}

bsStepperBoard::bssb_response_t bsStepperBoard::readStepper(uint8_t driver_num, uint16_t reg_num, uint16_t *response) {

    // Sanity check
    if (m_can_transmit == nullptr || m_can_receive == nullptr || m_async_sleep == nullptr) {
        assert(false && "Not implemented");
        return bssb_response_t::NOT_INIT;
    }

    // Create word
    uint32_t word = 0;
    word |= (driver_num & 0x0F) << 28;
    word |= (reg_num & 0x0FFF) << 16;

    // Destination CAN ID
    uint32_t identifier = constructIdentifier(Mailbox::READ_STEPPER, m_board_num);

    // Expect 1 frame response
    uint32_t response_buffer[1];
    bool     msg_received = CANTransfer(identifier, word, response_buffer);
    if (!msg_received) { return bssb_response_t::COMM_FAIL; }

    // Write Response Value
    *response = static_cast<uint16_t>(response_buffer[0] & 0xFFFF);

    // All Pass
    return bssb_response_t::SUCCESS;
}

bsStepperBoard::bssb_response_t bsStepperBoard::readStepper32b(uint8_t driver_num, uint16_t reg_num,
                                                               uint32_t *response) {
    // Sanity check
    if (m_can_transmit == nullptr || m_can_receive == nullptr || m_async_sleep == nullptr) {
        assert(false && "Not implemented");
        return bssb_response_t::NOT_INIT;
    }

    // Create CAN word
    uint32_t word = 0;
    word |= (driver_num & 0x0F) << 28;
    word |= (reg_num & 0x0FFF) << 16;

    // Destination CAN ID
    uint32_t identifier = constructIdentifier(Mailbox::READ_STEPPER, m_board_num);

    // Transfer
    uint32_t response_buffer[1];
    bool     msg_received = CANTransfer(identifier, word, response_buffer);
    if (!msg_received) { return bssb_response_t::COMM_FAIL; }

    *response = response_buffer[0];

    // Receive
    if (!msg_received) return bssb_response_t::COMM_FAIL;

    // All Pass
    return bssb_response_t::SUCCESS;
}

bsStepperBoard::bssb_response_t bsStepperBoard::readBoard(uint16_t reg_num, uint16_t *response) {
    // Sanity check
    if (m_can_transmit == nullptr || m_can_receive == nullptr || m_async_sleep == nullptr) {
        assert(false && "Not implemented");
        return bssb_response_t::NOT_INIT;
    }

    // Create CAN word
    uint32_t word = 0;
    word |= (reg_num & 0x0FFF) << 16;

    // Destination CAN ID
    uint32_t identifier = constructIdentifier(Mailbox::BOARD_READ, m_board_num);

    // Transfer
    uint32_t response_buffer[1];
    bool     msg_received = CANTransfer(identifier, word, response_buffer);
    if (!msg_received) { return bssb_response_t::COMM_FAIL; }

    // Write Response Value
    *response = static_cast<uint16_t>(response_buffer[0] & 0xFFFF);

    // All Pass
    return bssb_response_t::SUCCESS;
}

bsStepperBoard::bssb_response_t bsStepperBoard::writeBoard(uint16_t reg_num, uint16_t data, verifier_t verifier,
                                                           uint16_t *response) {
    // Sanity check
    if (m_can_transmit == nullptr || m_can_receive == nullptr || m_async_sleep == nullptr) {
        assert(false && "Not implemented");
        return bssb_response_t::NOT_INIT;
    }

    // Create CAN word
    uint32_t word = 0;
    word |= (reg_num & 0x0FFF) << 16;
    word |= (data & 0xFFFF);

    // Destination CAN ID
    uint32_t identifier = constructIdentifier(Mailbox::BOARD_WRITE, m_board_num);

    // Transfer
    uint32_t response_buffer[1];
    bool     msg_received = CANTransfer(identifier, word, response_buffer);
    if (!msg_received) { return bssb_response_t::COMM_FAIL; }

    uint32_t &can_response = response_buffer[0];

    // Compare bits 16-31
    bool cmd_tar_match = ((can_response & 0xFFFF0000) == (word & 0xFFFF0000));
    if (!cmd_tar_match) { return bssb_response_t::BAD_CAN_TIMING; }

    // Process Response
    uint16_t cmd_res = static_cast<uint16_t>(can_response & 0xFFFF);

    // Write Response Value
    if (response != nullptr) { *response = cmd_res; }

    // Compute Condition
    if (verifier != nullptr && !verifier(cmd_res)) { return bssb_response_t::CONDITION_FAIL; }

    // All Pass
    return bssb_response_t::SUCCESS;
}

bsStepperBoard::bssb_response_t bsStepperBoard::writeBuffer(uint32_t data, bool is_start_end_byte) {
    // Sanity check
    if (m_can_transmit == nullptr || m_can_receive == nullptr || m_async_sleep == nullptr) {
        assert(false && "Not implemented");
        return bssb_response_t::NOT_INIT;
    }

    // Destination CAN ID
    uint32_t identifier = constructIdentifier(Mailbox::BUFFER, m_board_num);

    // Response expected (START & STOP byte)
    if (is_start_end_byte) {
        uint32_t response_buffer[1];
        bool     msg_received = CANTransfer(identifier, data, response_buffer);
        if (!msg_received) { return bssb_response_t::COMM_FAIL; }

        uint32_t &can_response = response_buffer[0];

        // Process Response
        uint16_t cmd_res = static_cast<uint16_t>(can_response & 0xFFFF);

        // Compute Condition
        if (!getReadback_writeSuccessful(cmd_res)) { return bssb_response_t::CONDITION_FAIL; }

        return bssb_response_t::SUCCESS;
    }

    // Transmit Only
    if (m_can_mutex != nullptr) m_can_mutex(1);

    // Transmit
    bool transmitted = m_can_transmit(identifier, data, 4);

    // Release mutex (if provided)
    if (m_can_mutex != nullptr) m_can_mutex(0);

    // Mandatory Wait
    asyncSleep_ms(1);

    return transmitted ? bssb_response_t::SUCCESS : bssb_response_t::COMM_FAIL;
}

/* ================================================================================== */
/*                                       Utility                                      */
/* ================================================================================== */

uint16_t bsStepperBoard::getCurrentScaler(uint16_t current_ma) {
    constexpr uint16_t MIN_CURRENT        = 152;     // mA
    constexpr uint16_t MAX_CURRENT_NEMA17 = 3000;    // mA
    constexpr float    MAX_SCALE          = 4700.0f; // mA for scale 31
    constexpr uint8_t  MAX_STEP           = 31;

    if (current_ma <= MIN_CURRENT) return 1;
    if (current_ma > MAX_CURRENT_NEMA17) current_ma = MAX_CURRENT_NEMA17;

    float scale = (static_cast<float>(current_ma) / MAX_SCALE) * MAX_STEP;

    uint16_t scaler = static_cast<uint16_t>(roundf(scale));
    if (scaler > MAX_STEP) scaler = MAX_STEP;

    return scaler;
}

bsStepperBoard::UInt32Parts bsStepperBoard::floatToHighAndLowWords(float value) {
    UInt32Parts rslt;

    uint32_t int_value = static_cast<uint32_t>(value * 1000.0f);

    rslt.high = (int_value >> 16) & 0xFFFF;
    rslt.low  = int_value & 0xFFFF;

    return rslt;
}

bsStepperBoard::UInt32Parts bsStepperBoard::int32ToHighAndLowWords(int32_t value) {
    UInt32Parts rslt;

    uint32_t int_value = static_cast<uint32_t>(value);

    rslt.high = (int_value >> 16) & 0xFFFF;
    rslt.low  = int_value & 0xFFFF;

    return rslt;
}

void bsStepperBoard::asyncSleep_ms(uint8_t ms) {
    // Sanity check
    if (m_async_sleep == nullptr) { return; }

    for (int i = 0; i < ms; i++) {
        m_async_sleep();
    }
}

/* ---------------------------------------------------------------------------------- */