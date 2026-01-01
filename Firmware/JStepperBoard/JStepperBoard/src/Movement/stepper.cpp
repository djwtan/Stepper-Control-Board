/*
 * CPPFile1.cpp
 *
 * Created: 06/08/2025 15:19:53
 *  Author: Tan
 */

#include "stepper.h"

Stepper::Stepper(Spi *spi, xQueueHandle *spi_mutex, uint8_t spi_cs_pin, uint32_t fclk, uint8_t nfreeze_pin,
                 uint8_t nrst_pin, uint8_t intr_pin, uint8_t drv_en_pin)
    : TMC4361A(spi, spi_mutex, spi_cs_pin, fclk, nfreeze_pin, nrst_pin, intr_pin, drv_en_pin) {

    // Motion Queue
    xTaskCreate(Stepper::task_motionQueue,          // function name
                (const signed char *)"motionQueue", // task name
                216,                                // stack size (117)
                this,                               // stack parameters
                1,                                  // stack priority
                NULL                                // stack handle
    );
}

/* ---------------------------------------------------------------------------------- */
void Stepper::confMove(MoveSettings move_s) { m_move_s = move_s; }
void Stepper::confMove_resetMotionConfAfterEachMove(uint16_t enable) {
    m_move_s.reset_motion_conf_after_each_move = (bool)enable;
}
void Stepper::confMove_allowWriteMotionWhenBusy(uint16_t enable) {
    m_move_s.allow_write_motion_when_busy = (bool)enable;
}

/* ---------------------------------------------------------------------------------- */
void Stepper::confHome(HomeSettings home_s) { m_home_s = home_s; }
void Stepper::confHome_homingMode(uint16_t homing_mode) { m_home_s.homing_mode = (HomingMode)homing_mode; }
void Stepper::confHome_homingSensor(uint16_t homing_sensor) { m_home_s.homing_sensor = (HomingSensor)homing_sensor; }
void Stepper::confHome_sensorHomeValue(uint16_t home_value) { m_home_s.sensor_home_value = (bool)home_value; }
void Stepper::confHome_maxFind_h(uint16_t max_find_h) { m_home_s.max_find_h = max_find_h; }
void Stepper::confHome_maxFind_l(uint16_t max_find_l) { m_home_s.max_find_l = max_find_l; }
void Stepper::confHome_maxSpeed_h(uint16_t max_speed_h) { m_home_s.max_speed_h = max_speed_h; }
void Stepper::confHome_maxSpeed_l(uint16_t max_speed_l) { m_home_s.max_speed_l = max_speed_l; }
void Stepper::confHome_maxAccel_h(uint16_t max_accel_h) { m_home_s.max_accel_h = max_accel_h; }
void Stepper::confHome_maxAccel_l(uint16_t max_accel_l) { m_home_s.max_accel_l = max_accel_l; }
void Stepper::confHome_maxDecel_h(uint16_t max_decel_h) { m_home_s.max_decel_h = max_decel_h; }
void Stepper::confHome_maxDecel_l(uint16_t max_decel_l) { m_home_s.max_decel_l = max_decel_l; }
void Stepper::confHome_offset_h(uint16_t offset_h) { m_home_s.offset_h = offset_h; }
void Stepper::confHome_offset_l(uint16_t offset_l) { m_home_s.offset_l = offset_l; }
void Stepper::confHome_timeout_h(uint16_t timeout_ms_h) { m_home_s.timeout_ms_h = timeout_ms_h; }
void Stepper::confHome_timeout_l(uint16_t timeout_ms_l) { m_home_s.timeout_ms_l = timeout_ms_l; }

/* ---------------------------------------------------------------------------------- */
void Stepper::confPositioning(PositioningSettings pos_s) { m_pos_s = pos_s; }
void Stepper::confPositioning_posMode(uint16_t pos_mode) { m_pos_s.pos_mode = (PositioningMode)pos_mode; }
void Stepper::confPositioning_followMode(uint16_t follow_mode) { m_pos_s.follow_mode = (FollowMode)follow_mode; }
void Stepper::confPositioning_unitPerRev(uint16_t unit_per_rev) { m_pos_s.unit_per_rev = unit_per_rev; }

/* ---------------------------------------------------------------------------------- */
void Stepper::confRamp(RampSettings ramp_s) { m_ramp_s = ramp_s; }
void Stepper::confRamp_rampMode(uint16_t ramp_mode) { m_ramp_s.ramp_mode = (RampMode)ramp_mode; }
void Stepper::confRamp_rampType(uint16_t ramp_type) { m_ramp_s.ramp_type = (RampType)ramp_type; }

/* ---------------------------------------------------------------------------------- */
void Stepper::confSpeed(SpeedSettings speed_s) { m_spd_s = speed_s; }
void Stepper::confSpeed_maxSpeed_h(uint16_t max_h) { m_spd_s.max_h = max_h; }
void Stepper::confSpeed_maxSpeed_l(uint16_t max_l) { m_spd_s.max_l = max_l; }
void Stepper::confSpeed_startSpeed_h(uint16_t start_h) { m_spd_s.start_h = start_h; }
void Stepper::confSpeed_startSpeed_l(uint16_t start_l) { m_spd_s.start_l = start_l; }
void Stepper::confSpeed_stopSpeed_h(uint16_t stop_h) { m_spd_s.stop_h = stop_h; }
void Stepper::confSpeed_stopSpeed_l(uint16_t stop_l) { m_spd_s.stop_l = stop_l; }
void Stepper::confSpeed_breakSpeed_h(uint16_t break_h) { m_spd_s.break_h = break_h; }
void Stepper::confSpeed_breakSpeed_l(uint16_t break_l) { m_spd_s.break_l = break_l; }

/* ---------------------------------------------------------------------------------- */
void Stepper::confAccel(AccelerationSettings accel_s) { m_accel_s = accel_s; }
void Stepper::confAccel_maxAccel_h(uint16_t max_accel_h) { m_accel_s.max_accel_h = max_accel_h; }
void Stepper::confAccel_maxAccel_l(uint16_t max_accel_l) { m_accel_s.max_accel_l = max_accel_l; }
void Stepper::confAccel_maxDecel_h(uint16_t max_decel_h) { m_accel_s.max_decel_h = max_decel_h; }
void Stepper::confAccel_maxDecel_l(uint16_t max_decel_l) { m_accel_s.max_decel_l = max_decel_l; }
void Stepper::confAccel_startAccel_h(uint16_t start_accel_h) { m_accel_s.start_accel_h = start_accel_h; }
void Stepper::confAccel_startAccel_l(uint16_t start_accel_l) { m_accel_s.start_accel_l = start_accel_l; }
void Stepper::confAccel_finalDecel_h(uint16_t final_decel_h) { m_accel_s.final_decel_h = final_decel_h; }
void Stepper::confAccel_finalDecel_l(uint16_t final_decel_l) { m_accel_s.final_decel_l = final_decel_l; }

/* ---------------------------------------------------------------------------------- */
void Stepper::confBow(BowSettings bow_s) { m_bow_s = bow_s; }
void Stepper::confBow_bow1_h(uint16_t bow1_h) { m_bow_s.bow1_h = bow1_h; }
void Stepper::confBow_bow1_l(uint16_t bow1_l) { m_bow_s.bow1_l = bow1_l; }
void Stepper::confBow_bow2_h(uint16_t bow2_h) { m_bow_s.bow2_h = bow2_h; }
void Stepper::confBow_bow2_l(uint16_t bow2_l) { m_bow_s.bow2_l = bow2_l; }
void Stepper::confBow_bow3_h(uint16_t bow3_h) { m_bow_s.bow3_h = bow3_h; }
void Stepper::confBow_bow3_l(uint16_t bow3_l) { m_bow_s.bow3_l = bow3_l; }
void Stepper::confBow_bow4_h(uint16_t bow4_h) { m_bow_s.bow4_h = bow4_h; }
void Stepper::confBow_bow4_l(uint16_t bow4_l) { m_bow_s.bow4_l = bow4_l; }

/* ---------------------------------------------------------------------------------- */
void Stepper::d_emStop() {
    m_ctrl_init = false;
    m_run_task  = false;
    emStop(FreezeEvent::USER);
}

/* ---------------------------------------------------------------------------------- */
void Stepper::d_rampStop() {
    m_motion_conf = false;
    m_run_task    = false;
    rampStop();
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::d_setPosition(int32_t pos) {
    if (isRunning()) return false;
    setPosition(pos);
    return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::d_enableDriver() {
    if (isRunning()) return false;
    enableDriver();
    return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::d_releaseDriver() {
    if (isRunning()) return false;
    releaseDriver();
    return true;
}

/* ---------------------------------------------------------------------------------- */
Stepper::Status Stepper::getStatus() {
    if (isFrozen()) {
        FreezeEvent freeze_event = whyFrozen();
        switch (freeze_event) {
        case FreezeEvent::STALL            : return Stepper::Status::STALL; break;
        case FreezeEvent::POS_ERR          : return Stepper::Status::POS_ERR; break;
        case FreezeEvent::USER             : return Stepper::Status::STOPPED; break;
        case FreezeEvent::COIL_OL          : return Stepper::Status::COIL_OL; break;
        case FreezeEvent::COIL_SHORT       : return Stepper::Status::COIL_SHORT; break;
        case FreezeEvent::OVERTEMP         : return Stepper::Status::OVERTEMP; break;
        case FreezeEvent::TMC4361A_COMM_ERR: return Stepper::Status::TMC4361A_COMM_ERR; break;
        case FreezeEvent::TMC5160_COMM_ERR : return Stepper::Status::TMC5160_COMM_ERR; break;
        default                            : return Stepper::Status::UNDEFINED; break;
        }
    } else {
        if (m_is_homing)
            return Stepper::Status::HOMING;
        else if (m_is_vibrating)
            return Stepper::Status::VIBRATING;
        else if (!m_ctrl_init)
            return Stepper::Status::CTRL_NOT_INIT;
        else if (isRunning())
            return Stepper::Status::BUSY;
        else if (!m_motion_conf)
            return Stepper::Status::MOTION_NOT_INIT;
        else
            return Stepper::Status::READY;
    }
}

/* ---------------------------------------------------------------------------------- */
void Stepper::waitCompleteRun() {
    // Wait for status update (min 5 ticks after write)
    vTaskDelay(STATUS_UPDATE_FREQ_TICKS);

    // Check running status
    while (isRunning())
        vTaskDelay(STATUS_UPDATE_FREQ_TICKS);
}

/* ================================================================================== */
/*                                     CAN Support                                    */
/* ================================================================================== */
void Stepper::setTargetUnitsHigh(uint16_t unit_high) { m_move_s.unit_high = unit_high; }
void Stepper::setTargetUnitsLow(uint16_t unit_low) { m_move_s.unit_low = unit_low; }
void Stepper::setTimeMsHigh(uint16_t time_ms_high) { m_move_s.time_ms_high = time_ms_high; }
void Stepper::setTimeMsLow(uint16_t time_ms_low) { m_move_s.time_ms_low = time_ms_low; }
void Stepper::setVibrationIterations(uint16_t vib_i) { m_move_s.vib_i = vib_i; }
void Stepper::setDiminishingFactor(uint16_t vib_dim_factor) { m_move_s.vib_dim_factor = vib_dim_factor; }
void Stepper::setLoop(uint16_t vib_loop) { m_move_s.vib_loop = (bool)vib_loop; }
void Stepper::rstDrvFlag() { m_ctrl_init = false; }
void Stepper::rstMotionFlag() { m_motion_conf = false; }

/* ================================================================================== */
/*                                      Functions                                     */
/* ================================================================================== */
Stepper::PutQueueRes Stepper::move() {
    if (uxQueueMessagesWaiting(m_result_queue_handle) != 0) return PutQueueRes::Q_RESULTS_PENDING;

    // Combine word
    int32_t units;
    units = static_cast<int32_t>(combineWord(m_move_s.unit_high, m_move_s.unit_low));

    // Prepare args
    MotionItem *motion_item = new MotionItem;
    int32_t    *arg         = new int32_t(units);

    motion_item->motion_task = &Stepper::moveWrapper;
    motion_item->arg         = arg;

    if (xQueueSend(m_motion_queue_handle, &motion_item, 0) == pdTRUE)
        return PutQueueRes::Q_SUCCESS;
    else {
        delete motion_item; // free heap
        delete arg;
        return PutQueueRes::Q_FULL;
    }
}

/* ---------------------------------------------------------------------------------- */
Stepper::PutQueueRes Stepper::moveHoming() {
    if (uxQueueMessagesWaiting(m_result_queue_handle) != 0) return PutQueueRes::Q_RESULTS_PENDING;

    // Prepare args
    MotionItem *motion_item  = new MotionItem;
    motion_item->motion_task = &Stepper::moveHomingWrapper;

    if (xQueueSend(m_motion_queue_handle, &motion_item, 0) == pdTRUE)
        return PutQueueRes::Q_SUCCESS;
    else {
        delete motion_item; // free heap
        return PutQueueRes::Q_FULL;
    }
}

/* ---------------------------------------------------------------------------------- */
Stepper::PutQueueRes Stepper::moveInverseTime() {
    if (uxQueueMessagesWaiting(m_result_queue_handle) != 0) return PutQueueRes::Q_RESULTS_PENDING;

    // Combine word
    uint32_t time_ms_int;
    time_ms_int = combineWord(m_move_s.time_ms_high, m_move_s.time_ms_low);

    // Prepare args
    _IvtArgs *arg = new _IvtArgs;
    arg->units    = static_cast<int32_t>(combineWord(m_move_s.unit_high, m_move_s.unit_low));
    // arg->time_ms  = uintTofloat(time_ms_int, 0);
    arg->time_ms = static_cast<float>(time_ms_int);

    MotionItem *motion_item  = new MotionItem;
    motion_item->motion_task = &Stepper::moveInverseTimeWrapper;
    motion_item->arg         = arg;

    if (xQueueSend(m_motion_queue_handle, &motion_item, 0) == pdTRUE)
        return PutQueueRes::Q_SUCCESS;
    else {
        delete motion_item; // free heap
        delete arg;
        return PutQueueRes::Q_FULL;
    }
}

/* ---------------------------------------------------------------------------------- */
Stepper::PutQueueRes Stepper::moveVibration() {
    if (uxQueueMessagesWaiting(m_result_queue_handle) != 0) return PutQueueRes::Q_RESULTS_PENDING;

    // Prepare args
    _VibrationArgs *arg     = new _VibrationArgs;
    arg->units              = combineWord(m_move_s.unit_high, m_move_s.unit_low);
    arg->iterations         = (uint32_t)m_move_s.vib_i;
    arg->diminishing_factor = uintTofloat(m_move_s.vib_dim_factor, 0);
    arg->loop               = m_move_s.vib_loop;

    MotionItem *motion_item  = new MotionItem;
    motion_item->motion_task = &Stepper::moveVibrationWrapper;
    motion_item->arg         = arg;

    if (xQueueSend(m_motion_queue_handle, &motion_item, 0) == pdTRUE)
        return PutQueueRes::Q_SUCCESS;
    else {
        delete motion_item; // free heap
        delete arg;
        return PutQueueRes::Q_FULL;
    }
}

/* ---------------------------------------------------------------------------------- */
Stepper::PutQueueRes Stepper::initController() {
    if (uxQueueMessagesWaiting(m_result_queue_handle) != 0) return PutQueueRes::Q_RESULTS_PENDING;

    // Prepare args
    MotionItem *motion_item  = new MotionItem;
    motion_item->motion_task = &Stepper::execInitControllerWrapper; // special case for initController
    motion_item->arg         = nullptr;

    if (xQueueSend(m_motion_queue_handle, &motion_item, 0) == pdTRUE)
        return PutQueueRes::Q_SUCCESS;
    else {
        delete motion_item; // free heap
        return PutQueueRes::Q_FULL;
    }
}

/* ---------------------------------------------------------------------------------- */
Stepper::PutQueueRes Stepper::setMotion() {
    if (uxQueueMessagesWaiting(m_result_queue_handle) != 0) return PutQueueRes::Q_RESULTS_PENDING;

    // Prepare args
    MotionItem *motion_item  = new MotionItem;
    motion_item->motion_task = &Stepper::execSetMotionWrapper; // special case for setMotion
    motion_item->arg         = nullptr;

    if (xQueueSend(m_motion_queue_handle, &motion_item, 0) == pdTRUE)
        return PutQueueRes::Q_SUCCESS;
    else {
        delete motion_item; // free heap
        return PutQueueRes::Q_FULL;
    }
}

/* ================================================================================== */
/*                                        Query                                       */
/* ================================================================================== */
bool Stepper::getReadback(uint32_t *readback) {
    bool has_readback = xQueueReceive(m_result_queue_handle, readback, 0) == pdTRUE;
    stepper_debug("rb %d\n", has_readback);
    return has_readback;
}

/* ---------------------------------------------------------------------------------- */
int32_t Stepper::pulseToUnit(int32_t pulse) {
    // Convert integers to floats for precise math
    float mstep_per_fs = static_cast<float>(m_drv_s.mstep_per_fs);
    float unit_per_rev = static_cast<float>(m_pos_s.unit_per_rev);
    float fs_per_rev   = static_cast<float>(m_drv_s.fs_per_rev);
    float f_pulse      = static_cast<float>(pulse);
    bool  is_negative  = pulse < 0;

    // Compute unit as float (inverse of unitToPulse)
    float unit = f_pulse * (unit_per_rev / (mstep_per_fs * fs_per_rev));

    // Round to nearest integer
    return static_cast<int32_t>(unit + (is_negative ? -0.5f : 0.5f));
}

/* ================================================================================== */
/*                                      Wrappers                                      */
/* ================================================================================== */
uint32_t Stepper::moveWrapper(void *args) {
    auto       *motion_item_ptr = static_cast<MotionItem **>(args); // pointer to pointer from queue
    MotionItem *motion_item     = *motion_item_ptr;
    int32_t    *units           = static_cast<int32_t *>(motion_item->arg);

    ExecCode exec_code = execMove(*units);

    delete motion_item; // free heap
    delete units;
    return static_cast<uint32_t>(exec_code);
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::moveHomingWrapper(void *args) {
    auto       *motion_item_ptr = static_cast<MotionItem **>(args); // pointer to pointer from queue
    MotionItem *motion_item     = *motion_item_ptr;

    m_is_homing            = true;
    HomingCode homing_code = execMoveHoming();
    m_is_homing            = false;

    delete motion_item; // free heap
    return static_cast<uint32_t>(homing_code);
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::moveInverseTimeWrapper(void *args) {
    auto       *motion_item_ptr = static_cast<MotionItem **>(args); // pointer to pointer from queue
    MotionItem *motion_item     = *motion_item_ptr;
    _IvtArgs   *ivt_args        = static_cast<_IvtArgs *>(motion_item->arg);

    ExecCode exec_code = execMoveInverseTime(ivt_args->units, ivt_args->time_ms);

    delete motion_item; // free heap
    delete ivt_args;
    return static_cast<uint32_t>(exec_code);
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::moveVibrationWrapper(void *args) {
    auto           *motion_item_ptr = static_cast<MotionItem **>(args); // pointer to pointer from queue
    MotionItem     *motion_item     = *motion_item_ptr;
    _VibrationArgs *vib_args        = static_cast<_VibrationArgs *>(motion_item->arg);
    m_run_task                      = true;

    ExecCode exec_code =
        execMoveVibrate(vib_args->units, vib_args->iterations, vib_args->diminishing_factor, vib_args->loop);

    delete motion_item; // free heap
    delete vib_args;
    return static_cast<uint32_t>(exec_code);
}

/* ================================================================================== */
/*                                        Setup                                       */
/* ================================================================================== */
uint32_t Stepper::execInitControllerWrapper(void *args) {
    auto       *motion_item_ptr = static_cast<MotionItem **>(args); // pointer to pointer from queue
    MotionItem *motion_item     = *motion_item_ptr;

    bool exec = execInitController();

    delete motion_item; // free heap
    return static_cast<uint32_t>(exec);
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::execSetMotionWrapper(void *args) {
    auto       *motion_item_ptr = static_cast<MotionItem **>(args); // pointer to pointer from queue
    MotionItem *motion_item     = *motion_item_ptr;

    bool exec = execSetMotion();

    delete motion_item; // free heap
    return static_cast<uint32_t>(exec);
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::execInitController() {
    FreezeEvent freeze_event = whyFrozen();

    if (freeze_event > 3) return false; // require reboot

    hardReset();
    m_ctrl_init   = initialize();
    m_motion_conf = false;

    return m_ctrl_init;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::execSetMotion() {
    if (!m_ctrl_init) return false;

    bool conf = true;

    // Ramp
    conf &= setRampMode(m_ramp_s.ramp_mode, m_ramp_s.ramp_type);

    // Speed
    uint32_t max_speed   = combineWord(m_spd_s.max_h, m_spd_s.max_l);
    uint32_t start_speed = combineWord(m_spd_s.start_h, m_spd_s.start_l);
    uint32_t stop_speed  = combineWord(m_spd_s.stop_h, m_spd_s.stop_l);
    uint32_t break_speed = combineWord(m_spd_s.break_h, m_spd_s.break_l);

    float max_speed_f   = uintTofloat((max_speed), 1);
    float start_speed_f = uintTofloat((start_speed), 1);
    float stop_speed_f  = uintTofloat((stop_speed), 1);
    float break_speed_f = uintTofloat((break_speed), 1);

    conf &= setSpeeds(m_ramp_s.ramp_mode == TMC4361A::RampMode::VELOCITY_MODE ? 0 : max_speed_f, start_speed_f,
                      stop_speed_f, break_speed_f);

    // Acceleration
    uint32_t max_accel   = combineWord(m_accel_s.max_accel_h, m_accel_s.max_accel_l);
    uint32_t max_decel   = combineWord(m_accel_s.max_decel_h, m_accel_s.max_decel_l);
    uint32_t start_accel = combineWord(m_accel_s.start_accel_h, m_accel_s.start_accel_l);
    uint32_t final_decel = combineWord(m_accel_s.final_decel_h, m_accel_s.final_decel_l);

    float max_accel_f   = uintTofloat((max_accel), 0);
    float max_decel_f   = uintTofloat((max_decel), 0);
    float start_accel_f = uintTofloat((start_accel), 0);
    float final_decel_f = uintTofloat((final_decel), 0);

    conf &= setAccelerations(max_accel_f, max_decel_f, start_accel_f, final_decel_f);

    // Bow
    uint32_t bow1 = combineWord(m_bow_s.bow1_h, m_bow_s.bow1_l);
    uint32_t bow2 = combineWord(m_bow_s.bow2_h, m_bow_s.bow2_l);
    uint32_t bow3 = combineWord(m_bow_s.bow3_h, m_bow_s.bow3_l);
    uint32_t bow4 = combineWord(m_bow_s.bow4_h, m_bow_s.bow4_l);

    float bow1_f = uintTofloat((bow1), 0);
    float bow2_f = uintTofloat((bow2), 0);
    float bow3_f = uintTofloat((bow3), 0);
    float bow4_f = uintTofloat((bow4), 0);

    conf &= setBowValues(bow1_f, bow2_f, bow3_f, bow4_f);

    m_motion_conf = conf;

    return conf;
}

/* ================================================================================== */
/*                                      Movements                                     */
/* ================================================================================== */
Stepper::ExecCode Stepper::execMove(int32_t units) {
    // System check
    if (!m_ctrl_init) return ExecCode::E_CTRL_NOT_INIT;
    if (!m_motion_conf) return ExecCode::E_MOTION_NOT_INIT;
    if (isFrozen()) return ExecCode::E_IS_FROZEN;

    bool success = true;

    enableDriver();

    // Positioning mode
    if (m_ramp_s.ramp_mode == TMC4361A::RampMode::POSITIONING_MODE) {
        int32_t pulses = unitToPulse(units);

        // block if system is still running & write whilst busy isn't allowed
        if (!m_move_s.allow_write_motion_when_busy && isRunning()) return ExecCode::E_IS_BUSY;

        // Compute pulse
        int32_t target_pulse = computeTargetPulse(pulses);
        success &= setTargetPosition(target_pulse);

        if (!m_move_s.allow_write_motion_when_busy) waitCompleteRun();
    }
    // Velocity mode
    else {
        success &= setSpeeds((float)units);
    }

    if (!success) return ExecCode::E_WRITE_FAIL;
    // require motion write again
    if (m_move_s.reset_motion_conf_after_each_move) m_motion_conf = false;

    return ExecCode::E_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::setMotionInverseTime(float max_speed, float max_accel) {
    bool conf = true;

    RampSettings default_ramp_s; // position mode + trapezoidal

    conf &= setRampMode(default_ramp_s.ramp_mode, default_ramp_s.ramp_type);
    conf &= setSpeeds(max_speed, 0.0, 0.0, 0.0);
    conf &= setAccelerations(max_accel, max_accel, 0.0, 0.0);

    return conf;
}

/* ---------------------------------------------------------------------------------- */
Stepper::HomingCode Stepper::execMoveHoming() {
    // System check
    if (!m_ctrl_init) return HomingCode::H_CTRL_NOT_INIT;
    if (isFrozen()) return HomingCode::H_IS_FROZEN;
    if (isRunning()) return HomingCode::H_IS_BUSY;

    enableDriver();

    // Combine Word
    int32_t  max_find_unit;
    int32_t  homing_offset_unit;
    uint32_t timeout_ms;

    max_find_unit      = static_cast<int32_t>(combineWord(m_home_s.max_find_h, m_home_s.max_find_l));
    homing_offset_unit = static_cast<int32_t>(combineWord(m_home_s.offset_h, m_home_s.offset_l));
    timeout_ms         = combineWord(m_home_s.timeout_ms_h, m_home_s.timeout_ms_l);

    // Prepare args
    bool         write_success = true;
    _WaitRes     results;
    uint32_t     max_find_pulse_w          = static_cast<uint32_t>(unitToPulse(max_find_unit));
    uint32_t     max_find_pulse_reversed_w = static_cast<uint32_t>(unitToPulse(-max_find_unit));
    uint32_t     homing_offset_w           = static_cast<uint32_t>(unitToPulse(homing_offset_unit));
    portTickType timeout_ticks             = timeout_ms / portTICK_RATE_MS;

    write_success &= setPosition(0);    // Reset position & target
    write_success &= setHomingMotion(); // Configure motion for homing

    if (!write_success) return HomingCode::H_WRITE_FAIL;

    /* ================================ Immediate Homing ================================ */
    if (m_home_s.homing_mode == HomingMode::IMMEDIATE) {
        (void)0;
    }

    /* =============================== Sensor Active High =============================== */
    else if (m_home_s.homing_mode == HomingMode::SENSOR) {
        // ------------------------- Move towards Sensor
        if (readSensor(m_home_s.homing_sensor) != m_home_s.sensor_home_value) {
            // 1. Reset Position
            write_success &= setPosition(0);
            // 2. Start Move
            write_success &= setTargetPosition(max_find_pulse_w);
            // 3. Wait 5ms status update
            vTaskDelay(STATUS_UPDATE_FREQ_TICKS * 2);
            // 3. Block
            results = waitCondition([&]() { return readSensor(m_home_s.homing_sensor); }, m_home_s.sensor_home_value,
                                    timeout_ticks);
            // 4. Sthap it
            rampStop();

            // Premature breaks
            if (results.timed_out) { return HomingCode::H_TIMEOUT; }
            if (!results.condition_met) { return HomingCode::H_MAX_PULSE_REACHED; }
        }
        // ------------------------- Back away
        write_success &= setHomingMotion();
        // 7. position
        write_success &= setPosition(0);
        // 8. Move reverse direction
        write_success &= setTargetPosition(max_find_pulse_reversed_w);
        // 9. Wait 5ms status update
        vTaskDelay(STATUS_UPDATE_FREQ_TICKS * 2);
        // 10. Block
        results = waitCondition([&]() { return readSensor(m_home_s.homing_sensor); }, !m_home_s.sensor_home_value,
                                timeout_ticks);
        // 11. Sthap it again
        rampStop();

        // Premature breaks
        if (results.timed_out) { return HomingCode::H_TIMEOUT; }
        if (!results.condition_met) { return HomingCode::H_MAX_PULSE_REACHED; }

        // ------------------------- Move towards sensor again
        // Move to Sensor
        // 6. Reconf
        write_success &= setHomingMotion();
        // 7. position
        write_success &= setPosition(0);
        // 8. Move reverse direction
        write_success &= setTargetPosition(max_find_pulse_w);
        // 9. Wait 5ms status update
        vTaskDelay(STATUS_UPDATE_FREQ_TICKS * 2);
        // 10. Block
        results = waitCondition([&]() { return readSensor(m_home_s.homing_sensor); }, m_home_s.sensor_home_value,
                                timeout_ticks);
        // 11. Sthap it again
        rampStop();
    }

    /* ===================================== Torque ===================================== */
    else {
        // 1. Reset Position
        write_success &= setPosition(0);
        // 2. Start Move
        write_success &= setTargetPosition(max_find_pulse_w);
        // 3. Toggle value for open loop stall
        bool ori_stop_on_stall  = m_stall_s.stop_on_stall;
        m_stall_s.stop_on_stall = !m_cl_s.enable;
        // 3. Block
        results = waitCondition([&]() { return isFrozen(); }, true, timeout_ticks);
        // 4. Get freeze event
        FreezeEvent freeze_event = whyFrozen();
        // 5. Reassign condition
        if (m_cl_s.enable)
            results.condition_met = (freeze_event == FreezeEvent::POS_ERR);
        else
            results.condition_met = (freeze_event == FreezeEvent::STALL);
        // 6. Revert
        m_stall_s.stop_on_stall = ori_stop_on_stall;
    }

    write_success &= setPosition(0);

    // Premature breaks
    if (results.timed_out) { return HomingCode::H_TIMEOUT; }
    if (!results.condition_met) { return HomingCode::H_MAX_PULSE_REACHED; }

    // Reset Motion
    setHomingMotion();
    // Move to offset position
    write_success &= setTargetPosition(homing_offset_w);
    // Wait for status update
    vTaskDelay(STATUS_UPDATE_FREQ_TICKS * 2);
    // Wait for stop
    waitCompleteRun();
    // Check for errors
    if (isFrozen()) return HomingCode::H_FAILED_MIDWAY;
    // Reset position
    write_success &= setPosition(0);

    if (!write_success) return HomingCode::H_WRITE_FAIL;

    return HomingCode::H_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
Stepper::ExecCode Stepper::execMoveInverseTime(int32_t units, float time_ms) {
    // System check
    if (!m_ctrl_init) return ExecCode::E_CTRL_NOT_INIT;
    if (isFrozen()) return ExecCode::E_IS_FROZEN;
    if (isRunning()) return ExecCode::E_IS_BUSY;
    if (m_ramp_s.ramp_mode != TMC4361A::RampMode::POSITIONING_MODE) return ExecCode::E_BAD_SETTINGS;
    /* ---------------------------------------------------------------------------------- */

    enableDriver();

    int32_t pulses        = unitToPulse(units);
    int32_t target_pulse  = computeTargetPulse(pulses);
    int32_t current_pulse = (m_pos_s.follow_mode == FollowMode::ENCODER) ? getEncoderPosition() : getInternalPosition();

    // // aim for triangular shape
    uint32_t delta_pulse       = abs(target_pulse - current_pulse);
    float    magic_number      = 1000.0f / 571.0f;    // ? Why magic number needed
    float    time_s_f          = (time_ms / 1000.0f); // total time in seconds
    float    time_s_f_adjusted = time_s_f * magic_number;

    // Peak velocity for triangular motion: v = 2s / t
    float vel_f = 2.0f * static_cast<float>(delta_pulse) / time_s_f_adjusted;

    // Convert to RPM if needed
    float vel_f_scaled = spdToRpm(vel_f);

    // Average acceleration: a = v / (t/2) = 2v / t
    float acel_f_scaled = 2.0f * vel_f_scaled / time_s_f_adjusted;

    /* ---------------------------------------------------------------------------------- */
    bool success  = true;
    m_motion_conf = false;

    success &= setMotionInverseTime(vel_f_scaled, acel_f_scaled);
    success &= setTargetPosition(static_cast<uint32_t>(target_pulse));
    if (!success) return ExecCode::E_WRITE_FAIL;

    waitCompleteRun();
    return ExecCode::E_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
Stepper::ExecCode Stepper::execMoveVibrate(uint32_t units, uint32_t iterations, float diminishing_factor, bool loop) {
    // Preconditions
    if (!m_ctrl_init) return ExecCode::E_CTRL_NOT_INIT;
    if (isFrozen()) return ExecCode::E_IS_FROZEN;
    if (isRunning()) return ExecCode::E_IS_BUSY;
    if (m_cl_s.enable) return ExecCode::E_BAD_SETTINGS;
    if (m_stall_s.stop_on_stall) return ExecCode::E_BAD_SETTINGS;
    if (diminishing_factor > 1.0f) return ExecCode::E_BAD_SETTINGS;
    if (!setVibratingMotion()) return ExecCode::E_WRITE_FAIL;

    enableDriver();

    m_motion_conf          = false;
    m_is_vibrating         = true;
    uint32_t write_pos     = 0;
    int32_t  write_pos_int = 0;
    int32_t  ref_pos       = getInternalPosition();

    do {
        int32_t current_magnitude = unitToPulse(units / 2);

        for (uint32_t i = 0; i < iterations && m_run_task && current_magnitude != 0; ++i) {
            // Move positive
            write_pos_int = ref_pos + current_magnitude;
            write_pos     = static_cast<uint32_t>(write_pos_int);
            setTargetPosition(write_pos);
            waitCompleteRun();

            // Move negative
            write_pos_int = ref_pos - current_magnitude;
            write_pos     = static_cast<uint32_t>(write_pos_int);
            setTargetPosition(write_pos);
            waitCompleteRun();

            // Diminish amplitude
            current_magnitude *= diminishing_factor;

            // Stop conditions
            if (isFrozen()) {
                m_run_task = false;
                break;
            }
        }

        // Exit if not looping
        if (!loop) break;

    } while (m_run_task);

    m_is_vibrating = false;
    return ExecCode::E_SUCCESS;
}

/* ================================================================================== */
/*                                        Misc                                        */
/* ================================================================================== */
uint32_t Stepper::combineWord(uint16_t high, uint16_t low) {
    return (static_cast<uint32_t>(high) << 16) | static_cast<uint32_t>(low);
}

/* ---------------------------------------------------------------------------------- */
float Stepper::uintTofloat(uint32_t val, uint8_t sign) {
    float f;

    if (sign) {
        int32_t signedVal = static_cast<int32_t>(val);
        f                 = static_cast<float>(signedVal);
    } else {
        f = static_cast<float>(val);
    }

    return f / FLOAT_DIVISOR;
}

/* ---------------------------------------------------------------------------------- */
int32_t Stepper::unitToPulse(int32_t unit) {
    // Convert integers to floats for precise division
    float mstep_per_fs = static_cast<float>(m_drv_s.mstep_per_fs);
    float unit_per_rev = static_cast<float>(m_pos_s.unit_per_rev);
    float fs_per_rev   = static_cast<float>(m_drv_s.fs_per_rev);
    float f_unit       = static_cast<float>(unit);
    bool  is_negative  = unit < 0;

    // Compute pulses as float
    float pulse = f_unit * (mstep_per_fs / unit_per_rev) * fs_per_rev;

    return static_cast<int32_t>(pulse + (is_negative ? -0.5f : 0.5f)); // round
}

/* ---------------------------------------------------------------------------------- */
int32_t Stepper::correctRotationalPulse(int32_t pulse) {
    int32_t correction_pulse    = m_move_s.fs_per_correction * m_drv_s.mstep_per_fs;
    int32_t correction_thresh_l = (int32_t)((float)correction_pulse * 0.01f); // 1% dev
    int32_t correction_thresh_h = correction_pulse - correction_thresh_l;

    // Normalize remainder into [0, correction_pulse)
    int32_t remainder = pulse % correction_pulse;
    int32_t quotient  = pulse / correction_pulse;

    if (remainder < 0) {
        remainder += correction_pulse;
        quotient -= 1;
    }

    // Snap logic
    if (remainder < correction_thresh_l) {
        return quotient * correction_pulse; // snap down
    }
    if (remainder > correction_thresh_h) {
        return (quotient + 1) * correction_pulse; // snap up
    }

    // Otherwise keep as-is
    return pulse;
}

/* ---------------------------------------------------------------------------------- */
int32_t Stepper::computeTargetPulse(int32_t pulses) {
    int32_t target_pulse;

    // Absolute Positioning
    if (m_pos_s.pos_mode == PositioningMode::PM_ABSOLUTE) {
        target_pulse = pulses;
    }
    // Get current position
    else {
        int32_t pos_ref = (m_pos_s.follow_mode == FollowMode::ENCODER) ? getEncoderPosition() : getInternalPosition();

        bool will_overflow =
            (pulses > 0 && pos_ref > INT32_MAX - pulses) || (pulses < 0 && pos_ref < INT32_MIN - pulses);

        // Reset position if ovf
        if (will_overflow) {
            setPosition(0);
            target_pulse = pulses;
        } else {
            target_pulse = pos_ref + pulses;
        }

        target_pulse = correctRotationalPulse(target_pulse);
    }

    return target_pulse;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::setHomingMotion() {
    bool success = true;

    RampSettings default_ramp_s; // position mode + trapezoidal

    uint32_t max_speed   = combineWord(m_home_s.max_speed_h, m_home_s.max_speed_l);
    float    max_speed_f = uintTofloat((max_speed), 1);
    uint32_t max_accel   = combineWord(m_home_s.max_accel_h, m_home_s.max_accel_l);
    float    max_accel_f = uintTofloat((max_accel), 1);
    uint32_t max_decel   = combineWord(m_home_s.max_decel_h, m_home_s.max_decel_l);
    float    max_decel_f = uintTofloat((max_decel), 1);

    success &= setRampMode(default_ramp_s.ramp_mode, default_ramp_s.ramp_type);
    success &= setSpeeds(max_speed_f);
    success &= setAccelerations(max_accel_f, max_decel_f);

    return success;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::setVibratingMotion() {
    bool success = true;

    float dummy_rpm   = 1800.0f;
    float dummy_accel = 16777.0f;

    TMC4361A::RampMode ramp_mode = TMC4361A::RampMode::POSITIONING_MODE;
    TMC4361A::RampType ramp_type = TMC4361A::RampType::TRAPEZOIDAL_RAMP;

    success &= setRampMode(ramp_mode, ramp_type);
    success &= setSpeeds(dummy_rpm);
    success &= setAccelerations(dummy_accel, dummy_accel);

    return success;
}

/* ---------------------------------------------------------------------------------- */
template <typename Callable>
Stepper::_WaitRes Stepper::waitCondition(Callable payload_factory, bool value, portTickType timeout) {
    portTickType start_tick = xTaskGetTickCount();
    _WaitRes     wait_results{false, true};

    while (xTaskGetTickCount() - start_tick < timeout) {
        if (payload_factory() == value) {
            wait_results.condition_met = true;
            wait_results.timed_out     = false;
            break;
        }
        if (!isRunning()) {
            wait_results.timed_out = false;
            break;
        }
        vTaskDelay(STATUS_UPDATE_FREQ_TICKS);
    }
    return wait_results;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::readSensor(HomingSensor homing_sensor) {
    uint8_t sensor_status = getSensorReading();

    return sensor_status >> homing_sensor & 1;
}

/* ================================ Motion Queue Task =============================== */

void Stepper::task_motionQueue(void *parameters) {
    auto       *self = static_cast<Stepper *>(parameters);
    MotionItem *motion_item_ptr;
    uint32_t    result = 0;

    for (;;) {
        if (xQueuePeek(self->m_motion_queue_handle, &motion_item_ptr, 0) == pdTRUE) {
            if (motion_item_ptr->motion_task) {
                stepper_debug("Move started.\n");
                result = (self->*motion_item_ptr->motion_task)(&motion_item_ptr);
                // Send result
                xQueueSend(self->m_result_queue_handle, &result, 0);
                // Remove from queue
                xQueueReceive(self->m_motion_queue_handle, &motion_item_ptr, 0);
                stepper_debug("Move ended.\n");
            }
        } else {
            vTaskDelay(10 / portTICK_RATE_MS);
        }
    }
}

/* ---------------------------------------------------------------------------------- */