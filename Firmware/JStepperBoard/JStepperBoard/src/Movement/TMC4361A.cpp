#include "TMC4361A.h"

TMC4361A::TMC4361A(Spi *spi, xQueueHandle *spi_mutex, uint8_t spi_cs_pin, uint32_t fclk, uint8_t nfreeze_pin,
                   uint8_t nrst_pin, uint8_t intr_pin, uint8_t drv_en_pin)
    : m_spi(spi), m_spi_mutex(spi_mutex), m_spi_cs_pin(spi_cs_pin), m_fclk(fclk), m_nfreeze_pin(nfreeze_pin),
      m_nrst_pin(nrst_pin), m_intr_pin(intr_pin), m_drv_en_pin(drv_en_pin) {

    ioport_set_pin_dir(m_nrst_pin, IOPORT_DIR_OUTPUT);
    ioport_set_pin_dir(m_nfreeze_pin, IOPORT_DIR_OUTPUT);
    ioport_set_pin_dir(m_drv_en_pin, IOPORT_DIR_OUTPUT);

    resetFreeze();
    toggleNRST();
    releaseDriver();

    // Drive monitoring
    xTaskCreate(TMC4361A::task_stsMonitor,     // function name
                (const signed char *)"DrvMon", // task name
                200,                           // stack size (100)
                this,                          // task parameters
                1,                             // task priority
                &drv_monitor_handle            // task handle
    );
}

/* ================================================================================== */
/*                                   Motion Settings                                  */
/* ================================================================================== */
bool TMC4361A::setRampMode(TMC4361A::RampMode mode, TMC4361A::RampType type) {
    uint32_t write_value = mode | type;

    return writeRegister(TMC4361A_Reg::RAMPMODE, write_value);
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::setSpeeds(float max_speed, float start_speed, float stop_speed, float break_speed) {
    bool succeed = true;

    uint32_t max_speed_w   = floatToFixed(rpmToSpd(max_speed), 8);
    uint32_t start_speed_w = floatToFixed(rpmToSpd(start_speed), 8);
    uint32_t stop_speed_w  = floatToFixed(rpmToSpd(stop_speed), 8);
    uint32_t break_speed_w = floatToFixed(rpmToSpd(break_speed), 8); // Affects trapezoidal only

    succeed &= writeRegister(TMC4361A_Reg::VMAX, max_speed_w);
    succeed &= writeRegister(TMC4361A_Reg::VSTART, start_speed_w);
    succeed &= writeRegister(TMC4361A_Reg::VSTOP, stop_speed_w);
    succeed &= writeRegister(TMC4361A_Reg::VBREAK, break_speed_w);

    return succeed;
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::setAccelerations(float max_accel, float max_decel, float start_accel, float final_decel) {
    bool succeed = true;

    uint32_t max_accel_w   = floatToFixed(deltaRpmToDeltaSpd(max_accel), 2) & 0xFFFFFF;
    uint32_t max_decel_w   = floatToFixed(deltaRpmToDeltaSpd(max_decel), 2) & 0xFFFFFF;
    uint32_t start_accel_w = floatToFixed(deltaRpmToDeltaSpd(start_accel), 2) & 0xFFFFFF;
    uint32_t final_decel_w = floatToFixed(deltaRpmToDeltaSpd(final_decel), 2) & 0xFFFFFF;

    succeed &= writeRegister(TMC4361A_Reg::AMAX, max_accel_w);
    succeed &= writeRegister(TMC4361A_Reg::DMAX, max_decel_w);
    succeed &= writeRegister(TMC4361A_Reg::ASTART, start_accel_w);
    succeed &= writeRegister(TMC4361A_Reg::DFINAL, final_decel_w);

    return succeed;
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::setBowValues(float bow1, float bow2, float bow3, float bow4) {
    bool succeed = true;

    uint32_t bow1_w = deltaSqRpmToDeltaSqSpd(bow1) & 0xFFFFFF;
    uint32_t bow2_w = deltaSqRpmToDeltaSqSpd(bow2) & 0xFFFFFF;
    uint32_t bow3_w = deltaSqRpmToDeltaSqSpd(bow3) & 0xFFFFFF;
    uint32_t bow4_w = deltaSqRpmToDeltaSqSpd(bow4) & 0xFFFFFF;

    succeed &= writeRegister(TMC4361A_Reg::BOW1, bow1_w);
    succeed &= writeRegister(TMC4361A_Reg::BOW2, bow2_w);
    succeed &= writeRegister(TMC4361A_Reg::BOW3, bow3_w);
    succeed &= writeRegister(TMC4361A_Reg::BOW4, bow4_w);

    return succeed;
}

/* ================================================================================== */
/*                                   Motion Commands                                  */
/* ================================================================================== */
bool TMC4361A::setTargetPosition(uint32_t position) { return writeRegister(TMC4361A_Reg::X_TARGET, position); }

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::rampStop() { return writeRegister(TMC4361A_Reg::VMAX, 0); }

/* ---------------------------------------------------------------------------------- */
void TMC4361A::emStop(FreezeEvent freeze_event) {
    // releaseDriver(); // -> Eliminates the squeek
    raiseFreeze();
    m_freeze_event = freeze_event;
    m_is_running   = false;

    if (m_freeze_event == FreezeEvent::USER) {
        m_cached_position            = getInternalPosition();
        m_write_position_after_reset = true;
    }
}

/* ================================================================================== */
/*                                        Query                                       */
/* ================================================================================== */
bool                  TMC4361A::isTargetReached() { return m_target_reached; }
bool                  TMC4361A::isStalled() { return m_is_stalled; }
bool                  TMC4361A::isRunning() { return m_is_running; }
bool                  TMC4361A::isFrozen() { return m_is_frozen; }
uint8_t               TMC4361A::getSensorReading() { return m_sensor_status; }
bool                  TMC4361A::hasPositionError() { return m_has_position_error; }
TMC4361A::FreezeEvent TMC4361A::whyFrozen() { return m_freeze_event; }

/* ---------------------------------------------------------------------------------- */
int32_t TMC4361A::getInternalPosition() {
    uint32_t response;

    readRegister(TMC4361A_Reg::XACTUAL, &response);

    return static_cast<int32_t>(response);
}

/* ---------------------------------------------------------------------------------- */
int32_t TMC4361A::getEncoderPosition() {
    uint32_t response;

    readRegister(TMC4361A_Reg::ENC_POS, &response);

    return static_cast<int32_t>(response);
}

/* ---------------------------------------------------------------------------------- */
int32_t TMC4361A::getCurrentSpeed() {
    uint32_t response;

    readRegister(TMC4361A_Reg::VACTUAL, &response);

    return static_cast<int32_t>(response);
}

/* ---------------------------------------------------------------------------------- */
uint32_t TMC4361A::getEncoderPosDev() {
    uint32_t response;

    readRegister(TMC4361A_Reg::ENC_POS_DEV_RD, &response);

    return response;
}

/* ================================================================================== */
/*                                      Commands                                      */
/* ================================================================================== */
bool TMC4361A::setPosition(int32_t pos) {
    bool succeed = true;

    // ignore next pos err if new position set is > max error
    m_ignore_next_stall |= m_cl_s.enable && (Abs(pos - getEncoderPosition()) > m_cl_s.cl_tolerance);

    uint32_t pos_w = static_cast<uint32_t>(pos);

    succeed &= writeRegister(TMC4361A_Reg::XACTUAL, pos_w);
    succeed &= writeRegister(TMC4361A_Reg::X_TARGET, pos_w);
    succeed &= writeRegister(TMC4361A_Reg::ENC_POS, pos_w);

    return succeed;
}

/* ---------------------------------------------------------------------------------- */
void TMC4361A::releaseDriver() {
    if (m_drv_enabled) {
        ioport_set_pin_level(m_drv_en_pin, true);
        delay_ms(20);
        m_drv_enabled = false;
    }
}

/* ---------------------------------------------------------------------------------- */
void TMC4361A::enableDriver() {
    if (!m_drv_enabled) {
        ioport_set_pin_level(m_drv_en_pin, false);
        delay_ms(20);
        m_drv_enabled = true;
    }
}

/* ================================================================================== */
/*                                        Setup                                       */
/* ================================================================================== */
bool TMC4361A::initialize() {
    bool initialized = true;

    if (m_rtos_inited) { vTaskSuspend(drv_monitor_handle); }

    softReset();

    initialized &= setFixed();
    initialized &= setInterrupt();
    initialized &= setCurrent();
    initialized &= setClosedLoop();
    initialized &= setStallDetection();

    // Clear flag & event
    uint32_t dummy;
    clearEvents();
    readStatus(&dummy); // clear first reading

    // Write cached position
    if (m_write_position_after_reset) {
        initialized &= setPosition(m_cached_position);
        m_cached_position = 0;
    }

    if (m_rtos_inited) { vTaskResume(drv_monitor_handle); }
    delay_ms(1);

    return initialized;
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::softReset() {
    bool reset = true;
    reset &= writeRegister(TMC4361A_Reg::EVENTS, 0xFFFFFFFF);

    reset &= writeRegister(TMC4361A_Reg::RESET_REG, 0x52535400);
    m_freeze_event      = FreezeEvent::OK;
    m_ignore_next_stall = true;
    m_drv_no_comm       = false;

    // Clear flag & event
    uint32_t dummy;
    clearEvents();
    readStatus(&dummy); // clear first reading

    return reset;
}

/* ---------------------------------------------------------------------------------- */
void TMC4361A::hardReset() {
    resetFreeze();
    toggleNRST();
}

/* ---------------------------------------------------------------------------------- */
void TMC4361A::setDrv(TMC4361A::DrvSettings drv_s) { m_drv_s = drv_s; }
void TMC4361A::setDrv_mstepPerFs(uint16_t mstep_per_fs) { m_drv_s.mstep_per_fs = mstep_per_fs; }
void TMC4361A::setDrv_fsPerRev(uint16_t fs_per_rev) { m_drv_s.fs_per_rev = fs_per_rev; }

/* ---------------------------------------------------------------------------------- */
void TMC4361A::setCurrent(TMC4361A::CurrentSettings current_s) { m_current_s = current_s; }
void TMC4361A::setCurrent_iHold(uint16_t i_hold) { m_current_s.i_hold = (uint8_t)i_hold; }
void TMC4361A::setCurrent_iRun(uint16_t i_run) { m_current_s.i_run = (uint8_t)i_run; }

/* ---------------------------------------------------------------------------------- */
void TMC4361A::setStopOnStall(StallSettings stall_s) { m_stall_s = stall_s; }
void TMC4361A::setStopOnStall_enable(uint16_t enable) { m_stall_s.stop_on_stall = (bool)enable; }
void TMC4361A::setStopOnStall_thresh(uint16_t rpm) { m_stall_s.stall_thresh_rpm = rpm; }

/* ---------------------------------------------------------------------------------- */
void TMC4361A::setClosedLoop(ClosedLoopSettings cl_s) { m_cl_s = cl_s; }
void TMC4361A::setClosedLoop_enable(uint16_t enable) { m_cl_s.enable = (bool)enable; }
void TMC4361A::setClosedLoop_usePID(uint16_t enable_pid) { m_cl_s.enable_pid = (bool)enable_pid; }
void TMC4361A::setClosedLoop_encInRes(uint16_t enc_in_res) { m_cl_s.enc_in_res = enc_in_res; }
void TMC4361A::setClosedLoop_tolerance(uint16_t cl_tolerance) { m_cl_s.cl_tolerance = cl_tolerance; }

/* ---------------------------------------------------------------------------------- */
void TMC4361A::setStealthChopThreshold(uint16_t rpm) { m_stealthchop_thresh_rpm = rpm; }

/* ================================================================================== */
/*                                        Tasks                                       */
/* ================================================================================== */
void TMC4361A::task_stsMonitor(void *parameters) {
    auto *self = static_cast<TMC4361A *>(parameters);

    self->m_rtos_inited = true;

    // Declare
    uint8_t driver_status = 0;
    // bool    drv_standstill    = false;
    bool coil_b_ol         = false;
    bool coil_a_ol         = false;
    bool coil_b_short      = false;
    bool coil_a_short      = false;
    bool overtemp_warning  = false;
    bool overtemp_shutdown = false;
    // bool    sg2_status        = false;

    // Counter
    const uint32_t CHECK_COUNTER       = 1000 / self->STATUS_UPDATE_FREQ_MS; // 1 second
    const uint32_t MIN_COIL_OL_COUNTER = 500 / self->STATUS_UPDATE_FREQ_MS;  // from try and error
    uint32_t       loop_counter        = 0;
    uint32_t       coil_ol_counter     = 0;

    for (;;) {
        vTaskDelay(self->STATUS_UPDATE_FREQ_TICKS);

        uint32_t sts_resp     = 0;
        bool     read_success = false;

        if (self->m_is_frozen) {
            loop_counter    = 0;
            coil_ol_counter = 0;
            continue;
        }

        if (self->m_drv_no_comm) {
            self->emStop(FreezeEvent::TMC5160_COMM_ERR);
            loop_counter    = 0;
            coil_ol_counter = 0;
            continue;
        }

        read_success = self->readStatus(&sts_resp);

        if (!read_success) {
            self->emStop(FreezeEvent::TMC4361A_COMM_ERR);
            loop_counter    = 0;
            coil_ol_counter = 0;
            continue;
        }

        /* ---------------------------------------------------------------------------------- */
        self->m_is_running = self->extractStatus_running(sts_resp);

        /* ---------------------------------------------------------------------------------- */
        driver_status = self->extractStatus_driverStatus(sts_resp);

        // drv_standstill    = driver_status >> 7 & 1;
        coil_b_ol         = driver_status >> 6 & 1;
        coil_a_ol         = driver_status >> 5 & 1;
        coil_b_short      = driver_status >> 4 & 1;
        coil_a_short      = driver_status >> 3 & 1;
        overtemp_warning  = driver_status >> 2 & 1;
        overtemp_shutdown = driver_status >> 1 & 1;
        // sg2_status        = driver_status >> 0 & 1;

        // if (coil_a_ol || coil_b_ol) self->emStop(FreezeEvent::COIL_OL);
        // if (coil_a_ol && coil_b_ol) self->emStop(FreezeEvent::COIL_OL);
        // if (coil_a_ol || coil_b_ol) printf("OL %d\n", self);
        // if (coil_a_ol || coil_b_ol) { coil_ol_counter++; }
        if (coil_a_short || coil_b_short) { self->emStop(FreezeEvent::COIL_SHORT); }
        if (overtemp_warning || overtemp_shutdown) { self->emStop(FreezeEvent::OVERTEMP); }

        /* ---------------------------------------------------------------------------------- */
        self->m_sensor_status  = self->extractStatus_sensor(sts_resp);
        self->m_target_reached = self->extractStatus_flag(TMC4361A::StatusFlags::TARGET_REACHED_F, sts_resp);

        /* ---------------------------------------------------------------------------------- */
        self->m_is_stalled         = self->extractStatus_flag(TMC4361A::StatusFlags::ACTIVE_STALL_F, sts_resp);
        self->m_has_position_error = self->extractStatus_flag(TMC4361A::StatusFlags::ENC_FAIL_F, sts_resp);

        if (self->m_cl_s.enable && self->m_has_position_error) {
            if (!self->m_ignore_next_stall) { self->emStop(FreezeEvent::POS_ERR); }
            self->m_ignore_next_stall = false;
        } else if (self->m_stall_s.stop_on_stall && self->m_is_stalled) {
            self->emStop(FreezeEvent::STALL);
        }

        // Start counter after one coil OL detected
        if (coil_ol_counter != 0) { loop_counter++; }

        // Reset + OL Err
        if (loop_counter == CHECK_COUNTER) {
            // printf("%d OL Count %d | Over ? %d\n", self, coil_ol_counter, coil_ol_counter >= MIN_COIL_OL_COUNTER);
            if (coil_ol_counter >= MIN_COIL_OL_COUNTER) { self->emStop(FreezeEvent::COIL_OL); }
            loop_counter    = 0;
            coil_ol_counter = 0;
        }
    }
}

/* ================================================================================== */
/*                                        Maths                                       */
/* ================================================================================== */
float TMC4361A::spdToRpm(float speed) { return speed / m_drv_s.fs_per_rev / m_drv_s.mstep_per_fs * 60.0f; }

/* ================================================================================== */
/*                                        Test                                        */
/* ================================================================================== */
uint8_t TMC4361A::test_Comm() {
    // Write to motion controller
    bool write_tmc4361a_succeed = writeRegister(TMC4361A_Reg::SPIOUT_CONF, 0x4440128D);

    if (!write_tmc4361a_succeed) { return 0; }

    bool tmc4361_content_matches = verifyWrite(TMC4361A_Reg::SPIOUT_CONF, 0x4440128D);

    if (!tmc4361_content_matches) { return 1; }

    // Write to driver
    bool write_tmc5160_succeed = writeDrvRegister(TMC5160_Reg::CHOPCONF, 0x002101F4);

    if (!write_tmc5160_succeed) { return 2; }

    return 3;
}

/* ================================================================================== */
/*                                        Setup                                       */
/* ================================================================================== */
bool TMC4361A::setFixed() {
    bool succeed = true;
    // Defaults
    succeed &= writeRegister(TMC4361A_Reg::CLK_FREQ, m_fclk);    // set clock frequency
    succeed &= writeRegister(TMC4361A_Reg::DFREEZE, 0x00FFFFFF); // free axis, instant stop on freeze

    uint32_t general_conf_word = 0;
    general_conf_word |= (0 << 0);  // use_astart_and_vstart (for s-shape only)
    general_conf_word |= (1 << 1);  // direct_acc_val_en
    general_conf_word |= (1 << 2);  // direct_bow_val_en
    general_conf_word |= (1 << 5);  // pol_dir_out
    general_conf_word |= (0 << 15); // intr_pol (falling edge interrupt)
    general_conf_word |= (0 << 28); // reverse_motor_dir
    general_conf_word |= (1 << 29); // intr_tr_pu_pd_en
    general_conf_word |= (1 << 30); // intr_as_wired_and

    succeed &= writeRegister(TMC4361A_Reg::GENERAL_CONF, general_conf_word);

    uint32_t stp_length_add_word = 0;
    stp_length_add_word |= (4 << 0);  // STP_LENGTH_ADD
    stp_length_add_word |= (6 << 16); // DIR_SETUP_TIME

    succeed &= writeRegister(TMC4361A_Reg::STP_LENGTH_ADD, stp_length_add_word);
    succeed &= writeRegister(TMC4361A_Reg::SPIOUT_CONF, 0x4440128D);

    succeed &= writeDrvRegister(TMC5160_Reg::CHOPCONF, 0x00000000); // TOFF first for driver reenable
    succeed &= writeDrvRegister(TMC5160_Reg::CHOPCONF, 0x002101F4);
    // succeed &= writeDrvRegister(TMC5160_Reg::CHOPCONF, 0x000100C3);
    // succeed &= writeDrvRegister(TMC5160_Reg::CHOPCONF, 0x00410155);
    succeed &= writeDrvRegister(TMC5160_Reg::TPOWERDOWN, 0x0000000A);

    uint32_t gconf_word = 0;
    gconf_word |= (1 << 2); // en_pwm_mode
    gconf_word |= (1 << 3); //

    succeed &= writeDrvRegister(TMC5160_Reg::GCONF, gconf_word);

    float    tmc5160_internal_clock     = 12800000;
    float    stealthchop_thresh_rpm_f   = (float)m_stealthchop_thresh_rpm;
    uint32_t m_stealthchop_thresh_speed = floatToFixed(rpmToSpd(stealthchop_thresh_rpm_f), 0);

    uint32_t tpwnthrs_word = uint32_t(tmc5160_internal_clock / m_stealthchop_thresh_speed);

    succeed &= writeDrvRegister(TMC5160_Reg::TPWMTHRS, tpwnthrs_word);

    // Step config
    uint32_t step_confg_word = 0;

    if (m_cl_s.enable) m_drv_s.mstep_per_fs = 256; // closed loop override
    // microstepping (bits 3:0)
    switch (m_drv_s.mstep_per_fs) {
    case 1  : step_confg_word |= (8 << 0); break;
    case 2  : step_confg_word |= (7 << 0); break;
    case 4  : step_confg_word |= (6 << 0); break;
    case 8  : step_confg_word |= (5 << 0); break;
    case 16 : step_confg_word |= (4 << 0); break;
    case 32 : step_confg_word |= (3 << 0); break;
    case 64 : step_confg_word |= (2 << 0); break;
    case 128: step_confg_word |= (1 << 0); break;
    case 256: step_confg_word |= (0 << 0); break;
    default : break; // keep driver default value
    }

    step_confg_word |= (((uint32_t)m_drv_s.fs_per_rev & 0xFFF) << 4); // FS_PER_REV (bits 15:4)
    step_confg_word |= (m_drv_s.mstatus_selection << 16);             // MSTATUS_SELECTION (bits 16:23)

    succeed &= writeRegister(TMC4361A_Reg::STEP_CONF, step_confg_word);

    return succeed;
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::setInterrupt() {
    bool succeed = true;

    uint32_t intr_conf_word = 0;
    intr_conf_word |= (m_cl_s.enable << Events::CL_FIT);
    // ?(Bug) This never gets raised
    intr_conf_word |= (m_stall_s.stop_on_stall << Events::STOP_ON_STALL);

    succeed &= writeRegister(TMC4361A_Reg::INTR_CONF, intr_conf_word);
    succeed &= writeRegister(TMC4361A_Reg::EVENT_CLEAR_CONF, intr_conf_word);
    return succeed;
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::setCurrent() {
    uint32_t ihold_irun_word = 0;
    ihold_irun_word |= (m_current_s.i_hold << 0);        // IHOLD (0-31)
    ihold_irun_word |= (m_current_s.i_run << 8);         // IRUN  (0-31)
    ihold_irun_word |= (m_current_s.i_hold_delay << 16); // IHOLDDELAY

    return writeDrvRegister(TMC5160_Reg::IHOLD_IRUN, ihold_irun_word);
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::setClosedLoop() {
    // Sanity check
    if (!m_cl_s.enable) { return true; }

    bool succeed = true;

    // Encoder settings
    succeed &= writeRegister(TMC4361A_Reg::ENC_IN_RES_WR, static_cast<uint32_t>(m_cl_s.enc_in_res)); // encoder res
    succeed &=
        writeRegister(TMC4361A_Reg::CL_TOLERANCE_WR, min(static_cast<uint32_t>(m_cl_s.enc_in_res / 2), 256)); // cl_fit
    succeed &=
        writeRegister(TMC4361A_Reg::ENC_POS_DEV_TOL_WR, static_cast<uint32_t>(m_cl_s.cl_tolerance)); // max follow err
    succeed &=
        writeRegister(TMC4361A_Reg::CL_TR_TOLERANCE_WR, static_cast<uint32_t>(m_cl_s.cl_tolerance)); // target reach

    // PI regulator settings
    succeed &= writeRegister(TMC4361A_Reg::CL_BETA, 255);           // maximum compensation commutation angle
    succeed &= writeRegister(TMC4361A_Reg::CL_DELTA_P_WR, 0xFFFF);  // Keep 0xFFFF to minimize jitter
    succeed &= writeRegister(TMC4361A_Reg::CL_VMAX_CALC_P_WR, 2);   // P of PI regulator
    succeed &= writeRegister(TMC4361A_Reg::CL_VMAX_CALC_I_WR, 3);   // I of PI regulator (min 2)
    succeed &= writeRegister(TMC4361A_Reg::PID_DV_CLIP_WR, 512000); // 10 rps limit
    succeed &= writeRegister(TMC4361A_Reg::PID_I_CLIP_WR, 250);     // Integral clamp

    uint32_t enc_in_conf_word = 0;
    enc_in_conf_word |= (1 << 0);                  // enc_sel_decimal
    enc_in_conf_word |= (1 << 2);                  // clr_latch_cont_on_n
    enc_in_conf_word |= (0 << 9);                  // ignore_ab
    enc_in_conf_word |= (1 << 10);                 // latch_enc_on_n
    enc_in_conf_word |= (1 << 11);                 // latch_x_on_n
    enc_in_conf_word |= (m_cl_s.enable_pid << 22); // regulation_modus (!Do not use at high RPM)
    enc_in_conf_word |= (1 << 25);                 // cl_emf_en
    enc_in_conf_word |= (1 << 27);                 // cl_vlimit_en
    enc_in_conf_word |= (0 << 28);                 // cl_velocity_mode_en

#ifdef XPLAINED
    enc_in_conf_word |= (0 << 29); // invert_enc_dir
#else
    enc_in_conf_word |= (1 << 29); // invert_enc_dir
#endif

    succeed &= writeRegister(TMC4361A_Reg::ENC_IN_CONF, enc_in_conf_word);
    return succeed;
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::setStallDetection() {
    bool succeed = true;
    // Set stall to false if closed loop is active
    if (m_cl_s.enable) m_stall_s.stop_on_stall = false;

    // Stall config
    uint32_t reference_conf_word = 0;
    reference_conf_word |= (m_stall_s.stop_on_stall << 26); // stop_on_stall
    reference_conf_word |= (1 << 27);                       // drive_after_stall

    float stall_thresh_rpm_f = (float)(m_stall_s.stall_thresh_rpm); // 0 dp

    succeed &= writeRegister(TMC4361A_Reg::REFERENCE_CONF, reference_conf_word);
    succeed &= writeRegister(TMC4361A_Reg::VSTALL_LIMIT_WR, floatToFixed(rpmToSpd(stall_thresh_rpm_f), 0));
    return succeed;
}

/* ================================================================================== */
/*                                        Misc                                        */
/* ================================================================================== */
void TMC4361A::raiseStallFlag() { m_stall_detected = true; }

/* ---------------------------------------------------------------------------------- */
void TMC4361A::resetFreeze() {
    ioport_set_pin_level(m_nfreeze_pin, true);
    delay_ms(20);
    m_is_frozen = false;
}

/* ---------------------------------------------------------------------------------- */
void TMC4361A::raiseFreeze() {
    ioport_set_pin_level(m_nfreeze_pin, false);
    delay_ms(20);
    m_is_frozen = true;
}

/* ---------------------------------------------------------------------------------- */
void TMC4361A::toggleNRST() {
    ioport_set_pin_level(m_nrst_pin, false);
    delay_ms(20);
    ioport_set_pin_level(m_nrst_pin, true);
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::waitForCoverDone() {
    uint32_t events;
    while (readEvent(&events)) {
        if (events & (1 << TMC4361A::Events::COVER_DONE)) { return true; }
    }
    return false;
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::verifyWrite(uint8_t reg_addr, uint32_t write_value) {
    uint32_t readback;

    bool read_success = readRegister(reg_addr, &readback);

    return read_success & (write_value == readback);
}

/* ================================================================================== */
/*                                       Status                                       */
/* ================================================================================== */
bool    TMC4361A::readStatus(uint32_t *response) { return readRegister(TMC4361A_Reg::STATUS, response); }
bool    TMC4361A::extractStatus_running(uint32_t sts_resp) { return (sts_resp & 0x78) != 0; }
bool    TMC4361A::extractStatus_flag(StatusFlags flag, uint32_t sts_resp) { return (sts_resp >> flag) & 1; }
uint8_t TMC4361A::extractStatus_driverStatus(uint32_t sts_resp) { return (sts_resp >> 24) & 0xFF; }
uint8_t TMC4361A::extractStatus_sensor(uint32_t sts_resp) { return (sts_resp >> 7) & 0x03; }

/* ================================================================================== */
/*                                        Event                                       */
/* ================================================================================== */
bool TMC4361A::readEvent(uint32_t *response) { return readRegister(TMC4361A_Reg::EVENTS, response); }
bool TMC4361A::readAndClearEvent(TMC4361A::Events event) {
    uint32_t response;

    // Sanity Check
    if (!readEvent(&response)) { return false; }

    // Reset Event
    writeRegister(TMC4361A_Reg::EVENTS, 1 << event);

    return (response >> event) & 1;
}
void TMC4361A::clearEvents() { writeRegister(TMC4361A_Reg::EVENTS, UINT32_MAX); }

/* ================================================================================== */
/*                                        Maths                                       */
/* ================================================================================== */
uint32_t TMC4361A::deltaSqRpmToDeltaSqSpd(float deltaSq_rpm) {
    return static_cast<uint32_t>(deltaSq_rpm) * m_drv_s.fs_per_rev * m_drv_s.mstep_per_fs / 60 / 1024;
}
uint32_t TMC4361A::deltaRpmToDeltaSpd(float delta_rpm) {
    return static_cast<uint32_t>(delta_rpm) * m_drv_s.fs_per_rev * m_drv_s.mstep_per_fs / 60 / 1024;
}
float TMC4361A::rpmToSpd(float rpm) { return rpm * m_drv_s.fs_per_rev * m_drv_s.mstep_per_fs / 60.0f; }

uint32_t TMC4361A::floatToFixed(float value, uint8_t decimalPlaces) {
    value *= (float)(1 << decimalPlaces);
    int32_t signed_val = (int32_t)((value > 0.0) ? (value + 0.5) : (value - 0.5));

    return static_cast<uint32_t>(signed_val);
}

/* ================================================================================== */
/*                                    Communication                                   */
/* ================================================================================== */
bool TMC4361A::readDrvRegister(uint8_t drv_reg, uint32_t *response, uint8_t *drv_sts) {
    // Write to COVER registers
    uint32_t reg_write = static_cast<uint32_t>(drv_reg) + 0x80;
    writeRegister(TMC4361A_Reg::COVER_HIGH_WR, reg_write);
    writeRegister(TMC4361A_Reg::COVER_LOW_WR, 0x00000000);

    waitForCoverDone();

    // Readback
    bool     read_success;
    uint32_t readback_low;
    uint32_t readback_high;
    readRegister(TMC4361A_Reg::COVER_DRV_LOW_RD, &readback_high);
    read_success = readRegister(TMC4361A_Reg::COVER_DRV_HIGH_RD, &readback_low);

    if (drv_sts != nullptr) *drv_sts = readback_low & 0xFF;
    *response = readback_high;

    return read_success;
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::writeDrvRegister(uint8_t drv_reg, uint32_t data) {
    // Write to COVER registers
    uint32_t reg_write = static_cast<uint32_t>(drv_reg) + 0x80;
    writeRegister(TMC4361A_Reg::COVER_HIGH_WR, reg_write);
    writeRegister(TMC4361A_Reg::COVER_LOW_WR, data);

    waitForCoverDone();

    delay_us(100);

    // Readback
    bool     read_success = true;
    uint32_t readback_low;
    uint32_t readback_high;
    read_success &= readRegister(TMC4361A_Reg::COVER_DRV_LOW_RD, &readback_high);
    read_success &= readRegister(TMC4361A_Reg::COVER_DRV_HIGH_RD, &readback_low);

    if (data != 0 && data != 0xFFFFFFFF) {
        if (!read_success || (readback_low == 0 || readback_low == 0xFFFFFFFF)) {
            m_drv_no_comm = true;
            return false;
        }
    }

    delay_us(10);

    return true;
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::readRegister(uint8_t reg_addr, uint32_t *response) {
    // Sanity check
    if (response == nullptr) return false;

    xSemaphoreTake(*m_spi_mutex, portMAX_DELAY);
    spiTransfer(reg_addr & 0x7F, 0); // dummy call
    spiTransfer(reg_addr & 0x7F, 0, response);
    xSemaphoreGive(*m_spi_mutex);

    return *response != 0xFFFFFFFF;
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::writeRegister(uint8_t reg_addr, uint32_t data) {
    xSemaphoreTake(*m_spi_mutex, portMAX_DELAY);
    bool succeed = spiTransfer(reg_addr | 0x80, data);
    xSemaphoreGive(*m_spi_mutex);
    return succeed;
}

/* ---------------------------------------------------------------------------------- */
bool TMC4361A::spiTransfer(uint8_t address, uint32_t data, uint32_t *response) {
    uint8_t       tx_buf[5] = {0};
    uint8_t       rx_buf[5] = {0};
    status_code_t spi_status;

    tx_buf[0] = address;
    tx_buf[1] = (data >> 24) & 0xFF;
    tx_buf[2] = (data >> 16) & 0xFF;
    tx_buf[3] = (data >> 8) & 0xFF;
    tx_buf[4] = data & 0xFF;

    ioport_set_pin_level(m_spi_cs_pin, false);
    delay_us(1);
    spi_status = spi_transceive_packet(m_spi, tx_buf, rx_buf, 5);
    delay_us(1);
    ioport_set_pin_level(m_spi_cs_pin, true);

    if (response != nullptr) {
        *response = ((uint32_t)rx_buf[1] << 24) | ((uint32_t)rx_buf[2] << 16) | ((uint32_t)rx_buf[3] << 8) |
                    (uint32_t)rx_buf[4];
    }

    return spi_status == status_code::STATUS_OK;
}

/* ---------------------------------------------------------------------------------- */