/*
 * CPPFile1.cpp
 *
 * Created: 21/08/2025 10:24:54
 *  Author: Tan
 */
#include "can_interface.h"

CANInterface *CANInterface::p_can_interface = nullptr;

/* ================================================================================== */
/*                                  Interrupt Handler                                 */
/* ================================================================================== */
extern "C" void CAN1_Handler(void) {
    if (CANInterface::getInstance() == nullptr) return;

    uint32_t ul_status_stepper_r = 0;
    uint32_t ul_status_stepper_w = 0;
    uint32_t ul_status_board_r   = 0;
    uint32_t ul_status_board_w   = 0;
    uint32_t ul_status_buff      = 0;

    ul_status_stepper_r = can_mailbox_get_status(CAN1, CANInterface::RX_STEPPER_R_MB);
    ul_status_stepper_w = can_mailbox_get_status(CAN1, CANInterface::RX_STEPPER_W_MB);
    ul_status_board_r   = can_mailbox_get_status(CAN1, CANInterface::RX_BOARD_R_MB);
    ul_status_board_w   = can_mailbox_get_status(CAN1, CANInterface::RX_BOARD_W_MB);
    ul_status_buff      = can_mailbox_get_status(CAN1, CANInterface::RX_BUFF_MB);

    // if (ul_status_stepper_r & CAN_MSR_MRDY) {
    //     can_mailbox_read(CAN1, &CANInterface::getInstance()->rx_stepper_r_mb);

    //     can_mb_conf_t* mb = &CANInterface::getInstance()->rx_stepper_r_mb;

    //     printf("Mailbox index: %u\n", mb->ul_mb_idx);
    //     printf("Object type:   %u\n", mb->uc_obj_type);
    //     printf("ID version:    %u\n", mb->uc_id_ver);
    //     printf("Length:        %u\n", mb->uc_length);
    //     printf("TX priority:   %u\n", mb->uc_tx_prio);
    //     printf("Status:        0x%08X\n", mb->ul_status);
    //     printf("ID mask:       0x%08X\n", mb->ul_id_msk);
    //     printf("ID:            0x%08X\n", mb->ul_id);
    //     printf("FID:           0x%08X\n", mb->ul_fid);
    //     printf("Data low:      0x%08X\n", mb->ul_datal);
    //     printf("Data high:     0x%08X\n", mb->ul_datah);
    // }

    if (ul_status_stepper_r & CAN_MSR_MRDY) {
        can_mailbox_read(CAN1, &CANInterface::getInstance()->rx_stepper_r_mb);
        CANInterface::getInstance()->recv_mailbox = CANInterface::RX_STEPPER_R_MB;
        CANInterface::getInstance()->can_msg_rdy  = true;
    } else if (ul_status_stepper_w & CAN_MSR_MRDY) {
        can_mailbox_read(CAN1, &CANInterface::getInstance()->rx_stepper_w_mb);
        CANInterface::getInstance()->recv_mailbox = CANInterface::RX_STEPPER_W_MB;
        CANInterface::getInstance()->can_msg_rdy  = true;
    } else if (ul_status_board_r & CAN_MSR_MRDY) {
        can_mailbox_read(CAN1, &CANInterface::getInstance()->rx_board_r_mb);
        CANInterface::getInstance()->recv_mailbox = CANInterface::RX_BOARD_R_MB;
        CANInterface::getInstance()->can_msg_rdy  = true;
    } else if (ul_status_board_w & CAN_MSR_MRDY) {
        can_mailbox_read(CAN1, &CANInterface::getInstance()->rx_board_w_mb);
        CANInterface::getInstance()->recv_mailbox = CANInterface::RX_BOARD_W_MB;
        CANInterface::getInstance()->can_msg_rdy  = true;
    } else if (ul_status_buff & CAN_MSR_MRDY) {
        can_mailbox_read(CAN1, &CANInterface::getInstance()->rx_buff_mb);
        CANInterface::getInstance()->recv_mailbox = CANInterface::RX_BUFF_MB;
        CANInterface::getInstance()->can_msg_rdy  = true;
    }
}

/* ================================================================================== */
/*                                     Constructor                                    */
/* ================================================================================== */
CANInterface::CANInterface(uint8_t device_id) : m_device_id(device_id) {
    // Drive monitoring
    xTaskCreate(CANInterface::task_commandHandler, // function name
                (const signed char *)"cmdHandler", // task name
                2048,                              // stack size (100)
                this,                              // task parameters
                1,                                 // task priority
                NULL                               // task handle
    );
}

/* ---------------------------------------------------------------------------------- */
void CANInterface::init() {
    pmc_enable_periph_clk(ID_CAN1);
    uint32_t ul_sysclk = sysclk_get_cpu_hz();

    if (!can_init(CAN1, ul_sysclk, CAN_BPS_500K)) {
        m_configured    = false;
        p_can_interface = nullptr;
        return;
    }

    p_can_interface = this;
    m_configured    = true;

    can_disable_interrupt(CAN1, CAN_DISABLE_ALL_INTERRUPT_MASK);
    can_reset_all_mailbox(CAN1);

    // Create mailbox ids
    rx_stepper_r_mbid = createRxId(RX_STEPPER_R_MB);
    rx_stepper_w_mbid = createRxId(RX_STEPPER_W_MB);
    rx_board_r_mbid   = createRxId(RX_BOARD_R_MB);
    rx_board_w_mbid   = createRxId(RX_BOARD_W_MB);
    rx_buff_mbid      = createRxId(RX_BUFF_MB);

    // Initialize mailboxes
    initRxMailbox(&rx_stepper_r_mb, RX_STEPPER_R_MB, rx_stepper_r_mbid);
    initRxMailbox(&rx_stepper_w_mb, RX_STEPPER_W_MB, rx_stepper_w_mbid);
    initRxMailbox(&rx_board_r_mb, RX_BOARD_R_MB, rx_board_r_mbid);
    initRxMailbox(&rx_board_w_mb, RX_BOARD_W_MB, rx_board_w_mbid);
    initRxMailbox(&rx_buff_mb, RX_BUFF_MB, rx_buff_mbid);
    initTxMailbox(&tx_response_mb, TX_RESPONSE_MB);

    // Enable RX mailbox interrupts
    can_enable_interrupt(CAN1, 1 << RX_STEPPER_R_MB);
    can_enable_interrupt(CAN1, 1 << RX_STEPPER_W_MB);
    can_enable_interrupt(CAN1, 1 << RX_BOARD_R_MB);
    can_enable_interrupt(CAN1, 1 << RX_BOARD_W_MB);
    can_enable_interrupt(CAN1, 1 << RX_BUFF_MB);

    NVIC_EnableIRQ(CAN1_IRQn);
}

/* ---------------------------------------------------------------------------------- */
void CANInterface::deinit() {
    if (!m_configured) return;

    // Disable CAN interrupts
    can_disable_interrupt(CAN1, CAN_DISABLE_ALL_INTERRUPT_MASK);
    NVIC_DisableIRQ(CAN1_IRQn);

    // Reset mailboxes
    can_reset_all_mailbox(CAN1);

    pmc_disable_periph_clk(ID_CAN1);

    m_configured    = false;
    p_can_interface = nullptr;
}

/* ---------------------------------------------------------------------------------- */
void CANInterface::transmitMessage(uint32_t data_l, uint16_t mailbox_id) {
    // Sanity check
    if (!m_configured) return;

    // Minimum interval
    delay_us(200);

    tx_response_mb.ul_id     = CAN_MID_MIDvA(mailbox_id);
    tx_response_mb.uc_length = MAX_CAN_FRAME_DATA_LEN;
    tx_response_mb.ul_datal  = data_l;
    tx_response_mb.ul_datah  = 0;

    // Send the message
    can_mailbox_write(CAN1, &tx_response_mb);

    can_interface_debug("Send [%0.8X]\n", tx_response_mb.ul_datal);

    // Send out the information in the mailbox.
    can_global_send_transfer_cmd(CAN1, CAN_TCR_MB2);

    can_interface_debug("---\n");
}

/* ---------------------------------------------------------------------------------- */
bool CANInterface::isConfigured() { return m_configured; }

/* ---------------------------------------------------------------------------------- */
void CANInterface::registerSteppers(Stepper *steppers[MAX_STEPPERS]) {
    for (int i = 0; i < MAX_STEPPERS; i++) {
        m_steppers[i] = steppers[i]; // copy each pointer
    }
}

/* ---------------------------------------------------------------------------------- */
void CANInterface::registerMotionQueue(MotionQueue *motion_queue) { m_motion_queue = motion_queue; }

/* ---------------------------------------------------------------------------------- */
void CANInterface::registerSdCard(SDCard *sd_card) { m_sd_card = sd_card; }

/* ================================================================================== */
/*                                      Handlers                                      */
/* ================================================================================== */
void CANInterface::handleRXStepperR() {
    StepperReadPayload payload;
    payload.driver_num   = rx_stepper_r_mb.ul_datal >> 28;
    payload.register_num = (rx_stepper_r_mb.ul_datal >> 16) & 0xFFF;

    can_interface_debug("(STEPPER_R) %d, Reg %03X\n", payload.driver_num, payload.register_num);

    uint32_t response     = Response::UNKNOWN;
    bool     read_success = true;

    if (m_steppers[payload.driver_num] == nullptr) {
        response     = Response::DRIVER_IS_NULL;
        read_success = false;
    } else {
        Stepper *stepper = m_steppers[payload.driver_num];

        switch (payload.register_num) {
        case Registers_StepperRead::STATUS: {
            response = static_cast<uint32_t>(stepper->getStatus());
            break;
        }
        case Registers_StepperRead::READBACK: {
            uint32_t readback_value = 0;
            bool     has_readback   = static_cast<uint32_t>(stepper->getReadback(&readback_value));

            if (!has_readback) {
                response = Response::READ_FAIL;
            } else {
                response = readback_value;
            }
            break;
        }
        case Registers_StepperRead::INTERNAL_POSITION: {
            int32_t driver_pulse = stepper->getInternalPosition();
            int32_t user_units   = stepper->pulseToUnit(driver_pulse);

            response = static_cast<uint32_t>(user_units);
            break;
        }
        case Registers_StepperRead::ENCODER_POSITION: {
            int32_t driver_pulse = stepper->getEncoderPosition();
            int32_t user_units   = stepper->pulseToUnit(driver_pulse);

            response = static_cast<uint32_t>(user_units);
            break;
        }
        case Registers_StepperRead::CURRENT_SPEED: {
            int32_t driver_pulse = stepper->getCurrentSpeed();
            int32_t user_units   = stepper->pulseToUnit(driver_pulse);

            response = static_cast<uint32_t>(user_units / 6);
            break;
        }
        case Registers_StepperRead::SENSOR_STATUS: {
            response = static_cast<uint32_t>(stepper->getSensorReading());
            break;
        }
        case Registers_StepperRead::ENCODER_POS_DEV: {
            response = stepper->getEncoderPosDev();
            break;
        }
        default: {
            read_success = false;
            response     = Response::UNKNOWN;
        }
        }
    }

    // Create word
    uint32_t word;

    if (!read_success) {
        word = rx_stepper_w_mb.ul_datal & 0xFFFF0000;
        word |= static_cast<uint32_t>(response);
    } else {
        word = response;
    }

    transmitMessage(word, rx_stepper_r_mbid);
}

/* ---------------------------------------------------------------------------------- */
void CANInterface::handleRXStepperW() {
    StepperWritePayload payload;
    payload.driver_num   = rx_stepper_w_mb.ul_datal >> 28;
    payload.register_num = (rx_stepper_w_mb.ul_datal >> 16) & 0xFFF;
    payload.data         = rx_stepper_w_mb.ul_datal & 0xFFFF;

    can_interface_debug("(STEPPER_W) %d, Reg %03X, Data %04X\n", payload.driver_num, payload.register_num,
                        payload.data);
    uint16_t response = Response::UNKNOWN;

    if (m_steppers[payload.driver_num] == nullptr)
        response = Response::DRIVER_IS_NULL;
    else if (payload.register_num <= REG_WRITE_CONTROL_MAX)
        response = execWriteControl(payload);
    else if (payload.register_num <= REG_WRITE_DRIVER_MAX)
        response = execWriteDriver(payload);
    else if (payload.register_num <= REG_WRITE_MOTION_PARAM_MAX)
        response = execWriteMotionParam(payload);
    else if (payload.register_num <= REG_WRITE_HOME_MAX)
        response = execWriteHoming(payload);
    else if (payload.register_num <= REG_WRITE_MOTION_SETTINGS_MAX)
        response = execWriteMotionSettings(payload);
    else
        response = Response::UNKNOWN;

    // Create word
    uint32_t word;
    word = rx_stepper_w_mb.ul_datal & 0xFFFF0000;
    word |= static_cast<uint32_t>(response);

    transmitMessage(word, rx_stepper_w_mbid);
}

/* ---------------------------------------------------------------------------------- */
void CANInterface::handleRXBoardR() {
    BoardWritePayload payload;
    payload.register_num = (rx_board_r_mb.ul_datal >> 16) & 0xFFF;
    payload.data         = rx_board_r_mb.ul_datal & 0xFFFF;

    can_interface_debug("(BOARD_R) Reg %03X, Data %04X\n", payload.register_num, payload.data);

    // SD Card Related
    if (payload.register_num >= 0x700 && payload.register_num <= 0x7FF) {
        execSDRead(payload);
        return;
    }

    uint16_t response = Response::UNKNOWN;

    switch (payload.register_num) {
    case Registers_Board::FIRMWARE_VER: {
        transmitFrames(FIRMWARE_VERSION);
        return;
    }
    case Registers_Board::MOTION_READBACK: {
        uint32_t readback_value = 0;

        bool has_readback = m_motion_queue->getReadback(&readback_value);

        if (!has_readback) {
            response = Response::READ_FAIL;
        } else {
            response = readback_value;
        }
        break;
    }
    case Registers_Board::QSTATUS: {
        response = m_motion_queue->getQueueStatus();
        break;
    }
    default: {
        break;
    }
    }

    // Create word
    uint32_t word;
    word = rx_board_w_mb.ul_datal & 0x0FFF0000;
    word |= static_cast<uint32_t>(response);

    transmitMessage(word, rx_board_w_mbid);
}

/* ---------------------------------------------------------------------------------- */
void CANInterface::handleRXBoardW() {
    BoardWritePayload payload;
    payload.register_num = (rx_board_w_mb.ul_datal >> 16) & 0xFFF;
    payload.data         = rx_board_w_mb.ul_datal & 0xFFFF;

    can_interface_debug("(BOARD_W) Reg %03X, Data %04X\n", payload.register_num, payload.data);

    uint16_t response = Response::UNKNOWN;

    if (payload.register_num == Registers_Board::SELECT_MOTION_AND_QUEUE_ITEM) {
        response = selectMotionAndQueue(payload.data);
    } else if (payload.register_num == Registers_Board::MOTION_CONTROL_WORD) {
        if (payload.data == MotionControlWord::ABORT) {
            response = m_motion_queue->abortQueue() ? Response::SUCCESS : Response::WRITE_FAIL;
        } else {
            response = static_cast<uint16_t>(m_motion_queue->queueMotion(static_cast<uint8_t>(payload.data)));
        }
    } else if (payload.register_num == Registers_Board::RUN_PRELOAD) {
        uint8_t driver_num = 0xF;
        response           = static_cast<uint16_t>(
            m_motion_queue->queuePreloadedMotions(static_cast<uint8_t>(payload.data), &driver_num));
        response |= (driver_num << 8);
    } else if (payload.register_num >= 0x700 && payload.register_num <= 0x7FF) {
        response = execSDWrite(payload);
    } else {
        response = execBoardWrite(payload);
    }

    // Create word
    uint32_t word;
    word = rx_board_w_mb.ul_datal & 0x0FFF0000;
    word |= static_cast<uint32_t>(response);

    transmitMessage(word, rx_board_w_mbid);
}

/* ---------------------------------------------------------------------------------- */
void CANInterface::handleBuff() {
    uint32_t data     = rx_buff_mb.ul_datal;
    uint16_t response = Response::UNKNOWN;

    can_interface_debug("(BUFF) Data %0.8X", data);

    if (data == START_WORD) {
        if (buffer_listen_end) {
            response = Response::WRITE_FAIL;
            can_interface_debug("-> Clear buffer first!\n");
        } else {
            buffer_listen_start = true;
            can_interface_debug("-> Start Byte\n");
            response = Response::SUCCESS;
        }
    } else if (data == END_WORD) {
        if (!buffer_listen_start) {
            response = Response::WRITE_FAIL;
            can_interface_debug("-> End Byte received before starting!\n");
        } else {
            buffer_listen_end = true;
            can_interface_debug("-> End Byte\n");
            response = Response::SUCCESS;
        }
    } else {
        if (buffer_listen_start) {
            can_interface_debug("-> [%d]\n", buffer_len);
            buffer[buffer_len++] = (data >> 0) & 0xFF;
            buffer[buffer_len++] = (data >> 8) & 0xFF;
            buffer[buffer_len++] = (data >> 16) & 0xFF;
            buffer[buffer_len++] = (data >> 24) & 0xFF;
        } else {
            can_interface_debug("Start Word Not Received!\n", buffer_len);
        }
        // No transmit
        return;
    }

    transmitMessage(static_cast<uint32_t>(response), rx_buff_mbid);
}

/* ================================================================================== */
/*                                        Task                                        */
/* ================================================================================== */

void CANInterface::task_commandHandler(void *parameters) {
    for (;;) {
        if (CANInterface::getInstance()->can_msg_rdy) {
            switch (CANInterface::getInstance()->recv_mailbox) {
            case RX_STEPPER_R_MB: {
                CANInterface::getInstance()->handleRXStepperR();
                break;
            }
            case RX_STEPPER_W_MB: {
                CANInterface::getInstance()->handleRXStepperW();
                break;
            }
            case RX_BOARD_R_MB: {
                CANInterface::getInstance()->handleRXBoardR();
                break;
            }
            case RX_BOARD_W_MB: {
                CANInterface::getInstance()->handleRXBoardW();
                break;
            }
            case RX_BUFF_MB: {
                CANInterface::getInstance()->handleBuff();
                break;
            }
            }

            CANInterface::getInstance()->can_msg_rdy = false;
        }
    }
}

/* ================================================================================== */
/*                                       Utility                                      */
/* ================================================================================== */

void CANInterface::initRxMailbox(can_mb_conf_t *p_mailbox, uint8_t mailbox_id, uint16_t rx_id) {
    resetMailboxConf(p_mailbox);
    p_mailbox->ul_mb_idx   = mailbox_id;
    p_mailbox->uc_obj_type = CAN_MB_RX_MODE;
    p_mailbox->uc_length   = MAX_CAN_FRAME_DATA_LEN;
    p_mailbox->ul_id_msk   = CAN_MAM_MIDvA_Msk | CAN_MAM_MIDvB_Msk;
    p_mailbox->ul_id       = CAN_MID_MIDvA(rx_id);

    // p_mailbox->ul_id_msk = 0;
    // p_mailbox->ul_id     = 0;

    can_mailbox_init(CAN1, p_mailbox);
}

/* ---------------------------------------------------------------------------------- */
void CANInterface::initTxMailbox(can_mb_conf_t *p_mailbox, uint8_t mailbox_id) {
    resetMailboxConf(p_mailbox);
    p_mailbox->ul_mb_idx   = mailbox_id;
    p_mailbox->uc_obj_type = CAN_MB_TX_MODE;
    p_mailbox->uc_tx_prio  = CAN_TX_PRIO;
    p_mailbox->uc_id_ver   = 0;
    p_mailbox->ul_id_msk   = 0;
    can_mailbox_init(CAN1, p_mailbox);
}

/* ---------------------------------------------------------------------------------- */
uint32_t CANInterface::createRxId(uint8_t mailbox_id) { return (mailbox_id << 8) | m_device_id; }

/* ---------------------------------------------------------------------------------- */
void CANInterface::resetMailboxConf(can_mb_conf_t *p_mailbox) {
    p_mailbox->ul_mb_idx   = 0;
    p_mailbox->uc_obj_type = 0;
    p_mailbox->uc_id_ver   = 0;
    p_mailbox->uc_length   = 0;
    p_mailbox->uc_tx_prio  = 0;
    p_mailbox->ul_status   = 0;
    p_mailbox->ul_id_msk   = 0;
    p_mailbox->ul_id       = 0;
    p_mailbox->ul_fid      = 0;
    p_mailbox->ul_datal    = 0;
    p_mailbox->ul_datah    = 0;
}

/* ---------------------------------------------------------------------------------- */
void CANInterface::transmitFrames(const char *content) {
    // Send the start frame
    transmitMessage(START_WORD, rx_board_w_mbid);

    size_t len = strlen(content);
    size_t i   = 0;

    while (i < len) {
        uint32_t word = 0;

        // Pack up to 4 bytes into a 32-bit payload
        for (int b = 0; b < 4; b++) {
            word |= (uint32_t)(uint8_t)content[i] << (8 * b);

            if (i < len)
                i++;
            else
                break;
        }

        transmitMessage(word, rx_board_w_mbid);
    }

    // Send the end frame
    transmitMessage(END_WORD, rx_board_w_mbid);
}

/* ================================================================================== */
/*                                       Handler                                      */
/* ================================================================================== */
uint16_t CANInterface::execWriteControl(StepperWritePayload payload) {
    bool     stepper_used_by_queue = m_motion_queue->isStepperUsedInQueue(payload.driver_num);
    Stepper *stepper               = m_steppers[payload.driver_num];

    // EmStop aborts motion queue as well
    if (payload.data == ControlWord::EMERGENCY_STOP) {
        if (stepper_used_by_queue) { m_motion_queue->abortQueue(); } // abort whole queue
        stepper->d_emStop();
        return Response::SUCCESS;
    }

    // Block operations if used by queue
    if (stepper_used_by_queue) { return Response::USED_BY_QUEUE; }

    switch (payload.data) {
    // Setup
    case ControlWord::INIT: {
        return static_cast<uint16_t>(stepper->initController());
    }
    case ControlWord::SET_MOTION: {
        return static_cast<uint16_t>(stepper->setMotion());
    }
    case ControlWord::ENABLE_AXIS: {
        return stepper->d_enableDriver() ? Response::SUCCESS : Response::WRITE_FAIL;
    }
    case ControlWord::RELEASE_AXIS: {
        return stepper->d_releaseDriver() ? Response::SUCCESS : Response::WRITE_FAIL;
    }
    // Move
    case ControlWord::MOVE: {
        return static_cast<uint16_t>(stepper->move());
    }
    case ControlWord::MOVE_HOMING: {
        return static_cast<uint16_t>(stepper->moveHoming());
    }
    case ControlWord::MOVE_INVERSE_TIME: {
        return static_cast<uint16_t>(stepper->moveInverseTime());
    }
    case ControlWord::MOVE_VIBRATION: {
        return static_cast<uint16_t>(stepper->moveVibration());
    }
    // Stop
    case ControlWord::RAMP_STOP: {
        stepper->d_rampStop();
        return Response::SUCCESS;
    }
    // Misc
    case ControlWord::SET_ZERO: {
        return stepper->d_setPosition(0) ? Response::SUCCESS : Response::WRITE_FAIL;
    }
    default: return Response::INVALID_INPUT;
    }
}

/* ---------------------------------------------------------------------------------- */
uint16_t CANInterface::execWriteDriver(StepperWritePayload payload) {
    bool                  stepper_used_by_queue = m_motion_queue->isStepperUsedInQueue(payload.driver_num);
    Stepper              *stepper               = m_steppers[payload.driver_num];
    commandFunctionPtr    cmd_fnc               = nullptr;
    validationFunctionPtr val_fnc               = nullptr;
    bool                  rst_drv               = true;
    bool                  rst_motion            = false;

    // Block operations if used by queue
    if (stepper_used_by_queue) { return Response::USED_BY_QUEUE; }

    switch (payload.register_num) {
    case Registers_StepperWrite::DRV_MSTEP_PER_FS: {
        cmd_fnc = &Stepper::setDrv_mstepPerFs;
        val_fnc = val_ustep;
        break;
    }
    case Registers_StepperWrite::DRV_FS_PER_REV: {
        cmd_fnc = &Stepper::setDrv_fsPerRev;
        val_fnc = val_12b;
        break;
    }
    case Registers_StepperWrite::CURRENT_HOLD: {
        cmd_fnc = &Stepper::setCurrent_iHold;
        val_fnc = val_current;
        break;
    }
    case Registers_StepperWrite::CURRENT_RUN: {
        cmd_fnc = &Stepper::setCurrent_iRun;
        val_fnc = val_current;
        break;
    }
    case Registers_StepperWrite::STOP_ON_STALL_ENABLE: {
        cmd_fnc = &Stepper::setStopOnStall_enable;
        val_fnc = val_bool;
        break;
    }
    case Registers_StepperWrite::STOP_ON_STALL_THRESH: {
        cmd_fnc = &Stepper::setStopOnStall_thresh;
        break;
    }
    case Registers_StepperWrite::CL_ENABLE: {
        cmd_fnc = &Stepper::setClosedLoop_enable;
        val_fnc = val_bool;
        break;
    }
    case Registers_StepperWrite::CL_ENABLE_PID: {
        cmd_fnc = &Stepper::setClosedLoop_usePID;
        val_fnc = val_bool;
        break;
    }
    case Registers_StepperWrite::CL_ENC_IN_RES: {
        cmd_fnc = &Stepper::setClosedLoop_encInRes;
        break;
    }
    case Registers_StepperWrite::CL_TOLERANCE: {
        cmd_fnc = &Stepper::setClosedLoop_tolerance;
        break;
    }
    case Registers_StepperWrite::STEALTH_CHOP_THRESH: {
        cmd_fnc = &Stepper::setStealthChopThreshold;
        break;
    }
    default: {
        break;
    }
    }
    return execWriteCommand(stepper, cmd_fnc, payload.data, val_fnc, rst_drv, rst_motion);
}

/* ---------------------------------------------------------------------------------- */
uint16_t CANInterface::execWriteMotionParam(StepperWritePayload payload) {
    bool                  stepper_used_by_queue   = m_motion_queue->isStepperUsedInQueue(payload.driver_num);
    Stepper              *stepper                 = m_steppers[payload.driver_num];
    commandFunctionPtr    cmd_fnc                 = nullptr;
    validationFunctionPtr val_fnc                 = nullptr;
    bool                  rst_drv                 = false;
    bool                  rst_motion              = false;
    bool                  can_write_while_running = false;

    // Block operations if used by queue
    if (stepper_used_by_queue) { return Response::USED_BY_QUEUE; }

    switch (payload.register_num) {
    case Registers_StepperWrite::MOVE_UNIT_HIGH: {
        cmd_fnc                 = &Stepper::setTargetUnitsHigh;
        can_write_while_running = true;
        break;
    }
    case Registers_StepperWrite::MOVE_UNIT_LOW: {
        cmd_fnc                 = &Stepper::setTargetUnitsLow;
        can_write_while_running = true;
        break;
    }
    case Registers_StepperWrite::MOVE_TIME_MS_HIGH: {
        cmd_fnc = &Stepper::setTimeMsHigh;
        break;
    }
    case Registers_StepperWrite::MOVE_TIME_MS_LOW: {
        cmd_fnc = &Stepper::setTimeMsLow;
        break;
    }
    case Registers_StepperWrite::MOVE_VIB_I: {
        cmd_fnc = &Stepper::setVibrationIterations;
        break;
    }
    case Registers_StepperWrite::MOVE_VIB_DIM_FACTOR: {
        cmd_fnc = &Stepper::setDiminishingFactor;
        val_fnc = val_lessThan1000;
        break;
    }
    case Registers_StepperWrite::MOVE_VIB_LOOP: {
        cmd_fnc = &Stepper::setLoop;
        val_fnc = val_bool;
        break;
    }
    case Registers_StepperWrite::MOVE_RESET_MOTION_CONF_AFTER_EACH_MOVE: {
        cmd_fnc = &Stepper::confMove_resetMotionConfAfterEachMove;
        val_fnc = val_bool;
        break;
    }
    case Registers_StepperWrite::MOVE_ALLOW_WRITE_MOTION_WHEN_BUSY: {
        cmd_fnc = &Stepper::confMove_allowWriteMotionWhenBusy;
        val_fnc = val_bool;
        break;
    }
    case Registers_StepperWrite::SPEED_MAX_H: {
        cmd_fnc = &Stepper::confSpeed_maxSpeed_h;
        break;
    }
    case Registers_StepperWrite::SPEED_MAX_L: {
        cmd_fnc = &Stepper::confSpeed_maxSpeed_l;
        break;
    }
    case Registers_StepperWrite::SPEED_START_H: {
        cmd_fnc = &Stepper::confSpeed_startSpeed_h;
        break;
    }
    case Registers_StepperWrite::SPEED_START_L: {
        cmd_fnc = &Stepper::confSpeed_startSpeed_l;
        break;
    }
    case Registers_StepperWrite::SPEED_STOP_H: {
        cmd_fnc = &Stepper::confSpeed_stopSpeed_h;
        break;
    }
    case Registers_StepperWrite::SPEED_STOP_L: {
        cmd_fnc = &Stepper::confSpeed_stopSpeed_l;
        break;
    }
    case Registers_StepperWrite::SPEED_BREAK_H: {
        cmd_fnc = &Stepper::confSpeed_breakSpeed_h;
        break;
    }
    case Registers_StepperWrite::SPEED_BREAK_L: {
        cmd_fnc = &Stepper::confSpeed_breakSpeed_l;
        break;
    }
    case Registers_StepperWrite::ACC_MAX_ACCEL_H: {
        cmd_fnc = &Stepper::confAccel_maxAccel_h;
        break;
    }
    case Registers_StepperWrite::ACC_MAX_ACCEL_L: {
        cmd_fnc = &Stepper::confAccel_maxAccel_l;
        break;
    }
    case Registers_StepperWrite::ACC_MAX_DECEL_H: {
        cmd_fnc = &Stepper::confAccel_maxDecel_h;
        break;
    }
    case Registers_StepperWrite::ACC_MAX_DECEL_L: {
        cmd_fnc = &Stepper::confAccel_maxDecel_l;
        break;
    }
    case Registers_StepperWrite::ACC_START_ACCEL_H: {
        cmd_fnc = &Stepper::confAccel_startAccel_h;
        break;
    }
    case Registers_StepperWrite::ACC_START_ACCEL_L: {
        cmd_fnc = &Stepper::confAccel_startAccel_l;
        break;
    }
    case Registers_StepperWrite::ACC_FINAL_DECEL_H: {
        cmd_fnc = &Stepper::confAccel_finalDecel_h;
        break;
    }
    case Registers_StepperWrite::ACC_FINAL_DECEL_L: {
        cmd_fnc = &Stepper::confAccel_finalDecel_l;
        break;
    }
    case Registers_StepperWrite::BOW1_H: {
        cmd_fnc = &Stepper::confBow_bow1_h;
        break;
    }
    case Registers_StepperWrite::BOW1_L: {
        cmd_fnc = &Stepper::confBow_bow1_l;
        break;
    }
    case Registers_StepperWrite::BOW2_H: {
        cmd_fnc = &Stepper::confBow_bow2_h;
        break;
    }
    case Registers_StepperWrite::BOW2_L: {
        cmd_fnc = &Stepper::confBow_bow2_l;
        break;
    }
    case Registers_StepperWrite::BOW3_H: {
        cmd_fnc = &Stepper::confBow_bow3_h;
        break;
    }
    case Registers_StepperWrite::BOW3_L: {
        cmd_fnc = &Stepper::confBow_bow3_l;
        break;
    }
    case Registers_StepperWrite::BOW4_H: {
        cmd_fnc = &Stepper::confBow_bow4_h;
        break;
    }
    case Registers_StepperWrite::BOW4_L: {
        cmd_fnc = &Stepper::confBow_bow4_l;
        break;
    }
    default: {
        break;
    }
    }
    return execWriteCommand(stepper, cmd_fnc, payload.data, val_fnc, rst_drv, rst_motion, can_write_while_running);
}

/* ---------------------------------------------------------------------------------- */
uint16_t CANInterface::execWriteHoming(StepperWritePayload payload) {
    bool                  stepper_used_by_queue   = m_motion_queue->isStepperUsedInQueue(payload.driver_num);
    Stepper              *stepper                 = m_steppers[payload.driver_num];
    commandFunctionPtr    cmd_fnc                 = nullptr;
    validationFunctionPtr val_fnc                 = nullptr;
    bool                  rst_drv                 = false;
    bool                  rst_motion              = false;
    bool                  can_write_while_running = true;

    // Block operations if used by queue
    if (stepper_used_by_queue) { return Response::USED_BY_QUEUE; }

    switch (payload.register_num) {
    case Registers_StepperWrite::HOME_HOMING_MODE: {
        cmd_fnc = &Stepper::confHome_homingMode;
        val_fnc = val_homingMode;
        break;
    }
    case Registers_StepperWrite::HOME_HOMING_SENSOR: {
        cmd_fnc = &Stepper::confHome_homingSensor;
        val_fnc = val_homingSensor;
        break;
    }
    case Registers_StepperWrite::HOME_SENSOR_HOME_VALUE: {
        cmd_fnc = &Stepper::confHome_sensorHomeValue;
        val_fnc = val_bool;
        break;
    }
    case Registers_StepperWrite::HOME_MAX_FIND_H: {
        cmd_fnc = &Stepper::confHome_maxFind_h;
        break;
    }
    case Registers_StepperWrite::HOME_MAX_FIND_L: {
        cmd_fnc = &Stepper::confHome_maxFind_l;
        break;
    }
    case Registers_StepperWrite::HOME_MAX_SPEED_H: {
        cmd_fnc = &Stepper::confHome_maxSpeed_h;
        break;
    }
    case Registers_StepperWrite::HOME_MAX_SPEED_L: {
        cmd_fnc = &Stepper::confHome_maxSpeed_l;
        break;
    }
    case Registers_StepperWrite::HOME_MAX_ACCEL_H: {
        cmd_fnc = &Stepper::confHome_maxAccel_h;
        break;
    }
    case Registers_StepperWrite::HOME_MAX_ACCEL_L: {
        cmd_fnc = &Stepper::confHome_maxAccel_l;
        break;
    }
    case Registers_StepperWrite::HOME_MAX_DECEL_H: {
        cmd_fnc = &Stepper::confHome_maxDecel_h;
        break;
    }
    case Registers_StepperWrite::HOME_MAX_DECEL_L: {
        cmd_fnc = &Stepper::confHome_maxDecel_l;
        break;
    }
    case Registers_StepperWrite::HOME_OFFSET_H: {
        cmd_fnc = &Stepper::confHome_offset_h;
        break;
    }
    case Registers_StepperWrite::HOME_OFFSET_L: {
        cmd_fnc = &Stepper::confHome_offset_l;
        break;
    }
    case Registers_StepperWrite::HOME_TIMEOUT_MS_H: {
        cmd_fnc = &Stepper::confHome_timeout_h;
        break;
    }
    case Registers_StepperWrite::HOME_TIMEOUT_MS_L: {
        cmd_fnc = &Stepper::confHome_timeout_l;
        break;
    }
    default: {
        break;
    }
    }
    return execWriteCommand(stepper, cmd_fnc, payload.data, val_fnc, rst_drv, rst_motion, can_write_while_running);
}

/* ---------------------------------------------------------------------------------- */
uint16_t CANInterface::execWriteMotionSettings(StepperWritePayload payload) {
    bool                  stepper_used_by_queue = m_motion_queue->isStepperUsedInQueue(payload.driver_num);
    Stepper              *stepper               = m_steppers[payload.driver_num];
    commandFunctionPtr    cmd_fnc               = nullptr;
    validationFunctionPtr val_fnc               = nullptr;
    bool                  rst_drv               = false;
    bool                  rst_motion            = true;

    // Block operations if used by queue
    if (stepper_used_by_queue) { return Response::USED_BY_QUEUE; }

    switch (payload.register_num) {
    case Registers_StepperWrite::POS_MODE: {
        cmd_fnc = &Stepper::confPositioning_posMode;
        val_fnc = val_posMode;
        break;
    }
    case Registers_StepperWrite::POS_FOLLOW_MODE: {
        cmd_fnc = &Stepper::confPositioning_followMode;
        val_fnc = val_followMode;
        break;
    }
    case Registers_StepperWrite::POS_UNIT_PER_REV: {
        rst_drv    = false;
        rst_motion = true;
        cmd_fnc    = &Stepper::confPositioning_unitPerRev;
        break;
    }
    case Registers_StepperWrite::RAMP_MODE: {
        cmd_fnc = &Stepper::confRamp_rampMode;
        val_fnc = val_rampMode;
        break;
    }
    case Registers_StepperWrite::RAMP_TYPE: {
        cmd_fnc = &Stepper::confRamp_rampType;
        val_fnc = val_rampType;
        break;
    }
    default: {
        break;
    }
    }
    return execWriteCommand(stepper, cmd_fnc, payload.data, val_fnc, rst_drv, rst_motion);
}

/* ---------------------------------------------------------------------------------- */
uint16_t CANInterface::execWriteCommand(Stepper *stepper, commandFunctionPtr cmd_fnc, uint16_t value,
                                        validationFunctionPtr val_fnc, bool rst_drv, bool rst_motion,
                                        bool can_write_while_running) {

    // Sanity check
    if (cmd_fnc == nullptr) return Response::UNKNOWN;

    // Value check
    bool valid_value = (val_fnc != nullptr) ? val_fnc(value) : true;
    if (!valid_value) return Response::INVALID_INPUT;

    // Status check
    if (!can_write_while_running && stepper->isRunning()) return Response::BUSY;

    // Execute
    (stepper->*cmd_fnc)(value);

    // Flag require driver reset
    if (rst_drv) stepper->rstDrvFlag();

    // Flag require motion reset
    if (rst_motion) stepper->rstMotionFlag();

    return Response::SUCCESS;
}

/* ================================================================================== */
/*                                        Board                                       */
/* ================================================================================== */

uint16_t CANInterface::selectMotionAndQueue(uint16_t data) {
    uint8_t motion_num = static_cast<uint8_t>((data >> 8) & 0xFF);
    uint8_t item_num   = static_cast<uint8_t>(data & 0xFF);

    if ((motion_num > MAX_MOTIONS && motion_num != TEST_MOTION_NUM) || (item_num > MAX_ITEMS_PER_MOTION)) {
        return Response::VALUE_OUT_OF_BOUNDS;
    }

    m_motion_sel = motion_num;
    m_item_sel   = item_num;

    can_interface_debug("(Write) Board, Select [%d][%d]\n", m_motion_sel, m_item_sel);
    return Response::SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint16_t CANInterface::execBoardWrite(BoardWritePayload payload) {
    validationFunctionPtr     val = [](uint16_t data) { return true; };
    MotionQueue::QueueItemKey key;

    uint16_t queue_status       = m_motion_queue->getQueueStatus();
    bool     queue_is_executing = static_cast<bool>((queue_status >> 15) & 1);

    if (queue_is_executing) { return Response::USED_BY_QUEUE; }

    switch (payload.register_num) {
    case Registers_Board::QITEM_STEPPER_NUM: {
        val = val_maxSteppers;
        key = MotionQueue::QueueItemKey::DRIVER_NUMBER;
        break;
    }
    case Registers_Board::QITEM_MOTION_TYPE: {
        val = val_motionType;
        key = MotionQueue::QueueItemKey::MOTION_TYPE;
        break;
    }
    case Registers_Board::QITEM_SEQUENCE_NUMBER: {
        val = val_sequenceNumber;
        key = MotionQueue::QueueItemKey::SEQUENCE_NUMBER;
        break;
    }
    case Registers_Board::QITEM_POSITION_H: {
        key = MotionQueue::QueueItemKey::POSITION_H;
        break;
    }
    case Registers_Board::QITEM_POSITION_L: {
        key = MotionQueue::QueueItemKey::POSITION_L;
        break;
    }
    case Registers_Board::QITEM_RAMP_TYPE: {
        val = val_rampType;
        key = MotionQueue::QueueItemKey::RAMP_TYPE;
        break;
    }
    case Registers_Board::QITEM_MAX_SPEED_H: {
        key = MotionQueue::QueueItemKey::MAX_SPEED_H;
        break;
    }
    case Registers_Board::QITEM_MAX_SPEED_L: {
        key = MotionQueue::QueueItemKey::MAX_SPEED_L;
        break;
    }
    case Registers_Board::QITEM_ACCEL_H: {
        key = MotionQueue::QueueItemKey::MAX_ACCEL_H;
        break;
    }
    case Registers_Board::QITEM_ACCEL_L: {
        key = MotionQueue::QueueItemKey::MAX_ACCEL_L;
        break;
    }
    case Registers_Board::QITEM_DECEL_H: {
        key = MotionQueue::QueueItemKey::MAX_DECEL_H;
        break;
    }
    case Registers_Board::QITEM_DECEL_L: {
        key = MotionQueue::QueueItemKey::MAX_DECEL_L;
        break;
    }
    case Registers_Board::QITEM_BOW1_H: {
        key = MotionQueue::QueueItemKey::BOW1_H;
        break;
    }
    case Registers_Board::QITEM_BOW1_L: {
        key = MotionQueue::QueueItemKey::BOW1_L;
        break;
    }
    case Registers_Board::QITEM_BOW2_H: {
        key = MotionQueue::QueueItemKey::BOW2_H;
        break;
    }
    case Registers_Board::QITEM_BOW2_L: {
        key = MotionQueue::QueueItemKey::BOW2_L;
        break;
    }
    case Registers_Board::QITEM_BOW3_H: {
        key = MotionQueue::QueueItemKey::BOW3_H;
        break;
    }
    case Registers_Board::QITEM_BOW3_L: {
        key = MotionQueue::QueueItemKey::BOW3_L;
        break;
    }
    case Registers_Board::QITEM_BOW4_H: {
        key = MotionQueue::QueueItemKey::BOW4_H;
        break;
    }
    case Registers_Board::QITEM_BOW4_L: {
        key = MotionQueue::QueueItemKey::BOW4_L;
        break;
    }
    case Registers_Board::QITEM_USE_INVERSE_TIME: {
        val = val_bool;
        key = MotionQueue::QueueItemKey::USE_INVERSE_TIME;
        break;
    }
    case Registers_Board::QITEM_TIME_H: {
        key = MotionQueue::QueueItemKey::TIME_H;
        break;
    }
    case Registers_Board::QITEM_TIME_L: {
        key = MotionQueue::QueueItemKey::TIME_L;
        break;
    }
    case Registers_Board::QITEM_OFFSET_H: {
        key = MotionQueue::QueueItemKey::OFFSET_H;
        break;
    }
    case Registers_Board::QITEM_OFFSET_L: {
        key = MotionQueue::QueueItemKey::OFFSET_L;
        break;
    }
    case Registers_Board::QITEM_HOMING_MODE: {
        val = val_homingMode;
        key = MotionQueue::QueueItemKey::HOMING_MODE;
        break;
    }
    case Registers_Board::QITEM_HOMING_SENSOR: {
        val = val_homingSensor;
        key = MotionQueue::QueueItemKey::HOMING_SENSOR;
        break;
    }
    case Registers_Board::QITEM_SENSOR_HOME_VALUE: {
        val = val_bool;
        key = MotionQueue::QueueItemKey::SENSOR_HOME_VALUE;
        break;
    }
    default: {
        key = MotionQueue::QueueItemKey::SENSOR_HOME_VALUE; // Dummy so that compiler doesn't complain
        val = [](uint16_t data) { return false; };
        break;
    }
    }

    if (!val(payload.data)) {
        can_interface_debug("(execBoardWrite) Key %d failed check", key);
        return Response::INVALID_INPUT;
    }

    m_motion_queue->writeMotionItem(key, m_motion_sel, m_item_sel, payload.data);
    return Response::SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
void CANInterface::execSDRead(BoardWritePayload payload) {
    uint16_t response = Response::UNKNOWN;

    switch (payload.register_num) {
    // -------- Transmit driver config file from SD Card
    case Registers_Board::SD_FETCH_DRV_CONFIG: {
        if (payload.data >= MAX_STEPPERS) {
            response = Response::VALUE_OUT_OF_BOUNDS;
            break;
        }

        char  file_name[64];
        char *file_contents = NULL;
        snprintf(file_name, 64, "config/%d", payload.data);
        can_interface_debug("file_name: %s\n", file_name);

        m_sd_card->mount();
        bool file_read = m_sd_card->readFile(file_name, &file_contents);
        m_sd_card->unmount();

        if (!file_read) {
            response = Response::READ_FAIL;
            free(file_contents);
            break;
        }

        transmitFrames(file_contents);

        free(file_contents);

        return;
    }
    // -------- Transmit motion file from SD Card with file name appended
    case Registers_Board::SD_FETCH_MOTION_CONFIG: {
        // Get motion file name
        char motion_file_name[64];

        m_sd_card->mount();
        bool motion_file_exist = parse_motionFileName(m_sd_card, payload.data, motion_file_name);
        m_sd_card->unmount();

        if (!motion_file_exist) {
            response = Response::READ_FAIL;
            break;
        }

        // Get content from file name
        char file_name[64];
        snprintf(file_name, 64, "motion/%s", motion_file_name);

        can_interface_debug("file_name: %s\n", file_name);

        char *file_contents = NULL;
        m_sd_card->mount();
        bool file_read = m_sd_card->readFile(file_name, &file_contents);
        m_sd_card->unmount();

        if (!file_read) {
            response = Response::READ_FAIL;
            free(file_contents);
            break;
        }

        // Append filename
        char prefix[128];
        snprintf(prefix, sizeof(prefix), "\nFILE_NAME=%s\n", motion_file_name);

        size_t prefix_len  = strlen(prefix);
        size_t content_len = strlen(file_contents);

        char *new_contents = new char[prefix_len + content_len + 1]; // +1 for null terminator

        memcpy(new_contents, prefix, prefix_len);
        memcpy(new_contents + prefix_len, file_contents, content_len);
        new_contents[prefix_len + content_len] = '\0';

        free(file_contents);
        file_contents = new_contents;

        // Transmit content
        transmitFrames(file_contents);

        free(file_contents);

        return;
    }
    // -------- Transmit sequence file from SD Card
    case Registers_Board::SD_FETCH_SEQUENCE: {
        char  file_name[64];
        char *file_contents = NULL;
        snprintf(file_name, 64, "sequence/%d", payload.data);
        can_interface_debug("file_name: %s\n", file_name);

        m_sd_card->mount();
        bool file_read = m_sd_card->readFile(file_name, &file_contents);
        m_sd_card->unmount();

        if (!file_read) {
            response = Response::READ_FAIL;
            free(file_contents);
            break;
        }

        transmitFrames(file_contents);

        free(file_contents);

        return;
    }
    // -------- Current length of buffer (multiple of 4)
    case Registers_Board::BUF_LEN: {
        response = buffer_len;
        break;
    }
    // -------- Transmit buffer content
    case Registers_Board::READ_BUFFER: {
        transmitFrames(buffer);
        return;
    }
    // -------- Transmit path content
    case Registers_Board::READ_CURRENT_PATH: {
        transmitFrames(path);
        return;
    }
    // -------- Transmit content of file at PATH from sd card
    case Registers_Board::SD_READ_FILE_AT_PATH: {
        char *file_contents = NULL;

        m_sd_card->mount();
        bool file_read = m_sd_card->readFile(path, &file_contents);
        m_sd_card->unmount();

        if (!file_read) {
            response = Response::READ_FAIL;
            free(file_contents);
            break;
        }

        transmitFrames(file_contents);

        free(file_contents);

        return;
    }
    // --------
    default: {
        break;
    }
    }

    // Create word
    uint32_t word;
    word = payload.register_num << 16;
    word |= static_cast<uint32_t>(response);

    transmitMessage(word, rx_board_w_mbid);
}

/* ---------------------------------------------------------------------------------- */
uint16_t CANInterface::execSDWrite(BoardWritePayload payload) {
    uint16_t response = Response::UNKNOWN;

    switch (payload.register_num) {
    // -------- Clear Buffer & Reset Parameters
    case Registers_Board::BUF_CLEAR: {
        memset(buffer, 0, sizeof(buffer));
        buffer_len          = 0;
        buffer_listen_start = false;
        buffer_listen_end   = false;

        response = Response::SUCCESS;
        break;
    }
    // -------- Copy Buffer into Path and clear Buffer
    case Registers_Board::CP_BUFFER_AS_PATH: {
        if (buffer_listen_end) {
            memcpy(path, buffer, 256);
            response = Response::SUCCESS;
        } else {
            response = Response::WRITE_FAIL;
        }
        break;
    }
    // -------- Writes Buffer into file on SD Card at Path
    case Registers_Board::SD_SAVE_FILE_AT_PATH: {
        if (buffer_listen_end) {
            m_sd_card->mount();
            response = m_sd_card->writeFile(path, buffer) ? Response::SUCCESS : Response::WRITE_FAIL;
            m_sd_card->unmount();
        } else {
            response = Response::WRITE_FAIL;
        }
        break;
    }
    // -------- Deletes file at Path
    case Registers_Board::SD_DELETE_FILE_AT_PATH: {
        m_sd_card->mount();
        response = m_sd_card->deleteFile(path) ? Response::SUCCESS : Response::WRITE_FAIL;
        m_sd_card->unmount();
        break;
    }
    // -------- Wipes SD Card and Create Files
    case Registers_Board::SD_WIPE: {
        // TODO
        break;
    }
    // --------
    default: {
        break;
    }
    }

    return response;
}

/* ---------------------------------------------------------------------------------- */
