/*
 * IncFile1.h
 *
 * Created: 21/08/2025 10:24:46
 *  Author: Tan
 */
#pragma once

#include "asf.h"
#include "board_registers.h"
#include "motion_queue.h"
#include "parser.h"
#include "sd_card.h"
#include "stepper.h"
#include "stepper_registers.h"
#include "utils.h"
#include "version.h"

#ifndef CAN_INTERFACE_H
#    define CAN_INTERFACE_H

#    ifdef CAN_INTERFACE_DEBUG
#        include <stdio.h>
#        define can_interface_debug(...) printf(__VA_ARGS__)
#    else
#        define can_interface_debug(...)
#    endif

class CANInterface {
  public:
    // Mailbox settings
    static constexpr uint8_t MAX_CAN_FRAME_DATA_LEN = 4;
    static constexpr uint8_t CAN_TX_PRIO            = 15;

    // Receiving Mailbox IDs
    static constexpr uint8_t RX_STEPPER_R_MB = 0; // Individual Stepper Control
    static constexpr uint8_t RX_STEPPER_W_MB = 1; // Individual Stepper Control
    uint16_t                 rx_stepper_r_mbid;
    uint16_t                 rx_stepper_w_mbid;
    can_mb_conf_t            rx_stepper_r_mb;
    can_mb_conf_t            rx_stepper_w_mb;

    static constexpr uint8_t RX_BOARD_R_MB = 3;
    static constexpr uint8_t RX_BOARD_W_MB = 4;
    uint16_t                 rx_board_r_mbid;
    uint16_t                 rx_board_w_mbid;
    can_mb_conf_t            rx_board_r_mb;
    can_mb_conf_t            rx_board_w_mb;

    static constexpr uint8_t RX_BUFF_MB = 5;
    uint16_t                 rx_buff_mbid;
    can_mb_conf_t            rx_buff_mb;

    // Transmitting Mailbox ID
    static constexpr uint8_t TX_RESPONSE_MB = 2; // ? Why only works with 2 ?
    uint16_t                 tx_response_mbid;
    can_mb_conf_t            tx_response_mb;

    explicit CANInterface(uint8_t device_id);
    ~CANInterface() { deinit(); };

    // Singleton access
    static CANInterface *p_can_interface;
    static CANInterface *getInstance() { return p_can_interface; }

    /**
     * @brief Initialize can related ISR
     *
     */
    void init();

    /**
     * @brief Deinit ISR
     *
     */
    void deinit();

    /**
     * @brief Transmit a CAN message
     *
     * @param data_l
     * @param data_h
     */
    void transmitMessage(uint32_t data, uint16_t mailbox_id);

    /**
     * @brief Check if CAN interface is configured
     *
     * @return true
     * @return false
     */
    bool isConfigured();

    /**
     * @brief Stepper pointers
     *
     * @param steppers
     */
    void registerSteppers(Stepper *steppers[MAX_STEPPERS]);

    /**
     * @brief Motion queue pointer
     *
     * @param motion_queue
     */
    void registerMotionQueue(MotionQueue *motion_queue);

    /**
     * @brief SD Card pointer
     *
     * @param sd_card
     */
    void registerSdCard(SDCard *sd_card);

    /* ==================================== Handlers ==================================== */

    /**
     * @brief Handle Stepper Read Requests
     *
     */
    void handleRXStepperR();

    /**
     * @brief Handle Stepper Write Requests
     *
     * @note data_l
     * @note nibble 8 - stepper number
     * @note nibble 5-7 - register
     * @note nibble 1-4 - data
     *
     */
    void handleRXStepperW();

    /**
     * @brief Handle Board Write Requests
     *
     * @note data_l
     * @note nibble 8 - ignore
     * @note nibble 5-7 - register
     * @note nibble 1-4 - ignore
     *
     */
    void handleRXBoardR();

    /**
     * @brief Handle Board Read Requests
     *
     * @note data_l
     * @note nibble 8 - ignore
     * @note nibble 5-7 - register
     * @note nibble 1-4 - data
     *
     */
    void handleRXBoardW();

    /**
     * @brief Handle Buffer
     *
     */
    void handleBuff();

    /* ====================================== Task ====================================== */

    volatile bool    can_msg_rdy  = false;
    volatile uint8_t recv_mailbox = 0;
    static void      task_commandHandler(void *parameters);

  private:
    struct StepperWritePayload {
        uint8_t  driver_num;
        uint16_t register_num;
        uint16_t data;
    };

    struct StepperReadPayload {
        uint8_t  driver_num;
        uint16_t register_num;
    };

    uint8_t m_motion_sel = 0; // Motion select
    uint8_t m_item_sel   = 0; // Queue item select

    struct BoardWritePayload {
        uint16_t register_num;
        uint16_t data;
    };

    bool         m_configured = false;     // Configured flag
    uint8_t      m_device_id;              // Message ID
    Stepper     *m_steppers[MAX_STEPPERS]; // pointer to steppers
    SDCard      *m_sd_card;                // pointer to SD card
    MotionQueue *m_motion_queue;           // ptr to motion queue

    /* ===================================== Buffer ===================================== */
    static constexpr uint32_t START_WORD = 0xA0B0C0D;
    static constexpr uint32_t END_WORD   = 0xD0C0B0A;

    char   buffer[256]         = {0};
    char   path[256]           = {0};
    size_t buffer_len          = 0;
    bool   buffer_listen_start = false;
    bool   buffer_listen_end   = false;

    /* ===================================== Utility ==================================== */

    /**
     * @brief Initialize reception mailbox
     *
     * @param p_mailbox
     */
    void initRxMailbox(can_mb_conf_t *p_mailbox, uint8_t mailbox_id, uint16_t rx_id);

    /**
     * @brief Initialize transmit mailbox
     *
     * @param p_mailbox
     */
    void initTxMailbox(can_mb_conf_t *p_mailbox, uint8_t mailbox_id);

    /**
     * @brief Create ID for reception mailbox based on mailbox id and device id.
     *
     * @param mailbox_id Mailbox ID
     * @return uint32_t Created ID
     *
     * @note Format: (mailbox_id << 8) | device_id
     */
    uint32_t createRxId(uint8_t mailbox_id);

    /**
     * @brief Reset mailbox configure structure.
     *
     * @param p_mailbox Pointer to mailbox configure structure.
     */
    static void resetMailboxConf(can_mb_conf_t *p_mailbox);

    /**
     * @brief Transmit content wrapped in START and END word
     *
     * @param content char array
     */
    void transmitFrames(const char *content);

    /* ================================== Write Handler ================================= */

    uint16_t execWriteControl(StepperWritePayload payload);        // 0x000 - 0x0FF
    uint16_t execWriteDriver(StepperWritePayload payload);         // 0x100 - 0x1FF
    uint16_t execWriteMotionParam(StepperWritePayload payload);    // 0x200 - 0x2FF
    uint16_t execWriteHoming(StepperWritePayload payload);         // 0x300 - 0x3FF
    uint16_t execWriteMotionSettings(StepperWritePayload payload); // 0x400 - 0x4FF

    using commandFunctionPtr    = void (Stepper::*)(uint16_t);
    using validationFunctionPtr = bool (*)(uint16_t);

    /**
     * @brief Executes write command
     *
     * @param stepper stepper index
     * @param cmd_fnc function to call
     * @param value value to write
     * @param val_fnc pointer to value check function
     * @param rst_drv call reset driver
     * @param rst_motion call reset motion
     * @param can_write_while_running writing is possible when stepper is running
     * @return uint16_t
     */
    uint16_t execWriteCommand(Stepper *stepper, commandFunctionPtr cmd_fnc, uint16_t value,
                              validationFunctionPtr val_fnc = nullptr, bool rst_drv = false, bool rst_motion = false,
                              bool can_write_while_running = false);

    /* ====================================== Board ===================================== */

    uint16_t selectMotionAndQueue(uint16_t data);
    uint16_t execBoardWrite(BoardWritePayload payload);
    void     execSDRead(BoardWritePayload payload);
    uint16_t execSDWrite(BoardWritePayload payload);
};

/* ============================== Data Validity Checks ============================== */
inline constexpr bool val_enum(uint16_t value, uint16_t size) { return value <= (size - 1); };
inline constexpr bool val_not_equal_to(uint16_t value, uint16_t not_allowed) { return value != not_allowed; };

inline constexpr bool val_ustep(uint16_t value) { return (value <= 256) && (value > 0) && (value & (value - 1)) == 0; };
inline constexpr bool val_12b(uint16_t value) { return value <= 0xFFF; };
inline constexpr bool val_current(uint16_t value) { return (value >= 1) && (value <= 31); };
inline constexpr bool val_bool(uint16_t value) { return (value == 0 || value == 1); };
inline constexpr bool val_positioningMode(uint16_t value) { return val_enum(value, 2); };
inline constexpr bool val_followMode(uint16_t value) { return val_enum(value, 2); };
inline constexpr bool val_homingMode(uint16_t value) { return val_enum(value, 3); };
inline constexpr bool val_homingSensor(uint16_t value) { return val_enum(value, 2); };
inline constexpr bool val_posMode(uint16_t value) { return val_enum(value, 2); };
inline constexpr bool val_rampMode(uint16_t value) { return (value == 0 || value == 4); };
inline constexpr bool val_rampType(uint16_t value) { return val_enum(value, 3); };
inline constexpr bool val_lessThan1000(uint16_t value) { return value <= 1000; }
inline constexpr bool val_maxSteppers(uint16_t value) { return value < MAX_STEPPERS; }
inline constexpr bool val_sequenceNumber(uint16_t value) { return val_not_equal_to(value, 0xFF); }
inline constexpr bool val_motionType(uint16_t value) { return val_enum(value, 5); }

#endif /* CAN_INTERFACE_H */