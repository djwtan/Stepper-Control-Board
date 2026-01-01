#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>

#define MAX_STEPPERS         5
#define MAX_ACCELERATION     16777
#define MAX_MOTIONS          0xF  // Maximum number of motion sequences
#define TEST_MOTION_NUM      0xFF // Motion number used to invoke test queue item
#define MAX_ITEMS_PER_MOTION 32   // Maximum items per motion sequence

// #define BSSB_DEBUG

#ifdef BSSB_DEBUG
#    include <stdio.h>
#    define bssb_debug(...) printf(__VA_ARGS__)
#else
#    define bssb_debug(...)
#endif

/* ================================================================================== */
/*                                   Defined Values                                   */
/* ================================================================================== */
namespace RampMode {
static constexpr uint16_t VELOCITY_MODE    = 0x00;
static constexpr uint16_t POSITIONING_MODE = (0x01 << 2);
} // namespace RampMode

namespace RampType {
static constexpr uint16_t HOLD_RAMP   = 0x00;
static constexpr uint16_t TRAPEZOIDAL = 0x01;
static constexpr uint16_t S_CURVE     = 0x02;
} // namespace RampType

namespace HomingMode {
static constexpr uint16_t IMMEDIATE = 0x00;
static constexpr uint16_t TORQUE    = 0x01;
static constexpr uint16_t SENSOR    = 0x02;
} // namespace HomingMode

namespace HomingSensor {
static constexpr uint16_t STOP_L = 0x00;
static constexpr uint16_t STOP_R = 0x01;
} // namespace HomingSensor

namespace PositioningMode {
static constexpr uint16_t PM_RELATIVE = 0x00;
static constexpr uint16_t PM_ABSOLUTE = 0x01;
} // namespace PositioningMode

namespace FollowMode {
static constexpr uint16_t INTERNAL = 0x00;
static constexpr uint16_t ENCODER  = 0x01;
} // namespace FollowMode

/* ================================================================================== */
/*                                    Stepper Write                                   */
/* ================================================================================== */

/**
 * @brief Stepper Write Registers
 *
 * @note Directed to mailbox 1
 * @note Responses with <Response> (excluding CONTROL_WORD 0x000)
 *
 */
namespace RegS_W {
static constexpr uint16_t CONTROL_WORD = 0x000;
/* ----------------------------------------- */
// Driver Parameters
// Unit Settings
static constexpr uint16_t DRV_MSTEP_PER_FS = 0x100;
static constexpr uint16_t DRV_FS_PER_REV   = 0x101;
// Current Settings
static constexpr uint16_t CURRENT_HOLD = 0x102;
static constexpr uint16_t CURRENT_RUN  = 0x103;
// Stall Settings
static constexpr uint16_t STOP_ON_STALL_ENABLE = 0x104;
static constexpr uint16_t STOP_ON_STALL_THRESH = 0x105;
// Closed Loop Settings
static constexpr uint16_t CL_ENABLE     = 0x106;
static constexpr uint16_t CL_ENC_IN_RES = 0x107;
static constexpr uint16_t CL_TOLERANCE  = 0x108;
static constexpr uint16_t CL_ENABLE_PID = 0x109;
// StealthChop Threshold
static constexpr uint16_t STEALTH_CHOP_THRESH = 0x10A;

/* ----------------------------------------- */
// Motion Parameters
// Position Settings
static constexpr uint16_t MOVE_UNIT_HIGH                         = 0x200;
static constexpr uint16_t MOVE_UNIT_LOW                          = 0x201;
static constexpr uint16_t MOVE_TIME_MS_HIGH                      = 0x202;
static constexpr uint16_t MOVE_TIME_MS_LOW                       = 0x203;
static constexpr uint16_t MOVE_VIB_I                             = 0x204;
static constexpr uint16_t MOVE_VIB_DIM_FACTOR                    = 0x205;
static constexpr uint16_t MOVE_VIB_LOOP                          = 0x206;
static constexpr uint16_t MOVE_RESET_MOTION_CONF_AFTER_EACH_MOVE = 0x207;
static constexpr uint16_t MOVE_ALLOW_WRITE_MOTION_WHEN_BUSY      = 0x208;
// Speed Settings
static constexpr uint16_t SPEED_MAX_H   = 0x209;
static constexpr uint16_t SPEED_MAX_L   = 0x20A;
static constexpr uint16_t SPEED_START_H = 0x20B;
static constexpr uint16_t SPEED_START_L = 0x20C;
static constexpr uint16_t SPEED_STOP_H  = 0x20D;
static constexpr uint16_t SPEED_STOP_L  = 0x20E;
static constexpr uint16_t SPEED_BREAK_H = 0x20F;
static constexpr uint16_t SPEED_BREAK_L = 0x210;
// Acceleration Settings
static constexpr uint16_t ACC_MAX_ACCEL_H   = 0x211;
static constexpr uint16_t ACC_MAX_ACCEL_L   = 0x212;
static constexpr uint16_t ACC_MAX_DECEL_H   = 0x213;
static constexpr uint16_t ACC_MAX_DECEL_L   = 0x214;
static constexpr uint16_t ACC_START_ACCEL_H = 0x215;
static constexpr uint16_t ACC_START_ACCEL_L = 0x216;
static constexpr uint16_t ACC_FINAL_DECEL_H = 0x217;
static constexpr uint16_t ACC_FINAL_DECEL_L = 0x218;
// Bow Settings
static constexpr uint16_t BOW1_H = 0x219;
static constexpr uint16_t BOW1_L = 0x21A;
static constexpr uint16_t BOW2_H = 0x21B;
static constexpr uint16_t BOW2_L = 0x21C;
static constexpr uint16_t BOW3_H = 0x21D;
static constexpr uint16_t BOW3_L = 0x21E;
static constexpr uint16_t BOW4_H = 0x21F;
static constexpr uint16_t BOW4_L = 0x220;

/* ----------------------------------------- */
// Home Settings
static constexpr uint16_t HOME_HOMING_MODE       = 0x300;
static constexpr uint16_t HOME_HOMING_SENSOR     = 0x301;
static constexpr uint16_t HOME_SENSOR_HOME_VALUE = 0x302;
static constexpr uint16_t HOME_MAX_FIND_H        = 0x303;
static constexpr uint16_t HOME_MAX_FIND_L        = 0x304;
static constexpr uint16_t HOME_MAX_SPEED_H       = 0x305;
static constexpr uint16_t HOME_MAX_SPEED_L       = 0x306;
static constexpr uint16_t HOME_MAX_ACCEL_H       = 0x307;
static constexpr uint16_t HOME_MAX_ACCEL_L       = 0x308;
static constexpr uint16_t HOME_MAX_DECEL_H       = 0x309;
static constexpr uint16_t HOME_MAX_DECEL_L       = 0x30A;
static constexpr uint16_t HOME_OFFSET_H          = 0x30B;
static constexpr uint16_t HOME_OFFSET_L          = 0x30C;
static constexpr uint16_t HOME_TIMEOUT_MS_H      = 0x30D;
static constexpr uint16_t HOME_TIMEOUT_MS_L      = 0x30E;

/* ----------------------------------------- */
// Motion Shaping
// Positioning Settings
static constexpr uint16_t POS_MODE         = 0x400;
static constexpr uint16_t POS_FOLLOW_MODE  = 0x401;
static constexpr uint16_t POS_UNIT_PER_REV = 0x402;
// Ramp Settings
static constexpr uint16_t RAMP_MODE = 0x403;
static constexpr uint16_t RAMP_TYPE = 0x404;
} // namespace RegS_W

/* ---------------------------------------------------------------------------------- */
/**
 * @brief Valid values for CONTROL_WORD (0x000)
 *
 * @note Return value for operations that uses queue must be retrieved later with RegS_R::READBACK
 *
 */
namespace ControlWord {
// Setups
static constexpr uint16_t INIT           = 0x0000; // PutQueueRes -> Response
static constexpr uint16_t SET_MOTION     = 0x0001; // PutQueueRes -> Response
static constexpr uint16_t ENABLE_DRIVER  = 0x0002; // Response
static constexpr uint16_t RELEASE_DRIVER = 0x0003; // Response

// Move
static constexpr uint16_t MOVE              = 0x1000; // PutQueueRes -> ExecCode
static constexpr uint16_t MOVE_HOMING       = 0x1001; // PutQueueRes -> HomingCode
static constexpr uint16_t MOVE_INVERSE_TIME = 0x1002; // PutQueueRes -> ExecCode
static constexpr uint16_t MOVE_VIBRATION    = 0x1003; // PutQueueRes -> ExecCode

// Stop
static constexpr uint16_t RAMP_STOP      = 0x2000; // Response
static constexpr uint16_t EMERGENCY_STOP = 0x2001; // Response

// Misc
static constexpr uint16_t SET_ZERO = 0x3000; // Response
} // namespace ControlWord
/* ================================================================================== */
/*                                    Stepper Read                                    */
/* ================================================================================== */

/**
 * @brief Read registers
 *
 * @todo If other registers need to be exposed, use one set of registers.
 *
 */
namespace RegS_R {
static constexpr uint16_t STATUS            = 0x000; // DriverStatus
static constexpr uint16_t READBACK          = 0X001; // Result / ExecCode / HomingCode
static constexpr uint16_t INTERNAL_POSITION = 0X002; // int32
static constexpr uint16_t ENCODER_POSITION  = 0X003; // int32
static constexpr uint16_t CURRENT_SPEED     = 0X004; // int32
static constexpr uint16_t SENSOR_STATUS     = 0X005; // 2 bit
static constexpr uint16_t ENCODER_POS_DEV   = 0X006; // int32
}; // namespace RegS_R

/* ---------------------------------------------------------------------------------- */
enum DriverStatus : uint16_t {
    STS_READY = 1,
    STS_BUSY,
    STS_HOMING,
    STS_VIBRATING,
    STS_CTRL_NOT_INIT,
    STS_MOTION_NOT_INIT,
    STS_STALL,
    STS_POS_ERR = 8,
    STS_STOPPED,
    STS_COIL_OL,
    STS_COIL_SHORT,
    STS_OVERTEMP,
    STS_TMC4361A_COMM_ERR = 13,
    STS_TMC5160_COMM_ERR,
    STS_NO_COMM = 0xFFFF, // Not on driver
};

static inline const char *toString_stepperStatus(DriverStatus res) {
    switch (res) {
    case DriverStatus::STS_READY            : return "(DriverStatus) Ready"; break;
    case DriverStatus::STS_BUSY             : return "(DriverStatus) Busy"; break;
    case DriverStatus::STS_HOMING           : return "(DriverStatus) Busy Homing"; break;
    case DriverStatus::STS_VIBRATING        : return "(DriverStatus) Busy Vibrating"; break;
    case DriverStatus::STS_CTRL_NOT_INIT    : return "(DriverStatus) Controller Not Init"; break;
    case DriverStatus::STS_MOTION_NOT_INIT  : return "(DriverStatus) Motion Not Init"; break;
    case DriverStatus::STS_STALL            : return "(DriverStatus) Stalled"; break;
    case DriverStatus::STS_POS_ERR          : return "(DriverStatus) Position Error"; break;
    case DriverStatus::STS_STOPPED          : return "(DriverStatus) EM Stopped"; break;
    case DriverStatus::STS_COIL_OL          : return "(DriverStatus) Coil Open"; break;
    case DriverStatus::STS_COIL_SHORT       : return "(DriverStatus) Coil Short"; break;
    case DriverStatus::STS_OVERTEMP         : return "(DriverStatus) Overtemperature"; break;
    case DriverStatus::STS_TMC4361A_COMM_ERR: return "(DriverStatus) Tmc4361A err"; break;
    case DriverStatus::STS_TMC5160_COMM_ERR : return "(DriverStatus) Tmc5160 err"; break;
    case DriverStatus::STS_NO_COMM          : return "(DriverStatus) No Communication"; break;
    default                                 : return "(DriverStatus) Invalid";
    }
};

enum ExecCode : uint16_t {
    E_SUCCESS = 1,
    E_WRITE_FAIL,
    E_CTRL_NOT_INIT,
    E_MOTION_NOT_INIT,
    E_IS_FROZEN,
    E_IS_BUSY,
    E_BAD_SETTINGS
};

static inline const char *toString_execCode(ExecCode res) {
    switch (res) {
    case ExecCode::E_SUCCESS        : return "(ExecCode) Success"; break;
    case ExecCode::E_WRITE_FAIL     : return "(ExecCode) Write Failed"; break;
    case ExecCode::E_CTRL_NOT_INIT  : return "(ExecCode) Controller Not Init"; break;
    case ExecCode::E_MOTION_NOT_INIT: return "(ExecCode) Motion Not Init"; break;
    case ExecCode::E_IS_FROZEN      : return "(ExecCode) Is Frozen (Ask Status)"; break;
    case ExecCode::E_IS_BUSY        : return "(ExecCode) Busy"; break;
    case ExecCode::E_BAD_SETTINGS   : return "(ExecCode) Bad Settings"; break;
    default                         : return "(ExecCode) Invalid";
    }
};

enum HomingCode : uint16_t {
    H_SUCCESS = 1,
    H_WRITE_FAIL,
    H_CTRL_NOT_INIT,
    H_MOTION_NOT_INIT,
    H_IS_FROZEN,
    H_IS_BUSY,
    H_TIMEOUT,
    H_MAX_PULSE_REACHED,
    H_FAILED_MIDWAY
};

static inline const char *toString_homingCode(HomingCode res) {
    switch (res) {
    case HomingCode::H_SUCCESS          : return "(HomingCode) Success"; break;
    case HomingCode::H_WRITE_FAIL       : return "(HomingCode) Write Failed"; break;
    case HomingCode::H_CTRL_NOT_INIT    : return "(HomingCode) Controller Not Init"; break;
    case HomingCode::H_MOTION_NOT_INIT  : return "(HomingCode) Motion Not Init"; break;
    case HomingCode::H_IS_FROZEN        : return "(HomingCode) Is Frozen (Ask Status)"; break;
    case HomingCode::H_IS_BUSY          : return "(HomingCode) Busy"; break;
    case HomingCode::H_TIMEOUT          : return "(HomingCode) Timedout"; break;
    case HomingCode::H_MAX_PULSE_REACHED: return "(HomingCode) Maximum Position Reached"; break;
    case HomingCode::H_FAILED_MIDWAY    : return "(HomingCode) Failed Midway"; break;
    default                             : return "(HomingCode) Invalid";
    }
};

/* ================================================================================== */
/*                                        Board                                       */
/* ================================================================================== */
namespace Reg_Board {
// board info
static constexpr uint16_t FIRMWARE_VER = 0x001;

// execute motion
static constexpr uint16_t MOTION_CONTROL_WORD = 0x100;

// Select the the motion and queue for next write operation
static constexpr uint16_t SELECT_MOTION_AND_QUEUE_ITEM = 0x101;

static constexpr uint16_t MOTION_READBACK = 0x102; // Result / MotionQueueRes
static constexpr uint16_t QSTATUS         = 0x103; // QueueStatus

// Execute pre-loaded motion
static constexpr uint16_t RUN_PRELOAD = 0X104; // PreloadPutQueueRes / Readback from stepper

// Mandatory
static constexpr uint16_t QITEM_STEPPER_NUM     = 0x200;
static constexpr uint16_t QITEM_MOTION_TYPE     = 0x201;
static constexpr uint16_t QITEM_SEQUENCE_NUMBER = 0x202;
static constexpr uint16_t QITEM_POSITION_H      = 0x203;
static constexpr uint16_t QITEM_POSITION_L      = 0x204;

// Basic Motion
static constexpr uint16_t QITEM_RAMP_TYPE   = 0x300;
static constexpr uint16_t QITEM_MAX_SPEED_H = 0x301;
static constexpr uint16_t QITEM_MAX_SPEED_L = 0x302;
static constexpr uint16_t QITEM_ACCEL_H     = 0x303;
static constexpr uint16_t QITEM_ACCEL_L     = 0x304;
static constexpr uint16_t QITEM_DECEL_H     = 0x305;
static constexpr uint16_t QITEM_DECEL_L     = 0x306;
static constexpr uint16_t QITEM_BOW1_H      = 0x307;
static constexpr uint16_t QITEM_BOW1_L      = 0x308;
static constexpr uint16_t QITEM_BOW2_H      = 0x309;
static constexpr uint16_t QITEM_BOW2_L      = 0x30A;
static constexpr uint16_t QITEM_BOW3_H      = 0x30B;
static constexpr uint16_t QITEM_BOW3_L      = 0x30C;
static constexpr uint16_t QITEM_BOW4_H      = 0x30D;
static constexpr uint16_t QITEM_BOW4_L      = 0x30E;

// Inverse time move override
static constexpr uint16_t QITEM_USE_INVERSE_TIME = 0x400;

// Use for inverse time + homing timeout
static constexpr uint16_t QITEM_TIME_H = 0x500;
static constexpr uint16_t QITEM_TIME_L = 0x501;

// Homing mode
static constexpr uint16_t QITEM_OFFSET_H          = 0x600;
static constexpr uint16_t QITEM_OFFSET_L          = 0x601;
static constexpr uint16_t QITEM_HOMING_MODE       = 0x602;
static constexpr uint16_t QITEM_HOMING_SENSOR     = 0x603;
static constexpr uint16_t QITEM_SENSOR_HOME_VALUE = 0x604;

// SD Card Read
static constexpr uint16_t SD_FETCH_DRV_CONFIG    = 0x700;
static constexpr uint16_t SD_FETCH_MOTION_CONFIG = 0x701;
static constexpr uint16_t SD_FETCH_SEQUENCE      = 0X702;

// SD Card Write
static constexpr uint16_t BUF_CLEAR              = 0x703; // Clear buffer
static constexpr uint16_t BUF_LEN                = 0x704; // Size, has been terminated, etc.
static constexpr uint16_t CP_BUFFER_AS_PATH      = 0x705; // Copies current buffer as path then clear
static constexpr uint16_t READ_BUFFER            = 0x706; // Return current buffer
static constexpr uint16_t READ_CURRENT_PATH      = 0x707; // Return current path
static constexpr uint16_t SD_SAVE_FILE_AT_PATH   = 0x708; // Save file at path
static constexpr uint16_t SD_READ_FILE_AT_PATH   = 0x709; // More generic version for read
static constexpr uint16_t SD_DELETE_FILE_AT_PATH = 0x70A; // Delete file at path
} // namespace Reg_Board

/* ---------------------------------------------------------------------------------- */
namespace MotionType {
static constexpr uint16_t MOVE_RELATIVE = 0;
static constexpr uint16_t MOVE_ABSOLUTE = 1;
static constexpr uint16_t MOVE_HOMING   = 2;
static constexpr uint16_t WAIT_TIME     = 3;
static constexpr uint16_t WAIT_SENSOR   = 4;
} // namespace MotionType

/**
 * @brief Valid values for MOTION_CONTROL_WORD (0x100)
 *
 */
namespace MotionControlWord {
static constexpr uint16_t START_MOTION_0 = 0x0000;
static constexpr uint16_t START_MOTION_1 = 0x0001;
static constexpr uint16_t START_MOTION_2 = 0x0002;
static constexpr uint16_t START_MOTION_3 = 0x0003;
static constexpr uint16_t START_MOTION_4 = 0x0004;
static constexpr uint16_t START_MOTION_5 = 0x0005;
static constexpr uint16_t START_MOTION_6 = 0x0006;
static constexpr uint16_t START_MOTION_7 = 0x0007;
static constexpr uint16_t START_MOTION_8 = 0x0008;
static constexpr uint16_t START_MOTION_9 = 0x0009;
static constexpr uint16_t START_MOTION_A = 0x000A;
static constexpr uint16_t START_MOTION_B = 0x000B;
static constexpr uint16_t START_MOTION_C = 0x000C;
static constexpr uint16_t START_MOTION_D = 0x000D;
static constexpr uint16_t START_MOTION_E = 0x000E;
static constexpr uint16_t START_MOTION_F = 0x000F;
static constexpr uint16_t ABORT          = 0x1000;
} // namespace MotionControlWord

/* ---------------------------------------------------------------------------------- */
enum MotionQueueRes : uint16_t {
    MQ_SUCCESS = 1,
    MQ_CONDITIONAL_SUCCESS,
    MQ_FAILED_MIDWAY,
    MQ_INVALID_MOTION,
    MQ_EMPTY_QUEUE,
    MQ_EXCEED_MAX_CONCURRENT,
    MQ_REPEATED_ASSIGNMENT = 7,
    MQ_BAD_PARAMETERS,
    MQ_STEPPERS_NOT_READY,
};

static inline const char *toString_motionQueueRes(MotionQueueRes res) {
    switch (res) {
    case MotionQueueRes::MQ_SUCCESS              : return "(MotionQueueRes) Success"; break;
    case MotionQueueRes::MQ_CONDITIONAL_SUCCESS  : return "(MotionQueueRes) Conditional Success"; break;
    case MotionQueueRes::MQ_FAILED_MIDWAY        : return "(MotionQueueRes) Failed Midway"; break;
    case MotionQueueRes::MQ_INVALID_MOTION       : return "(MotionQueueRes) Invalid Motion"; break;
    case MotionQueueRes::MQ_EMPTY_QUEUE          : return "(MotionQueueRes) Empty Queue"; break;
    case MotionQueueRes::MQ_EXCEED_MAX_CONCURRENT: return "(MotionQueueRes) Exceed Max Concurrent Motions"; break;
    case MotionQueueRes::MQ_REPEATED_ASSIGNMENT  : return "(MotionQueueRes) Repeated Driver Assignment"; break;
    case MotionQueueRes::MQ_BAD_PARAMETERS       : return "(MotionQueueRes) Bad Parameters"; break;
    case MotionQueueRes::MQ_STEPPERS_NOT_READY   : return "(MotionQueueRes) Steppers Not Ready"; break;
    default                                      : return "(MotionQueueRes) Invalid";
    }
};

enum PreloadPutQueueRes : uint16_t {
    P_SUCCESS = 1,     // same as PutQueueRes
    P_FULL,            // same as PutQueueRes
    P_RESULTS_PENDING, // Same as PutQueueRes
    P_INVALID_MOTION,
    P_STEPPER_NOT_READY
};

static inline const char *toString_preloadPutQueueRes(PreloadPutQueueRes res) {
    switch (res) {
    case PreloadPutQueueRes::P_SUCCESS          : return "(PreloadPutQueueRes) Success"; break;
    case PreloadPutQueueRes::P_FULL             : return "(PreloadPutQueueRes) Queue Full"; break;
    case PreloadPutQueueRes::P_RESULTS_PENDING  : return "(PreloadPutQueueRes) Results Pending"; break;
    case PreloadPutQueueRes::P_INVALID_MOTION   : return "(PreloadPutQueueRes) Invalid Motion Type"; break;
    case PreloadPutQueueRes::P_STEPPER_NOT_READY: return "(PreloadPutQueueRes) Stepper Not Ready"; break;
    default                                     : return "(PreloadPutQueueRes) Invalid";
    }
};

/**
 * @brief Status of queue execution
 *
 * @note Bits 15    - Executing flag
 * @note Bits 11-14 - Current motion number
 * @note Bits 00-10 - Current sequence number
 *
 */
struct QueueStatus {
    uint16_t is_executing;
    uint16_t motion_num;
    uint16_t sequence_num;
};

/* ================================================================================== */
/*                                      Response                                      */
/* ================================================================================== */
/**
 * @brief Stepper Queue / Motion Queue response
 *
 */
enum PutQueueRes : uint16_t {
    Q_SUCCESS = 1,
    Q_FULL,
    Q_RESULTS_PENDING,
};

static inline const char *toString_putQueueRes(PutQueueRes res) {
    switch (res) {
    case PutQueueRes::Q_SUCCESS        : return "(PutQueueRes) Success"; break;
    case PutQueueRes::Q_FULL           : return "(PutQueueRes) Queue Full"; break;
    case PutQueueRes::Q_RESULTS_PENDING: return "(PutQueueRes) Results Pending"; break;
    default                            : return "(PutQueueRes) Invalid";
    }
};

/**
 * @brief Response value
 *
 */
enum Response : uint16_t {
    SUCCESS             = 0x0001,
    INVALID_INPUT       = 0x1000,
    VALUE_OUT_OF_BOUNDS = 0x1001,
    WRITE_FAIL          = 0x1002,
    READ_FAIL           = 0x1003,
    DRIVER_BUSY         = 0x1004,
    DRIVER_IS_NULL      = 0x1005,
    USED_BY_QUEUE       = 0x1006,
    UNKNOWN             = 0x1007,
};

static inline const char *toString_response(Response res) {
    switch (res) {
    case Response::SUCCESS            : return "(Response) Success"; break;
    case Response::INVALID_INPUT      : return "(Response) Invalid Input"; break;
    case Response::VALUE_OUT_OF_BOUNDS: return "(Response) Input Value Out of Bounds"; break;
    case Response::WRITE_FAIL         : return "(Response) Write Failed"; break;
    case Response::READ_FAIL          : return "(Response) Read Failed"; break;
    case Response::DRIVER_BUSY        : return "(Response) Driver Busy"; break;
    case Response::DRIVER_IS_NULL     : return "(Response) Invalid Driver Number"; break;
    case Response::UNKNOWN            : return "(Response) Tak mungkin!"; break;
    default                           : return "(Response) Invalid";
    }
};

/* ================================================================================== */
/*                                         CAN                                        */
/* ================================================================================== */

class bsStepperBoard {
  public:
    using CANTransmit = std::function<bool(uint32_t identifier, uint32_t data, uint8_t data_len)>;
    using CANReceive  = std::function<int(uint32_t *buffer, int buffer_size)>;
    using CANMutex    = std::function<void(bool)>;
    using AsyncSleep  = std::function<void(void)>;

    // Constructor
    bsStepperBoard(uint8_t board_num) : m_board_num(board_num) {};
    ~bsStepperBoard() {};

    // Override functions
    void setCANTransmit(CANTransmit func) { m_can_transmit = func; };
    void setCANReceive(CANReceive func) { m_can_receive = func; };
    void setCANMutex(CANMutex func) { m_can_mutex = func; };
    void setAsyncSleep_1ms(AsyncSleep func) { m_async_sleep = func; };

    static constexpr uint32_t START_WORD = 0xA0B0C0D;
    static constexpr uint32_t END_WORD   = 0xD0C0B0A;

    /* =================================== Board Info =================================== */

    /**
     * @brief Reads firmware version
     *
     * @param ver
     * @return read successful
     */
    bool readFirmwareVersion(std::string **ver);

    /* ===================================== SD Card ==================================== */

    /**
     * @brief Read content of 'config' file on SD card
     *
     * @param motion_index 0-4
     * @param file_content ptr to storage ptr
     * @return read successful
     */
    bool readDriverConfig(uint8_t driver_num, std::string **file_content);

    /**
     * @brief Read content of 'motion' file on SD card
     *
     * @param motion_index 0-31
     * @param file_content ptr to storage ptr
     * @return read successful
     */
    bool readMotionConfig(uint8_t motion_index, std::string **file_content);

    /**
     * @brief Read content of 'sequence' file on SD card
     *
     * @param sequence_index 0-15
     * @param file_content ptr to storage ptr
     * @return read successful
     */
    bool readSequenceConfig(uint8_t sequence_index, std::string **file_content);

    /**
     * @brief Reads content of BUFFER on bssb
     *
     * @param buf_content ptr to storage ptr
     * @return read successful
     */
    bool readBuffer(std::string **buf_content);

    /**
     * @brief Reads content of PATH on bssb
     *
     * @param path ptr to storage ptr
     * @return read successful
     */
    bool readPath(std::string **path);

    /**
     * @brief Read content of file at path on SD card
     *
     * @param path string
     * @param content ptr to storage ptr
     * @return read successful
     *
     * @note ignore root directory (0:)
     */
    bool readFileAtPath(std::string *path, std::string **file_content);

    /**
     * @brief Delete file at path on SD card
     *
     * @param path
     * @param content
     * @return delete successful
     *
     * @note ignore root directory (0:)
     */
    bool deleteFileAtPath(std::string *path);

    /**
     * @brief Write content to file at path on SD card
     *
     * @param path
     * @param content
     * @return write successful
     *
     * @note ignore root directory (0:)
     */
    bool writeToFileAtPath(std::string *path, std::string *content);

    /* ===================================== Decode ===================================== */

    /**
     * @brief Clear responses in Readback
     *
     * @param driver_num driver number
     *
     */
    void clearQueuedReadback(uint8_t driver_num);

    /**
     * @brief Fetches results of the last executed stepper move
     *
     * @param driver_num driver number
     * @param response ptr to store response
     * @return Communication is successful
     *
     * @note Blocks until timeout / result becomes available
     * @note compare / verify with appropriate decoder based on last move type
     */
    bool getQueueReadback(uint8_t driver_num, uint16_t *response);

    /**
     * @brief Decode response into HomingCode
     *
     * @param response
     * @return HomingCode
     */
    static HomingCode decodeQueueReadback_homingCode(uint16_t response);

    /**
     * @brief For move, moveInverseTime, moveVibrate
     *
     * @param response
     * @return is ExecCode::E_SUCCESS
     */
    static bool verifyQueueReadback_execSuccessful(uint16_t response);

    /**
     * @brief For initController, setMotion
     *
     * @param response
     * @return is Response::SUCCESS
     */
    static bool verifyQueueReadback_writeSuccessful(uint16_t response);

    /* ================================== Move Commands ================================= */

    /**
     * @brief Runs motion in SD card as mapped in _indexMap
     *
     * @param index number as in indexMap
     * @param driver_num call readback on this value upon execution
     * @return Successfully put into queue
     */
    bool q_movePreloaded(uint16_t index, uint8_t *driver_num);

    /**
     * @brief Moves
     *
     * @param driver_num driver number
     * @param position_units (positioning mode) user units, (Velocity mode) rpm
     *
     * @return Successfully put into queue
     *
     * @note readback is instantly available for velocity mode
     */
    bool q_moveStepper(uint8_t driver_num, int32_t position_units);

    /**
     * @brief Starts homing move with current configurations
     *
     * @param driver_num
     * @return Successfully put into queue
     *
     * @note call configureHoming beforehand
     */
    bool q_moveStepper_homing(uint8_t driver_num);

    /**
     * @brief Starts inverse time positioning move
     *
     * @param driver_num driver number
     * @param position_units user units
     * @param time_ms time in ms
     *
     * @return Successfully put into queue
     */
    bool q_moveStepper_inverseTime(uint8_t driver_num, int32_t position_units, uint32_t time_ms);

    /**
     * @brief Starts Vibration
     *
     * @param driver_num driver number
     * @param position_units recommended 1-3 degrees
     * @param iterations number of backforth motions
     * @param diminishing_factor 0.1-1.0. Pass 1.0 for constant vibration
     * @param loop loop forever (Ramp stop / emStop required to exit)
     *
     * @return Successfully put into queue
     */
    bool q_moveStepper_vibration(uint8_t driver_num, int32_t position_units, uint16_t iterations,
                                 float diminishing_factor, bool loop);

    /* ================================== Control Word ================================== */

    /**
     * @brief Graceful stop. Follows deceleration profile.
     *
     * @param driver_num
     * @return execute success
     */
    bool rampStop(uint8_t driver_num);

    /**
     * @brief Abrupt stop. Calls FREEZE on motion controller. Reset required after.
     *
     * @param driver_num
     * @return execute success
     */
    bool emergencyStop(uint8_t driver_num);

    /**
     * @brief Resets internal and encoder position to 0.
     *
     * @param driver_num
     * @return execute success
     *
     * @note Can only be called when stepper is not BUSY
     */
    bool resetPosition(uint8_t driver_num);

    /**
     * @brief Disables driver
     *
     * @param driver_num
     * @return execute success
     *
     * @note Can only be called when stepper is not BUSY
     * @note Any move commands automatically calls this
     */
    bool enableDriver(uint8_t driver_num);

    /**
     * @brief Disables driver
     *
     * @param driver_num
     * @return execute success
     *
     * @note Can only be called when stepper is not BUSY
     */
    bool releaseDriver(uint8_t driver_num);

    /* ================================== Stepper Read ================================== */

    /**
     * @brief Returns stepper status
     *
     * @param driver_num
     * @return DriverStatus
     */
    DriverStatus readDriverStatus(uint8_t driver_num);

    /**
     * @brief Returns internal position in unit defined by unit per rev
     *
     * @param driver_num
     * @return int32_t
     */
    int32_t readCurrentPosition(uint8_t driver_num);

    /**
     * @brief Returns encoder position in unit defined by unit per rev
     *
     * @param driver_num
     * @return int32_t
     */
    int32_t readEncoderPosition(uint8_t driver_num);

    /**
     * @brief Returns current speed in RPM
     *
     * @param driver_num
     * @return int32_t
     */
    int32_t readCurrentSpeed(uint8_t driver_num);

    /**
     * @brief Returns value of STOP_L and STOP_R sensors on driver
     *
     * @param driver_num
     * @return bit0 - STOP_L, bit1 - STOP_R
     */
    uint8_t readSensors(uint8_t driver_num);

    /**
     * @brief Returns difference between internal position and encoder
     * @brief position. (unit: encoder pulses)
     *
     * @param driver_num
     * @return int32_t
     */
    int32_t readEncoderPositionDeviation(uint8_t driver_num);

    /* ================================= Motion Planner ================================= */

    /**
     * @brief Fetches status of current motion
     *
     * @return QueueStatus
     */
    QueueStatus getMotionQueueStatus();

    /**
     * @brief Starts planned motion sequence
     *
     * @param motion_num
     * @return Successfully placed into motion queue
     */
    bool mq_startMotion(uint16_t motion_num);

    /**
     * @brief Ends current motion
     *
     * @return true
     * @return false
     *
     * @note Emergency stop will be called on all involved drivers
     */
    bool abortMotion();

    /**
     * @brief Clear readback
     *
     */
    void clearMotionQueueReadback();

    /**
     * @brief Fetches results of the last executed motion
     *
     * @param MotionQueueRes if communication is successful
     * @return Communication is successful
     *
     * @note Blocks until timeout / result becomes available
     */
    bool getMotionQueueReadback(MotionQueueRes *mq_res);

    struct QueueItemParameters {
        // For All
        uint16_t motion_type = 0;
        uint16_t driver_num  = 0; // use for extra check
        int32_t  position    = 0;
        /* ---------------------------------------- */
        // Move Homing
        int32_t  offset      = 0;
        uint16_t homing_mode = 0; // use for extra check
        /* ---------------------------------------- */
        uint16_t homing_sensor     = 0; // Home sensor / Wait Sensor
        uint16_t sensor_home_value = 0; // Home / Wait sensor condition
        /* ---------------------------------------- */
        // Move
        uint16_t ramp_type = 0;
        float    max_speed = 0;
        float    max_accel = 0;
        float    max_decel = 0;
        float    bow1      = 0;
        float    bow2      = 0;
        float    bow3      = 0;
        float    bow4      = 0;
        /* ---------------------------------------- */
        // Move Inverse Time
        uint16_t use_inverse_time = 0;
        /* ---------------------------------------- */
        // Homing timeout / Inverse Time Time / Wait Time / Wait Sensor Timeout
        uint32_t time_ms = 0;
        /* ---------------------------------------- */
    };

    /**
     * @brief Writes parameters to [motion][item]
     *
     * @param motion_num
     * @param item_num
     * @param sequence_number sequence in which motion should be executed
     * @param parameters QueueItemParameters
     * @return write successful
     *
     * @note Does not override content loaded from SD card.
     * @note Wiped after reset
     */
    bool writeQueueItem(uint16_t motion_num, uint16_t item_num, uint16_t sequence_number,
                        QueueItemParameters parameters);

    /**
     * @brief Starts test motion sequence
     *
     * @return Successfully placed into motion queue
     */
    bool mq_startTestQueueItem();

    /**
     * @brief Writes parameters to test motion
     *
     * @param parameters QueueItemParameters
     * @return write successful
     */
    bool writeTestQueueItem(QueueItemParameters parameters);

    /* ===================================== Driver ===================================== */

    struct DriverConfig {
        uint16_t mstep_per_fs         = 256;  // microsteps per full step (e.g. 1`6, 32, 64, 128, 256)
        uint16_t fs_per_rev           = 200;  // full steps per revolution (e.g. 200 for 1.8deg motor)
        uint16_t unit_per_rev         = 360;  // User unit per rev
        uint16_t holding_current      = 250;  // hold current in mA
        uint16_t peak_rms_current     = 500;  // run current in mA
        bool     stop_on_stall_enable = 0;    // enable stop on stall detection
        uint16_t stop_on_stall_thresh = 100;  // stall detection threshold (rpm)
        bool     cl_enable            = 0;    // closed loop enable
        bool     cl_enable_pid        = 0;    // closed loop enable PID regulator
        uint16_t cl_enc_in_res        = 4000; // closed loop encoder input resolution (pulses per revolution)
        uint16_t cl_tolerance         = 500;  // closed loop tolerance in encoder counts
        uint16_t stealth_chop_thresh  = 180;  // stealthChop threshold in rpm (0 to disable)
    };
    /**
     * @brief Configure stepper driver with basic settings.
     *
     * @param driver_num
     * @param config
     * @return successfully placed into queue
     *
     * @warning If using SD Card, this function will override driver config.
     */
    bool q_setAndInitDriver(uint8_t driver_num, DriverConfig config);

    /**
     * @brief Call init driver only
     *
     * @param driver_num
     * @return successfully placed into queue
     *
     * @note Use this if using SD Card to not override config.
     */
    bool q_initDriver(uint8_t driver_num);

    /* ===================================== Motion ===================================== */

    struct MotionConfig {
        uint16_t allow_write_motion_when_busy      = 0; // keep default
        uint16_t reset_motion_conf_after_each_move = 0; // keep default
        uint16_t pos_mode                          = PositioningMode::PM_RELATIVE;
        uint16_t follow_mode                       = FollowMode::INTERNAL;
        uint16_t ramp_mode                         = RampMode::POSITIONING_MODE;
        uint16_t ramp_type                         = RampType::TRAPEZOIDAL;
        float    max_speed                         = 0.0f; // rpm
        float    start_speed                       = 0.0f; // rpm
        float    stop_speed                        = 0.0f; // rpm
        float    break_speed                       = 0.0f; // rpm
        float    max_accel                         = 0.0f; // rpm/s
        float    max_decel                         = 0.0f; // rpm/s2
        float    start_accel                       = 0.0f; // rpm/s2
        float    final_decel                       = 0.0f; // rpm/s2
        float    bow1                              = 0.0f; // rpm/s3
        float    bow2                              = 0.0f; // rpm/s3
        float    bow3                              = 0.0f; // rpm/s3
        float    bow4                              = 0.0f; // rpm/s3
    };
    /**
     * @brief Configure stepper motion with basic settings
     *
     * @param driver_num
     * @param config
     * @return successfully placed into queue
     *
     */
    bool q_setMotion(uint8_t driver_num, MotionConfig config);

    /* ===================================== Homing ===================================== */

    struct HomingConfig {
        uint16_t homing_mode           = HomingMode::IMMEDIATE;
        uint16_t homing_sensor         = HomingSensor::STOP_L;
        uint16_t home_sensor_value     = 0;
        int32_t  homing_offset_units   = 0;
        int32_t  homing_distance_units = 0;
        float    max_speed             = 0.0f;             // rpm
        float    max_accel             = 0.0f;             // rpm/s
        float    max_decel             = MAX_ACCELERATION; // rpm/s2
        uint32_t homing_timeout_ms     = 10000;            // ms
    };
    /**
     * @brief Configure stepper homing with basic settings
     *
     * @param driver_num
     * @param config
     * @return successfully wrote all parameters
     */
    bool configureHoming(uint8_t driver_num, HomingConfig config);

    /* ---------------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------------------- */
  private:
    CANTransmit m_can_transmit = nullptr;
    CANReceive  m_can_receive  = nullptr;
    CANMutex    m_can_mutex    = nullptr;
    AsyncSleep  m_async_sleep  = nullptr;

    uint8_t m_board_num = 0;

    enum class bssb_response_t {
        NOT_INIT,
        SUCCESS = 1,
        COMM_FAIL,
        CONDITION_FAIL,
        BAD_CAN_TIMING,
    };

    bool writeQueueItemParameters(QueueItemParameters parameters);

    /**
     * @brief Write command into stepper motion queue
     *
     * @param driver_num driver number
     * @param control_word ControlWord
     *
     * @return Sucessfully written into queue
     */
    bool writeIntoQueue(uint8_t driver_num, uint16_t control_word);

    /* ============================ Driver Response Verifiers =========================== */

    static bool getReadback_writeSuccessful(uint16_t response);

    static bool getReadback_putQueueSuccessful(uint16_t response);

    /* ================================== Communication ================================= */

    /**
     * @brief Perform exchange of CAN messages
     *
     * @param identifier targets message to mailbox. Use constructIdentifier to construct.
     * @param word data to transmit
     * @param response_buffer buffer to store response
     * @param buffer_size buffer size
     *
     * @return transmission success
     */
    bool CANTransfer(uint32_t identifier, uint32_t word, uint32_t *response_buffer, int buffer_size = 1);

    /**
     * @brief Transmit string content to buffer mailbox in 32-bit frames
     *
     * @param content
     * @return content length matches
     */
    bool transferStringToBuffer(std::string *content);

    /**
     * @brief Fetch string as defined in sd register
     *
     * @param reg_num sd register
     * @param data write data (if applicable)
     * @param file_content ptr to storage ptr
     * @return bssb_response_t
     */
    bssb_response_t fetchString(uint16_t reg_num, uint16_t data, std::string **file_content);

    /* ================================== Mailbox Comm ================================== */

    using verifier_t = std::function<bool(uint16_t)>;

    /**
     * @brief Send message stepper write mailbox
     *
     * @param driver_num driver number
     * @param reg_num RegS_W
     * @param data datafield
     * @param verifier result decoder (Checks Response::SUCCESS on default)
     * @param response ptr to var to store response
     * @return bssb_response_t
     */
    bssb_response_t writeStepper(uint8_t driver_num, uint16_t reg_num, uint16_t data,
                                 verifier_t verifier = getReadback_writeSuccessful, uint16_t *response = nullptr);

    /**
     * @brief Send message stepper read mailbox
     *
     * @param driver_num driver number
     * @param reg_num RegS_R
     * @param response ptr to var to store 16-bit response
     * @return bssb_response_t
     */
    bssb_response_t readStepper(uint8_t driver_num, uint16_t reg_num, uint16_t *response);

    /**
     * @brief Send message stepper read mailbox
     *
     * @param driver_num driver number
     * @param reg_num RegS_R
     * @param response ptr to var to store 32-bit full response
     * @return bssb_response_t
     *
     */
    bssb_response_t readStepper32b(uint8_t driver_num, uint16_t reg_num, uint32_t *response);

    /**
     * @brief Send message to board read mailbox
     *
     * @param reg_num Selected Reg_Board
     * @param response ptr to var to store response
     * @return bssb_response_t
     */
    bssb_response_t readBoard(uint16_t reg_num, uint16_t *response);

    /**
     * @brief Send message to board write mailbox
     *
     * @param reg_num Selected Reg_Board
     * @param data data to write
     * @param verifier result decoder (Checks Response::SUCCESS on default)
     * @param response ptr to var to store response
     * @return bssb_response_t
     */
    bssb_response_t writeBoard(uint16_t reg_num, uint16_t data, verifier_t verifier = getReadback_writeSuccessful,
                               uint16_t *response = nullptr);

    /**
     * @brief Send message to buffer mailbox
     *
     * @param data data to write
     * @param is_start_end_byte
     * @return bssb_response_t
     */
    bssb_response_t writeBuffer(uint32_t data, bool is_start_end_byte = false);

    /* ===================================== Utility ==================================== */

    struct UInt32Parts {
        uint16_t high;
        uint16_t low;
    };

    /**
     * @brief maps current in mA to scale of 1-31
     *
     * @param current_ma
     * @return uint16_t
     */
    static uint16_t getCurrentScaler(uint16_t current_ma);

    /**
     * @brief For time(ms), speed(rpm), acceleration(rpm/s), and bow(rpm/s2) conversion
     *
     * @param value
     * @return word
     */
    static UInt32Parts floatToHighAndLowWords(float value);

    /**
     * @brief For position
     *
     * @param value
     * @return UInt32Parts
     */
    static UInt32Parts int32ToHighAndLowWords(int32_t value);

    /**
     * @brief call async sleep for x iterations
     *
     * @param ms
     */
    void asyncSleep_ms(uint8_t ms);

    enum Mailbox {
        READ_STEPPER  = 0,
        STEPPER_WRITE = 1,
        BOARD_READ    = 3,
        BOARD_WRITE   = 4,
        BUFFER        = 5,
    };

    /**
     * @brief Construct mailbox identifier for bssb
     *
     * @param mb_id
     * @param board_num
     * @return uint32_t
     *
     * @note bit31-16 - don't care
     * @note bit15-8 - mailbox_id
     * @note bit7-0 - board_num
     */
    static inline uint32_t constructIdentifier(uint8_t mb_id, uint8_t board_num) {
        return (static_cast<uint32_t>(mb_id << 8) | (board_num & 0xFF));
    }

    /**
     * @brief Construct select word for writing queue item
     *
     * @param motion_num
     * @param item_num
     * @return uint32_t
     *
     * @note bit15-8 - motion_num
     * @note bit7-0 - item_num
     */
    static inline uint16_t constructSelectWord(uint8_t motion_num, uint8_t item_num) {
        return (static_cast<uint16_t>((motion_num & 0xFF) << 8) | (item_num & 0xFF));
    }
};