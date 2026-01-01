/*
 * board_registers.h
 *
 * Created: 03/11/2025 15:09:41
 *  Author: Tan
 */

#ifndef BOARD_REGISTERS_H_
#define BOARD_REGISTERS_H_

#include <stdint.h>

namespace Registers_Board {
// board info
static constexpr uint16_t FIRMWARE_VER = 0x001;

// execute motion
static constexpr uint16_t MOTION_CONTROL_WORD = 0x100;

// Select the the motion and queue for next write operation
static constexpr uint16_t SELECT_MOTION_AND_QUEUE_ITEM = 0x101;

static constexpr uint16_t MOTION_READBACK = 0x102;
static constexpr uint16_t QSTATUS         = 0x103;

// Execute pre-loaded motion
static constexpr uint16_t RUN_PRELOAD = 0X104;

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
static constexpr uint16_t BUF_LEN                = 0x704; // Size
static constexpr uint16_t CP_BUFFER_AS_PATH      = 0x705; // Copies current buffer as path then clear
static constexpr uint16_t READ_BUFFER            = 0x706; // Return current buffer
static constexpr uint16_t READ_CURRENT_PATH      = 0x707; // Return current path
static constexpr uint16_t SD_SAVE_FILE_AT_PATH   = 0x708; // Save file at path
static constexpr uint16_t SD_READ_FILE_AT_PATH   = 0x709; // More generic version for read
static constexpr uint16_t SD_DELETE_FILE_AT_PATH = 0x70A; // Delete file at path
static constexpr uint16_t SD_WIPE                = 0x70B; // Delete file at path
} // namespace Registers_Board

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

static constexpr uint16_t ABORT = 0x1000;
} // namespace MotionControlWord

/* ================================================================================== */
/*                                      Response                                      */
/* ================================================================================== */

/**
 * @brief Response value
 *
 */
namespace Response {
static constexpr uint16_t SUCCESS             = 0x0001; // w-only
static constexpr uint16_t INVALID_INPUT       = 0x1000; // w-only
static constexpr uint16_t VALUE_OUT_OF_BOUNDS = 0x1001; // w-only
static constexpr uint16_t WRITE_FAIL          = 0x1002; // w-only
static constexpr uint16_t READ_FAIL           = 0x1003; // r-only
static constexpr uint16_t BUSY                = 0x1004; // w-only
static constexpr uint16_t DRIVER_IS_NULL      = 0x1005; // r/w
static constexpr uint16_t USED_BY_QUEUE       = 0x1006; // w-only
static constexpr uint16_t UNKNOWN             = 0x1007; // r/w
} // namespace Response

#endif /* BOARD_REGISTERS_H_ */