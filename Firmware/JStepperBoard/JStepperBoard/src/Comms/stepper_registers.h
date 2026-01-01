/*
 * define.h
 *
 * Created: 04/09/2025 10:09:51
 *  Author: Tan
 */

#ifndef STEPPER_REGISTERS_H_
#define STEPPER_REGISTERS_H_

#define REG_WRITE_CONTROL_MAX         0x0FF // Maximum number of control related registers
#define REG_WRITE_DRIVER_MAX          0x1FF // Maximum number of driver related registers
#define REG_WRITE_MOTION_PARAM_MAX    0x2FF // Maximum number of motion param related registers
#define REG_WRITE_HOME_MAX            0x3FF // Maximum number of homing related registers
#define REG_WRITE_MOTION_SETTINGS_MAX 0x4FF // Maximum number of motion shaping related registers

/* ================================================================================== */
/*                                        Write                                       */
/* ================================================================================== */

/**
 * @brief Write Registers
 *
 */
namespace Registers_StepperWrite {
static constexpr uint16_t CONTROL_WORD = 0x000;
/* ---------------------------------------------------------------------------------- */
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
/* ---------------------------------------------------------------------------------- */
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

/* ---------------------------------------------------------------------------------- */
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

/* ---------------------------------------------------------------------------------- */
// Motion Shaping
// Positioning Settings
static constexpr uint16_t POS_MODE         = 0x400;
static constexpr uint16_t POS_FOLLOW_MODE  = 0x401;
static constexpr uint16_t POS_UNIT_PER_REV = 0x402;
// Ramp Settings
static constexpr uint16_t RAMP_MODE = 0x403;
static constexpr uint16_t RAMP_TYPE = 0x404;
} // namespace Registers_StepperWrite

/**
 * @brief Valid values for CONTROL_WORD (0x0000)
 *
 */
namespace ControlWord {
// Setups
static constexpr uint16_t INIT         = 0x0000;
static constexpr uint16_t SET_MOTION   = 0x0001;
static constexpr uint16_t ENABLE_AXIS  = 0x0002;
static constexpr uint16_t RELEASE_AXIS = 0x0003;

// Move
static constexpr uint16_t MOVE              = 0x1000;
static constexpr uint16_t MOVE_HOMING       = 0x1001;
static constexpr uint16_t MOVE_INVERSE_TIME = 0x1002;
static constexpr uint16_t MOVE_VIBRATION    = 0x1003;

// Stop
static constexpr uint16_t RAMP_STOP      = 0x2000;
static constexpr uint16_t EMERGENCY_STOP = 0x2001;

// Misc
static constexpr uint16_t SET_ZERO = 0x3000;
} // namespace ControlWord

/* ================================================================================== */
/*                                        Read                                        */
/* ================================================================================== */

/**
 * @brief Read registers
 *
 * @todo If other registers need to be exposed, use one set of registers.
 *
 */
namespace Registers_StepperRead {
static constexpr uint16_t STATUS            = 0x000;
static constexpr uint16_t READBACK          = 0x001;
static constexpr uint16_t INTERNAL_POSITION = 0x002;
static constexpr uint16_t ENCODER_POSITION  = 0x003;
static constexpr uint16_t CURRENT_SPEED     = 0x004;
static constexpr uint16_t SENSOR_STATUS     = 0x005;
static constexpr uint16_t ENCODER_POS_DEV   = 0x006;
} // namespace Registers_StepperRead

#endif /* STEPPER_REGISTERS_H_ */