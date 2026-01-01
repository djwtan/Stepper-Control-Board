#ifndef TMC5160_H
#define TMC5160_H

#include "asf.h"

namespace TMC5160_Reg {
/* Register addresses */
/* General configuration registers */
constexpr uint8_t GCONF           = 0x00; // Global configuration flags
constexpr uint8_t GSTAT           = 0x01; // Global status flags
constexpr uint8_t IFCNT           = 0x02; // UART transmission counter
constexpr uint8_t SLAVECONF       = 0x03; // UART slave configuration
constexpr uint8_t IO_INPUT_OUTPUT = 0x04; // Read input / write output pins
constexpr uint8_t X_COMPARE       = 0x05; // Position comparison register
constexpr uint8_t OTP_PROG        = 0x06; // OTP programming register
constexpr uint8_t OTP_READ        = 0x07; // OTP read register
constexpr uint8_t FACTORY_CONF    = 0x08; // Factory configuration (clock trim)
constexpr uint8_t SHORT_CONF      = 0x09; // Short detector configuration
constexpr uint8_t DRV_CONF        = 0x0A; // Driver configuration
constexpr uint8_t GLOBAL_SCALER   = 0x0B; // Global scaling of motor current
constexpr uint8_t OFFSET_READ     = 0x0C; // Offset calibration results

/* Velocity dependent driver feature control registers */
constexpr uint8_t IHOLD_IRUN = 0x10; // Driver current control
constexpr uint8_t TPOWERDOWN = 0x11; // Delay before power down
constexpr uint8_t TSTEP      = 0x12; // Actual time between microsteps
constexpr uint8_t TPWMTHRS   = 0x13; // Upper velocity for stealthChop voltage PWM mode
constexpr uint8_t TCOOLTHRS  = 0x14; // Lower threshold velocity for switching on smart energy
                                     // coolStep and stallGuard feature
constexpr uint8_t THIGH = 0x15;      // Velocity threshold for switching into a different chopper mode and fullstepping

/* Ramp generator motion control registers */
constexpr uint8_t RAMPMODE = 0x20;  // Driving mode (Velocity, Positioning, Hold)
constexpr uint8_t XACTUAL  = 0x21;  // Actual motor position
constexpr uint8_t VACTUAL  = 0x22;  // Actual  motor  velocity  from  ramp  generator
constexpr uint8_t VSTART   = 0x23;  // Motor start velocity
constexpr uint8_t A_1      = 0x24;  // First acceleration between VSTART and V1
constexpr uint8_t V_1      = 0x25;  // First acceleration/deceleration phase target velocity
constexpr uint8_t AMAX     = 0x26;  // Second acceleration between V1 and VMAX
constexpr uint8_t VMAX     = 0x27;  // Target velocity in velocity mode
constexpr uint8_t DMAX     = 0x28;  // Deceleration between VMAX and V1
constexpr uint8_t D_1      = 0x2A;  // Deceleration between V1 and VSTOP
                                    // Attention:  Do  not  set  0  in  positioning  mode, even if V1=0!
constexpr uint8_t VSTOP = 0x2B;     // Motor stop velocity
                                    // Attention: Set VSTOP > VSTART!
                                    // Attention:  Do  not  set  0  in  positioning  mode, minimum 10 recommend!
constexpr uint8_t TZEROWAIT = 0x2C; // Waiting time after ramping down to zero velocity before
                                    // next movement or direction inversion can start.
constexpr uint8_t XTARGET = 0x2D;   // Target position for ramp mode

/* Ramp generator driver feature control registers */
constexpr uint8_t VDCMIN    = 0x33; // Velocity threshold for enabling automatic commutation dcStep
constexpr uint8_t SW_MODE   = 0x34; // Switch mode configuration
constexpr uint8_t RAMP_STAT = 0x35; // Ramp status and switch event status
constexpr uint8_t XLATCH    = 0x36; // Ramp generator latch position upon programmable switch event

/* Encoder registers */
constexpr uint8_t ENCMODE       = 0x38; // Encoder configuration and use of N channel
constexpr uint8_t X_ENC         = 0x39; // Actual encoder position
constexpr uint8_t ENC_CONST     = 0x3A; // Accumulation constant
constexpr uint8_t ENC_STATUS    = 0x3B; // Encoder status information
constexpr uint8_t ENC_LATCH     = 0x3C; // Encoder position latched on N event
constexpr uint8_t ENC_DEVIATION = 0x3D; // Maximum number of steps deviation between encoder
                                        // counter and XACTUAL for deviation warning

/* Motor driver registers */
constexpr uint8_t MSLUT_0_7  = 0x60; // Microstep table entries. Add 0...7 for the next registers
constexpr uint8_t MSLUTSEL   = 0x68; // Look up table segmentation definition
constexpr uint8_t MSLUTSTART = 0x69; // Absolute current at microstep table entries 0 and 256
constexpr uint8_t MSCNT      = 0x6A; // Actual position in the microstep table
constexpr uint8_t MSCURACT   = 0x6B; // Actual microstep current
constexpr uint8_t CHOPCONF   = 0x6C; // Chopper and driver configuration
constexpr uint8_t COOLCONF   = 0x6D; // coolStep smart current control register and stallGuard2 configuration
constexpr uint8_t DCCTRL     = 0x6E; // dcStep automatic commutation configuration register
constexpr uint8_t DRV_STATUS = 0x6F; // stallGuard2 value and driver error flags
constexpr uint8_t PWMCONF    = 0x70; // stealthChop voltage PWM mode chopper configuration
constexpr uint8_t PWM_SCALE  = 0x71; // Results of stealthChop amplitude regulator.
constexpr uint8_t PWM_AUTO   = 0x72; // Automatically determined PWM config values
constexpr uint8_t LOST_STEPS = 0x73; // Number of input steps skipped due to dcStep. only with SD_MODE = 1
} // namespace TMC5160_Reg

#endif /* TMC5160_H */