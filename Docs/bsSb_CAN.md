# BIGSIS STEPPER BOARD

# ၊၊||၊ CAN Communication

1.  CAN Settings:
    1. Baudrate `500k`
    2. Frame format `data`
    3. Frame type `standard`
    4. Data length `4`
2.  The identifier (16-bit) that is used to direct commands to respective mailboxes should be constructed as
    `MAILBOX_ID (8-bit)` `DEVICE_ID (8-bit)`. For example, messages directed to the **Stepper Write** mailbox `ID 1` on a stepper board configured with `ID 8` will have the identifier `0x0108`.
3.  There are **5 receiving mailboxes**:

    1.  **Stepper Read** `0x00` - Read command for individual drivers
    2.  **Stepper Write** `0x01` - Write command for individual drivers
    3.  **Board Read** `0x03` - Read command for board-level stuff
    4.  **Board Write** `0x04` - Write command for board-level stuff
    5.  **Buffer** `0x05` - Write into `BUFFER`

### **RESPONSE_CODE**

- `0x0001` Success
- `0x1000` Invalid Input
- `0x1001` Value out of Bounds
- `0x1002` Write Fail
- `0x1003` Read Fail
- `0x1004` Busy
- `0x1005` Driver is Null
- `0x1006` Unknown
- `0x1007` Used by Queue

## ᯓ Stepper Read

1. Message Format

   > | Bits | 31 - 28       | 27 - 16         | 15 - 0     |
   > | ---- | ------------- | --------------- | ---------- |
   > | ---  | Driver Number | Register Number | Don't Care |

2. Response Format

   a. Success

   > | Bits | 31 - 0   |
   > | ---- | -------- |
   > | ---  | Response |

   b. Fail

   > | Bits | 31 - 28       | 27 - 16         | 15 - 0   |
   > | ---- | ------------- | --------------- | -------- |
   > | ---  | Driver Number | Register Number | Response |

3. Registers

   > | Registers | Name              | Description                                                  | Response                                                                                                                                                                                                                                                               |
   > | --------- | ----------------- | ------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
   > | 0x000     | STATUS            | Fetches status of driver                                     | `1` READY<br> `2` BUSY<br>`3` HOMING<br>`4` VIBRATING<br>`5` CTRL\*NOT_INIT<br>`6` MOTION_NOT_INIT<br>`7` STALL<br>`8` POS_ERR<br>`9` STOPPED<br>`10` COIL_OL<br>`11` COIL_SHORT<br>`12` OVERTEMP<br>`13` TMC4361A_COMM_ERR<br>`14` TMC5160_COMM_ERR<br>`15` UNDEFINED |
   > | 0x001     | READBACK          | Fetches the result of queued command                         | Depends on previous command called, `0x1003` if no readback is available.                                                                                                                                                                                              |
   > | 0x002     | INTERNAL_POSITION | Driver step position, converted to user pulse                | `int32`                                                                                                                                                                                                                                                                |
   > | 0x003     | ENCODER_POSITION  | Encoder step position, converted to user pulse               | `int32`                                                                                                                                                                                                                                                                |
   > | 0x004     | CURRENT_SPEED     | Current RPM                                                  | `int32`                                                                                                                                                                                                                                                                |
   > | 0x005     | SENSOR_STATUS     | Current reading of `STOP_L` and `STOP_R` sensors             | `bit 0` - STOP_L<br>`bit 1` - STOP_R                                                                                                                                                                                                                                   |
   > | 0x006     | ENCODER_POS_DEV   | Difference between driver and encoder position in microsteps | `int32`                                                                                                                                                                                                                                                                |

## ᯓ Stepper Write

1. Message Format

   > | Bits | 31 - 28       | 27 - 16         | 15 - 0 |
   > | ---- | ------------- | --------------- | ------ |
   > | ---  | Driver Number | Register Number | Data   |

2. Response Format

   > | Bits | 31 - 28       | 27 - 16         | 15 - 0   |
   > | ---- | ------------- | --------------- | -------- |
   > | ---  | Driver Number | Register Number | Response |

3. Control Registers

   > | Registers | Name         | Description        | Data               | Response           |
   > | --------- | ------------ | ------------------ | ------------------ | ------------------ |
   > | 0x000     | CONTROL_WORD | Calls Control Word | _See Control Word_ | _See Control Word_ |

   ### **Control Word**

   > | Data   | Name              | Description                                   | Response                                                                                                                                                                            |
   > | ------ | ----------------- | --------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
   > | 0x0000 | INIT              | `Queue` Initialize Controller                 | `1` - Successfully put into Queue<br>`2` - Queue is full<br>`3` - Pending result in `READBACK`<br> <br> ℹ️ On `1`, `RESPONSE_CODE` will be available in `READBACK` upon completion. |
   > | 0x0001 | SET_MOTION        | `Queue` Sets Motion Parameters                | `1` - Successfully put into Queue<br>`2` - Queue is full<br>`3` - Pending result in `READBACK`<br> <br> ℹ️ On `1`, `RESPONSE_CODE` will be available in `READBACK` upon completion. |
   > | 0x0002 | ENABLE_AXIS       | Enables Driver                                | `RESPONSE_CODE`                                                                                                                                                                     |
   > | 0x0003 | RELEASE_AXIS      | Disables Driver                               | `RESPONSE_CODE`                                                                                                                                                                     |
   > | 0x1000 | MOVE              | `Queue` Runs Move                             | `1` - Successfully put into Queue<br>`2` - Queue is full<br>`3` - Pending result in `READBACK`<br> <br> ℹ️ On `1`, `EXEC_CODE` will be available in `READBACK` upon completion.     |
   > | 0x1001 | MOVE_HOMING       | `Queue` Runs Homing                           | `1` - Successfully put into Queue<br>`2` - Queue is full<br>`3` - Pending result in `READBACK`<br> <br> ℹ️ On `1`, `HOMING_CODE` will be available in `READBACK` upon completion.   |
   > | 0x1002 | MOVE_INVERSE_TIME | `Queue` Runs Move (Time Controlled)           | `1` - Successfully put into Queue<br>`2` - Queue is full<br>`3` - Pending result in `READBACK`<br> <br> ℹ️ On `1`, `EXEC_CODE` will be available in `READBACK` upon completion.     |
   > | 0x1003 | MOVE_VIBRATION    | `Queue` Runs Vibration                        | `1` - Successfully put into Queue<br>`2` - Queue is full<br>`3` - Pending result in `READBACK`<br> <br> ℹ️ On `1`, `EXEC_CODE` will be available in `READBACK` upon completion.     |
   > | 0x2000 | RAMP_STOP         | Stops gracefully (follows deceleration curve) | `RESPONSE_CODE`                                                                                                                                                                     |
   > | 0x2001 | EMERGENCY_STOP    | Stops abruptly. Raises Error                  | `RESPONSE_CODE`                                                                                                                                                                     |
   > | 0x3000 | SET_ZERO          | Resets internal position                      | `RESPONSE_CODE`                                                                                                                                                                     |

   ### **EXEC_CODE**

   - `1` E_SUCCESS
   - `2` E_WRITE_FAIL
   - `3` E_CTRL_NOT_INIT
   - `4` E_MOTION_NOT_INIT
   - `5` E_IS_FROZEN
   - `6` E_IS_BUSY
   - `7` E_BAD_SETTINGS

   ### **HOMING_CODE**

   - `1` H_SUCCESS
   - `2` H_WRITE_FAIL
   - `3` H_CTRL_NOT_INIT
   - `4` H_MOTION_NOT_INIT
   - `5` H_IS_FROZEN
   - `6` H_IS_BUSY
   - `7` H_TIMEOUT
   - `8` H_MAX_PULSE_REACHED
   - `9` H_FAILED_MIDWAY

4. Driver Registers

   > | Registers | Name                 | Description                                                | Data                                       | Response        |
   > | --------- | -------------------- | ---------------------------------------------------------- | ------------------------------------------ | --------------- |
   > | 0x100     | DRV_MSTEP_PER_FS     | Driver microstepping                                       | `1` `2` `4` `8` `16` `32` `64` `128` `256` | `RESPONSE_CODE` |
   > | 0x101     | DRV_FS_PER_REV       | Fullstep per Revolution                                    | Default `200`                              | `RESPONSE_CODE` |
   > | 0x102     | CURRENT_HOLD         | Holding Current                                            | 1-31                                       | `RESPONSE_CODE` |
   > | 0x103     | CURRENT_RUN          | Peak Current                                               | 1-31                                       | `RESPONSE_CODE` |
   > | 0x104     | STOP_ON_STALL_ENABLE | Toggle stall detection                                     | `0` - Off (default) <br>`1` - On           | `RESPONSE_CODE` |
   > | 0x105     | STOP_ON_STALL_THRESH | Minimum RPM to activate stall detection                    | 0-3000<br>Default `100`                    | `RESPONSE_CODE` |
   > | 0x106     | CL_ENABLE            | Toggle closed loop                                         | `0` - Off (default) <br>`1` - On           | `RESPONSE_CODE` |
   > | 0x107     | CL_ENC_IN_RES        | Encoder Resolution in pulse                                | Default `1000`                             | `RESPONSE_CODE` |
   > | 0x108     | CL_TOLERANCE         | Step difference in encoder pulse to trigger position error | Default `200`                              | `RESPONSE_CODE` |
   > | 0x109     | CL_ENABLE_PID        | Toggle closed loop PID control                             | `0` - Off (default) <br>`1` - On           | `RESPONSE_CODE` |
   > | 0x10A     | STEALTH_CHOP_THRESH  | RPM to transition from silent mode to power mode           | 0-3000<br>Default `100`                    | `RESPONSE_CODE` |

5. Motion Parameter Registers

   ⚠️ For registers that has `_H` and `_L`, data has to be separated into high and low bits and written into the respective registers.

   > | Registers | Name                                   | Description                                                              | Data                             | Response        |
   > | --------- | -------------------------------------- | ------------------------------------------------------------------------ | -------------------------------- | --------------- |
   > | 0x200     | MOVE_UNIT_HIGH                         | User position units                                                      | `int32`                          | `RESPONSE_CODE` |
   > | 0x201     | MOVE_UNIT_LOW                          |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x202     | MOVE_TIME_MS_HIGH                      | Time in _ms_ for _inverse time_ move                                     | `uint32`                         | `RESPONSE_CODE` |
   > | 0x203     | MOVE_TIME_MS_LOW                       |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x204     | MOVE_VIB_I                             | Iterations to vibrate for _vibrate_ move                                 | `uint16`                         | `RESPONSE_CODE` |
   > | 0x205     | MOVE_VIB_DIM_FACTOR                    | Magnitude scaling per back-forth iteration for _vibrate_ move _(x1000)_  | 0-1000                           | `RESPONSE_CODE` |
   > | 0x206     | MOVE_VIB_LOOP                          | Loop _vibrate_ move forever. Requires stop to exit                       | `0` - Off (default) <br>`1` - On | `RESPONSE_CODE` |
   > | 0x207     | MOVE_RESET_MOTION_CONF_AFTER_EACH_MOVE | Requires 'Set Motion' after each move command                            | `0` - Off (default) <br>`1` - On | `RESPONSE_CODE` |
   > | 0x208     | MOVE_ALLOW_WRITE_MOTION_WHEN_BUSY      | Allows _move_ to be called when driver status is _busy_                  | `0` - Off (default) <br>`1` - On | `RESPONSE_CODE` |
   > | 0x209     | SPEED_MAX_H                            | Maximum speed in RPM _(x1000)_                                           | 0-3000000                        | `RESPONSE_CODE` |
   > | 0x20A     | SPEED_MAX_L                            |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x20B     | SPEED_START_H                          | Starting speed in RPM _(x1000)_                                          | 0-3000000<br>Default `0`         | `RESPONSE_CODE` |
   > | 0x20C     | SPEED_START_L                          |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x20D     | SPEED_STOP_H                           | Stopping speed in RPM _(x1000)_                                          | 0-3000000<br>Default `0`         | `RESPONSE_CODE` |
   > | 0x20E     | SPEED_STOP_L                           |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x20F     | SPEED_BREAK_H                          | Breaking speed in RPM _(x1000)_<br><br>ℹ️Affects _Trapezoidal Ramp_ only | 0-3000000<br>Default `0`         | `RESPONSE_CODE` |
   > | 0x210     | SPEED_BREAK_L                          |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x211     | ACC_MAX_ACCEL_H                        | Maximum acceleration in RPM/s _(x1000)_                                  | 0-16777000                       | `RESPONSE_CODE` |
   > | 0x212     | ACC_MAX_ACCEL_L                        |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x213     | ACC_MAX_DECEL_H                        | Maximum deceleration in RPM/s _(x1000)_                                  | 0-16777000                       | `RESPONSE_CODE` |
   > | 0x214     | ACC_MAX_DECEL_L                        |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x215     | ACC_START_ACCEL_H                      | Starting acceleration in RPM/s _(x1000)_                                 | 0-16777000<br>Default `0`        | `RESPONSE_CODE` |
   > | 0x216     | ACC_START_ACCEL_L                      |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x217     | ACC_FINAL_DECEL_H                      | Final acceleration in RPM/s _(x1000)_                                    | 0-16777000<br>Default `0`        | `RESPONSE_CODE` |
   > | 0x218     | ACC_FINAL_DECEL_L                      |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x219     | BOW1_H                                 | Bow value (jerk) in RPM/s2 _(x1000)_<br><br>ℹ️Affects _SCurve Ramp_ only | `uint32`                         | `RESPONSE_CODE` |
   > | 0x21A     | BOW1_L                                 |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x21B     | BOW2_H                                 | Bow value (jerk) in RPM/s2 _(x1000)_<br><br>ℹ️Affects _SCurve Ramp_ only | `uint32`                         | `RESPONSE_CODE` |
   > | 0x21C     | BOW2_L                                 |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x21D     | BOW3_H                                 | Bow value (jerk) in RPM/s2 _(x1000)_<br><br>ℹ️Affects _SCurve Ramp_ only | `uint32`                         | `RESPONSE_CODE` |
   > | 0x21E     | BOW3_L                                 |                                                                          |                                  | `RESPONSE_CODE` |
   > | 0x21F     | BOW4_H                                 | Bow value (jerk) in RPM/s2 _(x1000)_<br><br>ℹ️Affects _SCurve Ramp_ only | `uint32`                         | `RESPONSE_CODE` |
   > | 0x220     | BOW4_L                                 |                                                                          |                                  | `RESPONSE_CODE` |

6. Homing Setting Registers

   ⚠️ For registers that has `_H` and `_L`, data has to be separated into high and low bits and written into the respective registers.

   > | Registers | Name                   | Description                                                                            | Data                                                       | Response        |
   > | --------- | ---------------------- | -------------------------------------------------------------------------------------- | ---------------------------------------------------------- | --------------- |
   > | 0x300     | HOME_HOMING_MODE       | Homing Type                                                                            | `0` - Immediate (default) <br>`1` - Torque<br>`2` - Sensor | `RESPONSE_CODE` |
   > | 0x301     | HOME_HOMING_SENSOR     | Sensor to use for _Sensor_ homing                                                      | `0` - STOP_L (default) <br>`1` - STOP_R                    | `RESPONSE_CODE` |
   > | 0x302     | HOME_SENSOR_HOME_VALUE | Sensor value to indicate home                                                          | 0-1                                                        | `RESPONSE_CODE` |
   > | 0x303     | HOME_MAX_FIND_H        | User position units                                                                    | `int32`                                                    | `RESPONSE_CODE` |
   > | 0x304     | HOME_MAX_FIND_L        |                                                                                        |                                                            | `RESPONSE_CODE` |
   > | 0x305     | HOME_MAX_SPEED_H       | Maximum homing speed in RPM _(x1000)_                                                  | 0-3000000                                                  | `RESPONSE_CODE` |
   > | 0x306     | HOME_MAX_SPEED_L       |                                                                                        |                                                            | `RESPONSE_CODE` |
   > | 0x307     | HOME_MAX_ACCEL_H       | Maximum homing acceleration in RPM/s _(x1000)_                                         | 0-16777000                                                 | `RESPONSE_CODE` |
   > | 0x308     | HOME_MAX_ACCEL_L       |                                                                                        |                                                            | `RESPONSE_CODE` |
   > | 0x309     | HOME_MAX_DECEL_H       | Maximum homing deceleration in RPM/s _(x1000)_                                         | 0-16777000                                                 | `RESPONSE_CODE` |
   > | 0x30A     | HOME_MAX_DECEL_L       |                                                                                        |                                                            | `RESPONSE_CODE` |
   > | 0x30B     | HOME_OFFSET_H          | Homing offset in user position units<br><br>⚠️Position will be zeroed before this move | `int32`                                                    | `RESPONSE_CODE` |
   > | 0x30C     | HOME_OFFSET_L          |                                                                                        |                                                            | `RESPONSE_CODE` |
   > | 0x30D     | HOME_TIMEOUT_MS_H      | Time in _ms_ for homing timeout                                                        | `uint32`                                                   | `RESPONSE_CODE` |
   > | 0x30E     | HOME_TIMEOUT_MS_L      |                                                                                        |                                                            | `RESPONSE_CODE` |

7. Motion Shaping Registers
   > | Registers | Name             | Description                              | Data                                                             | Response        |
   > | --------- | ---------------- | ---------------------------------------- | ---------------------------------------------------------------- | --------------- |
   > | 0x400     | POS_MODE         | Positioning mode                         | `0` - Relative (default) <br>`1` - Absolute                      | `RESPONSE_CODE` |
   > | 0x401     | POS_FOLLOW_MODE  | Positioning reference                    | `0` - Internal (default) <br>`1` - Encoder                       | `RESPONSE_CODE` |
   > | 0x402     | POS_UNIT_PER_REV | User position units per motor revolution | Default `200`                                                    | `RESPONSE_CODE` |
   > | 0x403     | RAMP_MODE        | Velocity / Position                      | `0` - Velocity <br>`4` - Positioning (default)                   | `RESPONSE_CODE` |
   > | 0x404     | RAMP_TYPE        | Shape of acceleration / deceleration     | `0` - Hold Ramp <br>`1` - Trapezoidal (default)<br>`2` - S-Curve | `RESPONSE_CODE` |

## Board Read

ℹ️ If _Data_ field isn't specified, it does not matter

1. Message Format

   > | Bits | 31 - 28    | 27 - 16         | 15 - 0 |
   > | ---- | ---------- | --------------- | ------ |
   > | ---  | Don't Care | Register Number | Data   |

2. Response Format

   > | Bits | 31 - 28    | 27 - 16         | 15 - 0   |
   > | ---- | ---------- | --------------- | -------- |
   > | ---  | Don't Care | Register Number | Response |

3. Registers
   > | Registers | Name            | Description                             | Response                                                                                                                                                                                                                                           |
   > | --------- | --------------- | --------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
   > | 0x001     | FIRMWARE_VER    | Fetches firmware version                | _Response Format:_<br>`0xA0B0C0D0` - Start Word<br>`CONTENT` - 32-bit Little Endian<br> `0xD0C0B0A` - End Word                                                                                                                                     |
   > | 0x102     | MOTION_READBACK | Readback of _Motion Queue_ operation    | `0x1003` if no result is available<br>`1` SUCCESS<br>`2` CONDITIONAL_SUCCESS<br>`3` FAILED_MIDWAY<br>`4` INVALID_MOTION<br>`5` EMPTY_QUEUE<br>`6` EXCEED_MAX_CONCURRENT<br>`7` REPEATED_ASSIGNMENT<br>`8` BAD_PARAMETERS<br>`9` STEPPERS_NOT_READY |
   > | 0x103     | QSTATUS         | Status of motion that is being executed | `bit 15` Executing flag<br>`bits 14-11` Motion number<br>`bits 10-0` Sequence number                                                                                                                                                               |
4. Buffer + SD Card
   > | Registers | Name                   | Description                                                                            | Data | Response                                                                                                                                  |
   > | --------- | ---------------------- | -------------------------------------------------------------------------------------- | ---- | ----------------------------------------------------------------------------------------------------------------------------------------- |
   > | 0x700     | SD_FETCH_DRV_CONFIG    | Returns Driver Config file from SD Card                                                | 0-4  | `0x1003` If read failed<br>_Response Format:_<br>`0xA0B0C0D0` - Start Word<br>`CONTENT` - 32-bit Little Endian<br> `0xD0C0B0A` - End Word |
   > | 0x701     | SD_FETCH_MOTION_CONFIG | Returns Motion file from SD Card<br>_index_ is defined in _\_indexMap_ file on SD Card | 0-31 | `0x1003` If read failed<br>_Response Format:_<br>`0xA0B0C0D0` - Start Word<br>`CONTENT` - 32-bit Little Endian<br> `0xD0C0B0A` - End Word |
   > | 0x702     | SD_FETCH_SEQUENCE      | Returns Sequence file from SD Card                                                     | 0-15 | `0x1003` If read failed<br>_Response Format:_<br>`0xA0B0C0D0` - Start Word<br>`CONTENT` - 32-bit Little Endian<br> `0xD0C0B0A` - End Word |
   > | 0x704     | BUF_LEN                | Returns stored `BUFFER` length                                                         | -    | `uint16`                                                                                                                                  |
   > | 0x706     | READ_BUFFER            | Returns stored `BUFFER`                                                                | -    | `0x1003` If read failed<br>_Response Format:_<br>`0xA0B0C0D0` - Start Word<br>`CONTENT` - 32-bit Little Endian<br> `0xD0C0B0A` - End Word |
   > | 0x707     | READ_CURRENT_PATH      | Returns stored `PATH`                                                                  | -    | `0x1003` If read failed<br>_Response Format:_<br>`0xA0B0C0D0` - Start Word<br>`CONTENT` - 32-bit Little Endian<br> `0xD0C0B0A` - End Word |
   > | 0x709     | READ_CURRENT_PATH      | Returns content of file at `PATH`                                                      | -    | `0x1003` If read failed<br>_Response Format:_<br>`0xA0B0C0D0` - Start Word<br>`CONTENT` - 32-bit Little Endian<br> `0xD0C0B0A` - End Word |

## ᯓ Board Write

1. Message Format

   > | Bits | 31 - 28    | 27 - 16         | 15 - 0 |
   > | ---- | ---------- | --------------- | ------ |
   > | ---  | Don't Care | Register Number | Data   |

2. Response Format

   > | Bits | 31 - 28    | 27 - 16         | 15 - 0   |
   > | ---- | ---------- | --------------- | -------- |
   > | ---  | Don't Care | Register Number | Response |

3. Control Registers

   > | Registers | Name                | Description                                                            | Data                                      | Response                                                                                                                                                                                                                                                                            |
   > | --------- | ------------------- | ---------------------------------------------------------------------- | ----------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
   > | 0x100     | MOTION_CONTROL_WORD |                                                                        | _See Motion Control Word_                 | _See Motion Control Word_                                                                                                                                                                                                                                                           |
   > | 0x104     | RUN_PRELOAD         | `Queue` Puts preloaded motion into the specified driver's motion queue | Motion index defined in _\_indexMap_ file | `bits 15-8` driver number <br>`bits 7-0`<br>`1` P\*SUCCESS<br>`2` P\*FULL<br>`3` P_RESULTS_PENDING<br>`4` P_INVALID_MOTION<br>`5` P_STEPPER_NOT_READY<br><br>ℹ️ On `1`, `EXEC_CODE` or `HOMING_CODE` will be available in `READBACK` of the returned driver number upon completion. |

   ### **Motion Control Word**

   > | Data        | Name         | Description                        | Response                                                                                                                                                                            |
   > | ----------- | ------------ | ---------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
   > | 0x000(0-F） | START_MOTION | `Motion Queue` Starts motion (0-F) | `1` - Successfully put into Queue<br>`2` - Queue is full<br>`3` - Pending result in `READBACK`<br> <br> ℹ️ On `1`, response will be available in `MOTION_READBACK` upon completion. |
   > | 0x1000      | ABORT        | Aborts motion queue                | `0x0001` - Successfully aborted motion queue<br>`0x1002` - Motion queue is not running<br> <br> ℹ️ On `0x0001`, response will be available in `MOTION_READBACK` upon completion.    |

4. Motion Queue Parameter Registers

   ⚠️ For registers that has `_H` and `_L`, data has to be separated into high and low bits and written into the respective registers.

   > | Registers | Name                         | Description                                                                            | Data                                                                                              | Response        |
   > | --------- | ---------------------------- | -------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- | --------------- |
   > | 0x101     | SELECT_MOTION_AND_QUEUE_ITEM | Maps subsequent write operations to item in selected motion                            | `bits 8-15` motion number<br>`bits 7-0` item number                                               | `RESPONSE_CODE` |
   > | 0x200     | QITEM_STEPPER_NUM            | Writes driver number of selected motion                                                | 0-4                                                                                               | `RESPONSE_CODE` |
   > | 0x201     | QITEM_MOTION_TYPE            | Writes motion type of selected motion                                                  | `0` MOVE_RELATIVE<br> `1` MOVE_ABSOLUTE<br> `2` MOVE_HOMING<br> `3` WAIT_TIME<br> `4` WAIT_SENSOR | `RESPONSE_CODE` |
   > | 0x202     | QITEM_SEQUENCE_NUMBER        | Writes sequence number of selected motion                                              | 0-4                                                                                               | `RESPONSE_CODE` |
   > | 0x203     | QITEM_POSITION_H             | Writes user position unit of selected motion                                           | `int32`                                                                                           | `RESPONSE_CODE` |
   > | 0x204     | QITEM_POSITION_L             |                                                                                        |                                                                                                   | `RESPONSE_CODE` |
   > | 0x300     | QITEM_RAMP_TYPE              | Shape of acceleration / deceleration                                                   | `0` - Hold Ramp <br>`1` - Trapezoidal (default)<br>`2` - S-Curve                                  | `RESPONSE_CODE` |
   > | 0x301     | QITEM_MAX_SPEED_H            | Maximum speed in RPM _(x1000)_                                                         | 0-3000000                                                                                         | `RESPONSE_CODE` |
   > | 0x302     | QITEM_MAX_SPEED_L            |                                                                                        |                                                                                                   | `RESPONSE_CODE` |
   > | 0x303     | QITEM_ACCEL_H                | Maximum acceleration in RPM/s _(x1000)_                                                | 0-16777000                                                                                        | `RESPONSE_CODE` |
   > | 0x304     | QITEM_ACCEL_L                |                                                                                        |                                                                                                   | `RESPONSE_CODE` |
   > | 0x305     | QITEM_DECEL_H                | Maximum deceleration in RPM/s _(x1000)_                                                | 0-16777000                                                                                        | `RESPONSE_CODE` |
   > | 0x306     | QITEM_DECEL_L                |                                                                                        |                                                                                                   | `RESPONSE_CODE` |
   > | 0x307     | QITEM_BOW1_H                 | Bow value (jerk) in RPM/s2 _(x1000)_<br><br>ℹ️Affects _SCurve Ramp_ only               | `uint32`                                                                                          | `RESPONSE_CODE` |
   > | 0x308     | QITEM_BOW1_L                 |                                                                                        |                                                                                                   | `RESPONSE_CODE` |
   > | 0x309     | QITEM_BOW2_H                 | Bow value (jerk) in RPM/s2 _(x1000)_<br><br>ℹ️Affects _SCurve Ramp_ only               | `uint32`                                                                                          | `RESPONSE_CODE` |
   > | 0x30A     | QITEM_BOW2_L                 |                                                                                        |                                                                                                   | `RESPONSE_CODE` |
   > | 0x30B     | QITEM_BOW3_H                 | Bow value (jerk) in RPM/s2 _(x1000)_<br><br>ℹ️Affects _SCurve Ramp_ only               | `uint32`                                                                                          | `RESPONSE_CODE` |
   > | 0x30C     | QITEM_BOW3_L                 |                                                                                        |                                                                                                   | `RESPONSE_CODE` |
   > | 0x30D     | QITEM_BOW4_H                 | Bow value (jerk) in RPM/s2 _(x1000)_<br><br>ℹ️Affects _SCurve Ramp_ only               | `uint32`                                                                                          | `RESPONSE_CODE` |
   > | 0x30E     | QITEM_BOW4_L                 |                                                                                        |                                                                                                   | `RESPONSE_CODE` |
   > | 0x400     | QITEM_USE_INVERSE_TIME       | Selected motion uses _move inverse time_                                               | `0` - Off (default) <br>`1` - On                                                                  | `RESPONSE_CODE` |
   > | 0x500     | QITEM_TIME_H                 | Timeout for `MOVE_HOMING` or wait time for `WAIT_TIME` (unit: ms)                      | `uint32`                                                                                          | `RESPONSE_CODE` |
   > | 0x501     | QITEM_TIME_L                 |                                                                                        |                                                                                                   | `RESPONSE_CODE` |
   > | 0x600     | QITEM_OFFSET_H               | Homing offset in user position units<br><br>⚠️Position will be zeroed before this move | `int32`                                                                                           | `RESPONSE_CODE` |
   > | 0x601     | QITEM_OFFSET_L               |                                                                                        |                                                                                                   | `RESPONSE_CODE` |
   > | 0x602     | QITEM_HOMING_MODE            | Homing Type                                                                            | `0` - Immediate (default) <br>`1` - Torque<br>`2` - Sensor                                        | `RESPONSE_CODE` |
   > | 0x603     | QITEM_HOMING_SENSOR          | Sensor to use for _Sensor_ homing                                                      | `0` - STOP_L (default) <br>`1` - STOP_R                                                           | `RESPONSE_CODE` |
   > | 0x604     | QITEM_SENSOR_HOME_VALUE      | Sensor value to indicate home                                                          | 0-1                                                                                               | `RESPONSE_CODE` |

5. Buffer + SD Card
   > | Registers | Name                   | Description                                                        | Data | Response        |
   > | --------- | ---------------------- | ------------------------------------------------------------------ | ---- | --------------- |
   > | 0x703     | BUF_CLEAR              | Clears stored `BUFFER`                                             | -    | `RESPONSE_CODE` |
   > | 0x705     | CP_BUFFER_AS_PATH      | Assign stored `BUFFER` as `PATH` and clear buffer                  | -    | `RESPONSE_CODE` |
   > | 0x708     | SD_SAVE_FILE_AT_PATH   | Saves `BUFFER` into `PATH` on SD Card                              | -    | `RESPONSE_CODE` |
   > | 0x70A     | SD_DELETE_FILE_AT_PATH | Deletes file at `PATH`                                             | -    | `RESPONSE_CODE` |
   > | 0x70B     | SD_WIPE                | Wipes SD Card and create template folders<br>⚠️Not implemented yet | -    | `RESPONSE_CODE` |

## ᯓ Buffer

1. Buffer
   > | Registers | Name         | Description                                                                                                                                                                        | Data        | Response        |
   > | --------- | ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------- | --------------- |
   > | -         | `START WORD` | Signals `BUFFER` to start listening                                                                                                                                                | `0xA0B0C0D` | `RESPONSE_CODE` |
   > | -         | `Content`    | Buffer content (32-bits, Little Endian)<br>⚠️ Message should be transmitted at a rate of no less than 1ms<br>⚠️ To verify content, read `BUFFER_LEN` and compare with sent content | -           | -               |
   > | -         | `STOP WORD`  | Signals `BUFFER` to end listening                                                                                                                                                  | `0xD0C0B0A` | `RESPONSE_CODE` |
