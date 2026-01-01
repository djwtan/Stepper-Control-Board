# BIGSIS STEPPER BOARD

# 💾 SD Card

- FAT32
- File Structure

  ```
  .
  ├── config/
  │   ├── 0
  │   ├── 1
  │   ├── 2
  │   ├── 3
  │   └── 4
  ├── motion/
  │   ├── _indexMap
  │   ├── motion_file1
  │   ├── motion_file2
  │   ├── ...
  │   └── motion_file31
  └── sequence/
      ├── 0
      ├── 1
      ├── ...
      └── 31
  ```

## 📁 `config`

- The `config` folder contains driver settings. If the file for the driver _(0-4)_ exists, the settings will be loaded on boot and `init` will be called automatically.

- Parameters

  - `MICROSTEP_PER_FULLSTEP`
    - Microstepping
    - Valid values are `1` `2` `4` `8` `16` `32` `64` `128` `256`
    - ℹ️ For closed-loop, use only `256`
  - `FULLSTEP_PER_REVOLUTION`
    - Number of Fullsteps per Revolution
    - Default value is `200`
  - `UNIT_PER_REVOLUTION`
    - Number of Units per Revolution
    - Example: If set to `360`, calling `q_moveStepper(30)` will move the driver 30 degrees.
    - ℹ️ If higher resolution is required, (eg: 5.625 degrees), set to `360000` and call `q_moveStepper(5625)`
    - ℹ️ Set this value to the end effector (consider gearbox, pulleys, etc.) to make life easier
  - `PEAK_CURRENT`
    - Current in mA
    - Maximum `3000`
  - `HOLDING_CURRENT`
    - Holding Current
    - Maximum `3000`
  - `STOP_ON_STALL_ENABLE`
    - Enable flag for stop-on-stall
    - `0` or `1`
  - `STOP_ON_STALL_THRESHOLD`
    - RPM to trigger stop-on-stall if enabled
    - ℹ️ Values of >100 RPM are more reliable
    - ⚠️ Very much application dependent. Not guaranteed to work.
  - `CLOSED_LOOP_ENABLE`
    - Enable Closed-Loop
    - `0` or `1`
  - `CLOSED_LOOP_USE_PID`
    - Enable PID Controller
    - `0` or `1`
    - ⚠️ Not recommended >600 RPM
  - `ENCODER_PULSE_PER_REVOLUTION`
    - Encode Pulse
    - ℹ️ Refer to motor datasheet
    - ℹ️ Use `4000` for stepperonline motors
  - `MAX_DELTA_PULSE`
    - Maximum position error in microsteps
  - `STEALTH_CHOP_THRESHOLD`
    - RPM to transition from "silent" mode to "power" mode
    - ⚠️ Value must not exceed `180` if using stop-on-stall
    - ℹ️ Simple guide:
    - Low rpm, low torque
      - Set above operation RPM
    - High rpm, low torque
      - Set at ~0.5 \* operation RPM
    - High torque
      - Set `50-150`

- Example (path: `config/1`)

  ```
  MICROSTEP_PER_FULLSTEP=256
  FULLSTEP_PER_REVOLUTION=200
  UNIT_PER_REVOLUTION=360
  PEAK_CURRENT=2000
  HOLDING_CURRENT=1000
  STOP_ON_STALL_ENABLE=0
  STOP_ON_STALL_THRESHOLD=500
  CLOSED_LOOP_ENABLE=0
  CLOSED_LOOP_USE_PID=0
  ENCODER_PULSE_PER_REVOLUTION=4000
  MAX_DELTA_PULSE=500
  STEALTH_CHOP_THRESHOLD=180
  ```

## 📁 `motion`

- The `motion` folder contains preconfigured motions. (Velocity and Vibration moves not supported)
- Fields not required by `MOTION_TYPE` may be omitted.
- File can be named whatever _(max length = 31)_, as long as it is properly mapped in \_indexMap
- Parameters

  - `MOTION_TYPE`
    - Command type to execute
      - `0` MOVE_RELATIVE
      - `1` MOVE_ABSOLUTE
      - `2` MOVE_HOMING
        - ⚠️ This move is relative
      - `3` WAIT_TIME
        - ⚠️ `q_movePreloaded()` cannot call this type
      - `4` WAIT_SENSOR
        - ⚠️ `q_movePreloaded()` cannot call this type
  - `DRIVER_NUMBER`
    - Used by Motion Types `0` `1` `2` `3`
    - Driver that executes the command
    - `0-4`
  - `POSITION`
    - Used by Motion Types `0` `1` `2`
    - Position units to move (as configured in `UNIT_PER_REVOLUTION` in driver settings)
  - `OFFSET`
    - Used by Motion Types `2`
    - Homing offset position
  - `HOMING_MODE`
    - Used by Motion Types `2`
    - `0` - Immediate <br>`1` - Torque<br>`2` - Sensor
  - `HOMING_SENSOR`
    - Used by Motion Types `2`, `4`
    - Sensor used for homing or wait (⚠️ driver specific)
    - `0` - STOP_L <br>`1` - STOP_R
  - `SENSOR_HOME_VALUE`
    - Used by Motion Types `2`, `4`
    - Trigger value
    - `0` or `1`
  - `RAMP_TYPE`
    - Used by Motion Types `0` `1` `2`
    - `0` - Hold Ramp <br>`1` - Trapezoidal (default)<br>`2` - S-Curve
  - `MAX_SPEED`
    - Used by Motion Types `0` `1` `2`
    - RPM
    - Supports values up to 3 decimal places
  - `MAX_ACCELERATION`
    - Used by Motion Types `0` `1` `2`
    - RPMs^-1
    - Maximum `16777`
    - Supports values up to 3 decimal places
  - `MAX_DECELERATION`
    - Used by Motion Types `0` `1` `2`
    - RPMs^-1
    - Maximum `16777`
    - Supports values up to 3 decimal places
  - `BOW1`
    - Used by Motion Types `0` `1`
    - RPMs^-2
    - Supports values up to 3 decimal places
    - ⚠️ Only valid for S-Curve Ramp Type
  - `BOW2`
    - Used by Motion Types `0` `1`
    - RPMs^-2
    - Supports values up to 3 decimal places
    - ⚠️ Only valid for S-Curve Ramp Type
  - `BOW3`
    - Used by Motion Types `0` `1`
    - RPMs^-2
    - Supports values up to 3 decimal places
    - ⚠️ Only valid for S-Curve Ramp Type
  - `BOW4`
    - Used by Motion Types `0` `1`
    - RPMs^-2
    - Supports values up to 3 decimal places
    - ⚠️ Only valid for S-Curve Ramp Type
  - `USE_INVERSE_TIME`

    - Used by Motion Types `0` `1`
    - Enables Inverse Time Mode
    - ℹ️ If enabled, ignore `RAMP_TYPE`, `MAX_SPEED`, `MAX_ACCELERATION`, `MAX_DECELERATION`, `BOW1`, `BOW2`, `BOW3`, `BOW4`,

  - `TIME_MS`

    - Used by Motion Types `0` `1` `2` `3` `4`
    - Time in ms
    - By Motion Type:

      - `MOVE_RELATIVE` - Time to complete move if `USE_INVERSE_TIME`
      - `MOVE_ABSOLUTE` - Time to complete move if `USE_INVERSE_TIME`
      - `MOVE_HOMING` - Homing timeout
      - `WAIT_TIME` - Total wait time
      - `WAIT_SENSOR` - Sensor wait timeout

- Example (path: `motion/input_z_home`)

  ```
  MOTION_TYPE=2
  DRIVER_NUMBER=0
  POSITION=360
  OFFSET=3
  HOMING_MODE=2
  HOMING_SENSOR=0
  SENSOR_HOME_VALUE=0
  RAMP_TYPE=0
  MAX_SPEED=20.0
  MAX_ACCELERATION=20.0
  MAX_DECELERATION=16777
  BOW1=0
  BOW2=0
  BOW3=0
  BOW4=0
  USE_INVERSE_TIME=0
  TIME_MS=10000
  ```

- The `_indexMap` file (path: `motion/_indexMap`) maps the motions to an index.
  - ⚠️ Do not assign same index, else the latter will override the prior.
  - ⚠️ If file is not present, no motion will be loaded.
  - Example
    ```
    0=cage_z_home
    1=cage_z_lower
    2=cage_z_raise
    3=input_x_home
    4=input_x_to_belt
    5=input_x_to_mid
    6=input_z_home
    7=input_z_lower
    8=input_z_raise
    9=output_x_home
    10=output_x_to_belt
    11=output_x_to_mid
    12=output_z_home
    13=output_z_lower
    14=output_z_raise
    ```
  - Result: `q_movePreloaded()` called with 0 will trigger `cage_z_home`

## 📁 `sequence`

- The sequence folder contains the sequence file (named as indexes) which motions can be arranged in a sequence when can then be called.
- Maximum concurrent motions is 5 (equivalent to the number of drivers).
- ⚠️ Motions that uses the same driver cannot be called together (except Motion Type `WAIT_TIME`)
- The format for each line is `sequence_number`=`motion_index`
- Example (path: `sequence/0`)

  ```
  0=14
  0=2
  1=11
  2=13
  3=5
  3=10
  4=8
  5=4
  6=7
  6=1
  ```

- The example provided is the _M1B Cage Sequence_
  - ℹ️ called with `mq_startMotion(0)` because filename is `0`
  - Lines1-2: `0=14` `0=2`
    - `output_z_raise (14)` and `cage_z_raise (2)` will be executed together (`sequence 0`)
  - Line3: `1=11`
    - `output_x_to_mid (11)` will be executed (`sequence 1`)
  - Line4: `2=13`
    - `output_z_lower (13)` will be executed (`sequence 2`)
  - Lines5-6: `3=5` `3=10`
    - `input_x_to_mid (5)` and `output_x_to_belt (10)` will be executed together (`sequence 3`)
  - `...`
