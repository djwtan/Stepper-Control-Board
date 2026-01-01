# BIGSIS STEPPER BOARD

# 💥 How to Make Things Move _(w. bsSb library)_

## 🗐 Calling Pre-Configured Motions (SD Card Required)

1. Configure driver settings _(see [bsSb_sdCard](bsSb_sdCard.md))_.
2. Configure individual motions _(see [bsSb_sdCard](bsSb_sdCard.md))_.
   - 💡 You can use `writeTestQueueItem()` -> `mq_startTestQueueItem()` -> `getMotionQueueReadback()` to find the desired parameters.
   - _to be automated_.
3. Driver should already be initialized on startup. You may call `readDriverStatus()` to confirm.
   - If DriverStatus is `STS_CTRL_NOT_INIT (5)`, `STS_STALL (7)`, `STS_POS_ERR (8)` or `STS_STOPPED (9)`, you need to reinitialize the driver by calling `q_initDriver()` -> `getQueueReadback()` and verify return value with `verifyQueueReadback_writeSuccessful()` (or simply readback = 1).
   - If DriverStatus is `STS_READY (1)` or `STS_MOTION_NOT_INIT (6)`, pre-configured motion is ready to be called.
4. Use `q_movePreloaded()` and call the motion number as defined in `_indexMap`_(see [bsSb_sdCard](bsSb_sdCard.md))_.
   - 💡 For concurrent movements, call this multiple times before going to the next step.
5. If (4) is successful, once the motion is complete, get readback with `getQueueReadback()` and verify with `ExecCode` or `HomingMode` based on configured _Motion Type_.
   - ℹ️ readback = 1 means successful regardless of type.
   - 💡 Use the `toString_` functions for debugging.
   - 💡 `getQueueReadback()` is a blocking function. For non-blocking, use `readDriverStatus()` until driver status is _**not**_ `STS_BUSY (2)` or `STS_HOMING (3)` and then call `getQueueReadback()` to acquire readback immediately. (Useful for concurrent operations where other movements need to be stopped if a movement fails)

Example

```cpp
uint8_t board_id = 1;  // Use your own board ID

bsStepperBoard bsSb(board_id);

// Override
bsSb.setCANTransmit(myCANTransmit);
bsSb.setCANReceive(myCANReceive);
bsSb.setCANMutex(myCANMutex);
bsSb.setAsyncSleep_1ms(myAsyncSleep);

// Run 2 motions together
uint8_t  drv0;
uint8_t  drv1;
uint16_t rb0;
uint16_t rb1;

bsSb.q_movePreloaded(12, &drv0);
bsSb.q_movePreloaded(6, &drv1);
bsSb.getQueueReadback(drv0, &rb0);
bsSb.getQueueReadback(drv1, &rb1);

printf("Output Z Home -> %s\n", toString_homingCode(static_cast<HomingCode>(rb0)));
printf("Input Z Home -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));

```

## 🗐 Calling Sequence (SD Card Required)

1. Configure driver settings _(see [bsSb_sdCard](bsSb_sdCard.md))_.
2. Configure individual motions _(see [bsSb_sdCard](bsSb_sdCard.md))_.
   - 💡 You can use `writeTestQueueItem()` -> `mq_startTestQueueItem()` -> `getMotionQueueReadback()` to find the desired parameters.
   - _to be automated_.
3. Configure sequence _(see [bsSb_sdCard](bsSb_sdCard.md))_.
4. Driver should already be initialized on startup. You may call `readDriverStatus()` to confirm.
   - If DriverStatus is `STS_CTRL_NOT_INIT (5)`, `STS_STALL (7)`, `STS_POS_ERR (8)` or `STS_STOPPED (9)`, you need to reinitialize the driver by calling `q_initDriver()` -> `getQueueReadback()` and verify return value with `verifyQueueReadback_writeSuccessful()` (or simply readback = 1).
   - If DriverStatus is `STS_READY (1)` or `STS_MOTION_NOT_INIT (6)` on all involved drivers, sequence is ready to be called.
5. Call `mq_startMotion()` with the configured motion number as in the `📁sequence` folder to start the sequence.
6. If (5) is successful, call `getmotionQueueReadback()` and verify with `MotionQueueRes`.
   - 💡 `getmotionQueueReadback()` is a blocking function. For non-blocking, use `getMotionQueueStatus()` which provides information on whether the motion is still executing, the motion number that is called, and the current sequence it is in. When the _is_executing_ flag is unset, call `getmotionQueueReadback()` to acquire readback immediately.

Example

```cpp
uint8_t board_id = 1;  // Use your own board ID

bsStepperBoard bsSb(board_id);

// Override
bsSb.setCANTransmit(myCANTransmit);
bsSb.setCANReceive(myCANReceive);
bsSb.setCANMutex(myCANMutex);
bsSb.setAsyncSleep_1ms(myAsyncSleep);

// Start Cage Homing Sequence
bsSb.mq_startMotion(0);

// Poll Status (Optional)

// QueueStatus q_sts;
// q_sts.is_executing = 1;

// while (q_sts.is_executing == 1) {
//   q_sts = bsSb.getMotionQueueStatus();
//   printf("E%d M%d S%d\n", q_sts.is_executing, q_sts.motion_num, q_sts.sequence_num);
// }

// Acquire Readback
MotionQueueRes res;
bsSb.getMotionQueueReadback(&res);

printf("Result -> %s\n", toString_motionQueueRes(static_cast<MotionQueueRes>(res)));

```

## ✒️ Manual Configuration (SD Card is Optional)

1. Configure driver settings _(see [bsSb_sdCard](bsSb_sdCard.md))_.
   - If SD card is not used, use `q_setAndInitDriver()` at least once on power up (or simply just use it everytime).
2. If SD card is not used, driver will not be initialized on startup. Use `q_setAndInitDriver()` and `getQueueReadback()` to write desired parameters.
3. Move:
   1. Homing
      1. On successful initialization, use `configureHoming()` to configure homing parameters. _(see struct for parameter details)_
         - ℹ️ If parameters do not change, skip this step in subsequent calls.
      2. Call `q_moveStepper_homing()` to start the homing move.
      3. If (2) is successful, get readback with `getQueueReadback()` and verify with `HomingCode`
         - 💡use `toString_homingCode` for debugging.
         - 💡 `getQueueReadback()` is a blocking function. For non-blocking, use `readDriverStatus()` until driver status is _**not**_ `STS_HOMING (3)` and then call `getQueueReadback()` to acquire readback immediately.
   2. Relative / Absolute
      1. On successful initialization, use `q_setMotion()` (or `q_moveStepper_inverseTime()` for inverse time) -> `getQueueReadback()` and verify return value with `verifyQueueReadback_writeSuccessful()` (or simply readback = 1).
      2. Call `q_moveStepper()` to move to desired position.
      3. If (2) is successful, get readback with `getQueueReadback()` and verify with `ExecCode`
         - 💡 Use the `toString_execCode` functions for debugging.
         - 💡 `getQueueReadback()` is a blocking function. For non-blocking, use `readDriverStatus()` until driver status is _**not**_ `STS_BUSY (2)` and then call `getQueueReadback()` to acquire readback immediately.
   3. Velocity
      1. On successful initialization, use `q_setMotion()` -> `getQueueReadback()` and verify return value with `verifyQueueReadback_writeSuccessful()` (or simply readback = 1).
      2. Call `q_moveStepper()` to accelerate towards desired RPM.
      3. If (2) is successful, get readback with `getQueueReadback()` and verify with `ExecCode`
         - 💡 Use the `toString_execCode` functions for debugging.
         - ℹ️ For velocity mode, readback is available immediately.
      4. Monitor with `readDriverStatus()`.
      5. To stop, use `rampStop()`.
   4. Vibration
      1. Call `q_moveStepper_vibration()` with desired parameters.
      2. If (1) is successful, get readback with `getQueueReadback()` and verify with `ExecCode`
         - 💡 Use the `toString_execCode` functions for debugging.
         - ℹ️ If `loop=true` readback will only become available if `rampStop()` / `emStop()` is called or if an error occurs. In this case, monitor with `readDriverStatus()`

Example

```cpp
uint8_t board_id   = 1;  // Use your own board ID
uint8_t driver_num = 0;  // Set to driver
uint16_t queue_readback;

bsStepperBoard bsSb(board_id);

// Override
bsSb.setCANTransmit(myCANTransmit);
bsSb.setCANReceive(myCANReceive);
bsSb.setCANMutex(myCANMutex);
bsSb.setAsyncSleep_1ms(myAsyncSleep);

// Configure Driver
bsStepperBoard::DriverConfig drv_config;
drv_config.holding_current     = 500;
drv_config.peak_rms_current    = 3000;
drv_config.unit_per_rev        = 360;
drv_config.stealth_chop_thresh = 400;

bool     inited = bsSb.q_setAndInitDriver(driver_num, drv_config);
inited &= bsSb.getQueueReadback(driver_num, &queue_readback);       // read success
inited &= bsSb.verifyQueueReadback_writeSuccessful(queue_readback); // condition pass

printf("drv_init -> %d\n", inited);

// Configure Motion
bsStepperBoard::MotionConfig motion_config;
motion_config.max_speed = 100.0f;
motion_config.max_accel = 100.0f / 1.0f;
motion_config.max_decel = 100.0f / 1.0f;
motion_config.pos_mode  = PositioningMode::PM_RELATIVE;

bool     minited = bsSb.q_setMotion(driver_num, motion_config);
minited &= bsSb.getQueueReadback(driver_num, &queue_readback);       // read success
minited &= bsSb.verifyQueueReadback_writeSuccessful(queue_readback); // condition pass

printf("motion_init -> %d\n", minited);

// Move
bool     moved = bsSb.q_moveStepper(driver_num, 360);
moved &= bsSb.getQueueReadback(driver_num, &queue_readback);      // read success
moved &= bsSb.verifyQueueReadback_execSuccessful(queue_readback); // condition pass

printf("moved -> %d, result -> %s\n", moved, toString_execCode(static_cast<ExecCode>(queue_readback)));
```
