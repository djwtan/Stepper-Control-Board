/*
 * IncFile1.h
 *
 * Created: 06/08/2025 15:20:03
 *  Author: Tan
 */
#include "TMC4361A.h"
#include "asf.h"
#include "sd_card.h"

#ifndef STEPPER_H
#    define STEPPER_H

#    ifdef STEPPER_DEBUG
#        include <stdio.h>
#        define stepper_debug(...) printf(__VA_ARGS__)
#    else
#        define stepper_debug(...)
#    endif

#    define FLOAT_DIVISOR 1000 // 3 dp

class Stepper : public TMC4361A {
  public:
    /* Command mode */
    enum PositioningMode { PM_RELATIVE, PM_ABSOLUTE };

    /* Position mode source */
    enum FollowMode { INTERNAL, ENCODER };

    /*
     * Homing modes:
     * a. Immediate
     *    - Homes immediately, then moves to offset position.
     *
     * b. Torque
     *    - CL (Closed Loop): Moves until position error is detected, then moves to offset.
     *    - OL (Open Loop): Moves until stall flag is triggered (i.e., RPM falls below stall
     * threshold), then moves to the offset.
     *
     * c. Sensor
     *    - Moves until rising/falling edge is detected at selected sensor, then moves to offset.
     */
    enum HomingMode { IMMEDIATE, TORQUE, SENSOR };

    /* Homing Sensor */
    enum HomingSensor { STOP_L, STOP_R };

    /* Exec return code */
    enum ExecCode {
        E_SUCCESS = 1,
        E_WRITE_FAIL,
        E_CTRL_NOT_INIT,
        E_MOTION_NOT_INIT,
        E_IS_FROZEN,
        E_IS_BUSY,
        E_BAD_SETTINGS,
    };

    /* Homing return code */
    enum HomingCode {
        H_SUCCESS = 1,
        H_WRITE_FAIL,
        H_CTRL_NOT_INIT,
        H_MOTION_NOT_INIT,
        H_IS_FROZEN,
        H_IS_BUSY = 6,
        H_TIMEOUT,
        H_MAX_PULSE_REACHED,
        H_FAILED_MIDWAY,
    };

    /* Drv Status */
    enum Status {
        READY = 1,
        BUSY,
        HOMING,
        VIBRATING,
        CTRL_NOT_INIT,
        MOTION_NOT_INIT,
        STALL,
        POS_ERR = 8,
        STOPPED,
        COIL_OL,
        COIL_SHORT,
        OVERTEMP,
        TMC4361A_COMM_ERR,
        TMC5160_COMM_ERR,
        UNDEFINED,
    };

    /* Put Queue */
    enum PutQueueRes {
        Q_SUCCESS = 1,
        Q_FULL,
        Q_RESULTS_PENDING,
    };

    /*
     *1. reset_motion_conf_after_each_move
     Require 'setMotion' after every successful 'move' call
     *2. allow_write_motion_when_busy
     Enables 'move' to override target position in running state
     *3. step correction at 360 degrees
     */
    struct MoveSettings {
        uint16_t unit_high                         = 0;     // unit high word
        uint16_t unit_low                          = 0;     // unit low word
        uint16_t time_ms_high                      = 0;     // time in ms high word
        uint16_t time_ms_low                       = 0;     // time in ms low word
        uint16_t vib_i                             = 0;     // vibration iterations
        uint16_t vib_dim_factor                    = 0;     // vibration diminishing factor
        uint8_t  fs_per_correction                 = 2;     // ? Expose or nah?
        bool     vib_loop                          = false; // loop vibration forever
        bool     reset_motion_conf_after_each_move = false;
        bool     allow_write_motion_when_busy      = false;
    };

    struct HomeSettings {
        HomingMode   homing_mode       = HomingMode::IMMEDIATE;
        HomingSensor homing_sensor     = HomingSensor::STOP_L;
        bool         sensor_home_value = true;
        uint16_t     max_find_h        = 0;
        uint16_t     max_find_l        = 0;
        uint16_t     max_speed_h       = 0;
        uint16_t     max_speed_l       = 0;
        uint16_t     max_accel_h       = 0;
        uint16_t     max_accel_l       = 0;
        uint16_t     max_decel_h       = 0;
        uint16_t     max_decel_l       = 0;
        uint16_t     offset_h          = 0;
        uint16_t     offset_l          = 0;
        uint16_t     timeout_ms_h      = 0;
        uint16_t     timeout_ms_l      = 5000;
    };

    struct PositioningSettings {
        PositioningMode pos_mode     = PositioningMode::PM_ABSOLUTE;
        FollowMode      follow_mode  = FollowMode::INTERNAL;
        uint16_t        unit_per_rev = 200;
    };

    struct RampSettings {
        TMC4361A::RampMode ramp_mode = TMC4361A::RampMode::POSITIONING_MODE;
        TMC4361A::RampType ramp_type = TMC4361A::RampType::TRAPEZOIDAL_RAMP;
    };

    struct SpeedSettings {
        uint16_t max_h   = 0;
        uint16_t max_l   = 0;
        uint16_t start_h = 0;
        uint16_t start_l = 0;
        uint16_t stop_h  = 0;
        uint16_t stop_l  = 0;
        uint16_t break_h = 0;
        uint16_t break_l = 0;
    };

    struct AccelerationSettings {
        uint16_t max_accel_h   = 0;
        uint16_t max_accel_l   = 250;
        uint16_t max_decel_h   = 0;
        uint16_t max_decel_l   = 250;
        uint16_t start_accel_h = 0;
        uint16_t start_accel_l = 0;
        uint16_t final_decel_h = 0;
        uint16_t final_decel_l = 0;
    };

    struct BowSettings {
        uint16_t bow1_h = 0;
        uint16_t bow1_l = 100;
        uint16_t bow2_h = 0;
        uint16_t bow2_l = 100;
        uint16_t bow3_h = 0;
        uint16_t bow3_l = 100;
        uint16_t bow4_h = 0;
        uint16_t bow4_l = 100;
    };

    Stepper(Spi *spi, xQueueHandle *spi_mutex, uint8_t spi_cs_pin, uint32_t fclk, uint8_t nfreeze_pin, uint8_t nrst_pin,
            uint8_t intr_pin, uint8_t drv_en_pin);

    /* ---------------------------------------------------------------------------------- */
    /**
     * @brief Configure settings related to the 'move' command
     *
     * @param move_s
     */
    void confMove(MoveSettings move_s);
    void confMove_resetMotionConfAfterEachMove(uint16_t enable);
    void confMove_allowWriteMotionWhenBusy(uint16_t enable);

    /* ---------------------------------------------------------------------------------- */
    /**
     * @brief Configure settings related to the 'moveHoming' command
     *
     * @param home_s
     */
    void confHome(HomeSettings home_s);
    void confHome_homingMode(uint16_t homing_mode);
    void confHome_homingSensor(uint16_t homing_sensor);
    void confHome_sensorHomeValue(uint16_t home_value);
    void confHome_maxFind_h(uint16_t max_find_h);
    void confHome_maxFind_l(uint16_t max_find_l);
    void confHome_maxSpeed_h(uint16_t max_speed_h);
    void confHome_maxSpeed_l(uint16_t max_speed_l);
    void confHome_maxAccel_h(uint16_t max_accel_h);
    void confHome_maxAccel_l(uint16_t max_accel_l);
    void confHome_maxDecel_h(uint16_t max_decel_h);
    void confHome_maxDecel_l(uint16_t max_decel_l);
    void confHome_offset_h(uint16_t offset_h);
    void confHome_offset_l(uint16_t offset_l);
    void confHome_timeout_h(uint16_t timeout_ms_h);
    void confHome_timeout_l(uint16_t timeout_ms_l);

    /* ---------------------------------------------------------------------------------- */
    /**
     * @brief Configure positioning settings
     *
     * @param pos_s
     */
    void confPositioning(PositioningSettings pos_s);
    void confPositioning_posMode(uint16_t pos_mode);
    void confPositioning_followMode(uint16_t follow_mode);
    void confPositioning_unitPerRev(uint16_t unit_per_rev);

    /* ---------------------------------------------------------------------------------- */
    /**
     * @brief Configure ramp settings
     *
     * @param ramp_s
     */
    void confRamp(RampSettings ramp_s);
    void confRamp_rampMode(uint16_t ramp_mode);
    void confRamp_rampType(uint16_t ramp_type);

    /* ---------------------------------------------------------------------------------- */
    /**
     * @brief Configure speed settings
     *
     * @param speed_s
     */
    void confSpeed(SpeedSettings speed_s);
    void confSpeed_maxSpeed_h(uint16_t max_h);
    void confSpeed_maxSpeed_l(uint16_t max_l);
    void confSpeed_startSpeed_h(uint16_t start_h);
    void confSpeed_startSpeed_l(uint16_t start_l);
    void confSpeed_stopSpeed_h(uint16_t stop_h);
    void confSpeed_stopSpeed_l(uint16_t stop_l);
    void confSpeed_breakSpeed_h(uint16_t break_h);
    void confSpeed_breakSpeed_l(uint16_t break_l);

    /* ---------------------------------------------------------------------------------- */
    /**
     * @brief Configure acceleration settings
     *
     * @param accel_s
     */
    void confAccel(AccelerationSettings accel_s);
    void confAccel_maxAccel_h(uint16_t max_accel_h);
    void confAccel_maxAccel_l(uint16_t max_accel_l);
    void confAccel_maxDecel_h(uint16_t max_decel_h);
    void confAccel_maxDecel_l(uint16_t max_decel_l);
    void confAccel_startAccel_h(uint16_t start_accel_h);
    void confAccel_startAccel_l(uint16_t start_accel_l);
    void confAccel_finalDecel_h(uint16_t final_decel_h);
    void confAccel_finalDecel_l(uint16_t final_decel_l);

    /* ---------------------------------------------------------------------------------- */
    /**
     * @brief Configure bow (jerk) settings
     *
     * @param bow_s
     * @note valid for S-Shaped ramp only
     */
    void confBow(BowSettings bow_s);
    void confBow_bow1_h(uint16_t bow1_h);
    void confBow_bow1_l(uint16_t bow1_l);
    void confBow_bow2_h(uint16_t bow2_h);
    void confBow_bow2_l(uint16_t bow2_l);
    void confBow_bow3_h(uint16_t bow3_h);
    void confBow_bow3_l(uint16_t bow3_l);
    void confBow_bow4_h(uint16_t bow4_h);
    void confBow_bow4_l(uint16_t bow4_l);

    /* ---------------------------------------------------------------------------------- */

    /**
     * @brief derived emergency stop that resets flags
     *
     * @note external calls must call this instead of the base class emStop()
     */
    void d_emStop();

    /**
     * @brief derived ramp stop that resets flags
     *
     * @note external calls must call this instead of the base class rampStop()
     */
    void d_rampStop();

    /**
     * @brief derived set position. Can only be called when not moving.
     *
     * @return true
     * @return false
     */
    bool d_setPosition(int32_t pos);

    /**
     * @brief derived enable driver. Can only be called when not moving.
     *
     * @return true
     * @return false
     */
    bool d_enableDriver();

    /**
     * @brief derived release driver. Can only be called when not moving.
     *
     * @return true
     * @return false
     */
    bool d_releaseDriver();

    /**
     * @brief Get driver status
     *
     * @return Status
     */
    Status getStatus();

    /**
     * @brief  Wait for running motion to complete (blocking)
     *
     */
    void waitCompleteRun();

    /* =================================== CAN Support ================================== */

    void setTargetUnitsHigh(uint16_t unit_high);
    void setTargetUnitsLow(uint16_t unit_low);
    void setTimeMsHigh(uint16_t time_ms_high);
    void setTimeMsLow(uint16_t time_ms_low);
    void setVibrationIterations(uint16_t vib_i);
    void setDiminishingFactor(uint16_t vib_dim_factor);
    void setLoop(uint16_t vib_loop);
    void rstDrvFlag();
    void rstMotionFlag();

    /* ==================================== Functions =================================== */

    /**
     * @brief Puts move command into queue
     *
     * @param units In positioning mode: target position in units.
     *              In velocity mode: speed in units/minute (sign determines direction).
     * @return PutQueueRes
     */
    PutQueueRes move();

    /**
     * @brief Puts homing sequence into queue
     *
     * @return PutQueueRes
     */
    PutQueueRes moveHoming();

    /**
     * @brief Puts move inverse time command into queue
     *
     * @return PutQueueRes
     */
    PutQueueRes moveInverseTime();

    /**
     * @brief Puts vibration command into queue
     *
     * @param units magnitude
     * @param iterations vibrating max iteration
     * @param diminishing_factor magnitude decrement every iteration
     * @param loop run forever
     * @return PutQueueRes
     */
    PutQueueRes moveVibration();

    /**
     * @brief Puts init controller command into queue
     *
     * @return PutQueueRes
     */
    PutQueueRes initController();

    /**
     * @brief Puts set motion command into queue
     *
     * @return PutQueueRes
     */
    PutQueueRes setMotion();

    /* ====================================== Query ===================================== */

    /**
     * @brief Get the Readback object
     *
     * @param readback pointer to store return value
     * @return readback value received
     */
    bool getReadback(uint32_t *readback);

    /**
     * @brief Pulse to user defined unit
     *
     * @param unit units relative to unit_per_rev
     * @return units (user)
     */
    int32_t pulseToUnit(int32_t pulse);

    /**
     * @brief (Re)initialize controller
     *
     * @return controller initialized
     * @note call directly to bypass queue (blocking)
     */
    bool execInitController();

    /**
     * @brief Write motion settings to controller
     *
     * @return controller initialized
     * @note call directly to bypass queue (blocking)
     */
    bool execSetMotion();

  private:
    struct _WaitRes {
        bool condition_met;
        bool timed_out;
    };

    MoveSettings         m_move_s;
    HomeSettings         m_home_s;
    PositioningSettings  m_pos_s;
    RampSettings         m_ramp_s;
    SpeedSettings        m_spd_s;
    AccelerationSettings m_accel_s;
    BowSettings          m_bow_s;

    bool m_is_vibrating = false; // homing flag
    bool m_is_homing    = false; // homing flag
    bool m_ctrl_init    = false; // controller initialized flag
    bool m_motion_conf  = false; // motion confd flag
    bool m_run_task     = true;  // flag to end tasks prematurely

    /* ==================================== Wrappers ==================================== */
    /**
     * @brief Wrapper for move command
     *
     * @param args
     * @return ExecCode (cast to uint32_t)
     */
    uint32_t moveWrapper(void *args);

    /**
     * @brief Wrapper for homing sequence
     *
     * @return HomingCode (cast to uint32_t)
     */
    uint32_t moveHomingWrapper(void *args);

    struct _IvtArgs {
        int32_t units;
        float   time_ms;
    };

    /**
     * @brief Wrapper for inverse time move command
     *
     * @param args
     * @return ExecCode (cast to uint32_t)
     */
    uint32_t moveInverseTimeWrapper(void *args);

    struct _VibrationArgs {
        int32_t  units;
        uint32_t iterations;
        float    diminishing_factor;
        bool     loop;
    };
    /**
     * @brief Wrapper for vibrate command
     *
     * @param args
     * @return ExecCode (cast to uint32_t)
     */
    uint32_t moveVibrationWrapper(void *args);

    /* ====================================== Setup ===================================== */
    /**
     * @brief Wrapper for init controller command
     *
     * @param args
     * @return uint32_t
     */
    uint32_t execInitControllerWrapper(void *args);

    /**
     * @brief Wrapper for set motion command
     *
     * @param args
     * @return uint32_t
     */
    uint32_t execSetMotionWrapper(void *args);

    /* ==================================== Movements =================================== */
    /**
     * @brief Executes a move or run command depending on the current mode.
     *
     * @param units In positioning mode: target position in units.
     *              In velocity mode: speed in units/minute (sign determines direction).
     *
     * @return ExecCode
     */
    ExecCode execMove(int32_t units);

    /**
     * @brief Blocking homing sequence
     *
     * @return HomingCode
     */
    HomingCode execMoveHoming();

    /**
     * @brief Computes motion profile and executes a blocking move.
     *
     * @param units target position in units.
     * @param time_ms target time to complete motion
     * @return ExecCode
     *
     * @note Positioning mode only.
     * @note Motion doesn't need to be set first
     * @note trapezoidal ramp only
     *
     * @todo S-shape ramp support
     */
    ExecCode execMoveInverseTime(int32_t units, float time_ms);

    /**
     * @brief Mimics vibration
     *
     * @param units magnitude in units
     * @param iterations num of times to move back and forth
     * @param diminishing_factor decrease in magnitude every iteration
     * @param loop flag to loop forever
     * @return ExecCode
     *
     * @note Works in open loop mode only
     * @note Motion doesn't need to be set first
     */
    ExecCode execMoveVibrate(uint32_t units, uint32_t iterations, float diminishing_factor, bool loop);

    /* ====================================== Misc ====================================== */

    /**
     * @brief Combine two 16-bit words into a 32-bit value
     *
     * @param high
     * @param low
     * @return uint32_t
     */
    static uint32_t combineWord(uint16_t high, uint16_t low);

    /**
     * @brief  Convert integer to float
     *
     * @param val
     * @param dp decimal places
     * @param sign 1 if signed, 0 if unsigned
     * @return float
     */
    static float uintTofloat(uint32_t val, uint8_t sign);

    /**
     * @brief Converts unit to pulse (microstep)
     *
     * @param unit units relative to unit_per_rev
     * @return pulse (microstep)
     */
    int32_t unitToPulse(int32_t unit);

    /**
     * @brief Auto correct stepping difference if value is divisible by a full rotation
     *
     * @param pulse
     * @return int32_t
     */
    int32_t correctRotationalPulse(int32_t pulse);

    /**
     * @brief Returns target pulse based on settings
     *
     * @return int32_t
     */
    int32_t computeTargetPulse(int32_t pulses);

    /**
     * @brief Set motion for inverse time mode
     *
     * @param max_speed
     * @param max_accel
     * @return write success
     */
    bool setMotionInverseTime(float max_speed, float max_accel);

    /**
     * @brief Settings for homing motion
     *
     * @return write success
     */
    bool setHomingMotion();

    /**
     * @brief Settings for vibrating motion
     *
     * @return write success
     *
     * @todo Set vibration rpm
     */
    bool setVibratingMotion();

    /**
     * @brief Wait for condition to meet true | timeout
     *
     * @tparam Callable
     * @param payload_factory function that returns condition bool
     * @param value bool value to compare with
     * @param timeout time in ticks
     * @return _WaitRes
     */
    template <typename Callable> _WaitRes waitCondition(Callable payload_factory, bool value, portTickType timeout);

    /**
     * @brief Read sensor stop_l or stop_r
     *
     * @param homing_sensor
     * @return sensor triggered
     */
    bool readSensor(HomingSensor homing_sensor);

    /* ================================ Motion Queue Task =============================== */
    using MotionTask_t = uint32_t (Stepper::*)(void *); // pointer to member class

    struct MotionItem {
        MotionTask_t motion_task;
        void        *arg;
    };

    xQueueHandle m_motion_queue_handle = xQueueCreate(1, sizeof(MotionItem));
    xQueueHandle m_result_queue_handle = xQueueCreate(1, sizeof(uint32_t));

    /**
     * @brief motion queue executor
     *
     */
    static void task_motionQueue(void *parameters);
};

#endif /* STEPPER_H */