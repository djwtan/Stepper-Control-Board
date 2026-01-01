/*
 * TMC4361A_Register.h
 *
 *  Created on: 18.07.2017
 *      Author: LK
 */

#ifndef TMC4361A_H
#define TMC4361A_H

#include "TMC5160.h"
#include "asf.h"

namespace TMC4361A_Reg {
constexpr uint8_t GENERAL_CONF         = 0x00;
constexpr uint8_t REFERENCE_CONF       = 0x01;
constexpr uint8_t START_CONF           = 0x02;
constexpr uint8_t INPUT_FILT_CONF      = 0x03;
constexpr uint8_t SPIOUT_CONF          = 0x04;
constexpr uint8_t CURRENT_CONF         = 0x05;
constexpr uint8_t SCALE_VALUES         = 0x06;
constexpr uint8_t ENC_IN_CONF          = 0x07;
constexpr uint8_t ENC_IN_DATA          = 0x08;
constexpr uint8_t ENC_OUT_DATA         = 0x09;
constexpr uint8_t STEP_CONF            = 0x0A;
constexpr uint8_t SPI_STATUS_SELECTION = 0x0B;
constexpr uint8_t EVENT_CLEAR_CONF     = 0x0C;
constexpr uint8_t INTR_CONF            = 0x0D;
constexpr uint8_t EVENTS               = 0x0E;
constexpr uint8_t STATUS               = 0x0F;

constexpr uint8_t STP_LENGTH_ADD     = 0x10;
constexpr uint8_t DIR_SETUP_TIME     = 0x10;
constexpr uint8_t START_OUT_ADD      = 0x11;
constexpr uint8_t GEAR_RATIO         = 0x12;
constexpr uint8_t START_DELAY        = 0x13;
constexpr uint8_t CLK_GATING_DELAY   = 0x14;
constexpr uint8_t STDBY_DELAY        = 0x15;
constexpr uint8_t FREEWHEEL_DELAY    = 0x16;
constexpr uint8_t VDRV_SCALE_LIMIT   = 0x17;
constexpr uint8_t PWM_VMAX           = 0x17;
constexpr uint8_t UP_SCALE_DELAY     = 0x18;
constexpr uint8_t CL_UPSCALE_DELAY   = 0x18;
constexpr uint8_t HOLD_SCALE_DELAY   = 0x19;
constexpr uint8_t CL_DOWNSCALE_DELAY = 0x19;
constexpr uint8_t DRV_SCALE_DELAY    = 0x1A;
constexpr uint8_t BOOST_TIME         = 0x1B;
constexpr uint8_t CL_BETA            = 0x1C;
constexpr uint8_t CL_GAMMA           = 0x1C;
constexpr uint8_t DAC_ADDR_A         = 0x1D;
constexpr uint8_t DAC_ADDR_B         = 0x1D;
constexpr uint8_t SPI_SWITCH_VEL     = 0x1D;
constexpr uint8_t HOME_SAFETY_MARGIN = 0x1E;
constexpr uint8_t PWM_FREQ           = 0x1F;
constexpr uint8_t CHOPSYNC_DIV       = 0x1F;

constexpr uint8_t RAMPMODE  = 0x20;
constexpr uint8_t XACTUAL   = 0x21;
constexpr uint8_t VACTUAL   = 0x22;
constexpr uint8_t AACTUAL   = 0x23;
constexpr uint8_t VMAX      = 0x24;
constexpr uint8_t VSTART    = 0x25;
constexpr uint8_t VSTOP     = 0x26;
constexpr uint8_t VBREAK    = 0x27;
constexpr uint8_t AMAX      = 0x28;
constexpr uint8_t DMAX      = 0x29;
constexpr uint8_t ASTART    = 0x2A;
constexpr uint8_t SIGN_AACT = 0x2A;
constexpr uint8_t DFINAL    = 0x2B;
constexpr uint8_t DSTOP     = 0x2C;
constexpr uint8_t BOW1      = 0x2D;
constexpr uint8_t BOW2      = 0x2E;
constexpr uint8_t BOW3      = 0x2F;
constexpr uint8_t BOW4      = 0x30;
constexpr uint8_t CLK_FREQ  = 0x31;

constexpr uint8_t POS_COMP        = 0x32;
constexpr uint8_t VIRT_STOP_LEFT  = 0x33;
constexpr uint8_t VIRT_STOP_RIGHT = 0x34;
constexpr uint8_t X_HOME          = 0x35;
constexpr uint8_t X_LATCH_RD      = 0x36;
constexpr uint8_t REV_CNT_RD      = 0x36;
constexpr uint8_t X_RANGE_WR      = 0x36;
constexpr uint8_t X_TARGET        = 0x37;

constexpr uint8_t X_PIPE0 = 0x38;
constexpr uint8_t X_PIPE1 = 0x39;
constexpr uint8_t X_PIPE2 = 0x3A;
constexpr uint8_t X_PIPE3 = 0x3B;
constexpr uint8_t X_PIPE4 = 0x3C;
constexpr uint8_t X_PIPE5 = 0x3D;
constexpr uint8_t X_PIPE6 = 0x3E;
constexpr uint8_t X_PIPE7 = 0x3F;

constexpr uint8_t SH_REG0  = 0x40;
constexpr uint8_t SH_REG1  = 0x41;
constexpr uint8_t SH_REG2  = 0x42;
constexpr uint8_t SH_REG3  = 0x43;
constexpr uint8_t SH_REG4  = 0x44;
constexpr uint8_t SH_REG5  = 0x45;
constexpr uint8_t SH_REG6  = 0x46;
constexpr uint8_t SH_REG7  = 0x47;
constexpr uint8_t SH_REG8  = 0x48;
constexpr uint8_t SH_REG9  = 0x49;
constexpr uint8_t SH_REG10 = 0x4A;
constexpr uint8_t SH_REG11 = 0x4B;
constexpr uint8_t SH_REG12 = 0x4C;
constexpr uint8_t SH_REG13 = 0x4D;

constexpr uint8_t DFREEZE        = 0x4E;
constexpr uint8_t IFREEZE        = 0x4E;
constexpr uint8_t CLK_GATING_REG = 0x4F;
constexpr uint8_t RESET_REG      = 0x4F;

constexpr uint8_t ENC_POS             = 0x50;
constexpr uint8_t ENC_LATCH_RD        = 0x51;
constexpr uint8_t ENC_RESET_VAL_WR    = 0x51;
constexpr uint8_t ENC_POS_DEV_RD      = 0x52;
constexpr uint8_t CL_TR_TOLERANCE_WR  = 0x52;
constexpr uint8_t ENC_POS_DEV_TOL_WR  = 0x53;
constexpr uint8_t ENC_IN_RES_WR       = 0x54;
constexpr uint8_t ENC_CONST_RD        = 0x54;
constexpr uint8_t MANUAL_ENC_CONST0   = 0x54;
constexpr uint8_t ENC_OUT_RES         = 0x55;
constexpr uint8_t SER_CLK_IN_HIGH_WR  = 0x56;
constexpr uint8_t SER_CLK_IN_LOW_WR   = 0x56;
constexpr uint8_t SSI_IN_CLK_DELAY_WR = 0x57;
constexpr uint8_t SSI_IN_WTIME_WR     = 0x57;
constexpr uint8_t SER_PTIME_WR        = 0x58;

constexpr uint8_t CL_OFFSET         = 0x59;
constexpr uint8_t PID_P_WR          = 0x5A;
constexpr uint8_t CL_VMAX_CALC_P_WR = 0x5A;
constexpr uint8_t PID_VEL_RD        = 0x5A;
constexpr uint8_t PID_I_WR          = 0x5B;
constexpr uint8_t CL_VMAX_CALC_I_WR = 0x5B;
constexpr uint8_t PID_ISUM_RD       = 0x5B;
constexpr uint8_t PID_D_WR          = 0x5C;
constexpr uint8_t CL_DELTA_P_WR     = 0x5C;
constexpr uint8_t PID_I_CLIP_WR     = 0x5D;
constexpr uint8_t PID_D_CLKDIV_WR   = 0x5D;
constexpr uint8_t PID_E_RD          = 0x5D;
constexpr uint8_t PID_DV_CLIP_WR    = 0x5E;
constexpr uint8_t PID_TOLERANCE_WR  = 0x5F;
constexpr uint8_t CL_TOLERANCE_WR   = 0x5F;

constexpr uint8_t FS_VEL_WR            = 0x60;
constexpr uint8_t DC_VEL_WR            = 0x60;
constexpr uint8_t CL_VMIN_EMF_WR       = 0x60;
constexpr uint8_t DC_TIME_WR           = 0x61;
constexpr uint8_t DC_SG_WR             = 0x61;
constexpr uint8_t DC_BLKTIME_WR        = 0x61;
constexpr uint8_t CL_VADD_EMF          = 0x61;
constexpr uint8_t DC_LSPTM_WR          = 0x62;
constexpr uint8_t ENC_VEL_ZERO_WR      = 0x62;
constexpr uint8_t ENC_VMEAN_WAIT_WR    = 0x63;
constexpr uint8_t ENC_VMEAN_FILTER_WR  = 0x63;
constexpr uint8_t ENC_VMEAN_INT_WR     = 0x63;
constexpr uint8_t SER_ENC_VARIATION_WR = 0x63;
constexpr uint8_t CL_CYCLE_WR          = 0x63;
constexpr uint8_t SYNCHRO_SET          = 0x64;
constexpr uint8_t V_ENC_RD             = 0x65;
constexpr uint8_t V_ENC_MEAN_RD        = 0x66;
constexpr uint8_t VSTALL_LIMIT_WR      = 0x67;

constexpr uint8_t ADDR_TO_ENC       = 0x68;
constexpr uint8_t DATA_TO_ENC       = 0x69;
constexpr uint8_t ADDR_FROM_ENC     = 0x6A;
constexpr uint8_t DATA_FROM_ENC     = 0x6B;
constexpr uint8_t COVER_LOW_WR      = 0x6C;
constexpr uint8_t COVER_HIGH_WR     = 0x6D;
constexpr uint8_t COVER_DRV_LOW_RD  = 0x6E;
constexpr uint8_t COVER_DRV_HIGH_RD = 0x6F;

constexpr uint8_t MSLUT_0_WR         = 0x70;
constexpr uint8_t MSLUT_1_WR         = 0x71;
constexpr uint8_t MSLUT_2_WR         = 0x72;
constexpr uint8_t MSLUT_3_WR         = 0x73;
constexpr uint8_t MSLUT_4_WR         = 0x74;
constexpr uint8_t MSLUT_5_WR         = 0x75;
constexpr uint8_t MSLUT_6_WR         = 0x76;
constexpr uint8_t MSLUT_7_WR         = 0x77;
constexpr uint8_t MSLUTSEL_WR        = 0x78;
constexpr uint8_t MSCNT_RD           = 0x79;
constexpr uint8_t MSOFFSET_WR        = 0x79;
constexpr uint8_t CURRENTA_RD        = 0x7A;
constexpr uint8_t CURRENTB_RD        = 0x7A;
constexpr uint8_t CURRENTA_SPI_RD    = 0x7B;
constexpr uint8_t CURRENTB_SPI_RD    = 0x7B;
constexpr uint8_t TZEROWAIT_WR       = 0x7B;
constexpr uint8_t SCALE_PARAM_RD     = 0x7C;
constexpr uint8_t CIRCULAR_DEC_WR    = 0x7C;
constexpr uint8_t ENC_COMP_XOFFSET   = 0x7D;
constexpr uint8_t ENC_COMP_YOFFSET   = 0x7D;
constexpr uint8_t ENC_COMP_AMPL      = 0x7D;
constexpr uint8_t START_SIN_WR       = 0x7E;
constexpr uint8_t START_SIN90_120_WR = 0x7E;
constexpr uint8_t DAC_OFFSET_WR      = 0x7E;
constexpr uint8_t VERSION_NO_RD      = 0x7F;
} // namespace TMC4361A_Reg

class TMC4361A {
  public:
    enum RampMode { VELOCITY_MODE = 0x00, POSITIONING_MODE = (0x01 << 2) };

    enum RampType { HOLD_RAMP = 0x00, TRAPEZOIDAL_RAMP = 0x01, S_SHAPED_RAMP = 0x02 };

    enum StatusFlags {
        TARGET_REACHED_F   = 0,
        POS_COMP_REACHED_F = 1,
        VEL_REACHED_F      = 2,
        VEL_STATE_F_L      = 3,
        VEL_STATE_F_H      = 4,
        RAMP_STATE_F_L     = 5,
        RAMP_STATE_F_H     = 6,
        STOPL_ACTIVE_F     = 7,
        STOPR_ACTIVE_F     = 8,
        VSTOPL_ACTIVE_F    = 9,
        VSTOPR_ACTIVE_F    = 10,
        ACTIVE_STALL_F     = 11,
        HOME_ERROR_F       = 12,
        FS_ACTIVE_F        = 13,
        ENC_FAIL_F         = 14,
        N_ACTIVE_F         = 15,
        ENC_LATCH_F        = 16,
        CL_FIT_F           = 19,
        DRV_SG             = 24, // SG2 Status
        DRV_OT             = 25, // Overtemperature Shutdown
        DRV_OTPW           = 26, // Overtemperature Warning
        DRV_S2GA           = 27, // Coil A Short
        DRV_S2GB           = 28, // Coil B Short
        DRV_OLA            = 29, // Coil A Open Load
        DRV_OLB            = 30, // Coil B Open Load
        DRV_STST           = 31, // Standstill Indicator
    };

    enum Events {
        TARGET_REACHED    = 0,
        POS_COMP_REACHED  = 1,
        VEL_REACHED       = 2,
        VEL_STATE0        = 3,
        VEL_STATE1        = 4,
        VEL_STATE2        = 5,
        RAMP_STATE0       = 6,
        RAMP_STATE1       = 7,
        RAMP_STATE2       = 8,
        MAX_PHASE_TRAP    = 9,
        FROZEN            = 10, // NFREEZE set low.Reset Required.
        STOPL_ACTIVE      = 11,
        STOPR_ACTIVE      = 12,
        VSTOPL_ACTIVE     = 13,
        VSTOPR_ACTIVE     = 14,
        HOME_ERROR        = 15,
        XLATCH_DONE       = 16,
        FS_ACTIVE         = 17,
        ENC_FAIL          = 18,
        N_ACTIVE          = 19,
        ENC_DONE          = 20,
        SER_ENC_DATA_FAIL = 21,
        SER_DATA_DONE     = 23,
        SERIAL_ENC_SET    = 24,
        COVER_DONE        = 25,
        ENC_VEL0          = 26,
        CL_MAX            = 27,
        CL_FIT            = 28,
        STOP_ON_STALL     = 29,
        MOTOR_EV          = 30,
        RST_EV            = 31,
    };

    enum FreezeEvent {
        OK,                // no freeze
        USER,              // User called
        STALL,             // motor stall
        POS_ERR = 3,       // position error
        COIL_OL,           // coil open circuit
        COIL_SHORT,        // coil short circuit
        OVERTEMP,          // overtemp
        TMC4361A_COMM_ERR, // motion controller communication error
        TMC5160_COMM_ERR,  // motor driver communication error
    };

    struct DrvSettings {
        uint16_t mstep_per_fs      = 256;
        uint16_t fs_per_rev        = 0x0C8;
        uint8_t  mstatus_selection = 0xFB;
    };

    struct CurrentSettings {
        uint8_t i_hold       = 0;  // default 25%
        uint8_t i_run        = 16; // default 50%
        uint8_t i_hold_delay = 4;
    };

    struct StallSettings {
        bool     stop_on_stall    = false; // stop / warn only
        uint16_t stall_thresh_rpm = 100;   // min rpm for stallguard to become active
    };

    struct ClosedLoopSettings {
        bool     enable       = false;
        bool     enable_pid   = false;
        uint16_t cl_tolerance = 200;  // max position error
        uint16_t enc_in_res   = 1000; // encoder resolution
    };

    TMC4361A(Spi *spi, xQueueHandle *spi_mutex, uint8_t spi_cs_pin, uint32_t fclk, uint8_t nfreeze_pin,
             uint8_t nrst_pin, uint8_t intr_pin, uint8_t drv_en_pin);

    /* ================================= Motion Settings ================================ */

    /**
     * @brief Set the Ramp Mode object
     *
     * @param mode
     * @param type
     * @return write success
     */
    bool setRampMode(RampMode mode, RampType type);

    /**
     * @brief Set the Speeds object
     *
     * @param max_speed
     * @param start_speed
     * @param stop_speed
     * @param break_speed
     * @return write success
     *
     * @warning breakSpeed affects trapezoidal ramp only
     */
    bool setSpeeds(float max_speed, float start_speed = 0, float stop_speed = 0, float break_speed = 0);

    /**
     * @brief Set the Accelerations object
     *
     * @param max_accel
     * @param max_decel
     * @param start_accel
     * @param final_decel
     * @return write success
     */
    bool setAccelerations(float max_accel, float max_decel, float start_accel = 0, float final_decel = 0);

    /**
     * @brief Set the Bow Values object
     *
     * @param bow1
     * @param bow2
     * @param bow3
     * @param bow4
     * @return write success
     */
    bool setBowValues(float bow1, float bow2, float bow3, float bow4);

    /* ====================================== Query ===================================== */

    /**
     * @brief Checks 'target reached' flag
     *
     * @return write success
     */
    bool isTargetReached();

    /**
     * @brief Checks 'stallguard' flag
     *
     * @return write success
     */
    bool isStalled();

    /**
     * @brief Checks whether the motor is currently running.
     *
     * @return motor is running
     */
    bool isRunning();

    /**
     * @brief Checks 'frozen' flag
     *
     * @return is frozen flag raised
     */
    bool isFrozen();

    /**
     * @brief Return current status of stopl and stopr
     *
     * @return bit1 - stopl, bit2 - stopr
     */
    uint8_t getSensorReading();

    /**
     * @brief Checks 'cl_fit_f' flag
     *
     * @return has position error
     * @note the flag is inversed for some reason
     */
    bool hasPositionError();

    /**
     * @brief Checks the event that caused last emStop
     *
     * @return freeze event
     */
    FreezeEvent whyFrozen();

    /**
     * @brief Return the current internal position
     *
     * @return internal position in microsteps
     */
    int32_t getInternalPosition();

    /**
     * @brief Return the current encoder position (in pulse)
     *
     * @return encoder position in microsteps
     */
    int32_t getEncoderPosition();

    /**
     * @brief Return the current speed (in pulse / second)
     *
     * @return speed in microsteps per second
     */
    int32_t getCurrentSpeed();

    /**
     * @brief Get the Encoder Pos Deviation (in microsteps)
     *
     * @return int32_t
     */
    uint32_t getEncoderPosDev();

    /* ====================================== Setup ===================================== */
    /**
     * @brief Write all configurations
     *
     * @return write success
     */
    bool initialize();

    /**
     * @brief Software Reset
     *
     * @return write success
     */
    bool softReset();

    /**
     * @brief Hardware Reset
     */
    void hardReset();

    /* ---------------------------------------------------------------------------------- */
    /**
     * @brief Configure motor driver settings
     *
     * @param drv_s
     */
    void setDrv(DrvSettings drv_s);
    void setDrv_mstepPerFs(uint16_t mstep_per_fs);
    void setDrv_fsPerRev(uint16_t drv_s);

    /* ---------------------------------------------------------------------------------- */
    /**
     * @brief Configure motor current settings
     *
     * @param current_s
     */
    void setCurrent(CurrentSettings current_s);
    void setCurrent_iHold(uint16_t i_hold);
    void setCurrent_iRun(uint16_t i_run);

    /* ---------------------------------------------------------------------------------- */
    /**
     * @brief Configure Stop-on-stall
     *
     * @param stall_s
     */
    void setStopOnStall(StallSettings stall_s);
    void setStopOnStall_enable(uint16_t enable);
    void setStopOnStall_thresh(uint16_t rpm);

    /* ---------------------------------------------------------------------------------- */
    /**
     * @brief Configure Closed Loop
     *
     * @param cl_s
     */
    void setClosedLoop(ClosedLoopSettings cl_s);
    void setClosedLoop_enable(uint16_t enable);
    void setClosedLoop_usePID(uint16_t enable_pid);
    void setClosedLoop_encInRes(uint16_t enc_in_res);
    void setClosedLoop_tolerance(uint16_t cl_tolerance);

    /* ---------------------------------------------------------------------------------- */
    /**
     * @brief Configure stealthChop threshold
     *
     * @param rpm
     */
    void setStealthChopThreshold(uint16_t rpm);

    /* ====================================== Tasks ===================================== */

    /**
     * @brief Fetches status to raise events
     *
     * @param TMC4361A*
     */
    static void task_stsMonitor(void *parameters);

    /* ====================================== Maths ===================================== */

    /**
     * @brief Converts speed (pps) to rpm
     *
     * @param speed
     * @return float
     */
    float spdToRpm(float speed);

    /* ====================================== Test ====================================== */

    uint8_t test_Comm();

    /* ======================================= ISR ====================================== */
    /**
     * @brief Raise stall flag (closed loop)
     *
     */
    void raiseStallFlag();

  protected:
    DrvSettings        m_drv_s;                        // driver settings
    CurrentSettings    m_current_s;                    // current settings
    StallSettings      m_stall_s;                      // stall settings
    ClosedLoopSettings m_cl_s;                         // closed loop settings
    uint16_t           m_stealthchop_thresh_rpm = 180; // default threshold (rpm)

    xTaskHandle                   drv_monitor_handle       = NULL;
    xTaskHandle                   stall_monitor_handle     = NULL;
    static constexpr int          STATUS_UPDATE_FREQ_MS    = 5;
    static constexpr portTickType STATUS_UPDATE_FREQ_TICKS = STATUS_UPDATE_FREQ_MS / portTICK_RATE_MS;

    /**
     * @brief Jogs to target position with current motion configuration
     *
     * @param position number of microsteps
     * @return write success
     */
    bool setTargetPosition(uint32_t position);

    /**
     * @brief stops gracefully
     *
     * @warning direct calls must be made from internal functions only
     * @return write success
     */
    bool rampStop();

    /**
     * @brief raises nfreeze pin
     *
     * @param freeze_event
     * @warning direct calls must be made from internal functions only
     */
    void emStop(FreezeEvent freeze_event);

    /**
     * @brief Override position and encoder position
     *
     * @param pos
     * @return write success
     * @warning this function must only be called in IDLE state.
     * @note calling this when new position is > max pos error causes next pos err flag to be ignored
     */
    bool setPosition(int32_t pos);

    /**
     * @brief Set DRV_EN pin to release motor
     *
     */
    void releaseDriver();

    /**
     * @brief Unset DRV_EN pin to enable motor
     *
     */
    void enableDriver();

  private:
    struct IntrPinInfo {
        uint32_t  ul_pin;
        uint32_t  ul_flag;
        Pio      *p_pio;
        uint32_t  ul_mask;
        uint32_t  ul_attr;
        uint32_t  ul_id;
        IRQn_Type irqn_type;
        uint32_t  pin_index;
    };

    Spi          *m_spi;
    xQueueHandle *m_spi_mutex;
    uint8_t       m_spi_cs_pin;  // CS Pin
    uint32_t      m_fclk;        // External clock frequency (Hz)
    uint8_t       m_nfreeze_pin; // NFREEZE Pin
    uint8_t       m_nrst_pin;    // RESET Pin
    uint8_t       m_intr_pin;    // INTERRUPT Pin
    uint8_t       m_drv_en_pin;  // TMC5160 EN Pin

    bool    m_rtos_inited                = false;
    bool    m_is_interrupt_configured    = false;
    bool    m_drv_enabled                = false;
    bool    m_write_position_after_reset = false; // when raised, write saved position
    int32_t m_cached_position            = 0;     // stored position after freezing

    // Status
    bool          m_drv_no_comm        = false;
    bool          m_has_position_error = false;
    bool          m_is_stalled         = false;
    bool          m_is_running         = false;
    uint8_t       m_sensor_status      = 0;
    bool          m_target_reached     = false;
    bool          m_is_frozen          = false;
    FreezeEvent   m_freeze_event       = FreezeEvent::OK; // freeze event tracker
    bool          m_ignore_next_stall  = true;            // ignore first stall
    volatile bool m_stall_detected     = false;           // raised by stall detection ISR

    /* ====================================== Setup ===================================== */

    /**
     * @brief Write default registers
     *
     * @return write success
     */
    bool setFixed();

    /**
     * @brief Write intr registers
     *
     * @return write success
     */
    bool setInterrupt();

    /**
     * @brief Write current registers
     *
     * @return write success
     */
    bool setCurrent();

    /**
     * @brief cl settings register
     *
     * @return write success
     */
    bool setClosedLoop();

    /**
     * @brief Write stall detection registers
     *
     * @return write success
     */
    bool setStallDetection();

    /* ====================================== Misc ====================================== */

    /**
     * @brief Reset freeze
     *
     */
    void resetFreeze();

    /**
     * @brief Raise freeze
     *
     */
    void raiseFreeze();

    /**
     * @brief Toggles NRST pin
     *
     */
    void toggleNRST();

    /**
     * @brief Wait for COVER_DONE event to become high (or fail)
     *
     * @return cover done
     */
    bool waitForCoverDone();

    /**
     * @brief Verify contents of register with value
     *
     * @param reg_addr
     * @param write_value
     * @return value matches
     * @note Seems redundant. Not using this atm.
     */
    bool verifyWrite(uint8_t reg_addr, uint32_t write_value);

    /* ===================================== Status ===================================== */

    /**
     * @brief Read status
     *
     * @param response pointer to store value
     * @return read success
     */
    bool readStatus(uint32_t *response);

    /**
     * @brief Extract bits from status to determine if motor is running.
     *
     * @param sts_resp Status (0x0F)
     * @return bits 3:6 is non-zero
     */
    static bool extractStatus_running(uint32_t sts_resp);

    /**
     * @brief Read single bit of status based on flag
     *
     * @param flag
     * @param sts_resp Status (0x0F)
     * @return bit is set
     */
    static bool extractStatus_flag(StatusFlags flag, uint32_t sts_resp);

    /**
     * @brief Extract driver status flags from status
     *
     * @param sts_resp Status (0x0F)
     * @return bits 24:31
     */
    static uint8_t extractStatus_driverStatus(uint32_t sts_resp);

    /**
     * @brief Extract sensor status flags from status
     *
     * @param sts_resp Status (0x0F)
     * @return bits 7:8
     */
    static uint8_t extractStatus_sensor(uint32_t sts_resp);

    /* ====================================== Event ===================================== */

    /**
     * @brief Read events
     *
     * @param response pointer to store value
     * @return read success
     */
    bool readEvent(uint32_t *response);

    /**
     * @brief Read bit of event and clears it
     *
     * @param event
     * @return event is set
     */
    bool readAndClearEvent(Events event);

    /**
     * @brief Clears all events
     *
     */
    void clearEvents();

    /* ====================================== Maths ===================================== */
    /**
     * @brief Converts ΔΔRPM to ΔΔSpeed (pps3) [Jerk]
     *
     * @param deltaSq_rpm
     * @return uint32_t
     */
    uint32_t deltaSqRpmToDeltaSqSpd(float deltaSq_rpm);

    /**
     * @brief Converts ΔRPM to ΔSpeed (pps2) [Acceleration]
     *
     * @param delta_rpm
     * @return uint32_t
     */
    uint32_t deltaRpmToDeltaSpd(float delta_rpm);

    /**
     * @brief Converts RPM to speed (pps) [Speed]
     *
     * @param rpm
     * @return float
     */
    float rpmToSpd(float rpm);

    /**
     * @brief Converts float to uint32_t shifted by x decimal places
     *
     * @param value
     * @param decimalPlaces
     * @return uint32_t
     */
    static uint32_t floatToFixed(float value, uint8_t decimalPlaces);

    /* ================================== Communication ================================= */
    /**
     * @brief Read motor driver register
     *
     * @param drv_reg register of tmc5160
     * @param response pointer to store 32bit response
     * @param drv_sts pointer to store 8bit driver status
     * @return read success
     * @bug doesn't always work. Not used anywhere
     */
    bool readDrvRegister(uint8_t drv_reg, uint32_t *response, uint8_t *drv_sts = nullptr);

    /**
     * @brief Write to motor driver register
     *
     * @param drv_reg register of tmc5160
     * @param data write value
     * @return write success
     */
    bool writeDrvRegister(uint8_t drv_reg, uint32_t data);

    /**
     * @brief Read motion controller register
     *
     * @param reg_addr register of tmc4361a
     * @param response pointer to store response
     * @return read success
     */
    bool readRegister(uint8_t reg_addr, uint32_t *response);

    /**
     * @brief Write motion controller register
     *
     * @param reg_addr register of tmc4361a
     * @param data value to write
     * @return write success
     */
    bool writeRegister(uint8_t reg_addr, uint32_t data);

    /**
     * @brief Spi transfer protocol
     *
     * @param address resolved address
     * @param data value to write
     * @param response pointer to store response
     * @return is spi transfer successful
     */
    bool spiTransfer(uint8_t address, uint32_t data, uint32_t *response = nullptr);
};

#endif /* TMC4361A_H */
