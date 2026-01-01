/*
 * queue.h
 *
 * Created: 29/10/2025 12:06:07
 *  Author: Tan
 */

#ifndef QUEUE_H_
#define QUEUE_H_

#include "stepper.h"
#include "utils.h"

#define MAX_MOTIONS              0x0F         // Maximum number of motion sequences
#define TEST_MOTION_NUM          0xFF         // Motion number used to invoke test queue item
#define MAX_ITEMS_PER_MOTION     32           // Maximum items per motion sequence
#define MAX_CONCURRENT_SEQUENCES MAX_STEPPERS // Maximum concurrent actions

#ifdef MOTION_QUEUE_DEBUG
#    include <stdio.h>
#    define motion_queue_debug(...) printf(__VA_ARGS__)
#else
#    define motion_queue_debug(...)
#endif

class MotionQueue {
  public:
    /* Put Queue */
    enum PutQueueRes {
        Q_SUCCESS = 1,
        Q_FULL,
        Q_RESULTS_PENDING,
    };

    enum MotionType {
        MOVE_RELATIVE,
        MOVE_ABSOLUTE,
        MOVE_HOMING,
        WAIT_TIME,
        WAIT_SENSOR,
    };

    enum QueueItemKey {
        MOTION_TYPE,
        SEQUENCE_NUMBER,
        DRIVER_NUMBER = 2,
        POSITION_H,
        POSITION_L,
        OFFSET_H,
        OFFSET_L,
        HOMING_MODE,
        HOMING_SENSOR,
        SENSOR_HOME_VALUE,
        RAMP_TYPE,
        MAX_SPEED_H,
        MAX_SPEED_L,
        MAX_ACCEL_H,
        MAX_ACCEL_L,
        MAX_DECEL_H,
        MAX_DECEL_L,
        BOW1_H,
        BOW1_L,
        BOW2_H,
        BOW2_L,
        BOW3_H,
        BOW3_L,
        BOW4_H,
        BOW4_L,
        USE_INVERSE_TIME = 25,
        TIME_H,
        TIME_L
    };

    struct MotionParameters {
        // For All
        uint16_t driver_num = 0xFF; // use for extra check
        uint16_t position_h = 0;
        uint16_t position_l = 0;
        /* ---------------------------------------------------------------------------------- */
        // Move Homing
        uint16_t offset_h    = 0;
        uint16_t offset_l    = 0;
        uint16_t homing_mode = 0xFF; // use for extra check
        /* ---------------------------------------------------------------------------------- */
        uint16_t homing_sensor     = 0; // Home sensor / Wait Sensor
        uint16_t sensor_home_value = 0; // Home / Wait sensor condition
        /* ---------------------------------------------------------------------------------- */
        // Move
        uint16_t ramp_type   = 0;
        uint16_t max_speed_h = 0;
        uint16_t max_speed_l = 0;
        uint16_t max_accel_h = 0;
        uint16_t max_accel_l = 0;
        uint16_t max_decel_h = 0;
        uint16_t max_decel_l = 0;
        uint16_t bow1_h      = 0;
        uint16_t bow1_l      = 0;
        uint16_t bow2_h      = 0;
        uint16_t bow2_l      = 0;
        uint16_t bow3_h      = 0;
        uint16_t bow3_l      = 0;
        uint16_t bow4_h      = 0;
        uint16_t bow4_l      = 0;
        /* ---------------------------------------------------------------------------------- */
        // Move Inverse Time
        uint16_t use_inverse_time = 0;
        /* ---------------------------------------------------------------------------------- */
        // Homing timeout / Inverse Time Time / Wait Time / Wait Sensor Timeout
        uint16_t time_h = 0;
        uint16_t time_l = 0;
        /* ---------------------------------------------------------------------------------- */
    };

    enum Result {
        SUCCESS = 1,
        CONDITIONAL_SUCCESS, // If homing is involved
        FAILED_MIDWAY,
        INVALID_MOTION,
        EMPTY_QUEUE,
        EXCEED_MAX_CONCURRENT,
        REPEATED_ASSIGNMENT,
        BAD_PARAMETERS,
        STEPPERS_NOT_READY,
    };

    struct QueueItem {
        MotionType       motion_type;
        MotionParameters parameters;
        uint8_t          sequence_number = 0xFF; // use for extra check
    };

    struct Queue {
        QueueItem items[MAX_ITEMS_PER_MOTION]; // list of queue items
    };

    Queue     motion_list[MAX_MOTIONS];                // Stored motions sequences
    QueueItem test_item;                               // Holds test item to be exported into SD card
    QueueItem preloaded_motions[MAX_ITEMS_PER_MOTION]; // Preloaded motions based on map

    /* ---------------------------------------------------------------------------------- */
    MotionQueue(); // constructor

    /**
     * @brief Places motion number into motion queue task to be executed
     *
     * @param motion_num
     * @return PutQueueRes
     */
    PutQueueRes queueMotion(uint8_t motion_num);

    enum PreloadPutQueueRes {
        P_SUCCESS = 1,     // same as PutQueueRes
        P_FULL,            // same as PutQueueRes
        P_RESULTS_PENDING, // Same as PutQueueRes
        P_INVALID_MOTION,
        P_STEPPER_NOT_READY
    };
    /**
     * @brief Places preloaded motions into the specific driver's task queue
     *
     * @param motion_num
     * @return PutQueueRes
     */
    PreloadPutQueueRes queuePreloadedMotions(uint8_t motion_num, uint8_t *driver_num);

    /**
     * @brief Assign stepper object pointer
     *
     * @param stepper
     * @param index
     */
    void registerSteppers(Stepper *steppers[MAX_STEPPERS]);

    /**
     * @brief Get the Readback object
     *
     * @param readback pointer to store return value
     * @return readback value received
     */
    bool getReadback(uint32_t *readback);

    /**
     * @brief Get the Queue Status object
     *
     * @note Bits 15    - is_executing
     * @note Bits 11-14 - motion_num
     * @note Bits 0-10  - item_num
     *
     * @return uint16_t
     */
    uint16_t getQueueStatus();

    /**
     * @brief Returns whether stepper is held by queue system
     *
     * @param driver_num
     * @return yes or no
     */
    bool isStepperUsedInQueue(uint8_t driver_num);

    /**
     * @brief Abort current motion queue
     *
     */
    bool abortQueue();

    /**
     * @brief Directly modify motion parameter in motion list
     *
     * @param q_key
     * @param motion_num
     * @param item_num
     * @param val
     */
    void writeMotionItem(QueueItemKey q_key, uint8_t motion_num, uint8_t item_num, uint16_t val);

  private:
    Stepper *m_steppers[MAX_STEPPERS];
    bool     stepper_used_in_queue[MAX_STEPPERS];

    // Commands
    bool m_abort_queue = false;

    // Trackers
    struct QueueStatus {
        uint16_t is_executing = 0;
        uint16_t motion_num   = 0;
        uint16_t sequence_num = 0;
    };

    QueueStatus m_queue_status;

    /**
     * @brief Execute motion queue
     *
     * @param motion_num
     * @return Result
     */
    Result executeMotionQueue(uint8_t *motion_num_ptr);

    /**
     * @brief Execute queue item. Called by executeMotionQueue.
     *
     * @param items
     * @return true
     * @return false
     */
    Result executeItemInMotionQueue(QueueItem items[], const uint8_t num_items);

    /**
     * @brief Check if stepper is ready to receive a command. Called by executeItemInMotionQueue.
     *
     * @param stepper
     * @return true
     * @return false
     */
    bool stepperIsReadyForMovement(uint8_t driver_num);

    /**
     * @brief Flag stepper as USED in current motion queue
     *
     * @param driver_num
     */
    void flagStepperUsedInQueue(uint8_t driver_num);

    /**
     * @brief Flag all steppers as NOT USED in motion queue
     *
     */
    void unflagAllSteppers();

    /**
     * @brief Set the Motion Parameters based on motion type
     *
     * @param motion_type
     * @param parameters
     * @return true
     * @return false
     */
    bool setMotionParameters(MotionType motion_type, MotionParameters parameters);

    /**
     * @brief Starts all motions in the same sequence
     *
     * @param motion_type
     * @param parameters
     * @return true
     * @return false
     */
    bool startMotion(MotionType motion_type, MotionParameters parameters);

    enum MotionStatus { M_RUNNING = 1, M_ERR, COMPLETE };
    /**
     * @brief Fetches individual driver status
     *
     * @param driver_num
     * @return MotionStatus
     */
    MotionStatus getMotionStatus(MotionType motion_type, MotionParameters parameters, portTickType start_tick);

    /**
     * @brief Execution results
     *
     * @param motion_type
     * @param parameters
     * @return Result
     */
    Result execSuccessful(MotionType motion_type, MotionParameters parameters);

    /* ================================ Motion Queue Task =============================== */
    xQueueHandle m_motion_queue_handle = xQueueCreate(1, sizeof(uint8_t *));
    xQueueHandle m_result_queue_handle = xQueueCreate(1, sizeof(Result));

    /**
     * @brief motion queue executor RTOS task
     *
     */
    static void task_motionQueue(void *parameters);
};

#endif /* QUEUE_H_ */