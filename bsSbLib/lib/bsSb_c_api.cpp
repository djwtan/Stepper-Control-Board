// bsSb_c_api.cpp
#ifdef _WIN32
#    define BS_API __declspec(dllexport)
#else
#    define BS_API __attribute__((visibility("default")))
#endif

#include "bsSb.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// opaque handle type for C
typedef void *bsStepperBoardHandle;

// create object
bsStepperBoardHandle bsSb_create(uint8_t board_num) { return new bsStepperBoard(board_num); }

// destroy object
void bsSb_destroy(bsStepperBoardHandle h) { delete static_cast<bsStepperBoard *>(h); }

/* ================================================================================== */
/*                                 override functions                                 */
/* ================================================================================== */

typedef bool (*C_CANTransmit)(uint32_t, uint32_t, uint8_t);
typedef int (*C_CANReceive)(uint32_t *, int);
typedef void (*C_CANMutex)(bool);
typedef void (*C_AsyncSleep)(void);

void setCANTransmit(bsStepperBoardHandle h, C_CANTransmit func) {
    auto obj = static_cast<bsStepperBoard *>(h);

    // wrap C function pointer into std::function
    bsStepperBoard::CANTransmit cb = [func](uint32_t id, uint32_t data, uint8_t len) { return func(id, data, len); };

    obj->setCANTransmit(cb);
}

void setCANReceive(bsStepperBoardHandle h, C_CANReceive func) {
    auto obj = static_cast<bsStepperBoard *>(h);

    bsStepperBoard::CANReceive cb = [func](uint32_t *buf, int size) { return func(buf, size); };

    obj->setCANReceive(cb);
}

void setCANMutex(bsStepperBoardHandle h, C_CANMutex func) {
    auto obj = static_cast<bsStepperBoard *>(h);

    bsStepperBoard::CANMutex cb = [func](bool locked) { func(locked); };

    obj->setCANMutex(cb);
}

void setAsyncSleep(bsStepperBoardHandle h, C_AsyncSleep func) {
    auto obj = static_cast<bsStepperBoard *>(h);

    bsStepperBoard::AsyncSleep cb = [func]() { func(); };

    obj->setAsyncSleep_1ms(cb);
}

/* ================================================================================== */
/*                                     Board Info                                     */
/* ================================================================================== */

bool readFirmwareVersion(bsStepperBoardHandle h, const char **ver) {
    auto obj = static_cast<bsStepperBoard *>(h);

    std::string *cpp_ver = nullptr;
    bool         success = obj->readFirmwareVersion(&cpp_ver);

    if (success && cpp_ver) {
        *ver = strdup(cpp_ver->c_str());
    } else {
        *ver = nullptr;
    }

    return success; // must return bool
}

/* ================================================================================== */
/*                                       SD Card                                      */
/* ================================================================================== */

bool readDriverConfig(bsStepperBoardHandle h, uint8_t driver_num, const char **file_content) {
    auto obj = static_cast<bsStepperBoard *>(h);

    std::string *cpp_file_content = nullptr;
    bool         success          = obj->readDriverConfig(driver_num, &cpp_file_content);

    if (success && cpp_file_content) {
        *file_content = strdup(cpp_file_content->c_str());
    } else {
        *file_content = nullptr;
    }

    return success;
}

bool readMotionConfig(bsStepperBoardHandle h, uint8_t motion_index, const char **file_content) {
    auto obj = static_cast<bsStepperBoard *>(h);

    std::string *cpp_file_content = nullptr;
    bool         success          = obj->readMotionConfig(motion_index, &cpp_file_content);

    if (success && cpp_file_content) {
        *file_content = strdup(cpp_file_content->c_str());
    } else {
        *file_content = nullptr;
    }

    return success;
}

bool readSequenceConfig(bsStepperBoardHandle h, uint8_t sequence_index, const char **file_content) {
    auto obj = static_cast<bsStepperBoard *>(h);

    std::string *cpp_file_content = nullptr;
    bool         success          = obj->readSequenceConfig(sequence_index, &cpp_file_content);

    if (success && cpp_file_content) {
        *file_content = strdup(cpp_file_content->c_str());
    } else {
        *file_content = nullptr;
    }

    return success;
}

bool readBuffer(bsStepperBoardHandle h, const char **buf_content) {
    auto obj = static_cast<bsStepperBoard *>(h);

    std::string *cpp_buf_content = nullptr;
    bool         success         = obj->readBuffer(&cpp_buf_content);

    if (success && cpp_buf_content) {
        *buf_content = strdup(cpp_buf_content->c_str());
    } else {
        *buf_content = nullptr;
    }

    return success;
}

bool readPath(bsStepperBoardHandle h, const char **path) {
    auto obj = static_cast<bsStepperBoard *>(h);

    std::string *cpp_path = nullptr;
    bool         success  = obj->readPath(&cpp_path);

    if (success && cpp_path) {
        *path = strdup(cpp_path->c_str());
    } else {
        *path = nullptr;
    }

    return success;
}

bool readFileAtPath(bsStepperBoardHandle h, const char *path, const char **file_content) {
    auto obj = static_cast<bsStepperBoard *>(h);

    std::string *cpp_file_content = nullptr;
    std::string  path_str(path);

    bool success = obj->readFileAtPath(&path_str, &cpp_file_content);

    if (success && cpp_file_content) {
        *file_content = strdup(cpp_file_content->c_str());
    } else {
        *file_content = nullptr;
    }

    return success;
}

bool deleteFileAtPath(bsStepperBoardHandle h, const char *path) {
    auto obj = static_cast<bsStepperBoard *>(h);

    std::string path_str(path);

    bool success = obj->deleteFileAtPath(&path_str);

    return success;
}

bool writeToFileAtPath(bsStepperBoardHandle h, const char *path, const char *file_content) {
    auto obj = static_cast<bsStepperBoard *>(h);

    std::string file_content_str(file_content);
    std::string path_str(path);

    bool success = obj->writeToFileAtPath(&path_str, &file_content_str);

    return success;
}

/* ================================================================================== */
/*                                  Readback (Driver)                                 */
/* ================================================================================== */

void clearQueuedReadback(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    obj->clearQueuedReadback(driver_num);
}

bool getQueueReadback(bsStepperBoardHandle h, uint8_t driver_num, uint16_t *response) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->getQueueReadback(driver_num, response);
}

/* ================================================================================== */
/*                                    Move Commands                                   */
/* ================================================================================== */

bool q_movePreloaded(bsStepperBoardHandle h, uint8_t index, uint8_t *driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->q_movePreloaded(index, driver_num);
}

bool q_moveStepper(bsStepperBoardHandle h, uint8_t driver_num, int32_t position_units) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->q_moveStepper(driver_num, position_units);
}

bool q_moveStepper_homing(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->q_moveStepper_homing(driver_num);
}

bool q_moveStepper_inverseTime(bsStepperBoardHandle h, uint8_t driver_num, int32_t position_units, uint32_t time_ms) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->q_moveStepper_inverseTime(driver_num, position_units, time_ms);
}

bool q_moveStepper_vibration(bsStepperBoardHandle h, uint8_t driver_num, int32_t position_units, uint16_t iterations,
                             float diminishing_factor, bool loop) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->q_moveStepper_vibration(driver_num, position_units, iterations, diminishing_factor, loop);
}

/* ================================================================================== */
/*                                    Control Word                                    */
/* ================================================================================== */

bool rampStop(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->rampStop(driver_num);
}

bool emergencyStop(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->emergencyStop(driver_num);
}

bool resetPosition(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->resetPosition(driver_num);
}

bool enableDriver(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->enableDriver(driver_num);
}

bool releaseDriver(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->releaseDriver(driver_num);
}

/* ================================================================================== */
/*                                    Stepper Read                                    */
/* ================================================================================== */

uint8_t readDriverStatus(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return static_cast<uint8_t>(obj->readDriverStatus(driver_num));
}

int32_t readCurrentPosition(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->readCurrentPosition(driver_num);
}

int32_t readEncoderPosition(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->readEncoderPosition(driver_num);
}

int32_t readCurrentSpeed(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->readCurrentSpeed(driver_num);
}

uint8_t readSensors(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->readSensors(driver_num);
}

int32_t readEncoderPositionDeviation(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->readEncoderPositionDeviation(driver_num);
}

/* ================================================================================== */
/*                                   Motion Planner                                   */
/* ================================================================================== */

QueueStatus getMotionQueueStatus(bsStepperBoardHandle h) {
    auto obj = static_cast<bsStepperBoard *>(h);

    QueueStatus qsts = obj->getMotionQueueStatus();

    return qsts;
}

bool mq_startMotion(bsStepperBoardHandle h, uint8_t motion_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->mq_startMotion(motion_num);
}

bool abortMotion(bsStepperBoardHandle h) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->abortMotion();
}

void clearMotionQueueReadback(bsStepperBoardHandle h) {
    auto obj = static_cast<bsStepperBoard *>(h);

    obj->clearMotionQueueReadback();
}

bool getMotionQueueReadback(bsStepperBoardHandle h, uint16_t *res) {
    auto obj = static_cast<bsStepperBoard *>(h);

    MotionQueueRes mq_res;

    bool success = obj->getMotionQueueReadback(&mq_res);

    if (success) { *res = static_cast<uint16_t>(mq_res); }

    return success;
}

typedef struct {
    uint16_t motion_type;
    uint16_t driver_num;
    int32_t  position;
    int32_t  offset;
    uint16_t homing_mode;
    uint16_t homing_sensor;
    uint16_t sensor_home_value;
    uint16_t ramp_type;
    float    max_speed;
    float    max_accel;
    float    max_decel;
    float    bow1;
    float    bow2;
    float    bow3;
    float    bow4;
    uint16_t use_inverse_time;
    uint32_t time_ms;

} QueueItemParameters;

bool writeQueueItem(bsStepperBoardHandle h, uint16_t motion_num, uint16_t item_num, uint16_t sequence_number,
                    const QueueItemParameters *params) {
    auto obj = static_cast<bsStepperBoard *>(h);

    if (!params) return false;

    bsStepperBoard::QueueItemParameters p;

    // Copy every field (1-to-1)
    p.motion_type       = params->motion_type;
    p.driver_num        = params->driver_num;
    p.position          = params->position;
    p.offset            = params->offset;
    p.homing_mode       = params->homing_mode;
    p.homing_sensor     = params->homing_sensor;
    p.sensor_home_value = params->sensor_home_value;
    p.ramp_type         = params->ramp_type;
    p.max_speed         = params->max_speed;
    p.max_accel         = params->max_accel;
    p.max_decel         = params->max_decel;
    p.bow1              = params->bow1;
    p.bow2              = params->bow2;
    p.bow3              = params->bow3;
    p.bow4              = params->bow4;
    p.use_inverse_time  = params->use_inverse_time;
    p.time_ms           = params->time_ms;

    return obj->writeQueueItem(motion_num, item_num, sequence_number, p);
}

bool mq_startTestQueueItem(bsStepperBoardHandle h) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->mq_startTestQueueItem();
}

bool writeTestQueueItem(bsStepperBoardHandle h, const QueueItemParameters *params) {
    auto obj = static_cast<bsStepperBoard *>(h);

    if (!params) return false;

    bsStepperBoard::QueueItemParameters p;

    // Copy every field (1-to-1)
    p.motion_type       = params->motion_type;
    p.driver_num        = params->driver_num;
    p.position          = params->position;
    p.offset            = params->offset;
    p.homing_mode       = params->homing_mode;
    p.homing_sensor     = params->homing_sensor;
    p.sensor_home_value = params->sensor_home_value;
    p.ramp_type         = params->ramp_type;
    p.max_speed         = params->max_speed;
    p.max_accel         = params->max_accel;
    p.max_decel         = params->max_decel;
    p.bow1              = params->bow1;
    p.bow2              = params->bow2;
    p.bow3              = params->bow3;
    p.bow4              = params->bow4;
    p.use_inverse_time  = params->use_inverse_time;
    p.time_ms           = params->time_ms;

    return obj->writeTestQueueItem(p);
}

/* ================================================================================== */
/*                                       Driver                                       */
/* ================================================================================== */

typedef struct {
    uint16_t mstep_per_fs;
    uint16_t fs_per_rev;
    uint16_t unit_per_rev;
    uint16_t holding_current;
    uint16_t peak_rms_current;
    bool     stop_on_stall_enable;
    uint16_t stop_on_stall_thresh;
    bool     cl_enable;
    bool     cl_enable_pid;
    uint16_t cl_enc_in_res;
    uint16_t cl_tolerance;
    uint16_t stealth_chop_thresh;
} DriverConfig;

bool q_setAndInitDriver(bsStepperBoardHandle h, uint8_t driver_num, const DriverConfig *params) {
    auto obj = static_cast<bsStepperBoard *>(h);

    if (!params) return false;

    bsStepperBoard::DriverConfig p;

    // Copy every field (1-to-1)
    p.mstep_per_fs         = params->mstep_per_fs;
    p.fs_per_rev           = params->fs_per_rev;
    p.unit_per_rev         = params->unit_per_rev;
    p.holding_current      = params->holding_current;
    p.peak_rms_current     = params->peak_rms_current;
    p.stop_on_stall_enable = params->stop_on_stall_enable;
    p.stop_on_stall_thresh = params->stop_on_stall_thresh;
    p.cl_enable            = params->cl_enable;
    p.cl_enable_pid        = params->cl_enable_pid;
    p.cl_enc_in_res        = params->cl_enc_in_res;
    p.cl_enc_in_res        = params->cl_enc_in_res;
    p.cl_tolerance         = params->cl_tolerance;
    p.stealth_chop_thresh  = params->stealth_chop_thresh;

    return obj->q_setAndInitDriver(driver_num, p);
}

bool q_initDriver(bsStepperBoardHandle h, uint8_t driver_num) {
    auto obj = static_cast<bsStepperBoard *>(h);

    return obj->q_initDriver(driver_num);
}

/* ================================================================================== */
/*                                       Motion                                       */
/* ================================================================================== */

typedef struct {
    uint16_t allow_write_motion_when_busy;
    uint16_t reset_motion_conf_after_each_move;
    uint16_t pos_mode;
    uint16_t follow_mode;
    uint16_t ramp_mode;
    uint16_t ramp_type;
    float    max_speed;
    float    start_speed;
    float    stop_speed;
    float    break_speed;
    float    max_accel;
    float    max_decel;
    float    start_accel;
    float    final_decel;
    float    bow1;
    float    bow2;
    float    bow3;
    float    bow4;
} MotionConfig;

bool q_setMotion(bsStepperBoardHandle h, uint8_t driver_num, const MotionConfig *params) {
    auto obj = static_cast<bsStepperBoard *>(h);

    bsStepperBoard::MotionConfig p;

    // Copy every field (1-to-1)
    p.allow_write_motion_when_busy      = params->allow_write_motion_when_busy;
    p.reset_motion_conf_after_each_move = params->reset_motion_conf_after_each_move;
    p.pos_mode                          = params->pos_mode;
    p.follow_mode                       = params->follow_mode;
    p.ramp_mode                         = params->ramp_mode;
    p.ramp_type                         = params->ramp_type;
    p.max_speed                         = params->max_speed;
    p.start_speed                       = params->start_speed;
    p.stop_speed                        = params->stop_speed;
    p.break_speed                       = params->break_speed;
    p.max_accel                         = params->max_accel;
    p.max_decel                         = params->max_decel;
    p.start_accel                       = params->start_accel;
    p.final_decel                       = params->final_decel;
    p.bow1                              = params->bow1;
    p.bow2                              = params->bow2;
    p.bow3                              = params->bow3;
    p.bow4                              = params->bow4;

    return obj->q_setMotion(driver_num, p);
}

/* ================================================================================== */
/*                                       Homing                                       */
/* ================================================================================== */

typedef struct {
    uint16_t homing_mode;
    uint16_t homing_sensor;
    uint16_t home_sensor_value;
    int32_t  homing_offset_units;
    int32_t  homing_distance_units;
    float    max_speed;
    float    max_accel;
    float    max_decel;
    uint32_t homing_timeout_ms;
} HomingConfig;

bool configureHoming(bsStepperBoardHandle h, uint8_t driver_num, const HomingConfig *params) {
    auto obj = static_cast<bsStepperBoard *>(h);

    bsStepperBoard::HomingConfig p;

    // Copy every field (1-to-1)
    p.homing_mode           = params->homing_mode;
    p.homing_sensor         = params->homing_sensor;
    p.home_sensor_value     = params->home_sensor_value;
    p.homing_offset_units   = params->homing_offset_units;
    p.homing_distance_units = params->homing_distance_units;
    p.max_speed             = params->max_speed;
    p.max_accel             = params->max_accel;
    p.max_decel             = params->max_decel;
    p.homing_timeout_ms     = params->homing_timeout_ms;

    return obj->configureHoming(driver_num, p);
}

#ifdef __cplusplus
}
#endif
