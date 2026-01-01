/*
 * queue.cpp
 *
 * Created: 29/10/2025 12:06:19
 *  Author: Tan
 */

#include "motion_queue.h"

MotionQueue::MotionQueue() {
    // Motion Queue
    xTaskCreate(MotionQueue::task_motionQueue,      // function name
                (const signed char *)"motionQueue", // task name
                496,                                // stack size (117)
                this,                               // stack parameters
                2,                                  // stack priority
                NULL                                // stack handle
    );
}

/* ---------------------------------------------------------------------------------- */
MotionQueue::PutQueueRes MotionQueue::queueMotion(uint8_t motion_num) {
    if (uxQueueMessagesWaiting(m_result_queue_handle) != 0) {
        motion_queue_debug("Put queue failed! Results pending\n");
        return PutQueueRes::Q_RESULTS_PENDING;
    }

    // Allocate motion number on heap
    uint8_t *motion_num_ptr = new uint8_t(motion_num);

    // Try to queue it
    if (xQueueSend(m_motion_queue_handle, &motion_num_ptr, 0) == pdTRUE) {
        motion_queue_debug("Motion %d queued\n", motion_num);
        return PutQueueRes::Q_SUCCESS;
    } else {
        // Queue full, free heap memory
        motion_queue_debug("Put queue failed! Queue is full\n", motion_num);
        delete motion_num_ptr;
        return PutQueueRes::Q_FULL;
    }
}

/* ---------------------------------------------------------------------------------- */
MotionQueue::PreloadPutQueueRes MotionQueue::queuePreloadedMotions(uint8_t motion_num, uint8_t *driver_num) {
    if (motion_num >= MAX_MOTIONS) {
        motion_queue_debug("Invalid motion num %d\n", motion_num);
        return PreloadPutQueueRes::P_INVALID_MOTION;
    }

    QueueItem &item = preloaded_motions[motion_num];

    if (item.motion_type == MotionType::WAIT_SENSOR || item.motion_type == MotionType::WAIT_TIME) {
        motion_queue_debug("Preloaded motion %d cannot be WAIT type\n", motion_num);
        return PreloadPutQueueRes::P_INVALID_MOTION;
    }

    if (item.parameters.driver_num >= MAX_STEPPERS) {
        motion_queue_debug("Preloaded motion %d has invalid driver num %d\n", motion_num, item.parameters.driver_num);
        return PreloadPutQueueRes::P_INVALID_MOTION;
    }

    Stepper &stepper = *(m_steppers[item.parameters.driver_num]); // Stepper, dereferenced
    *driver_num      = item.parameters.driver_num;                // Assign driver_num

    Stepper::Status sts = stepper.getStatus();

    if (sts != Stepper::Status::READY && sts != Stepper::Status::MOTION_NOT_INIT) {
        motion_queue_debug("Stepper %d not ready for preloaded motion %d\n", item.parameters.driver_num, motion_num);
        return PreloadPutQueueRes::P_STEPPER_NOT_READY;
    }

    if (isStepperUsedInQueue(item.parameters.driver_num)) {
        motion_queue_debug("Stepper %d is held by motion queue\n", item.parameters.driver_num);
        return PreloadPutQueueRes::P_STEPPER_NOT_READY;
    }

    setMotionParameters(item.motion_type, item.parameters);

    Stepper::PutQueueRes res;
    if (item.motion_type == MotionType::MOVE_HOMING) {
        res = stepper.moveHoming();

    } else if (item.parameters.use_inverse_time == 1) {
        res = stepper.moveInverseTime();
    } else {

        res = stepper.move();
    }

    motion_queue_debug("Motion %d with driver num %d returned %d\n", motion_num, item.parameters.driver_num, res);

    return static_cast<PreloadPutQueueRes>(res);
}

/* ---------------------------------------------------------------------------------- */
void MotionQueue::registerSteppers(Stepper *steppers[MAX_STEPPERS]) {
    for (int i = 0; i < MAX_STEPPERS; i++) {
        m_steppers[i] = steppers[i]; // copy each pointer
    }
}

/* ---------------------------------------------------------------------------------- */
bool MotionQueue::getReadback(uint32_t *readback) {
    Result res;
    bool   has_readback = xQueueReceive(m_result_queue_handle, &res, 0) == pdTRUE;

    if (has_readback) {
        motion_queue_debug("Readback -> %d\n", res);
        *readback = static_cast<uint32_t>(res);
    }

    return has_readback;
}

/* ---------------------------------------------------------------------------------- */
uint16_t MotionQueue::getQueueStatus() {
    uint16_t word = 0;

    word |= ((m_queue_status.is_executing & 0x1) << 15);
    word |= ((m_queue_status.motion_num & 0xF) << 11);
    word |= ((m_queue_status.sequence_num & 0x7FF) << 0);

    return word;
}

/* ---------------------------------------------------------------------------------- */
bool MotionQueue::isStepperUsedInQueue(uint8_t driver_num) {
    if (driver_num >= MAX_STEPPERS) { return false; }
    return stepper_used_in_queue[driver_num];
}

/* ---------------------------------------------------------------------------------- */
bool MotionQueue::abortQueue() {
    if (m_queue_status.is_executing == 1) {
        m_abort_queue = true;
        return true;
    }
    return false;
}

/* ---------------------------------------------------------------------------------- */
void MotionQueue::writeMotionItem(QueueItemKey q_key, uint8_t motion_num, uint8_t item_num, uint16_t val) {
    QueueItem &q_item = (motion_num == TEST_MOTION_NUM) ? test_item : motion_list[motion_num].items[item_num];

    motion_queue_debug("(M%d%d) Write Key %d -> %d\n", motion_num, item_num, q_key, val);
    switch (q_key) {
    case QueueItemKey::MOTION_TYPE: {
        q_item.motion_type = static_cast<MotionType>(val);
        break;
    }
    case QueueItemKey::SEQUENCE_NUMBER: {
        q_item.sequence_number = val;
        break;
    }
    case QueueItemKey::DRIVER_NUMBER: {
        q_item.parameters.driver_num = val;
        break;
    }
    case QueueItemKey::POSITION_H: {
        q_item.parameters.position_h = val;
        break;
    }
    case QueueItemKey::POSITION_L: {
        q_item.parameters.position_l = val;
        break;
    }
    case QueueItemKey::OFFSET_H: {
        q_item.parameters.offset_h = val;
        break;
    }
    case QueueItemKey::OFFSET_L: {
        q_item.parameters.offset_l = val;
        break;
    }
    case QueueItemKey::HOMING_MODE: {
        q_item.parameters.homing_mode = val;
        break;
    }
    case QueueItemKey::HOMING_SENSOR: {
        q_item.parameters.homing_sensor = val;
        break;
    }
    case QueueItemKey::SENSOR_HOME_VALUE: {
        q_item.parameters.sensor_home_value = val;
        break;
    }
    case QueueItemKey::RAMP_TYPE: {
        q_item.parameters.ramp_type = val;
        break;
    }
    case QueueItemKey::MAX_SPEED_H: {
        q_item.parameters.max_speed_h = val;
        break;
    }
    case QueueItemKey::MAX_SPEED_L: {
        q_item.parameters.max_speed_l = val;
        break;
    }
    case QueueItemKey::MAX_ACCEL_H: {
        q_item.parameters.max_accel_h = val;
        break;
    }
    case QueueItemKey::MAX_ACCEL_L: {
        q_item.parameters.max_accel_l = val;
        break;
    }
    case QueueItemKey::MAX_DECEL_H: {
        q_item.parameters.max_decel_h = val;
        break;
    }
    case QueueItemKey::MAX_DECEL_L: {
        q_item.parameters.max_decel_l = val;
        break;
    }
    case QueueItemKey::BOW1_H: {
        q_item.parameters.bow1_h = val;
        break;
    }
    case QueueItemKey::BOW1_L: {
        q_item.parameters.bow1_l = val;
        break;
    }
    case QueueItemKey::BOW2_H: {
        q_item.parameters.bow2_h = val;
        break;
    }
    case QueueItemKey::BOW2_L: {
        q_item.parameters.bow2_l = val;
        break;
    }
    case QueueItemKey::BOW3_H: {
        q_item.parameters.bow3_h = val;
        break;
    }
    case QueueItemKey::BOW3_L: {
        q_item.parameters.bow3_l = val;
        break;
    }
    case QueueItemKey::BOW4_H: {
        q_item.parameters.bow4_h = val;
        break;
    }
    case QueueItemKey::BOW4_L: {
        q_item.parameters.bow4_l = val;
        break;
    }
    case QueueItemKey::USE_INVERSE_TIME: {
        q_item.parameters.use_inverse_time = val;
        break;
    }
    case QueueItemKey::TIME_H: {
        q_item.parameters.time_h = val;
        break;
    }
    case QueueItemKey::TIME_L: {
        q_item.parameters.time_l = val;
        break;
    }
    default: {
        break;
    }
    }
}

/* ---------------------------------------------------------------------------------- */
MotionQueue::Result MotionQueue::executeMotionQueue(uint8_t *motion_num_ptr) {
    uint8_t motion_num = *motion_num_ptr;
    delete motion_num_ptr;

    // Sanity check
    if ((motion_num >= MAX_MOTIONS) && (motion_num != TEST_MOTION_NUM)) {
        motion_queue_debug("Max motion is %d. Received -> %d\n", MAX_MOTIONS - 1, motion_num);
        return Result::INVALID_MOTION;
    }

    // Update flags
    m_queue_status.is_executing = 1;
    m_queue_status.motion_num   = motion_num;
    m_queue_status.sequence_num = 0;

    /* =================================== Test Motion ================================== */
    if (motion_num == TEST_MOTION_NUM) {
        motion_queue_debug("------------ Executing Test Motion %d ------------\n", motion_num);
        flagStepperUsedInQueue(test_item.parameters.driver_num);

        QueueItem test_items[] = {test_item};
        Result    res          = executeItemInMotionQueue(test_items, 1);

        unflagAllSteppers();
        return res;
    }

    /* ================================== Normal Motion ================================= */
    motion_queue_debug("------------ Executing motion %d ------------\n", motion_num);
    // Fetch the list of motions (ref)
    Queue &q = motion_list[motion_num];

    // Find highest sequence number
    uint8_t max_seq        = 0;
    bool    has_queue_item = false;

    for (int i = 0; i < MAX_ITEMS_PER_MOTION; i++) {
        if (q.items[i].sequence_number != 0xFF) {
            has_queue_item = true;
            if (q.items[i].sequence_number > max_seq) { max_seq = q.items[i].sequence_number; }
            // Flag all steppers used in this motion queue
            flagStepperUsedInQueue(q.items[i].parameters.driver_num);
        }
    }

    // Queue check
    if (!has_queue_item) {
        motion_queue_debug("No items found in this queue!\n");
        return Result::EMPTY_QUEUE;
    }

    motion_queue_debug("Highest sequence number %d\n", max_seq);

    Result res;

    // Loop through all sequences
    for (int sq_num = 0; sq_num <= max_seq; sq_num++) {

        // Max concurrent actions = max steppers
        QueueItem items_this_sequence[MAX_CONCURRENT_SEQUENCES];
        uint8_t   num_items = 0;

        // Find all motions for current sequence to execute concurrently
        for (int q_item = 0; q_item < MAX_ITEMS_PER_MOTION; q_item++) {
            if (q.items[q_item].sequence_number == sq_num) {
                num_items++;

                // Check if exceed, return if so
                if (num_items > MAX_CONCURRENT_SEQUENCES) { return Result::EXCEED_MAX_CONCURRENT; }

                // Assign to list
                items_this_sequence[num_items - 1] = q.items[q_item];
            }
        }

        // Skip sequences that have no items
        if (num_items == 0) continue;

        // Pass list to be executed
        m_queue_status.sequence_num = sq_num; // update sequence number
        motion_queue_debug("Executing %d items from sequence %d\n", num_items, sq_num);
        res = executeItemInMotionQueue(items_this_sequence, num_items);

        // Early return on failure
        if (res != Result::SUCCESS) {
            unflagAllSteppers();
            return res;
        };
    }

    unflagAllSteppers();
    return Result::SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
MotionQueue::Result MotionQueue::executeItemInMotionQueue(QueueItem items[], const uint8_t num_items) {
    uint8_t i;
    uint8_t j;
    uint8_t driver_num;
    bool    success = true;

    // Check stepper validity (allow exception for wait time)
    motion_queue_debug("-> Checking parameters...", num_items);
    for (i = 0; i < num_items; i++) {
        if ((items[i].parameters.driver_num >= MAX_STEPPERS) && (items[i].motion_type != MotionType::WAIT_TIME)) {
            motion_queue_debug("Bad Parameters\n\n", num_items);
            return Result::BAD_PARAMETERS;
        }
    }

    // Each stepper can only be assigned once in a sequence
    for (i = 0; i < num_items; i++) {
        for (j = 0; j < num_items; j++) {
            if ((i != j) && (items[i].parameters.driver_num == items[j].parameters.driver_num)) {
                motion_queue_debug("Repeated Assignment\n\n", num_items);
                return Result::REPEATED_ASSIGNMENT;
            }
        }
    }
    motion_queue_debug("OK\n");

    motion_queue_debug("-> Checking stepper status...");
    // Check if stepper is ready to be actuated. (allow exception for wait time)
    for (i = 0; i < num_items; i++) {
        driver_num = items[i].parameters.driver_num;
        if ((items[i].motion_type != MotionType::WAIT_TIME) && (!stepperIsReadyForMovement(driver_num))) {
            motion_queue_debug("Steppers not Ready\n\n", num_items);
            return Result::STEPPERS_NOT_READY;
        }
    }
    motion_queue_debug("OK\n");

    motion_queue_debug("-> Writing motion...");
    // Write motion for all steppers
    for (i = 0; i < num_items; i++) {
        success &= setMotionParameters(items[i].motion_type, items[i].parameters);
    }

    if (!success) {
        motion_queue_debug("FAILED\n");
        return Result::FAILED_MIDWAY;
    }
    motion_queue_debug("OK\n");

    motion_queue_debug("-> Starting motion...");
    // Start all steppers
    for (i = 0; i < num_items; i++) {
        success &= startMotion(items[i].motion_type, items[i].parameters);
    }

    if (!success) {
        motion_queue_debug("FAILED\n");
        return Result::FAILED_MIDWAY;
    }
    motion_queue_debug("OK\n");

    // Wait start
    vTaskDelay(100 / portTICK_RATE_MS);

    motion_queue_debug("-> Running...");
    // Wait for all to stop
    MotionStatus motion_status[num_items] = {MotionStatus::M_RUNNING};

    // Blocking Loop
    portTickType start_tick = xTaskGetTickCount();
    for (;;) {
        vTaskDelay((10 * MAX_STEPPERS) / portTICK_RATE_MS);
        // Update status
        for (i = 0; i < num_items; i++) {
            motion_status[i] = getMotionStatus(items[i].motion_type, items[i].parameters, start_tick);
        }

        // Has Err?
        for (i = 0; i < num_items; i++) {
            if ((motion_status[i] == MotionStatus::M_ERR) || m_abort_queue) {
                // EmStop all
                for (j = 0; j < num_items; j++) {
                    driver_num = items[j].parameters.driver_num;
                    m_steppers[driver_num]->d_emStop();
                }
                motion_queue_debug("STOPPED\n");
                m_abort_queue = false; // reset
                return Result::FAILED_MIDWAY;
            }
        }

        // Complete?
        bool complete = true;
        for (i = 0; i < num_items; i++) {
            complete &= (motion_status[i] == MotionStatus::COMPLETE);
        }

        if (complete) { break; }
    }

    motion_queue_debug("OK\n");

    motion_queue_debug("-> Result...");
    // Get Readback
    bool all_complete         = true;
    bool conditional_complete = true;
    for (i = 0; i < num_items; i++) {
        Result res = execSuccessful(items[i].motion_type, items[i].parameters);

        all_complete &= (res == Result::SUCCESS);
        conditional_complete &= (res == Result::SUCCESS || res == Result::CONDITIONAL_SUCCESS);
    }

    // Final
    if (all_complete) {
        motion_queue_debug("OKK\n\n");
        return Result::SUCCESS;
    }
    if (conditional_complete) {
        motion_queue_debug("OK?\n\n");
        return Result::CONDITIONAL_SUCCESS;
    }
    motion_queue_debug("FAILED\n\n");
    return Result::FAILED_MIDWAY;
}

/* ---------------------------------------------------------------------------------- */
bool MotionQueue::stepperIsReadyForMovement(uint8_t driver_num) {
    Stepper::Status sts = m_steppers[driver_num]->getStatus();

    return (sts == Stepper::Status::READY || sts == Stepper::Status::MOTION_NOT_INIT);
}

/* ---------------------------------------------------------------------------------- */
void MotionQueue::flagStepperUsedInQueue(uint8_t driver_num) {
    if (driver_num >= MAX_STEPPERS) { return; }
    stepper_used_in_queue[driver_num] = true;
}

/* ---------------------------------------------------------------------------------- */
void MotionQueue::unflagAllSteppers() {
    for (int i = 0; i < MAX_STEPPERS; i++) {
        stepper_used_in_queue[i] = false;
    }
}

/* ---------------------------------------------------------------------------------- */
bool MotionQueue::setMotionParameters(MotionType motion_type, MotionParameters parameters) {
    uint8_t driver_num = parameters.driver_num;

    if (motion_type == MotionType::WAIT_SENSOR || motion_type == MotionType::WAIT_TIME) {
        motion_queue_debug(" [Set motion skipped] ");
        return true;
    }

    // Homing
    if (motion_type == MotionType::MOVE_HOMING) {

        // Config check
        if (parameters.homing_mode == 0xFF) {
            motion_queue_debug(" [No Homing Mode Provided !] ");
            return false;
        }

        Stepper::HomeSettings home_s;
        home_s.homing_mode       = static_cast<Stepper::HomingMode>(parameters.homing_mode);
        home_s.homing_sensor     = static_cast<Stepper::HomingSensor>(parameters.homing_sensor);
        home_s.sensor_home_value = static_cast<bool>(parameters.sensor_home_value);
        home_s.max_find_h        = parameters.position_h;
        home_s.max_find_l        = parameters.position_l;
        home_s.max_speed_h       = parameters.max_speed_h;
        home_s.max_speed_l       = parameters.max_speed_l;
        home_s.max_accel_h       = parameters.max_accel_h;
        home_s.max_accel_l       = parameters.max_accel_l;
        home_s.max_decel_h       = parameters.max_decel_h;
        home_s.max_decel_l       = parameters.max_decel_l;
        home_s.offset_h          = parameters.offset_h;
        home_s.offset_l          = parameters.offset_l;
        home_s.timeout_ms_h      = parameters.time_h;
        home_s.timeout_ms_l      = parameters.time_l;

        m_steppers[driver_num]->confHome(home_s);

        return true;
    }

    // Relative / Absolute
    Stepper::MoveSettings move_s;
    move_s.unit_high    = parameters.position_h;
    move_s.unit_low     = parameters.position_l;
    move_s.time_ms_high = parameters.time_h;
    move_s.time_ms_low  = parameters.time_l;

    m_steppers[driver_num]->confMove(move_s);

    Stepper::RampSettings ramp_s;
    ramp_s.ramp_mode = Stepper::RampMode::POSITIONING_MODE;
    ramp_s.ramp_type = static_cast<Stepper::RampType>(parameters.ramp_type);

    m_steppers[driver_num]->confRamp(ramp_s);

    Stepper::SpeedSettings m_spd_s;
    m_spd_s.max_h = parameters.max_speed_h;
    m_spd_s.max_l = parameters.max_speed_l;

    m_steppers[driver_num]->confSpeed(m_spd_s);

    Stepper::AccelerationSettings m_accel_s;
    m_accel_s.max_accel_h = parameters.max_accel_h;
    m_accel_s.max_accel_l = parameters.max_accel_l;
    m_accel_s.max_decel_h = parameters.max_decel_h;
    m_accel_s.max_decel_l = parameters.max_decel_l;

    m_steppers[driver_num]->confAccel(m_accel_s);

    Stepper::BowSettings m_bow_s;
    m_bow_s.bow1_h = parameters.bow1_h;
    m_bow_s.bow1_l = parameters.bow1_l;
    m_bow_s.bow2_h = parameters.bow2_h;
    m_bow_s.bow2_l = parameters.bow2_l;
    m_bow_s.bow3_h = parameters.bow3_h;
    m_bow_s.bow3_l = parameters.bow3_l;
    m_bow_s.bow4_h = parameters.bow4_h;
    m_bow_s.bow4_l = parameters.bow4_l;

    m_steppers[driver_num]->confBow(m_bow_s);

    if (motion_type == MotionType::MOVE_RELATIVE) {
        m_steppers[driver_num]->confPositioning_posMode(Stepper::PositioningMode::PM_RELATIVE);
    } else {
        m_steppers[driver_num]->confPositioning_posMode(Stepper::PositioningMode::PM_ABSOLUTE);
    }

    return m_steppers[driver_num]->execSetMotion();
}

/* ---------------------------------------------------------------------------------- */
bool MotionQueue::startMotion(MotionType motion_type, MotionParameters parameters) {
    if (motion_type == MotionType::WAIT_SENSOR || motion_type == MotionType::WAIT_TIME) {
        motion_queue_debug(" [Start motion skipped] ");
        return true;
    }

    uint8_t              driver_num = parameters.driver_num;
    Stepper::PutQueueRes res;

    if (motion_type == MotionType::MOVE_HOMING) {
        res = m_steppers[driver_num]->moveHoming();

    } else if (parameters.use_inverse_time == 1) {
        res = m_steppers[driver_num]->moveInverseTime();
    } else {

        res = m_steppers[driver_num]->move();
    }

    return res == Stepper::PutQueueRes::Q_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
MotionQueue::MotionStatus MotionQueue::getMotionStatus(MotionType motion_type, MotionParameters parameters,
                                                       portTickType start_tick) {
    // Sensor wait
    if (motion_type == MotionType::WAIT_SENSOR) {
        uint8_t      sensor_sts;
        uint16_t     sensor_value;
        uint32_t     timeout_ms;
        portTickType timeout_ticks;
        bool         timeout = false;

        sensor_sts    = m_steppers[parameters.driver_num]->getSensorReading();
        sensor_value  = static_cast<uint16_t>((sensor_sts >> parameters.homing_sensor) & 1);
        timeout_ms    = (static_cast<uint32_t>(parameters.time_h) << 16) | static_cast<uint32_t>(parameters.time_l);
        timeout_ticks = timeout_ms / portTICK_RATE_MS;

        bool condition_met = (sensor_value == parameters.sensor_home_value);
        motion_queue_debug("%d", sensor_value);

        if (timeout_ticks != 0) { timeout = (xTaskGetTickCount() - start_tick) >= timeout_ticks; }

        return (condition_met || timeout) ? MotionStatus::COMPLETE : MotionStatus::M_RUNNING;
    }

    // Blind wait
    if (motion_type == MotionType::WAIT_TIME) {
        uint32_t     target_wait_ms;
        portTickType target_wait_ticks;

        target_wait_ms    = (static_cast<uint32_t>(parameters.time_h) << 16) | static_cast<uint32_t>(parameters.time_l);
        target_wait_ticks = target_wait_ms / portTICK_RATE_MS;

        bool condition_met = (xTaskGetTickCount() - start_tick) >= target_wait_ticks;

        motion_queue_debug(condition_met ? "Wait complete" : ".");
        return condition_met ? MotionStatus::COMPLETE : MotionStatus::M_RUNNING;
    }

    // Motor block
    Stepper::Status sts = m_steppers[parameters.driver_num]->getStatus();
    if (sts == Stepper::Status::BUSY || sts == Stepper::Status::HOMING) { return MotionStatus::M_RUNNING; }
    if (sts == Stepper::Status::READY || sts == Stepper::Status::MOTION_NOT_INIT) { return MotionStatus::COMPLETE; }
    return MotionStatus::M_ERR;
}

/* ---------------------------------------------------------------------------------- */
MotionQueue::Result MotionQueue::execSuccessful(MotionType motion_type, MotionParameters parameters) {
    if (motion_type == MotionType::WAIT_SENSOR) {
        uint8_t sensor_sts = m_steppers[parameters.driver_num]->getSensorReading();

        bool condition_met = ((sensor_sts >> parameters.homing_sensor) & 1) == parameters.sensor_home_value;

        return condition_met ? Result::SUCCESS : Result::FAILED_MIDWAY;
    }

    if (motion_type == MotionType::WAIT_TIME) { return Result::SUCCESS; }

    uint32_t readback_value    = 0;
    bool     readback_acquired = false;

    while (!readback_acquired) {
        readback_acquired = m_steppers[parameters.driver_num]->getReadback(&readback_value);
        vTaskDelay(5 / portTICK_RATE_MS);
    }

    // Homing
    if (motion_type == MotionType::MOVE_HOMING) {
        Stepper::HomingCode homing_code = static_cast<Stepper::HomingCode>(readback_value);
        motion_queue_debug("[ Homing Code %d ]", homing_code);

        if (homing_code == Stepper::HomingCode::H_SUCCESS) { return Result::SUCCESS; }
        if (homing_code == Stepper::HomingCode::H_MAX_PULSE_REACHED || homing_code == Stepper::HomingCode::H_TIMEOUT) {
            return Result::CONDITIONAL_SUCCESS;
        }
        return Result::FAILED_MIDWAY;
    }

    Stepper::ExecCode exec_code = static_cast<Stepper::ExecCode>(readback_value);

    return (exec_code == Stepper::ExecCode::E_SUCCESS) ? Result::SUCCESS : Result::FAILED_MIDWAY;
}

/* ---------------------------------------------------------------------------------- */
void MotionQueue::task_motionQueue(void *parameters) {
    auto               *self = static_cast<MotionQueue *>(parameters);
    uint8_t            *motion_number_ptr;
    MotionQueue::Result result;

    // motion_queue_debug("Motion Queue Task Start\n");

    for (;;) {
        if (xQueuePeek(self->m_motion_queue_handle, &motion_number_ptr, 0) == pdTRUE) {
            // motion_queue_debug("Received command to run motion %d\n", *motion_number_ptr);

            // Execute and free heap memory inside executeMotionQueue
            result = self->executeMotionQueue(motion_number_ptr);

            self->m_queue_status.is_executing = 0; // reset status

            // Send result
            xQueueSend(self->m_result_queue_handle, &result, 0);

            // Remove from queue
            xQueueReceive(self->m_motion_queue_handle, &motion_number_ptr, 0);
        } else {
            vTaskDelay(10 / portTICK_RATE_MS);
        }
        // print_task_memory();
    }
}
/* ---------------------------------------------------------------------------------- */