/*
 * startup.cpp
 *
 * Created: 07/11/2025 15:58:06
 *  Author: Tan
 */

#include "parser.h"

/* ================================================================================== */
/*                                       Utility                                      */
/* ================================================================================== */

static uint16_t getCurrentScaler(uint16_t current_ma) {
    constexpr uint16_t MIN_CURRENT        = 152;  // mA
    constexpr uint16_t MAX_CURRENT_NEMA17 = 3000; // mA
    constexpr uint8_t  MAX_STEP           = 31;

    if (current_ma <= MIN_CURRENT) return 1;
    if (current_ma > MAX_CURRENT_NEMA17) current_ma = MAX_CURRENT_NEMA17;

    float scale = (static_cast<float>(current_ma) / DRIVER_RMS_MAX) * MAX_STEP;

    uint16_t scaler = static_cast<uint16_t>(roundf(scale));
    if (scaler > MAX_STEP) scaler = MAX_STEP;

    // parser_debug("Current scale -> %d\n", scaler);

    return scaler;
};

struct UInt32Parts {
    uint16_t high;
    uint16_t low;
};

static UInt32Parts floatToHighAndLowWords(float value) {
    UInt32Parts res;

    uint32_t int_value = static_cast<uint32_t>(value * 1000.0f);

    res.high = (int_value >> 16) & 0xFFFF;
    res.low  = int_value & 0xFFFF;

    return res;
}

static UInt32Parts int32ToHighAndLowWords(int32_t value) {
    UInt32Parts res;

    uint32_t int_value = static_cast<uint32_t>(value);

    res.high = (int_value >> 16) & 0xFFFF;
    res.low  = int_value & 0xFFFF;

    return res;
}

/* ================================================================================== */
/*                                       Parsers                                      */
/* ================================================================================== */

uint8_t parse_driverConfig(SDCard *sd_card, Stepper *steppers[MAX_STEPPERS]) {
    char   *file_contents = NULL;
    char    filename[32];
    uint8_t rslt = 0;

    parser_debug("\n=====================================\n");
    parser_debug("|        PARSING DRIVER CONFIG      |");
    parser_debug("\n=====================================\n");

    for (int i = 0; i < MAX_STEPPERS; i++) {
        // Expected path: "0:motion/sequence/<motion_num>"
        snprintf(filename, sizeof(filename), "config/%d", i);

        if (!sd_card->readFile(filename, &file_contents)) {
            parser_debug("Failed to read driver %d config.\n", i);
            continue;
        }

        parser_debug("\nReading driver %d config...\n", i);

        char *ptr = file_contents;
        while (*ptr) {
            char *end = strchr(ptr, '\n');
            if (end) *end = '\0';

            size_t len = strlen(ptr);
            if (len > 0 && ptr[len - 1] == '\r') ptr[len - 1] = '\0';

            char cmd[32];
            int  val;
            if (sscanf(ptr, "%31[^=]=%d", cmd, &val) == 2) {
                parser_debug("(%d) %s=%d\n", i, cmd, val);

                if (strcmp(cmd, "MICROSTEP_PER_FULLSTEP") == 0) {
                    steppers[i]->setDrv_mstepPerFs(val);
                } else if (strcmp(cmd, "FULLSTEP_PER_REVOLUTION") == 0) {
                    steppers[i]->setDrv_fsPerRev(val);
                } else if (strcmp(cmd, "UNIT_PER_REVOLUTION") == 0) {
                    steppers[i]->confPositioning_unitPerRev(val);
                } else if (strcmp(cmd, "PEAK_CURRENT") == 0) {
                    steppers[i]->setCurrent_iRun(getCurrentScaler(val));
                } else if (strcmp(cmd, "HOLDING_CURRENT") == 0) {
                    steppers[i]->setCurrent_iHold(getCurrentScaler(val));
                } else if (strcmp(cmd, "STOP_ON_STALL_ENABLE") == 0) {
                    steppers[i]->setStopOnStall_enable(val);
                } else if (strcmp(cmd, "STOP_ON_STALL_THRESHOLD") == 0) {
                    steppers[i]->setStopOnStall_thresh(val);
                } else if (strcmp(cmd, "CLOSED_LOOP_ENABLE") == 0) {
                    steppers[i]->setClosedLoop_enable(val);
                } else if (strcmp(cmd, "CLOSED_LOOP_USE_PID") == 0) {
                    steppers[i]->setClosedLoop_usePID(val);
                } else if (strcmp(cmd, "ENCODER_PULSE_PER_REVOLUTION") == 0) {
                    steppers[i]->setClosedLoop_encInRes(val);
                } else if (strcmp(cmd, "MAX_DELTA_PULSE") == 0) {
                    steppers[i]->setClosedLoop_tolerance(val);
                } else if (strcmp(cmd, "STEALTH_CHOP_THRESHOLD") == 0) {
                    steppers[i]->setStealthChopThreshold(val);
                }
            }

            if (!end) break;
            ptr = end + 1;
        }

        free(file_contents);
        rslt |= 1 << i;
    }

    parser_debug("\n=====================================\n");
    parser_debug("Result: |");
    for (int j = 0; j < MAX_STEPPERS; j++) {
        parser_debug(" %d |", (rslt >> j) & 1);
    }
    parser_debug("\n=====================================\n");

    return rslt;
}

/* ---------------------------------------------------------------------------------- */

static bool parseIndexMap(SDCard *sd_card, uint8_t out_index_list[MAX_ITEMS_PER_MOTION],
                          char out_file_name_list[MAX_ITEMS_PER_MOTION][32], uint8_t *out_num_items) {
    // Read "_indexMap" file
    char *file_contents = NULL;

    if (!sd_card->readFile("motion/_indexMap", &file_contents)) {
        parser_debug("Failed to read _indexMap file.\n");
        return false;
    }

    parser_debug("\nReading _indexMap file...\n");
    char   *ptr       = file_contents;
    uint8_t num_items = 0;

    while (*ptr) {
        char *end = strchr(ptr, '\n');
        if (end) *end = '\0';

        size_t len = strlen(ptr);
        if (len > 0 && ptr[len - 1] == '\r') ptr[len - 1] = '\0';

        int  motion_num;
        char motion_file_name[32];

        if (sscanf(ptr, "%d=%31s", &motion_num, motion_file_name) == 2) {
            parser_debug("%d=%s\n", motion_num, motion_file_name);

            out_index_list[num_items] = motion_num;
            strcpy(out_file_name_list[num_items], motion_file_name);

            num_items++;
        }

        if (!end) break;
        ptr = end + 1;
    }

    free(file_contents);

    *out_num_items = num_items;

    return true;
}

static bool parsePreloadedItems(SDCard *sd_card, MotionQueue *motion_queue, uint8_t index_list[MAX_ITEMS_PER_MOTION],
                                char file_name_list[MAX_ITEMS_PER_MOTION][32], uint8_t num_items) {

    uint8_t current_item = 0;

    for (int i = 0; i < num_items; i++) {
        char *file_contents = NULL;
        char  filename[64];

        // Expected path: "0:motion/<filename>"
        snprintf(filename, sizeof(filename), "motion/%s", file_name_list[i]);

        if (!sd_card->readFile(filename, &file_contents)) {
            parser_debug("Failed to read %s.\n", filename);
            continue;
        }

        current_item = index_list[i];

        parser_debug("\nParsing [%s] as item [%d]\n", filename, current_item);
        char *line = file_contents;
        while (*line) {
            char *end = strchr(line, '\n');
            if (end) *end = '\0';

            size_t len = strlen(line);
            if (len > 0 && line[len - 1] == '\r') line[len - 1] = '\0';

            char cmd[32];
            int  val;
            if (sscanf(line, "%31[^=]=%d", cmd, &val) == 2) {
                parser_debug("(%d) %s=%d\n", current_item, cmd, val);

                if (strcmp(cmd, "MOTION_TYPE") == 0) {
                    motion_queue->preloaded_motions[current_item].motion_type =
                        static_cast<MotionQueue::MotionType>(val);
                } else if (strcmp(cmd, "DRIVER_NUMBER") == 0) {
                    motion_queue->preloaded_motions[current_item].parameters.driver_num = val;
                } else if (strcmp(cmd, "POSITION") == 0) {
                    UInt32Parts position = int32ToHighAndLowWords(static_cast<int32_t>(val));
                    motion_queue->preloaded_motions[current_item].parameters.position_h = position.high;
                    motion_queue->preloaded_motions[current_item].parameters.position_l = position.low;
                } else if (strcmp(cmd, "OFFSET") == 0) {
                    UInt32Parts offset = int32ToHighAndLowWords(static_cast<int32_t>(val));
                    motion_queue->preloaded_motions[current_item].parameters.offset_h = offset.high;
                    motion_queue->preloaded_motions[current_item].parameters.offset_l = offset.low;
                } else if (strcmp(cmd, "HOMING_MODE") == 0) {
                    motion_queue->preloaded_motions[current_item].parameters.homing_mode = val;
                } else if (strcmp(cmd, "HOMING_SENSOR") == 0) {
                    motion_queue->preloaded_motions[current_item].parameters.homing_sensor = val;
                } else if (strcmp(cmd, "SENSOR_HOME_VALUE") == 0) {
                    motion_queue->preloaded_motions[current_item].parameters.sensor_home_value = val;
                } else if (strcmp(cmd, "RAMP_TYPE") == 0) {
                    motion_queue->preloaded_motions[current_item].parameters.ramp_type = val;
                } else if (strcmp(cmd, "MAX_SPEED") == 0) {
                    UInt32Parts max_speed = floatToHighAndLowWords(static_cast<float>(val));
                    motion_queue->preloaded_motions[current_item].parameters.max_speed_h = max_speed.high;
                    motion_queue->preloaded_motions[current_item].parameters.max_speed_l = max_speed.low;
                } else if (strcmp(cmd, "MAX_ACCELERATION") == 0) {
                    UInt32Parts max_acceleration = floatToHighAndLowWords(static_cast<float>(val));
                    motion_queue->preloaded_motions[current_item].parameters.max_accel_h = max_acceleration.high;
                    motion_queue->preloaded_motions[current_item].parameters.max_accel_l = max_acceleration.low;
                } else if (strcmp(cmd, "MAX_DECELERATION") == 0) {
                    UInt32Parts max_deceleration = floatToHighAndLowWords(static_cast<float>(val));
                    motion_queue->preloaded_motions[current_item].parameters.max_decel_h = max_deceleration.high;
                    motion_queue->preloaded_motions[current_item].parameters.max_decel_l = max_deceleration.low;
                } else if (strcmp(cmd, "BOW1") == 0) {
                    UInt32Parts bow1 = floatToHighAndLowWords(static_cast<float>(val));
                    motion_queue->preloaded_motions[current_item].parameters.bow1_h = bow1.high;
                    motion_queue->preloaded_motions[current_item].parameters.bow1_l = bow1.low;
                } else if (strcmp(cmd, "BOW2") == 0) {
                    UInt32Parts bow2 = floatToHighAndLowWords(static_cast<float>(val));
                    motion_queue->preloaded_motions[current_item].parameters.bow2_h = bow2.high;
                    motion_queue->preloaded_motions[current_item].parameters.bow2_l = bow2.low;
                } else if (strcmp(cmd, "BOW3") == 0) {
                    UInt32Parts bow3 = floatToHighAndLowWords(static_cast<float>(val));
                    motion_queue->preloaded_motions[current_item].parameters.bow3_h = bow3.high;
                    motion_queue->preloaded_motions[current_item].parameters.bow3_l = bow3.low;
                } else if (strcmp(cmd, "BOW4") == 0) {
                    UInt32Parts bow4 = floatToHighAndLowWords(static_cast<float>(val));
                    motion_queue->preloaded_motions[current_item].parameters.bow4_h = bow4.high;
                    motion_queue->preloaded_motions[current_item].parameters.bow4_l = bow4.low;
                } else if (strcmp(cmd, "USE_INVERSE_TIME") == 0) {
                    motion_queue->preloaded_motions[current_item].parameters.use_inverse_time =
                        static_cast<uint16_t>(val);
                } else if (strcmp(cmd, "TIME_MS") == 0) {
                    UInt32Parts time_ms = int32ToHighAndLowWords(static_cast<int32_t>(val));
                    motion_queue->preloaded_motions[current_item].parameters.time_h = time_ms.high;
                    motion_queue->preloaded_motions[current_item].parameters.time_l = time_ms.low;
                }
            }

            if (!end) break;
            line = end + 1;
        }

        free(file_contents);
    }

    return true;
}

static bool parseItemsIntoSequence(SDCard *sd_card, MotionQueue *motion_queue) {
    for (int sq_idx = 0; sq_idx <= MAX_MOTIONS; sq_idx++) {
        // Read Sequence file
        char *file_contents = NULL;
        char  filename[32];

        // Expected path: "0:motion/sequence/<motion_num>"
        snprintf(filename, sizeof(filename), "sequence/%d", sq_idx);

        if (!sd_card->readFile(filename, &file_contents)) {
            parser_debug("\nSequence file for motion %d not found.\n", sq_idx);
            continue;
        }

        parser_debug("\nLoading sequence [%d]...\n", sq_idx);

        // Read Sequence
        char   *ptr         = file_contents;
        uint8_t sq_item_idx = 0;

        while (*ptr) {
            char *end = strchr(ptr, '\n');
            if (end) *end = '\0';

            size_t len = strlen(ptr);
            if (len > 0 && ptr[len - 1] == '\r') ptr[len - 1] = '\0';

            int sq_num;
            int item_index;

            if (sscanf(ptr, "%d=%d", &sq_num, &item_index) == 2) {
                parser_debug("(%d%d) %d=>Sq%d\n", sq_idx, sq_item_idx, item_index, sq_num);

                // Load preloaded item into motion sequence
                motion_queue->motion_list[sq_idx].items[sq_item_idx] = motion_queue->preloaded_motions[item_index];
                // Assign sequence number
                motion_queue->motion_list[sq_idx].items[sq_item_idx].sequence_number = sq_num;

                sq_item_idx++;
            }

            if (!end) break;
            ptr = end + 1;
        }

        free(file_contents);
    }
    return true;
}

/* ---------------------------------------------------------------------------------- */

bool parse_motionSequence(SDCard *sd_card, MotionQueue *motion_queue) {
    parser_debug("\n=====================================\n");
    parser_debug("|     PARSING MOTION & SEQUENCE     |");
    parser_debug("\n=====================================\n");

    uint8_t num_items = 0;
    uint8_t index_list[MAX_ITEMS_PER_MOTION];
    char    file_name_list[MAX_ITEMS_PER_MOTION][32];

    // Read _indexMap
    if (!parseIndexMap(sd_card, index_list, file_name_list, &num_items)) { return false; }

    // Read Preloaded
    if (!parsePreloadedItems(sd_card, motion_queue, index_list, file_name_list, num_items)) { return false; }

    // Read Sequence
    if (!parseItemsIntoSequence(sd_card, motion_queue)) { return false; }

    return true;
}

bool parse_motionFileName(SDCard *sd_card, const uint8_t motion_index, char *out_file_name) {
    // Read "_indexMap" file
    const char file_name[64] = "motion/_indexMap";
    char      *file_contents = NULL;

    bool file_read = sd_card->readFile(file_name, &file_contents);

    if (!file_read) {
        parser_debug("Failed to read _indexMap file.\n");
        return false;
    }

    parser_debug("\nReading _indexMap file...\n");
    char *ptr         = file_contents;
    bool  match_found = false;

    while (*ptr) {
        char *end = strchr(ptr, '\n');
        if (end) *end = '\0';

        size_t len = strlen(ptr);
        if (len > 0 && ptr[len - 1] == '\r') ptr[len - 1] = '\0';

        int  motion_num;
        char motion_file_name[32];

        if (sscanf(ptr, "%d=%31s", &motion_num, motion_file_name) == 2) {

            if (motion_num == motion_index) {
                strcpy(out_file_name, motion_file_name);
                parser_debug("%d=%s\n", motion_num, motion_file_name);
                match_found = true;
                break;
            }
        }

        if (!end) break;
        ptr = end + 1;
    }

    free(file_contents);

    if (!match_found) parser_debug("No matching filename for %d\n", motion_index);

    return match_found;
}