/*
 * parser.h
 *
 * Created: 07/11/2025 15:57:53
 *  Author: Tan
 */
#pragma once

#ifndef PARSER_H_

#    define PARSER_H_

#    include "asf.h"
#    include "motion_queue.h"
#    include "sd_card.h"
#    include "stepper.h"
#    include <cmath>
#    include <stdio.h>
#    include <stdlib.h>
#    include <string.h>

#    ifdef PARSER_DEBUG
#        include <stdio.h>
#        define parser_debug(...) printf(__VA_ARGS__)
#    else
#        define parser_debug(...)
#    endif

/* ========================== Load Parameters from SD Card ========================== */
/**
 * @brief Loads driver config from sd card
 *
 * @param sd_card ptr to sd card
 * @param steppers full stepper array
 * @return binary of driver initialized
 */
extern uint8_t parse_driverConfig(SDCard *sd_card, Stepper *steppers[MAX_STEPPERS]);

/**
 * @brief Loads motion items from sequence file on sd card into motion queue
 *
 * @param sd_card ptr to sd card
 * @param motion_queue ptr to motion queue
 * @return load successful
 */
extern bool parse_motionSequence(SDCard *sd_card, MotionQueue *motion_queue);

/**
 * @brief Returns name of motion file associated to motion index
 *
 * @param sd_card ptr to sd card
 * @param motion_index index as defined in _indexMap (eg: '1=input_z_raise')
 * @param out_file_name filename (eg: 'input_z_raise')
 * @return file exist
 */
extern bool parse_motionFileName(SDCard *sd_card, const uint8_t motion_index, char *out_file_name);

extern void parseJSON(char *file_contents, char **json_string);

#endif /* PARSER_H_ */