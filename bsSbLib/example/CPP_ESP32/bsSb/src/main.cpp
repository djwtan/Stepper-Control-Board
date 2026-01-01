#include "bsSb.h"
#include "cstring"
#include "driver/twai.h"
#include "test.h"
#include <Arduino.h>

#define CAN_RX_PIN 21
#define CAN_TX_PIN 22
#define CAN_BAUD   TWAI_TIMING_CONFIG_500KBITS()

#define BSSB_DEBUG

// Direct printf to uart
extern "C" int _write(int fd, const void *buf, size_t count) {
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
        Serial.write((const uint8_t *)buf, count);
        return count;
    }
    return -1;
}

/* ---------------------------------------------------------------------------------- */
void configureCAN(void) {
    // Configure TWAI general settings
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);

    // Configure TWAI timing settings (500 kbps)
    twai_timing_config_t t_config = CAN_BAUD;

    // Configure TWAI filter settings (accept all frames)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("TWAI driver installed");
    } else {
        Serial.println("Failed to install TWAI driver");
        while (1)
            ;
    }

    // Start the TWAI driver
    if (twai_start() == ESP_OK) {
        Serial.println("TWAI driver started, ready to receive and echo messages");
    } else {
        Serial.println("Failed to start TWAI driver");
        while (1)
            ;
    }

    // Clear buffer
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {}
}

uint32_t last_transmit_word;

bool myCANTransmit(uint32_t identifier, uint32_t data, uint8_t data_len) {
    twai_message_t message;
    message.identifier       = identifier;
    message.extd             = 0;
    message.rtr              = 0;
    message.data_length_code = data_len;

    message.data[7] = 0;
    message.data[6] = 0;
    message.data[5] = 0;
    message.data[4] = 0;
    message.data[3] = (data >> 24) & 0xFF;
    message.data[2] = (data >> 16) & 0xFF;
    message.data[1] = (data >> 8) & 0xFF;
    message.data[0] = data & 0xFF;

    // printf("ID[%0.8X] TX[%0.8X]\n ", message.identifier, data);

    bool res = (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK);
    if (res) { last_transmit_word = data; }

    return res;
}

int myCANReceive(uint32_t *buffer, int buffer_size) {
    // Sanity check
    if (buffer == nullptr) return 0;

    twai_message_t message;
    esp_err_t      err;
    int            count                   = 0;
    bool           recv_last_transmit_word = false;

    while (true) {
        // Try to receive one CAN frame
        err = twai_receive(&message, pdMS_TO_TICKS(200));

        if (err == ESP_OK) {
            // Convert the 4 data bytes into uint32_t
            uint32_t data = 0;
            data |= ((uint32_t)message.data[3]) << 24;
            data |= ((uint32_t)message.data[2]) << 16;
            data |= ((uint32_t)message.data[1]) << 8;
            data |= ((uint32_t)message.data[0]);

            // Loopback filtering
            if (data == last_transmit_word) {
                recv_last_transmit_word = true;
                continue;
            }

            printf("ID[%0.8X] RX[%0.8X]\n ", message.identifier, data);

            buffer[count++] = data;

            if (count >= buffer_size) { break; }
        }

        if (err == ESP_ERR_TIMEOUT) {
            // If last transmit word happens to be the response :)
            if (recv_last_transmit_word && count == 0) { buffer[count++] = last_transmit_word; }

            // No more frames inside the per-frame timeout
            break;
        }
    }

    return count; // how many frames we read
}

SemaphoreHandle_t can_mutex = xSemaphoreCreateMutex();

void myCANMutex(bool get) { get ? xSemaphoreTake(can_mutex, portMAX_DELAY) : xSemaphoreGive(can_mutex); }

void myAsyncSleep() { vTaskDelay(pdMS_TO_TICKS(1)); }

/* ---------------------------------------------------------------------------------- */
bsStepperBoard bsSb1(1);
bsStepperBoard bsSb2(2);

test_parameters_t test_param[5];

uint8_t driver;

void setup() {
    Serial.begin(115200);
    configureCAN();

    /* ---------------------------------------------------------------------------------- */
    // // ID[00000301] TX[00010000]
    // for (;;) {
    //     uint32_t dummy[1];
    //     int      recv = myCANReceive(dummy, 1);

    //     // if (recv > 0) myCANTransmit(0x0104, 0x12345678, 4);
    //     // myCANTransmit(0x0104, 0x12345678, 4);
    // }
    /* ---------------------------------------------------------------------------------- */

    // Clear buffer
    uint32_t dummy[1024];
    int      cleared = myCANReceive(dummy, 1024);
    printf("CAN Cleared %d messages\n", cleared);

    pinMode(BUTTON, INPUT);
    pinMode(LED_PIN, OUTPUT);

    bsSb1.setCANTransmit(myCANTransmit);
    bsSb1.setCANReceive(myCANReceive);
    bsSb1.setCANMutex(myCANMutex);
    bsSb1.setAsyncSleep_1ms(myAsyncSleep);

    // for (;;) {
    //     std::string *firmware_version = nullptr;
    //     if (bsSb1.readFirmwareVersion(&firmware_version)) {
    //         // printf("Firmware version: %s\n", firmware_v/ersion->c_str());
    //         free(firmware_version);
    //     }
    // }

    bsSb1.clearQueuedReadback(0);
    bsSb1.clearQueuedReadback(1);
    bsSb1.clearQueuedReadback(2);
    bsSb1.clearQueuedReadback(3);
    bsSb1.clearQueuedReadback(4);

    bsSb2.setCANTransmit(myCANTransmit);
    bsSb2.setCANReceive(myCANReceive);
    bsSb2.setCANMutex(myCANMutex);
    bsSb2.setAsyncSleep_1ms(myAsyncSleep);

    bsSb2.clearQueuedReadback(0);
    bsSb2.clearQueuedReadback(1);
    bsSb2.clearQueuedReadback(2);
    bsSb2.clearQueuedReadback(3);
    bsSb2.clearQueuedReadback(4);

    test_param[0].bsSb           = &bsSb2;
    test_param[0].driver_num     = 0;
    test_param[0].driver_type    = DriverType::OPEN_LOOP;
    test_param[0].position_units = 50;
    test_param[0].test_type      = TestType::JOGGING;

    test_param[1].bsSb           = &bsSb2;
    test_param[1].driver_num     = 1;
    test_param[1].driver_type    = DriverType::OPEN_LOOP;
    test_param[1].position_units = 1800;
    test_param[1].test_type      = TestType::MOVE_VELOCITY;

    test_param[2].bsSb           = &bsSb2;
    test_param[2].driver_num     = 2;
    test_param[2].driver_type    = DriverType::OPEN_LOOP;
    test_param[2].position_units = 100;
    test_param[2].test_type      = TestType::MOVE_VELOCITY;

    test_param[3].bsSb           = &bsSb2;
    test_param[3].driver_num     = 3;
    test_param[3].driver_type    = DriverType::OPEN_LOOP;
    test_param[3].position_units = 100;
    test_param[3].test_type      = TestType::MOVE_VELOCITY;

    test_param[4].bsSb           = &bsSb2;
    test_param[4].driver_num     = 4;
    test_param[4].driver_type    = DriverType::OPEN_LOOP;
    test_param[4].position_units = 100;
    test_param[4].test_type      = TestType::MOVE_VELOCITY;

    // xTaskCreate(testTask_RTOS,   // Task function
    //             "testTask_RTOS", // Task name
    //             4096,            // Stack size
    //             &test_param[0],  // Parameter: stepper number
    //             1,               // Priority
    //             nullptr          // Task handle
    // );
    // xTaskCreate(testTask_RTOS,   // Task function
    //             "testTask_RTOS", // Task name
    //             4096,            // Stack size
    //             &test_param[1],  // Parameter: stepper number
    //             1,               // Priority
    //             nullptr          // Task handle
    // );
    // xTaskCreate(testTask_RTOS,    // Task function
    //             "testTask_RTOS",  // Task name
    //             4096,           // Stack size
    //             &test_param[2], // Parameter: stepper number
    //             1,              // Priority
    //             nullptr         // Task handle
    // );
    // xTaskCreate(testTask_RTOS,    // Task function
    //             "testTask_RTOS",  // Task name
    //             4096,           // Stack size
    //             &test_param[3], // Parameter: stepper number
    //             1,              // Priority
    //             nullptr         // Task handle
    // );
    // xTaskCreate(testTask_RTOS,    // Task function
    //             "testTask_RTOS",  // Task name
    //             4096,           // Stack size
    //             &test_param[4], // Parameter: stepper number
    //             1,              // Priority
    //             nullptr         // Task handle
    // );
    /* ---------------------------------------------------------------------------------- */
    // xTaskCreate(cageSequence_manualCall, // Task function
    //             "cage sequence",         // Task name
    //             4096,                    // Stack size
    //             &bsSb1,                  // Parameter: stepper number
    //             1,                       // Priority
    //             nullptr                  // Task handle
    // );
    // xTaskCreate(cageSequence_queue, // Task function
    //             "cage sequence",    // Task name
    //             4096,               // Stack size
    //             &bsSb1,             // Parameter: stepper number
    //             1,                  // Priority
    //             nullptr             // Task handle
    // );
    // xTaskCreate(cageSequence_preloadedMotion, // Task function
    //             "cage sequence",              // Task name
    //             4096,                         // Stack size
    //             &bsSb1,                       // Parameter: stepper number
    //             1,                            // Priority
    //             nullptr                       // Task handle
    // );
}

void loop() {
    /* =================================== Read Sensor ================================== */
    uint8_t sensor_status = 0xFF;

    for (int i = 0; i <= 4; i++) {

        // bsSb1.releaseDriver(i);
        sensor_status = bsSb1.readSensors(i);
        // printf("| %d | L: %d, R: %d ", i, ((sensor_status >> 0) & 1), ((sensor_status >> 1) & 1));
        printf("%d%d ", ((sensor_status >> 0) & 1), ((sensor_status >> 1) & 1));
        if (i == 4) { printf("\n"); }
    }
    delay(1);

    /* ================================= Move preloaded ================================= */
    // uint8_t  drv0 = 0xF;
    // uint8_t  drv1 = 0xF;
    // uint16_t rb0;
    // uint16_t rb1;

    // bsSb1.q_movePreloaded(13, &drv0);
    // bsSb1.q_movePreloaded(8, &drv1);
    // bsSb1.getQueueReadback(drv0, &rb0);
    // bsSb1.getQueueReadback(drv1, &rb1);

    // printf("\nYee\n");
    // printf("rb0 -> %s\n", toString_homingCode(static_cast<HomingCode>(rb0)));
    // printf("rb1 -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));

    // bsSb1.q_movePreloaded(14, &drv0);
    // bsSb1.q_movePreloaded(7, &drv1);
    // bsSb1.getQueueReadback(drv0, &rb0);
    // bsSb1.getQueueReadback(drv1, &rb1);

    // printf("\nHaw\n");
    // printf("rb0 -> %s\n", toString_homingCode(static_cast<HomingCode>(rb0)));
    // printf("rb1 -> %s\n", toString_homingCode(static_cast<HomingCode>(rb1)));

    /* =================================== Fetch File =================================== */
    // for (int i = 0; i < MAX_STEPPERS; i++) {
    //     std::string *file_content = nullptr;
    //     bool         res          = bsSb1.readDriverConfig(i, &file_content);

    //     printf("\n-----%d----\n", i);
    //     if (res) {
    //         printf("%s", file_content->c_str());
    //         delete file_content; // free memory
    //     } else {
    //         printf("fetch failed");
    //     }
    //     printf("\n----------\n");
    // }
    // for (int i = 0; i < 31; i++) {
    //     std::string *file_content = nullptr;
    //     bool         res          = bsSb1.readMotionConfig(i, &file_content);

    //     printf("\n-----%d----\n", i);
    //     if (res) {
    //         printf("%s", file_content->c_str());
    //         delete file_content; // free memory
    //     } else {
    //         printf("fetch failed");
    //     }
    //     printf("\n----------\n");
    // }
    // for (int i = 0; i < 15; i++) {
    //     std::string *file_content = nullptr;
    //     bool         res          = bsSb1.readSequenceConfig(i, &file_content);

    //     printf("\n-----%d----\n", i);
    //     if (res) {
    //         printf("%s", file_content->c_str());
    //         delete file_content; // free memory
    //     } else {
    //         printf("fetch failed");
    //     }
    //     printf("\n----------\n");
    // }

    /* ======================== SD Card Read and Write with Path ======================== */
    // std::string  path                    = "config/testing123";
    // std::string  write_content           = "abc testing 123\n321 gnitset cba\n";
    // std::string *read_path               = nullptr;
    // std::string *read_content            = nullptr;
    // std::string *read_file_content       = nullptr;
    // std::string *read_file_content_again = nullptr;

    // if (bsSb1.writeToFileAtPath(&path, &write_content)) {
    //     if (bsSb1.readBuffer(&read_content)) {
    //         printf("%s", read_content->c_str());
    //         printf("\n");
    //     }
    //     if (bsSb1.readPath(&read_path)) {
    //         printf("%s", read_path->c_str());
    //         printf("\n");
    //     }
    //     if (bsSb1.readFileAtPath(&path, &read_file_content)) {
    //         printf("%s", read_file_content->c_str());
    //         printf("\n");
    //     }
    //     bool deleted = bsSb1.deleteFileAtPath(&path);
    //     printf("deleted -> %d", deleted);

    //     if (bsSb1.readFileAtPath(&path, &read_file_content_again)) {
    //         printf("%s", read_file_content_again->c_str());
    //         printf("\n");
    //     }
    // }

    // delete read_path;
    // delete read_content;
    // delete read_file_content;
    // delete read_file_content_again;

    /* ================================= Test Queue Item ================================ */
    // bsStepperBoard::QueueItemParameters qip;
    // qip.motion_type      = 0;
    // qip.driver_num       = 1;
    // qip.position         = 360;
    // qip.use_inverse_time = 1;
    // qip.time_ms          = 2000;

    // if (bsSb1.writeTestQueueItem(qip) && bsSb1.mq_startTestQueueItem()) {
    //     MotionQueueRes res;
    //     if (bsSb1.getMotionQueueReadback(&res)) {
    //         printf("Test Item Res -> %s\n", toString_motionQueueRes(res));
    //     } else {
    //         printf("Test Item Res -> Failed to get readback\n");
    //     }
    // }

    // bsStepperBoard::QueueItemParameters qip2;
    // qip2.motion_type      = 0;
    // qip2.driver_num       = 1;
    // qip2.position         = -360;
    // qip2.use_inverse_time = 1;
    // qip2.time_ms          = 2000;

    // if (bsSb1.writeTestQueueItem(qip2) && bsSb1.mq_startTestQueueItem()) {
    //     MotionQueueRes res;
    //     if (bsSb1.getMotionQueueReadback(&res)) {
    //         printf("Test Item Res -> %s\n", toString_motionQueueRes(res));
    //     } else {
    //         printf("Test Item Res -> Failed to get readback\n");
    //     }
    // }

    // delay(100);
    /* ---------------------------------------------------------------------------------- */
}
