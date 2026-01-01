/*
 * JStepperBoard.cpp
 *
 * Created: 30/09/2025 11:44:59
 * Author : Tan
 */

extern "C" {
#include "asf.h"
#include "conf_access.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "stdio_serial.h"
#include <string.h>

void vApplicationMallocFailedHook(void) {}
}

#include "board_test.h"
#include "can_interface.h"
#include "motion_queue.h"
#include "parser.h"
#include "sd_card.h"
#include "stepper.h"
#include "version.h"

void disable_watchdog(void) {
    // Disable the watchdog
    WDT->WDT_MR = WDT_MR_WDDIS;
}

/* ====================================== Init ====================================== */
xQueueHandle SPI_MUTEX = xSemaphoreCreateMutex();

Stepper      *steppers[MAX_STEPPERS] = {nullptr};
CANInterface *can_interface          = nullptr;
SDCard        sd_card;
MotionQueue   motion_queue;

/* ====================================== UART ====================================== */
static void configure_console(void) {
    usart_serial_options_t uart_serial_options = {CONF_UART_BAUDRATE, 8, CONF_UART_PARITY, 1};

    sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
    stdio_serial_init(CONF_UART, &uart_serial_options);
}

/* ======================================= SPI ====================================== */
static void configure_spi(void) {
    static struct spi_device DUMMY;
    spi_master_init(SPI_BUS);
    spi_master_setup_device(SPI_BUS, &DUMMY, SPI_MODE_3, 1000000, 0);
    spi_enable(SPI_BUS);
}

/* ==================================== Steppers ==================================== */
static void configure_steppers(void) {
    steppers[0] = new Stepper(SPI_BUS, &SPI_MUTEX, DRIVER_0_CS_4361, FCLK, DRIVER_0_NFREEZE_4361, RST_TMC43610,
                              DRIVER_0_INTR, DRIVER_0_5160_EN);
    steppers[1] = new Stepper(SPI_BUS, &SPI_MUTEX, DRIVER_1_CS_4361, FCLK, DRIVER_1_NFREEZE_4361, RST_TMC43611,
                              DRIVER_1_INTR, DRIVER_1_5160_EN);
    steppers[2] = new Stepper(SPI_BUS, &SPI_MUTEX, DRIVER_2_CS_4361, FCLK, DRIVER_2_NFREEZE_4361, RST_TMC43612,
                              DRIVER_2_INTR, DRIVER_2_5160_EN);
    steppers[3] = new Stepper(SPI_BUS, &SPI_MUTEX, DRIVER_3_CS_4361, FCLK, DRIVER_3_NFREEZE_4361, RST_TMC43613,
                              DRIVER_3_INTR, DRIVER_3_5160_EN);
    steppers[4] = new Stepper(SPI_BUS, &SPI_MUTEX, DRIVER_4_CS_4361, FCLK, DRIVER_4_NFREEZE_4361, RST_TMC43614,
                              DRIVER_4_INTR, DRIVER_4_5160_EN);
}

/* ======================================= CAN ====================================== */
static void configure_can(void) {
    can_interface = new CANInterface(1);

    can_interface->registerSteppers(steppers);
    can_interface->registerMotionQueue(&motion_queue);
    can_interface->registerSdCard(&sd_card);

    can_interface->init();
}

/* ==================================== Interrupt =================================== */
// !Not quite working yet
static void interruptHandler_portA(uint32_t id, uint32_t mask) {
    // printf("Port A Interrupt: ID %lu, Mask %lu\n", id, mask);
    if (mask & DRIVER_0_INTR_MASK)
        if (steppers[0] != nullptr) steppers[0]->raiseStallFlag();
}
static void interruptHandler_portC(uint32_t id, uint32_t mask) {
    // printf("Port C Interrupt: ID %lu, Mask %lu\n", id, mask);
    if (mask & DRIVER_3_INTR_MASK)
        if (steppers[3] != nullptr) steppers[3]->raiseStallFlag();
}
static void interruptHandler_portD(uint32_t id, uint32_t mask) {
    // printf("Port D Interrupt: ID %lu, Mask %lu\n", id, mask);
    if (mask & DRIVER_1_INTR_MASK)
        if (steppers[1] != nullptr) steppers[1]->raiseStallFlag();
    if (mask & DRIVER_2_INTR_MASK)
        if (steppers[2] != nullptr) steppers[2]->raiseStallFlag();
    if (mask & DRIVER_4_INTR_MASK)
        if (steppers[4] != nullptr) steppers[4]->raiseStallFlag();
}

static void configure_interrupt(void) {
    pmc_enable_periph_clk(ID_PIOA);
    pmc_enable_periph_clk(ID_PIOC);
    pmc_enable_periph_clk(ID_PIOD);

    pio_set_debounce_filter(PIOA, DRIVER_0_INTR_MASK, 10);
    pio_set_debounce_filter(PIOD, DRIVER_1_INTR_MASK, 10);
    pio_set_debounce_filter(PIOD, DRIVER_2_INTR_MASK, 10);
    pio_set_debounce_filter(PIOC, DRIVER_3_INTR_MASK, 10);
    pio_set_debounce_filter(PIOD, DRIVER_4_INTR_MASK, 10);

    pio_set_input(PIOA, DRIVER_0_INTR_MASK, PIO_DEFAULT);
    pio_set_input(PIOD, DRIVER_1_INTR_MASK, PIO_DEFAULT);
    pio_set_input(PIOD, DRIVER_2_INTR_MASK, PIO_DEFAULT);
    pio_set_input(PIOC, DRIVER_3_INTR_MASK, PIO_DEFAULT);
    pio_set_input(PIOD, DRIVER_4_INTR_MASK, PIO_DEFAULT);

    pio_handler_set(PIOA, ID_PIOA, DRIVER_0_INTR_MASK, PIO_IT_FALL_EDGE, interruptHandler_portA);
    pio_handler_set(PIOD, ID_PIOD, DRIVER_1_INTR_MASK, PIO_IT_FALL_EDGE, interruptHandler_portD);
    pio_handler_set(PIOD, ID_PIOD, DRIVER_2_INTR_MASK, PIO_IT_FALL_EDGE, interruptHandler_portD);
    pio_handler_set(PIOC, ID_PIOC, DRIVER_3_INTR_MASK, PIO_IT_FALL_EDGE, interruptHandler_portC);
    pio_handler_set(PIOD, ID_PIOD, DRIVER_4_INTR_MASK, PIO_IT_FALL_EDGE, interruptHandler_portD);

    pio_configure_interrupt(PIOA, DRIVER_0_INTR_MASK, PIO_IT_FALL_EDGE);
    pio_configure_interrupt(PIOD, DRIVER_1_INTR_MASK, PIO_IT_FALL_EDGE);
    pio_configure_interrupt(PIOD, DRIVER_2_INTR_MASK, PIO_IT_FALL_EDGE);
    pio_configure_interrupt(PIOC, DRIVER_3_INTR_MASK, PIO_IT_FALL_EDGE);
    pio_configure_interrupt(PIOD, DRIVER_4_INTR_MASK, PIO_IT_FALL_EDGE);

    NVIC_EnableIRQ(PIOA_IRQn);
    NVIC_EnableIRQ(PIOC_IRQn);
    NVIC_EnableIRQ(PIOD_IRQn);

    pio_enable_pin_interrupt(DRIVER_0_INTR);
    pio_enable_pin_interrupt(DRIVER_1_INTR);
    pio_enable_pin_interrupt(DRIVER_2_INTR);
    pio_enable_pin_interrupt(DRIVER_3_INTR);
    pio_enable_pin_interrupt(DRIVER_4_INTR);
}

/* ===================================== SD Card ==================================== */
static void configure_sdcard(void) {
    /* Initialize SD MMC stack */
    sd_mmc_init();
}

/* ================================== Motion Queue ================================== */
static void configure_motionQueue(void) { motion_queue.registerSteppers(steppers); }

/* ========================== Initial Loading from SD Card ========================== */
static void initializeDriver(void) {
    uint8_t drivers_loaded = 0;

    sd_card.mount();
    drivers_loaded = parse_driverConfig(&sd_card, steppers);
    parse_motionSequence(&sd_card, &motion_queue);
    sd_card.unmount();

    // First Initialize
    for (int i; i < MAX_STEPPERS; i++) {
        if ((drivers_loaded >> i) & 1 == 1) { steppers[i]->execInitController(); }
    }
}

/* ================================================================================== */
/*                                        Main                                        */
/* ================================================================================== */
testItem *s0;
testItem *s1;
testItem *s2;
testItem *s3;
testItem *s4;

testMotionQueue test;

int main(void) {
    disable_watchdog();
    SystemInit();

    sysclk_init();
    board_init();
    configure_sdcard();
    configure_console();
    configure_spi();
    configure_steppers();
    configure_motionQueue();
    initializeDriver();

#ifdef BOARD_TEST_H_
#    ifdef RUN_THROUGH_CAN
    configure_can();

    // #  elif RUN_TEST == test_motionQueue
    //   test.stepper_ptr[0] = stepper0;
    //   test.stepper_ptr[1] = stepper1;
    //   test.stepper_ptr[2] = stepper2;
    //   test.stepper_ptr[3] = stepper3;
    //   test.stepper_ptr[4] = stepper4;
    //   test.motion_queue   = &motion_queue;
    //   xTaskCreate(test_motionQueue, (const signed char *)"Test0", 1000, &test, 2, NULL); // create test for drv0

#    else
    /* ================================== Stepper Test ================================== */
    /** Available Tests */
    // test_moveRelative
    // test_moveRelative_inverseTime
    // test_moveVelocity
    // test_moveVibration
    s0 = new testItem{stepper[0], 0};
    s1 = new testItem{stepper[1], 1};
    s2 = new testItem{stepper[2], 2};
    s3 = new testItem{stepper[3], 3};
    s4 = new testItem{stepper[4], 4};

#        ifdef DRV0
    xTaskCreate(RUN_TEST, (const signed char *)"Test0", 496, s0, 2, NULL); // create test for drv0
#        endif
#        ifdef DRV1
    xTaskCreate(RUN_TEST, (const signed char *)"Test1", 496, s1, 2, NULL); // create test for drv2
#        endif
#        ifdef DRV2
    xTaskCreate(RUN_TEST, (const signed char *)"Test2", 496, s2, 2, NULL); // create test for drv1
#        endif
#        ifdef DRV3
    xTaskCreate(RUN_TEST, (const signed char *)"Test3", 496, s3, 2, NULL); // create test for drv3
#        endif
#        ifdef DRV4
    xTaskCreate(RUN_TEST, (const signed char *)"Test4", 496, s4, 2, NULL); // create test for drv4
#        endif
#    endif
#else
    configure_can();
#endif

    /* ==================================== Can Test ==================================== */
    // xTaskCreate(test_canTransmit, (const signed char *)"CanTransmit", 496, &can_interface, 2, NULL); // create test

    vTaskStartScheduler(); // start all tasks
}
