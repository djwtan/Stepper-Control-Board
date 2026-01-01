/**
 * \file
 *
 * \brief User board definition template
 *
 */

/* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include "compiler.h"
#include "exceptions.h"
#include "system_sam4e.h"
#include <conf_board.h>

/* ================================================================================== */
/*                                    BOARD SELECT                                    */
/* ================================================================================== */
// #define XPLAINED // Uncomment if using SAM4E XPlained

/*----------------------------------------------------------------------------*/

/** Board oscillator settings */
#define BOARD_FREQ_SLCK_XTAL     (32768U)
#define BOARD_FREQ_SLCK_BYPASS   (32768U)
#define BOARD_FREQ_MAINCK_XTAL   (12000000U)
#define BOARD_FREQ_MAINCK_BYPASS (12000000U)

/** Master clock frequency */
#define BOARD_MCK                CHIP_FREQ_CPU_MAX

/** board main clock xtal statup time */
#define BOARD_OSC_STARTUP_US     15625

/*----------------------------------------------------------------------------*/
/** Name of the board */
#define BOARD_NAME               "JSTEPPER"
/** Family definition */
#define sam4e
/** Core definition */
#define cortexm4

/*----------------------------------------------------------------------------*/
/** UART0 pins (UTXD0 and URXD0) definitions, PA10,9. */
#define PINS_UART0           (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0)
#define PINS_UART0_FLAGS     (IOPORT_MODE_MUX_A)

#define PINS_UART0_PORT      IOPORT_PIOA
#define PINS_UART0_MASK      (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0)
#define PINS_UART0_PIO       PIOA
#define PINS_UART0_ID        ID_PIOA
#define PINS_UART0_TYPE      PIO_PERIPH_A
#define PINS_UART0_ATTR      PIO_DEFAULT

#define CONSOLE_UART         UART0
#define CONSOLE_UART_ID      ID_UART0

/*----------------------------------------------------------------------------*/
/**
 * CAN
 * - \ref PIN_CAN1_TXD
 * - \ref PIN_CAN1_RXD
 */

/** CAN1 PIN RX. */
#define PIN_CAN1_RX_IDX      PIO_PC12_IDX
#define PIN_CAN1_RX_FLAGS    IOPORT_MODE_MUX_C

/** CAN1 PIN TX. */
#define PIN_CAN1_TX_IDX      PIO_PC15_IDX
#define PIN_CAN1_TX_FLAGS    IOPORT_MODE_MUX_C

/*----------------------------------------------------------------------------*/
/*
 * SPI
 * Used by 5x TMC4361A Motion Controller
 */
#define SPI_BUS              SPI

/** SPI MISO pin definition. */
#define SPI_MISO_GPIO        (PIO_PA12_IDX)
#define SPI_MISO_FLAGS       (IOPORT_MODE_MUX_A)
/** SPI MOSI pin definition. */
#define SPI_MOSI_GPIO        (PIO_PA13_IDX)
#define SPI_MOSI_FLAGS       (IOPORT_MODE_MUX_A)
/** SPI SPCK pin definition. */
#define SPI_SPCK_GPIO        (PIO_PA14_IDX)
#define SPI_SPCK_FLAGS       (IOPORT_MODE_MUX_A)

/** SPI chip select 0 pin definition. (Only one configuration is possible) */
#define SPI_NPCS0_PA11_GPIO  (PIO_PA11_IDX)
#define SPI_NPCS0_PA11_FLAGS (IOPORT_MODE_MUX_A)

/** SPI chip select 1 pin definition. (multiple configurations are possible) */
#define SPI_NPCS1_PA9_GPIO   (PIO_PA9_IDX)
#define SPI_NPCS1_PA9_FLAGS  (IOPORT_MODE_MUX_B)
#define SPI_NPCS1_PA31_GPIO  (PIO_PA31_IDX)
#define SPI_NPCS1_PA31_FLAGS (IOPORT_MODE_MUX_A)
#define SPI_NPCS1_PB14_GPIO  (PIO_PB14_IDX)
#define SPI_NPCS1_PB14_FLAGS (IOPORT_MODE_MUX_A)
#define SPI_NPCS1_PC4_GPIO   (PIO_PC4_IDX)
#define SPI_NPCS1_PC4_FLAGS  (IOPORT_MODE_MUX_B)

/** SPI chip select 2 pin definition. (multiple configurations are possible) */
#define SPI_NPCS2_PA10_GPIO  (PIO_PA10_IDX)
#define SPI_NPCS2_PA10_FLAGS (IOPORT_MODE_MUX_B)
#define SPI_NPCS2_PA30_GPIO  (PIO_PA30_IDX)
#define SPI_NPCS2_PA30_FLAGS (IOPORT_MODE_MUX_B)
#define SPI_NPCS2_PB2_GPIO   (PIO_PB2_IDX)
#define SPI_NPCS2_PB2_FLAGS  (IOPORT_MODE_MUX_B)

/** SPI chip select 3 pin definition. (multiple configurations are possible) */
#define SPI_NPCS3_PA3_GPIO   (PIO_PA3_IDX)
#define SPI_NPCS3_PA3_FLAGS  (IOPORT_MODE_MUX_B)
#define SPI_NPCS3_PA5_GPIO   (PIO_PA5_IDX)
#define SPI_NPCS3_PA5_FLAGS  (IOPORT_MODE_MUX_B)
#define SPI_NPCS3_PA22_GPIO  (PIO_PA22_IDX)
#define SPI_NPCS3_PA22_FLAGS (IOPORT_MODE_MUX_B)

/** Software SPI */
#define SPI_SW_PA16_GPIO     PIO_PA16_IDX

/* ======================================= SPI ====================================== */
#define IRQ_PIN              IOPORT_CREATE_PIN(PIOA, 11)
#define SPI_MOSI             IOPORT_CREATE_PIN(PIOA, 13)
#define SPI_MISO             IOPORT_CREATE_PIN(PIOA, 12)
#define SPI_SCK              IOPORT_CREATE_PIN(PIOA, 14)

#ifdef XPLAINED
#  define DRIVER_0_CS_4361 SPI_NPCS1_PB14_GPIO
#else
#  define DRIVER_0_CS_4361 SPI_NPCS1_PC4_GPIO
#endif
#define DRIVER_1_CS_4361 SPI_NPCS0_PA11_GPIO
#define DRIVER_2_CS_4361 SPI_NPCS2_PB2_GPIO
#define DRIVER_3_CS_4361 SPI_NPCS3_PA22_GPIO
#define DRIVER_4_CS_4361 SPI_SW_PA16_GPIO

// !Not in use (using software cs)
#ifdef XPLAINED
#  define DRIVER_0_SPI_CS_FLAGS SPI_NPCS1_PB14_FLAGS
#else
#  define DRIVER_0_SPI_CS_FLAGS SPI_NPCS1_PC4_FLAGS
#endif
#define DRIVER_1_SPI_CS_FLAGS SPI_NPCS0_PA11_FLAGS
#define DRIVER_2_SPI_CS_FLAGS SPI_NPCS2_PB2_FLAGS
#define DRIVER_3_SPI_CS_FLAGS SPI_NPCS3_PA22_FLAGS

// !Not in use (using software cs)
/** Chip select number */
#define MC0_SPI_CS_ID         1
#define MC1_SPI_CS_ID         0
#define MC2_SPI_CS_ID         2
#define MC3_SPI_CS_ID         3
#define MC4_SPI_CS_ID         4 // Invalid

/*----------------------------------------------------------------------------*/
/*
 * TMC4361A + TMC5160
 */

#ifdef XPLAINED
#  define FCLK 6000000 // EXPLAINED
#else
#  define FCLK 16000000
#endif

/** NFREEZE pin */
#ifdef XPLAINED
#  define DRIVER_0_NFREEZE_4361 PIO_PD25_IDX // EXPLAINED
#else
#  define DRIVER_0_NFREEZE_4361 PIO_PD23_IDX
#endif
#define DRIVER_1_NFREEZE_4361 PIO_PD19_IDX
#define DRIVER_2_NFREEZE_4361 PIO_PA0_IDX
#define DRIVER_3_NFREEZE_4361 PIO_PD8_IDX
#define DRIVER_4_NFREEZE_4361 PIO_PC23_IDX

/** NRST pin */
#ifdef XPLAINED
#  define RST_TMC43610 PIO_PA24_IDX // EXPLAINED
#else
#  define RST_TMC43610 PIO_PD24_IDX
#endif
#define RST_TMC43611 PIO_PD21_IDX
#define RST_TMC43612 PIO_PC16_IDX
#define RST_TMC43613 PIO_PD29_IDX
#define RST_TMC43614 PIO_PC24_IDX

/** INTR pin */
#ifdef XPLAINED
#  define DRIVER_0_INTR PIO_PD23_IDX // EXPLAINED
#else
#  define DRIVER_0_INTR PIO_PA24_IDX
#endif
#define DRIVER_1_INTR             PIO_PD20_IDX
#define DRIVER_2_INTR             PIO_PD10_IDX
#define DRIVER_3_INTR             PIO_PC18_IDX
#define DRIVER_4_INTR             PIO_PD3_IDX

#define DRIVER_0_INTR_MASK        1 << (DRIVER_0_INTR % 32)
#define DRIVER_1_INTR_MASK        1 << (DRIVER_1_INTR % 32)
#define DRIVER_2_INTR_MASK        1 << (DRIVER_2_INTR % 32)
#define DRIVER_3_INTR_MASK        1 << (DRIVER_3_INTR % 32)
#define DRIVER_4_INTR_MASK        1 << (DRIVER_4_INTR % 32)

/** What is this? */
#define DRIVER_0_5160DIG0         PIO_PC5_IDX
#define DRIVER_1_5160DIG0         PIO_PD18_IDX
#define DRIVER_2_5160DIG0         PIO_PC17_IDX
#define DRIVER_3_5160DIG0         PIO_PC19_IDX
#define DRIVER_4_5160DIG0         PIO_PD4_IDX

#define DRIVER_0_5160DIG1         PIO_PA25_IDX
#define DRIVER_1_5160DIG2         PIO_PD28_IDX
#define DRIVER_2_5160DIG3         PIO_PB4_IDX
#define DRIVER_3_5160DIG4         PIO_PD7_IDX
#define DRIVER_4_5160DIG5         PIO_PD5_IDX

/** 5160 Enable */
#define DRIVER_0_5160_EN          PIO_PC6_IDX
#define DRIVER_1_5160_EN          PIO_PD22_IDX
#define DRIVER_2_5160_EN          PIO_PA1_IDX
#define DRIVER_3_5160_EN          PIO_PD15_IDX
#define DRIVER_4_5160_EN          PIO_PD2_IDX

/* ===================================== TMC5160 ==================================== */
#define MAX_STEPPERS              5 // Maximum number of steppers supported

/** Documented RSense values on RMS Current */
#define RMS_MAX_RSENSE_220MILLI   1100.0
#define RMS_MAX_RSENSE_150MILLI   1600.0
#define RMS_MAX_RSENSE_120MILLI   2000.0
#define RMS_MAX_RSENSE_100MILLI   2300.0
#define RMS_MAX_RSENSE_75MILLI    3100.0
#define RMS_MAX_RSENSE_66MILLI    3500.0
#define RMS_MAX_RSENSE_50MILLI    4700.0
#define RMS_MAX_RSENSE_33MILLI    7100.0
#define RMS_MAX_RSENSE_22MILLI    10600.0

#define POWER_SEQUENCING_DELAY_MS 10
#define FUSE_VALUE_MA             15000.0 // In mA. For current limiting
#define DRIVER_RMS_MAX            RMS_MAX_RSENSE_50MILLI

/* ====================================== HSMCI ===================================== */
#define SD_MMC_HSMCI_MEM_CNT      1
#define SD_MMC_HSMCI_SLOT_0_SIZE  4
#define PINS_HSMCI                {0x3fUL << 26, PIOA, ID_PIOA, PIO_PERIPH_C, PIO_PULLUP}

#define PIN_HSMCI_MCCDA_GPIO      (PIO_PA28_IDX) /* CMD */
#define PIN_HSMCI_MCCDA_FLAGS     (IOPORT_MODE_MUX_C | IOPORT_MODE_PULLUP)
#define PIN_HSMCI_MCCK_GPIO       (PIO_PA29_IDX)      /* CLK */
#define PIN_HSMCI_MCCK_FLAGS      (IOPORT_MODE_MUX_C) /* no pull-up on clock */
#define PIN_HSMCI_MCDA0_GPIO      (PIO_PA30_IDX)
#define PIN_HSMCI_MCDA0_FLAGS     (IOPORT_MODE_MUX_C | IOPORT_MODE_PULLUP)
#define PIN_HSMCI_MCDA1_GPIO      (PIO_PA31_IDX)
#define PIN_HSMCI_MCDA1_FLAGS     (IOPORT_MODE_MUX_C | IOPORT_MODE_PULLUP)
#define PIN_HSMCI_MCDA2_GPIO      (PIO_PA26_IDX)
#define PIN_HSMCI_MCDA2_FLAGS     (IOPORT_MODE_MUX_C | IOPORT_MODE_PULLUP)
#define PIN_HSMCI_MCDA3_GPIO      (PIO_PA27_IDX)
#define PIN_HSMCI_MCDA3_FLAGS     (IOPORT_MODE_MUX_C | IOPORT_MODE_PULLUP)
#define SD_MMC_ENABLE

#endif // USER_BOARD_H
