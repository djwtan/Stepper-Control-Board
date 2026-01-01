/**
 * \file
 *
 * \brief User board initialization template
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "ioport.h"
#include <asf.h>
#include <board.h>
#include <conf_board.h>

/**
 * \brief Set peripheral mode for IOPORT pins.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param port IOPORT port to configure
 * \param masks IOPORT pin masks to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_port_peripheral_mode(port, masks, mode)                                                             \
  do {                                                                                                                 \
    ioport_set_port_mode(port, masks, mode);                                                                           \
    ioport_disable_port(port, masks);                                                                                  \
  } while (0)

/**
 * \brief Set peripheral mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 */
#define ioport_set_pin_peripheral_mode(pin, mode)                                                                      \
  do {                                                                                                                 \
    ioport_set_pin_mode(pin, mode);                                                                                    \
    ioport_disable_pin(pin);                                                                                           \
  } while (0)

/**
 * \brief Set input mode for one single IOPORT pin.
 * It will configure port mode and disable pin mode (but enable peripheral).
 * \param pin IOPORT pin to configure
 * \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
 * \param sense Sense for interrupt detection (\ref ioport_sense)
 */
#define ioport_set_pin_input_mode(pin, mode, sense)                                                                    \
  do {                                                                                                                 \
    ioport_set_pin_dir(pin, IOPORT_DIR_INPUT);                                                                         \
    ioport_set_pin_mode(pin, mode);                                                                                    \
    ioport_set_pin_sense_mode(pin, sense);                                                                             \
  } while (0)

void board_init(void) {
  /* Initialize IOPORTs */
  ioport_init();

#ifdef CONF_BOARD_UART_CONSOLE
  /* Configure UART pins */
  ioport_set_port_peripheral_mode(PINS_UART0_PORT, PINS_UART0, PINS_UART0_FLAGS);
#endif

#ifdef CONF_BOARD_CAN
  /* Configure CAN1 TX and RX pin. */
  ioport_set_pin_peripheral_mode(PIN_CAN1_RX_IDX, PIN_CAN1_RX_FLAGS);
  ioport_set_pin_peripheral_mode(PIN_CAN1_TX_IDX, PIN_CAN1_TX_FLAGS);
#endif

#ifdef CONF_BOARD_SPI
  /* Configure SPI Master, Slave, and Clock */
  ioport_set_pin_peripheral_mode(SPI_MISO, SPI_MISO_FLAGS);
  ioport_set_pin_peripheral_mode(SPI_MOSI, SPI_MOSI_FLAGS);
  ioport_set_pin_peripheral_mode(SPI_SCK, SPI_SPCK_FLAGS);

  ioport_set_pin_dir(DRIVER_0_CS_4361, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(DRIVER_1_CS_4361, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(DRIVER_2_CS_4361, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(DRIVER_3_CS_4361, IOPORT_DIR_OUTPUT);
  ioport_set_pin_dir(DRIVER_4_CS_4361, IOPORT_DIR_OUTPUT);
  ioport_set_pin_level(DRIVER_0_CS_4361, true);
  ioport_set_pin_level(DRIVER_1_CS_4361, true);
  ioport_set_pin_level(DRIVER_2_CS_4361, true);
  ioport_set_pin_level(DRIVER_3_CS_4361, true);
  ioport_set_pin_level(DRIVER_4_CS_4361, true);
#endif

#ifdef CONF_BOARD_SD_MMC_HSMCI
  /* Configure HSMCI pins */
  ioport_set_pin_peripheral_mode(PIN_HSMCI_MCCDA_GPIO, PIN_HSMCI_MCCDA_FLAGS);
  ioport_set_pin_peripheral_mode(PIN_HSMCI_MCCK_GPIO, PIN_HSMCI_MCCK_FLAGS);
  ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA0_GPIO, PIN_HSMCI_MCDA0_FLAGS);
  ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA1_GPIO, PIN_HSMCI_MCDA1_FLAGS);
  ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA2_GPIO, PIN_HSMCI_MCDA2_FLAGS);
  ioport_set_pin_peripheral_mode(PIN_HSMCI_MCDA3_GPIO, PIN_HSMCI_MCDA3_FLAGS);
#endif
}
