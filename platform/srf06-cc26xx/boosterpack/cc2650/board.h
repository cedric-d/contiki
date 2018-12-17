/*
 * Copyright (c) 2015, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/** \addtogroup boosterpack-peripherals
 * @{
 *
 * \defgroup boosterpack-cc26xx-specific CC2650 BoosterPack Peripherals
 *
 * Defines related to the CC2650 BoosterPack.
 *
 * This file provides connectivity information on LEDs, UART and other
 * peripherals.
 *
 * This file is not meant to be modified by the user.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the TI
 * CC2650 BoosterPack.
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
/*---------------------------------------------------------------------------*/
#ifndef BOARD_H_
#define BOARD_H_
/*---------------------------------------------------------------------------*/
#include "ioc.h"
/*---------------------------------------------------------------------------*/
/**
 * \name LED configurations
 *
 * Those values are not meant to be modified by the user
 * @{
 */
/* Some files include leds.h before us, so we need to get rid of defaults in
 * leds.h before we provide correct definitions */
#undef LEDS_GREEN
#undef LEDS_YELLOW
#undef LEDS_RED
#undef LEDS_CONF_ALL

#define LEDS_RED       1
#define LEDS_GREEN     2
#define LEDS_YELLOW    LEDS_GREEN
#define LEDS_ORANGE    LEDS_RED

#define LEDS_CONF_ALL  3

/* Notify various examples that we have LEDs */
#define PLATFORM_HAS_LEDS        1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_LED_1          IOID_4
#define BOARD_IOID_LED_2          IOID_2
#define BOARD_LED_1               (1 << BOARD_IOID_LED_1)
#define BOARD_LED_2               (1 << BOARD_IOID_LED_2)
#define BOARD_LED_ALL             (BOARD_LED_1 | BOARD_LED_2)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name UART IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_UART_RX        IOID_1
#define BOARD_IOID_UART_TX        IOID_0
#define BOARD_IOID_UART_CTS       IOID_UNUSED
#define BOARD_IOID_UART_RTS       IOID_UNUSED
#define BOARD_UART_RX             (1 << BOARD_IOID_UART_RX)
#define BOARD_UART_TX             (1 << BOARD_IOID_UART_TX)
#define BOARD_UART_CTS            (1 << BOARD_IOID_UART_CTS)
#define BOARD_UART_RTS            (1 << BOARD_IOID_UART_RTS)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Button configuration
 *
 * Those values are not meant to be modified by the user
 * @{
 */
/* Notify various examples that we do not have buttons */
#undef PLATFORM_HAS_BUTTON
#define PLATFORM_HAS_BUTTON      0
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief SPI IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_SPI_SCK        IOID_10
#define BOARD_IOID_SPI_MOSI       IOID_11
#define BOARD_IOID_SPI_MISO       IOID_12
#define BOARD_SPI_SCK             (1 << BOARD_IOID_SPI_SCK)
#define BOARD_SPI_MOSI            (1 << BOARD_IOID_SPI_MOSI)
#define BOARD_SPI_MISO            (1 << BOARD_IOID_SPI_MISO)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief ROM bootloader configuration
 *
 * Change SET_CCFG_BL_CONFIG_BL_PIN_NUMBER to BOARD_IOID_KEY_xyz to select
 * which button triggers the bootloader on reset.
 *
 * The remaining values are not meant to be modified by the user
 * @{
 */
#if ROM_BOOTLOADER_ENABLE
#define SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE            0xC5
#define SET_CCFG_BL_CONFIG_BL_LEVEL                     0x00
#define SET_CCFG_BL_CONFIG_BL_PIN_NUMBER                IOID_UNUSED
#define SET_CCFG_BL_CONFIG_BL_ENABLE                    0xC5
#else
#define SET_CCFG_BL_CONFIG_BOOTLOADER_ENABLE            0x00
#define SET_CCFG_BL_CONFIG_BL_LEVEL                     0x01
#define SET_CCFG_BL_CONFIG_BL_PIN_NUMBER                0xFF
#define SET_CCFG_BL_CONFIG_BL_ENABLE                    0xFF
#endif
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Remaining pins
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_DIO3           IOID_3
#define BOARD_IOID_DIO5           IOID_5
#define BOARD_IOID_DIO6           IOID_6
#define BOARD_IOID_MRDY           IOID_7
#define BOARD_IOID_SRDY           IOID_8
#define BOARD_IOID_CS             IOID_9
#define BOARD_IOID_DIO13          IOID_13
#define BOARD_IOID_DIO14          IOID_14

#define BOARD_UNUSED_PINS { \
    BOARD_IOID_DIO3, BOARD_IOID_DIO5, BOARD_IOID_DIO6, BOARD_IOID_MRDY, \
    BOARD_IOID_SRDY, BOARD_IOID_CS, BOARD_IOID_DIO13, BOARD_IOID_DIO14, \
    IOID_UNUSED \
  }
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name RF Front End configuration
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define RF_CORE_CONF_RADIO_SETUP_FRONT_END_MODE 0x0     /* Differential mode */
#define RF_CORE_CONF_RADIO_SETUP_BIAS_MODE      0x1     /* External bias */
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "TI CC2650 BoosterPack"

/** @} */
/*---------------------------------------------------------------------------*/
#endif /* BOARD_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
