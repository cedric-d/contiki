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
/**
 * \addtogroup rf-core-ble
 * @{
 *
 * \file
 * Implementation of the CC13xx/CC26xx RF BLE driver
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "sys/process.h"
#include "sys/clock.h"
#include "sys/cc.h"
#include "sys/etimer.h"
#include "net/netstack.h"
#include "net/linkaddr.h"
#include "dev/oscillators.h"
#include "rf-core/rf-core.h"
#include "rf-core/rf-switch.h"
#include "rf-core/rf-ble.h"
#include "driverlib/rf_ble_cmd.h"
#include "driverlib/rf_common_cmd.h"
#include "ti-lib.h"
/*---------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* BLE Intervals: Send a burst of advertisements every BLE_ADV_INTERVAL secs */
#define BLE_ADV_INTERVAL      (CLOCK_SECOND * 5)
#define BLE_ADV_DUTY_CYCLE    (CLOCK_SECOND / 10)
#define BLE_ADV_MESSAGES      10

/* BLE Advertisement-related macros */
#define BLE_ADV_TYPE_DEVINFO      0x01
#define BLE_ADV_TYPE_NAME         0x09
#define BLE_ADV_TYPE_MANUFACTURER 0xFF
#define BLE_ADV_NAME_BUF_LEN        32
#define BLE_ADV_PAYLOAD_BUF_LEN     64
#define BLE_UUID_SIZE               16

/* Eddystone BLE Advertisement-related macros */
#define BLE_EDDYSTONE_UID_LENGTH 16
#define BLE_EDDYSTONE_URL_MAXLENGTH 17
/*---------------------------------------------------------------------------*/
static unsigned char ble_params_buf[32] CC_ALIGN(4);
static uint8_t ble_mode_on = RF_BLE_IDLE;
static struct etimer ble_adv_et;
/*---------------------------------------------------------------------------*/
static uint16_t tx_power = 0x9330;
static uint32_t ble_adv_count = 0;
/*---------------------------------------------------------------------------*/
/* BLE beacond config */
static struct ble_beacond_config {
  clock_time_t interval;
  clock_time_t duty_cycle;
  int burst_count;
  int payload_length;
  uint8_t payload[BLE_ADV_PAYLOAD_BUF_LEN];
} beacond_config = { .interval = BLE_ADV_INTERVAL,
                     .duty_cycle = BLE_ADV_DUTY_CYCLE,
                     .burst_count = BLE_ADV_MESSAGES,
                     .payload_length = 0 };
/*---------------------------------------------------------------------------*/
#ifdef RF_BLE_CONF_BOARD_OVERRIDES
#define RF_BLE_BOARD_OVERRIDES RF_BLE_CONF_BOARD_OVERRIDES
#else
#define RF_BLE_BOARD_OVERRIDES
#endif
/*---------------------------------------------------------------------------*/
/* BLE overrides */
static uint32_t ble_overrides[] = {
  0x00364038, /* Synth: Set RTRIM (POTAILRESTRIM) to 6 */
  0x000784A3, /* Synth: Set FREF = 3.43 MHz (24 MHz / 7) */
  0xA47E0583, /* Synth: Set loop bandwidth after lock to 80 kHz (K2) */
  0xEAE00603, /* Synth: Set loop bandwidth after lock to 80 kHz (K3, LSB) */
  0x00010623, /* Synth: Set loop bandwidth after lock to 80 kHz (K3, MSB) */
  0x00456088, /* Adjust AGC reference level */
  RF_BLE_BOARD_OVERRIDES
  0xFFFFFFFF, /* End of override list */
};
/*---------------------------------------------------------------------------*/
PROCESS(rf_ble_beacon_process, "CC13xx / CC26xx RF BLE Beacon Process");
/*---------------------------------------------------------------------------*/
static int
send_ble_adv_nc(int channel, uint8_t *adv_payload, int adv_payload_len)
{
  uint32_t cmd_status;
  rfc_CMD_BLE_ADV_NC_t cmd;
  rfc_bleAdvPar_t *params;

  params = (rfc_bleAdvPar_t *)ble_params_buf;

  /* Clear both buffers */
  memset(&cmd, 0x00, sizeof(cmd));
  memset(ble_params_buf, 0x00, sizeof(ble_params_buf));

  /* Adv NC */
  cmd.commandNo = CMD_BLE_ADV_NC;
  cmd.condition.rule = COND_NEVER;
  cmd.whitening.bOverride = 0;
  cmd.whitening.init = 0;
  cmd.pParams = params;
  cmd.channel = channel;

  /* Set up BLE Advertisement parameters */
  params->pDeviceAddress = (uint16_t *)&linkaddr_node_addr.u8[LINKADDR_SIZE - 2];
  params->endTrigger.triggerType = TRIG_NEVER;
  params->endTime = TRIG_NEVER;

  /* Set up BLE Advertisement parameters */
  params = (rfc_bleAdvPar_t *)ble_params_buf;
  params->advLen = adv_payload_len;
  params->pAdvData = adv_payload;

  if(rf_core_send_cmd((uint32_t)&cmd, &cmd_status) == RF_CORE_CMD_ERROR) {
    PRINTF("send_ble_adv_nc: Chan=%d CMDSTA=0x%08lx, status=0x%04x\n",
           channel, cmd_status, cmd.status);
    return RF_CORE_CMD_ERROR;
  }

  /* Wait until the command is done */
  if(rf_core_wait_cmd_done(&cmd) != RF_CORE_CMD_OK) {
    PRINTF("send_ble_adv_nc: Chan=%d CMDSTA=0x%08lx, status=0x%04x\n",
           channel, cmd_status, cmd.status);
    return RF_CORE_CMD_ERROR;
  }

  ble_adv_count++;

  return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
void
rf_ble_beacond_config(clock_time_t interval, const char *name)
{
  if(RF_BLE_ENABLED == 0) {
    return;
  }

  if(name != NULL) {
    int p;
    size_t namelen = strlen(name);
    if(namelen == 0 || namelen >= BLE_ADV_NAME_BUF_LEN) {
      return;
    }

    p = 0;
    memset(beacond_config.payload, 0, BLE_ADV_PAYLOAD_BUF_LEN);
    beacond_config.payload[p++] = 0x02; /* 2 bytes */
    beacond_config.payload[p++] = BLE_ADV_TYPE_DEVINFO;
    beacond_config.payload[p++] = 0x1a; /* LE general discoverable + BR/EDR */
    beacond_config.payload[p++] = 1 + namelen;
    beacond_config.payload[p++] = BLE_ADV_TYPE_NAME;
    memcpy(&beacond_config.payload[p], name, namelen);
    p += namelen;
    beacond_config.payload_length = p;
  }

  if(interval != 0) {
    beacond_config.interval = interval;
  }
}
/*---------------------------------------------------------------------------*/
void
rf_ble_beacond_config_raw(clock_time_t interval, const uint8_t *payload,
                          size_t payload_length)
{
  if(RF_BLE_ENABLED == 0) {
    return;
  }

  if(payload != NULL) {
    if(payload_length == 0 || payload_length > BLE_ADV_PAYLOAD_BUF_LEN) {
      return;
    }

    memcpy(beacond_config.payload, payload, payload_length);
    memset(beacond_config.payload + payload_length, 0,
           BLE_ADV_PAYLOAD_BUF_LEN - payload_length);
    beacond_config.payload_length = payload_length;
  }

  if(interval != 0) {
    beacond_config.interval = interval;
  }
}
/*---------------------------------------------------------------------------*/
void
rf_ble_beacond_config_eddystone_uid(clock_time_t interval,
                                    const uint8_t uid[BLE_EDDYSTONE_UID_LENGTH])
{
  if(RF_BLE_ENABLED == 0) {
    return;
  }

  if(uid != NULL) {
    int p;

    radio_value_t tx_level = 0;
    NETSTACK_RADIO.get_value(RADIO_PARAM_TXPOWER, &tx_level);

    p = 0;
    memset(beacond_config.payload, 0, BLE_ADV_PAYLOAD_BUF_LEN);

    beacond_config.payload[p++] = 0x02; /* 2 bytes */
    beacond_config.payload[p++] = 0x01; /* Flags */
    beacond_config.payload[p++] = 0x06; /* LE general discoverable without BR/EDR */

    beacond_config.payload[p++] = 0x03; /* 3 bytes */
    beacond_config.payload[p++] = 0x03; /* Complete list of 16-bit Service UUIDs */
    beacond_config.payload[p++] = 0xAA; /* 16-bit Eddystone UUID (LSB) */
    beacond_config.payload[p++] = 0xFE; /* 16-bit Eddystone UUID (MSB) */

    beacond_config.payload[p++] = 0x17; /* 23 bytes */
    beacond_config.payload[p++] = 0x16; /* Service Data */
    beacond_config.payload[p++] = 0xAA; /* 16-bit Eddystone UUID (LSB) */
    beacond_config.payload[p++] = 0xFE; /* 16-bit Eddystone UUID (MSB) */
    beacond_config.payload[p++] = 0x00; /* UID Frame */
    beacond_config.payload[p++] = tx_level; /* Calibrated Tx power at 0 m */
    memcpy(&beacond_config.payload[p], uid, BLE_EDDYSTONE_UID_LENGTH);
    p += BLE_EDDYSTONE_UID_LENGTH;
    beacond_config.payload[p++] = 0x00; /* Reserved for future use */
    beacond_config.payload[p++] = 0x00; /* Reserved for future use */

    beacond_config.payload_length = p;
  }

  if(interval != 0) {
    beacond_config.interval = interval;
  }
}
/*---------------------------------------------------------------------------*/
void
rf_ble_beacond_config_eddystone_url(clock_time_t interval, const char *url)
{
  static const char *eddystoneURLSchemePrefix[] = {
    "http://www.",  /* 0x00 */
    "https://www.", /* 0x01 */
    "http://",      /* 0x02 */
    "https://",     /* 0x03 */
  };
#define BLE_EDDYSTONE_URL_SCHEME_PREFIX_COUNT \
    (sizeof(eddystoneURLSchemePrefix) / sizeof(eddystoneURLSchemePrefix[0]))

  static const char *eddystoneURLEncoding[] = {
    ".com/",  /* 0x00 */
    ".org/",  /* 0x01 */
    ".edu/",  /* 0x02 */
    ".net/",  /* 0x03 */
    ".info/", /* 0x04 */
    ".biz/",  /* 0x05 */
    ".gov/",  /* 0x06 */
    ".com",   /* 0x07 */
    ".org",   /* 0x08 */
    ".edu",   /* 0x09 */
    ".net",   /* 0x0a */
    ".info",  /* 0x0b */
    ".biz",   /* 0x0c */
    ".gov",   /* 0x0d */
  };
#define BLE_EDDYSTONE_URL_ENCODING_COUNT \
    (sizeof(eddystoneURLEncoding) / sizeof(eddystoneURLEncoding[0]))

  if(RF_BLE_ENABLED == 0) {
    return;
  }

  if(url != NULL) {
    int i, p, c = 0;
    size_t len;

    radio_value_t tx_level = 0;
    NETSTACK_RADIO.get_value(RADIO_PARAM_TXPOWER, &tx_level);

    p = 0;
    memset(beacond_config.payload, 0, BLE_ADV_PAYLOAD_BUF_LEN);
    beacond_config.payload_length = 0;

    beacond_config.payload[p++] = 0x02; /* 2 bytes */
    beacond_config.payload[p++] = 0x01; /* Flags */
    beacond_config.payload[p++] = 0x06; /* LE general discoverable without BR/EDR */

    beacond_config.payload[p++] = 0x03; /* 3 bytes */
    beacond_config.payload[p++] = 0x03; /* Complete list of 16-bit Service UUIDs */
    beacond_config.payload[p++] = 0xAA; /* 16-bit Eddystone UUID (LSB) */
    beacond_config.payload[p++] = 0xFE; /* 16-bit Eddystone UUID (MSB) */

    beacond_config.payload[p++] = 0x06; /* Length (incl. URL scheme prefix) */
    beacond_config.payload[p++] = 0x16; /* Service Data */
    beacond_config.payload[p++] = 0xAA; /* 16-bit Eddystone UUID (LSB) */
    beacond_config.payload[p++] = 0xFE; /* 16-bit Eddystone UUID (MSB) */
    beacond_config.payload[p++] = 0x10; /* URL Frame */
    beacond_config.payload[p++] = tx_level; /* Calibrated Tx power at 0 m */

    /* encode the URL Scheme Prefix */
    for (i = 0; i < BLE_EDDYSTONE_URL_SCHEME_PREFIX_COUNT; i++) {
      len = strlen(eddystoneURLSchemePrefix[i]);
      if (strncmp(url, eddystoneURLSchemePrefix[i], len) == 0) {
        beacond_config.payload[p++] = i;
        c += len;
        break;
      }
    }
    if (i == BLE_EDDYSTONE_URL_SCHEME_PREFIX_COUNT) {
      return;
    }

    /* copy URL replacing tokens as they are found */
    while (p <= (14 + BLE_EDDYSTONE_URL_MAXLENGTH) && url[c] != '\0') {
      for (i = 0; i < BLE_EDDYSTONE_URL_ENCODING_COUNT; i++) {
        len = strlen(eddystoneURLEncoding[i]);
        if (strncmp(&url[c], eddystoneURLEncoding[i], len) == 0) {
          beacond_config.payload[p++] = i;
          c += len;
          break;
        }
      }
      if (i == BLE_EDDYSTONE_URL_ENCODING_COUNT) {
        beacond_config.payload[p++] = url[c++];
      }
    }
    if (url[c] != '\0') {
      return;
    }

    /* add the URL to the length within the payload */
    beacond_config.payload[7] += (p - 14);

    beacond_config.payload_length = p;
  }

  if(interval != 0) {
    beacond_config.interval = interval;
  }
}
/*---------------------------------------------------------------------------*/
void
rf_ble_beacond_config_eddystone_tlm(clock_time_t interval, uint16_t vbat,
                                    int16_t temp)
{
  uint32_t tmp32;
  int p;

  if(RF_BLE_ENABLED == 0) {
    return;
  }

  p = 0;
  memset(beacond_config.payload, 0, BLE_ADV_PAYLOAD_BUF_LEN);

  beacond_config.payload[p++] = 0x02; /* 2 bytes */
  beacond_config.payload[p++] = 0x01; /* Flags */
  beacond_config.payload[p++] = 0x06; /* LE general discoverable without BR/EDR */

  beacond_config.payload[p++] = 0x03; /* 3 bytes */
  beacond_config.payload[p++] = 0x03; /* Complete list of 16-bit Service UUIDs */
  beacond_config.payload[p++] = 0xAA; /* 16-bit Eddystone UUID (LSB) */
  beacond_config.payload[p++] = 0xFE; /* 16-bit Eddystone UUID (MSB) */

  beacond_config.payload[p++] = 0x11; /* 17 bytes */
  beacond_config.payload[p++] = 0x16; /* Service Data */
  beacond_config.payload[p++] = 0xAA; /* 16-bit Eddystone UUID (LSB) */
  beacond_config.payload[p++] = 0xFE; /* 16-bit Eddystone UUID (MSB) */
  beacond_config.payload[p++] = 0x20; /* TLM Frame */
  beacond_config.payload[p++] = 0x00; /* TLM version */
  beacond_config.payload[p++] = (vbat >> 8) & 0xFF; /* Battery voltage (MSB) */
  beacond_config.payload[p++] = vbat & 0xFF;        /* Battery voltage (LSB) */
  beacond_config.payload[p++] = temp / 256;         /* Temperature (MSB) */
  beacond_config.payload[p++] = temp % 256;         /* Temperature (LSB) */
  /* Advertising PDU count */
  tmp32 = uip_htonl(ble_adv_count);
  memcpy(&beacond_config.payload[p], &tmp32, sizeof(uint32_t));
  p += sizeof(uint32_t);
  /* Time since power-on or reboot */
  tmp32 = uip_htonl(clock_seconds() * 10
                    + (clock_time() % CLOCK_SECOND) / (CLOCK_SECOND/10));
  memcpy(&beacond_config.payload[p], &tmp32, sizeof(uint32_t));
  p += sizeof(uint32_t);

  beacond_config.payload_length = p;

  if(interval != 0) {
    beacond_config.interval = interval;
  }
}
/*---------------------------------------------------------------------------*/
void
rf_ble_beacond_burst_config(int burst_count, clock_time_t duty_cycle)
{
  if(burst_count > 0 ) {
    beacond_config.burst_count = burst_count;
  }

  beacond_config.duty_cycle = duty_cycle;
}
/*---------------------------------------------------------------------------*/
uint8_t
rf_ble_beacond_start()
{
  if(RF_BLE_ENABLED == 0) {
    return RF_CORE_CMD_ERROR;
  }

  if(ti_lib_chipinfo_supports_ble() == false) {
    return RF_CORE_CMD_ERROR;
  }

  if(beacond_config.payload_length == 0) {
    return RF_CORE_CMD_ERROR;
  }

  ble_mode_on = RF_BLE_IDLE;

  process_start(&rf_ble_beacon_process, NULL);

  return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
uint8_t
rf_ble_is_active()
{
  return ble_mode_on;
}
/*---------------------------------------------------------------------------*/
void
rf_ble_beacond_stop()
{
  process_exit(&rf_ble_beacon_process);
}
/*---------------------------------------------------------------------------*/
static uint8_t
rf_radio_setup()
{
  uint32_t cmd_status;
  rfc_CMD_RADIO_SETUP_t cmd;

  rf_switch_select_path(RF_SWITCH_PATH_2_4GHZ);

  /* Create radio setup command */
  rf_core_init_radio_op((rfc_radioOp_t *)&cmd, sizeof(cmd), CMD_RADIO_SETUP);

  cmd.txPower = tx_power;
  cmd.pRegOverride = ble_overrides;
  cmd.config.frontEndMode = RF_CORE_RADIO_SETUP_FRONT_END_MODE;
  cmd.config.biasMode = RF_CORE_RADIO_SETUP_BIAS_MODE;
  cmd.mode = 0;

  /* Send Radio setup to RF Core */
  if(rf_core_send_cmd((uint32_t)&cmd, &cmd_status) != RF_CORE_CMD_OK) {
    PRINTF("rf_radio_setup: CMDSTA=0x%08lx, status=0x%04x\n",
           cmd_status, cmd.status);
    return RF_CORE_CMD_ERROR;
  }

  /* Wait until radio setup is done */
  if(rf_core_wait_cmd_done(&cmd) != RF_CORE_CMD_OK) {
    PRINTF("rf_radio_setup: wait, CMDSTA=0x%08lx, status=0x%04x\n",
           cmd_status, cmd.status);
    return RF_CORE_CMD_ERROR;
  }

  return RF_CORE_CMD_OK;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rf_ble_beacon_process, ev, data)
{
  uint8_t was_on;
  static int i;
  uint32_t cmd_status;
  bool interrupts_disabled;

  PROCESS_BEGIN();

  while(1) {
    etimer_set(&ble_adv_et, beacond_config.interval);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&ble_adv_et) || ev == PROCESS_EVENT_EXIT);

    if(ev == PROCESS_EVENT_EXIT) {
      PROCESS_EXIT();
    }

    for(i = 0; i < beacond_config.burst_count; i++) {
      /*
       * Under ContikiMAC, some IEEE-related operations will be called from an
       * interrupt context. We need those to see that we are in BLE mode.
       */
      interrupts_disabled = ti_lib_int_master_disable();
      ble_mode_on = RF_BLE_ACTIVE;
      if(!interrupts_disabled) {
        ti_lib_int_master_enable();
      }

      /*
       * Send beacond_config.burst_count beacon bursts. Each burst on all three
       * channels, with a beacond_config.duty_cycle interval between bursts
       *
       * First, determine our state:
       *
       * If we are running NullRDC, we are likely in IEEE RX mode. We need to
       * abort the IEEE BG Op before entering BLE mode.
       * If we are ContikiMAC, we are likely off, in which case we need to
       * boot the CPE before entering BLE mode
       */
      was_on = rf_core_is_accessible();

      if(was_on) {
        /*
         * We were on: If we are in the process of receiving a frame, abort the
         * BLE beacon burst. Otherwise, terminate the primary radio Op so we
         * can switch to BLE mode
         */
        if(NETSTACK_RADIO.receiving_packet()) {
          PRINTF("rf_ble_beacon_process: We were receiving\n");

          /* Abort this pass */
          break;
        }

        rf_core_primary_mode_abort();
      } else {
        /* Request the HF XOSC to source the HF clock. */
        oscillators_request_hf_xosc();

        /* We were off: Boot the CPE */
        if(rf_core_boot() != RF_CORE_CMD_OK) {
          PRINTF("rf_ble_beacon_process: rf_core_boot() failed\n");

          /* Abort this pass */
          break;
        }

        /* Trigger a switch to the XOSC, so that we can use the FS */
        oscillators_switch_to_hf_xosc();
      }

      /* Enter BLE mode */
      if(rf_radio_setup() != RF_CORE_CMD_OK) {
        PRINTF("cc26xx_rf_ble_beacon_process: Error entering BLE mode\n");
        /* Continue so we can at least try to restore our previous state */
      } else {
        int j;
        /* Send advertising packets on all 3 advertising channels */
        for(j = 37; j <= 39; j++) {
          if(send_ble_adv_nc(j, beacond_config.payload,
                             beacond_config.payload_length) != RF_CORE_CMD_OK) {
            PRINTF("cc26xx_rf_ble_beacon_process: Channel=%d, "
                   "Error advertising\n", j);
            /* Break the loop, but don't return just yet */
            break;
          }
        }
      }

      /* Send a CMD_STOP command to RF Core */
      if(rf_core_send_cmd(CMDR_DIR_CMD(CMD_STOP), &cmd_status) != RF_CORE_CMD_OK) {
        PRINTF("cc26xx_rf_ble_beacon_process: status=0x%08lx\n", cmd_status);
        /* Continue... */
      }

      if(was_on) {
        /* We were on, go back to previous primary mode */
        rf_core_primary_mode_restore();
      } else {
        /* We were off. Shut back off */
        rf_core_power_down();

        /* Switch HF clock source to the RCOSC to preserve power */
        oscillators_switch_to_hf_rc();
      }

      /* Wait unless this is the last burst */
      if(i < beacond_config.burst_count - 1) {
        etimer_set(&ble_adv_et, beacond_config.duty_cycle);

        interrupts_disabled = ti_lib_int_master_disable();

        ble_mode_on = RF_BLE_IDLE;

        if(!interrupts_disabled) {
          ti_lib_int_master_enable();
        }

        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&ble_adv_et));
      }
    }

    interrupts_disabled = ti_lib_int_master_disable();

    ble_mode_on = RF_BLE_IDLE;

    if(!interrupts_disabled) {
      ti_lib_int_master_enable();
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
