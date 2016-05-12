/*
 * Copyright (c) 2016, Cedric D.
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
 *
 * This file is part of the Contiki operating system.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup cc26xx-aes-128
 * @{
 *
 * \file
 * Implementation of the CC13xx/CC26xx AES-128 driver.
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "dev/cc26xx-aes-128.h"
#include "ti-lib.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>
/*---------------------------------------------------------------------------*/
#define MODULE_NAME	"cc26xx-aes-128"
 
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
static void
set_key(const uint8_t *key)
{
  uint32_t aligned_key[(AES_128_KEY_LENGTH+3)/4];
  uint32_t ret;

  memcpy(aligned_key, key, AES_128_KEY_LENGTH);

  ret = ti_lib_crypto_aes_load_key(aligned_key, CC26XX_AES_128_KEY_AREA);

  if(ret != AES_SUCCESS) {
    PRINTF("%s: ti_lib_crypto_aes_load_key() error %u\n", MODULE_NAME, ret);
  }
}
/*---------------------------------------------------------------------------*/
static void
encrypt(uint8_t *plaintext_and_result)
{
  uint32_t in[(AES_128_BLOCK_SIZE+3)/4];
  uint32_t out[(AES_128_BLOCK_SIZE+3)/4];
  uint32_t ret;

  memcpy(in, plaintext_and_result, AES_128_BLOCK_SIZE);

  ret = ti_lib_crypto_aes_ecb(in, out, CC26XX_AES_128_KEY_AREA, true, false);
  if(ret != AES_SUCCESS) {
    PRINTF("%s: ti_lib_crypto_aes_ecb() error %u\n", MODULE_NAME, ret);
    return;
  }

  while((ret = ti_lib_crypto_aes_ecb_status()) == AES_DMA_BSY);
  if(ret != AES_SUCCESS) {
    PRINTF("%s: ti_lib_crypto_aes_ecb_status() error %u\n", MODULE_NAME, ret);
  }

  memcpy(plaintext_and_result, out, AES_128_BLOCK_SIZE);
}
/*---------------------------------------------------------------------------*/
const struct aes_128_driver cc26xx_aes_128_driver = {
  set_key,
  encrypt
};

/** @} */
