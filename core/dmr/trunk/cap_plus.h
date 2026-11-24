// SPDX-License-Identifier: MIT
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "dmr/trunk/t3_ms.h"

#define CAP_FID_MOTOROLA 0x10u
#define CAP_CSBKO_STATUS 0x3Eu
#define CAP_SLCO_REST    0x0Fu

bool cap_plus_is_csbk(const uint8_t d[T3_CSBK_BYTES]);
bool cap_plus_rx_csbk(t3_ms_t *ms, uint32_t now, uint8_t slot, uint8_t cc,
                      const uint8_t d[T3_CSBK_BYTES]);
bool cap_plus_rx_mbc_cont(t3_ms_t *ms, uint32_t now, uint8_t slot, uint8_t cc,
                          const uint8_t d[T3_CSBK_BYTES]);
bool cap_plus_rx_slc(t3_ms_t *ms, uint32_t now, uint32_t slc);
bool cap_plus_on_call_burst(t3_ms_t *ms, uint32_t now, uint8_t slot, uint8_t cc);
