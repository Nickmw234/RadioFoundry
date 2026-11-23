// SPDX-License-Identifier: MIT
#ifndef RFOUNDRY_CAP_PLUS_H
#define RFOUNDRY_CAP_PLUS_H

#include <stdbool.h>
#include <stdint.h>
#include "dmr/trunk/t3_ms.h"

#define CAP_FID_MOTOROLA     0x10
#define CAP_CSBKO_STATUS     0x3E
#define CAP_SLCO_REST        0x0F

/* Returns true if the CSBK is a Motorola Cap+ Channel Status burst (FID=0x10, CSBKO=0x3E) */
bool cap_plus_is_csbk(const uint8_t d[T3_CSBK_BYTES]);

/* Feed a Cap+ CSBK into the trunking engine; returns true if handled */
bool cap_plus_rx_csbk(t3_ms_t *ms, uint32_t now, uint8_t slot, uint8_t cc,
                      const uint8_t d[T3_CSBK_BYTES]);

/* Feed a CACH Short LC into the trunking engine; returns true if Cap+ SLCO 0x0F */
bool cap_plus_rx_slc(t3_ms_t *ms, uint32_t now, uint32_t slc);

#endif
