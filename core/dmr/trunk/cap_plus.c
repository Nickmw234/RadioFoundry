// SPDX-License-Identifier: MIT
#include "dmr/trunk/cap_plus.h"

/* Map a 1-based Cap+ LSN (1..16) to a programmed channel index and TDMA slot (1 or 2) */
static bool cap_lsn_to_chan(const t3_ms_t *ms, uint8_t lsn, t3_chan_t *chan, uint8_t *slot) {
    if (!ms->cfg || lsn == 0) return false;
    uint8_t lcn_idx = (uint8_t)((lsn - 1u) / 2u);
    if (lcn_idx >= ms->cfg->nctrl) return false;
    *chan = ms->cfg->ctrl[lcn_idx];
    *slot = (uint8_t)(((lsn - 1u) % 2u) + 1u);
    return true;
}

static void cap_queue_tune(t3_ms_t *ms, t3_chan_t chan, uint8_t slot, bool payload) {
    if (ms->nq >= T3_ACT_QUEUE) return;
    t3_act_t *a = &ms->q[ms->nq++];
    a->type = T3_ACT_TUNE;
    a->chan = chan;
    a->slot = slot;
    a->payload = payload;
    a->udt = false;
    a->nblocks = 0;
    a->blocks = 0;
}

bool cap_plus_is_csbk(const uint8_t d[T3_CSBK_BYTES]) {
    uint8_t csbko = d[0] & 0x3Fu;
    uint8_t fid   = d[1];
    return (csbko == CAP_CSBKO_STATUS && fid == CAP_FID_MOTOROLA);
}

bool cap_plus_rx_csbk(t3_ms_t *ms, uint32_t now, uint8_t slot, uint8_t cc,
                      const uint8_t d[T3_CSBK_BYTES]) {
    (void)slot;
    (void)cc;
    if (!cap_plus_is_csbk(d)) return false;

    ms->st.csbk++;
    ms->last_valid_ms = now;

    uint8_t rest_lsn = d[2] & 0x0Fu;
    uint8_t ch_mask  = d[3]; /* Bit 7 = LSN 1 .. Bit 0 = LSN 8 */

    t3_chan_t rest_chan;
    uint8_t rest_slot = 1;
    if (cap_lsn_to_chan(ms, rest_lsn, &rest_chan, &rest_slot)) {
        bool moved = (!ms->confirmed ||
                      ms->tscc.rx_hz != rest_chan.rx_hz ||
                      ms->tscc_slots != (uint8_t)(1u << (rest_slot - 1u)));
        ms->confirmed = true;
        ms->tscc = rest_chan;
        ms->tscc_slots = (uint8_t)(1u << (rest_slot - 1u));

        if (ms->state == T3_HUNT) {
            ms->state = T3_CAMPED;
            cap_queue_tune(ms, rest_chan, rest_slot, false);
        } else if (ms->state == T3_CAMPED && moved) {
            cap_queue_tune(ms, rest_chan, rest_slot, false);
        }
    }

    /* If camped on Rest Channel, check active Group Calls in ch_mask */
    if (ms->state == T3_CAMPED && ch_mask != 0) {
        uint8_t tg_idx = 4;
        for (uint8_t bit = 0; bit < 8 && tg_idx < 10; bit++) {
            if (ch_mask & (0x80u >> bit)) {
                uint8_t lsn = (uint8_t)(bit + 1u);
                uint32_t tg = d[tg_idx++];
                if (lsn != rest_lsn && t3_ms_wants_group(ms, tg, false)) {
                    t3_chan_t vchan;
                    uint8_t vslot = 1;
                    if (cap_lsn_to_chan(ms, lsn, &vchan, &vslot)) {
                        ms->st.grants++;
                        ms->st.followed++;
                        ms->state = T3_FOLLOW;
                        ms->last_act_ms = now;
                        ms->call.op = CAP_CSBKO_STATUS;
                        ms->call.dst = tg;
                        ms->call.src = 0;
                        ms->call.group = true;
                        ms->call.emergency = false;
                        ms->call.chan = vchan;
                        ms->call.slot = vslot;
                        ms->call.start_ms = now;
                        ms->call.ours = false;
                        ms->call.data = false;
                        cap_queue_tune(ms, vchan, vslot, true);
                        return true;
                    }
                }
            }
        }
    }

    return true;
}

bool cap_plus_rx_slc(t3_ms_t *ms, uint32_t now, uint32_t slc) {
    /* 32-bit Short LC: [31:28]=SLCO, [27:20]=MFID, [19:18]=op, [12:8]=Rest LSN */
    uint8_t slco = (uint8_t)((slc >> 28) & 0x0Fu);
    uint8_t mfid = (uint8_t)((slc >> 20) & 0xFFu);
    if (slco != CAP_SLCO_REST || mfid != CAP_FID_MOTOROLA) return false;

    uint8_t rest_lsn = (uint8_t)((slc >> 8) & 0x1Fu);
    t3_chan_t rest_chan;
    uint8_t rest_slot = 1;
    if (cap_lsn_to_chan(ms, rest_lsn, &rest_chan, &rest_slot)) {
        ms->last_valid_ms = now;
        bool moved = (!ms->confirmed ||
                      ms->tscc.rx_hz != rest_chan.rx_hz ||
                      ms->tscc_slots != (uint8_t)(1u << (rest_slot - 1u)));
        ms->confirmed = true;
        ms->tscc = rest_chan;
        ms->tscc_slots = (uint8_t)(1u << (rest_slot - 1u));

        if (ms->state == T3_HUNT) {
            ms->state = T3_CAMPED;
            cap_queue_tune(ms, rest_chan, rest_slot, false);
        } else if (ms->state == T3_CAMPED && moved) {
            cap_queue_tune(ms, rest_chan, rest_slot, false);
        }
    }
    return true;
}
