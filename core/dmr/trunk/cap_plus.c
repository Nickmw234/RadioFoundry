// SPDX-License-Identifier: MIT
#include <string.h>
#include "dmr/trunk/cap_plus.h"

static uint8_t cap_mbc_hdr[2][T3_CSBK_BYTES];
static uint32_t cap_mbc_ms[2];
static bool cap_mbc_open[2];

static bool cap_lsn_to_chan(const t3_ms_t *ms, uint8_t lsn, t3_chan_t *chan, uint8_t *slot) {
    if (!ms->cfg || lsn == 0 || lsn > 16u) return false;
    uint16_t lcn_num = (uint16_t)(((lsn - 1u) / 2u) + 1u);
    *slot = (uint8_t)(((lsn - 1u) % 2u) + 1u);

    for (uint8_t i = 0; i < ms->cfg->nctrl; i++) {
        if (ms->cfg->ctrl[i].lcn == lcn_num && ms->cfg->ctrl[i].rx_hz != 0) {
            *chan = ms->cfg->ctrl[i];
            return true;
        }
    }
    for (uint16_t i = 0; i < ms->cfg->plan.ntable; i++) {
        if (ms->cfg->plan.table[i].lcn == lcn_num && ms->cfg->plan.table[i].rx_hz != 0) {
            *chan = ms->cfg->plan.table[i];
            return true;
        }
    }
    uint8_t idx = (uint8_t)(lcn_num - 1u);
    if (idx < ms->cfg->nctrl && ms->cfg->ctrl[idx].rx_hz != 0) {
        *chan = ms->cfg->ctrl[idx];
        return true;
    }
    return false;
}

static void cap_force_listen_only(t3_ms_t *ms) {
    ms->reg = T3_REG_LISTEN;
    ms->ax.state = T3_AX_IDLE;
    ms->ax.retry_ms = 0;
    ms->massreg = false;
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

static bool cap_try_follow(t3_ms_t *ms, uint32_t now, uint8_t lsn, uint8_t rest_lsn, uint32_t tg) {
    if (lsn == rest_lsn || !t3_ms_wants_group(ms, tg, false)) return false;

    t3_chan_t vchan;
    uint8_t vslot = 1;
    if (!cap_lsn_to_chan(ms, lsn, &vchan, &vslot)) return false;

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

static void cap_parse_status(t3_ms_t *ms, uint32_t now, const uint8_t *buf, uint8_t len) {
    cap_force_listen_only(ms);
    ms->st.csbk++;
    ms->last_valid_ms = now;

    uint8_t rest_lsn = buf[2] & 0x1Fu;
    uint8_t ch_mask1 = buf[3];
    bool    has_ext  = (ms->cfg && ms->cfg->plan.ntable > 4u) || ((buf[2] & 0x20u) != 0);
    uint8_t ch_mask2 = has_ext ? buf[4] : 0u;
    uint8_t tg_idx   = has_ext ? 5u : 4u;

    t3_chan_t rest_chan;
    uint8_t rest_slot = 1;
    if (cap_lsn_to_chan(ms, rest_lsn, &rest_chan, &rest_slot)) {
        bool moved = (!ms->confirmed ||
                      ms->tscc.rx_hz != rest_chan.rx_hz ||
                      ms->tscc_slots != 0x3u);
        ms->confirmed = true;
        ms->tscc = rest_chan;
        ms->tscc_slots = 0x3u;

        if (ms->state == T3_HUNT) {
            ms->state = T3_CAMPED;
            cap_queue_tune(ms, rest_chan, rest_slot, false);
        } else if (ms->state == T3_CAMPED && moved) {
            cap_queue_tune(ms, rest_chan, rest_slot, false);
        }
    }

    if (ms->state == T3_CAMPED) {
        for (uint8_t bit = 0; bit < 8u && tg_idx < len; bit++) {
            if (ch_mask1 & (0x80u >> bit)) {
                uint8_t lsn = (uint8_t)(bit + 1u);
                uint32_t tg = buf[tg_idx++];
                if (cap_try_follow(ms, now, lsn, rest_lsn, tg)) {
                    return;
                }
            }
        }
        for (uint8_t bit = 0; bit < 8u && tg_idx < len; bit++) {
            if (ch_mask2 & (0x80u >> bit)) {
                uint8_t lsn = (uint8_t)(bit + 9u);
                uint32_t tg = buf[tg_idx++];
                if (cap_try_follow(ms, now, lsn, rest_lsn, tg)) {
                    return;
                }
            }
        }
    }
}

bool cap_plus_is_csbk(const uint8_t d[T3_CSBK_BYTES]) {
    uint8_t csbko = d[0] & 0x3Fu;
    uint8_t fid   = d[1];
    return (csbko == CAP_CSBKO_STATUS && fid == CAP_FID_MOTOROLA);
}

bool cap_plus_rx_csbk(t3_ms_t *ms, uint32_t now, uint8_t slot, uint8_t cc,
                      const uint8_t d[T3_CSBK_BYTES]) {
    (void)cc;
    if (!cap_plus_is_csbk(d)) return false;

    uint8_t si = (slot >= 1u && slot <= 2u) ? (uint8_t)(slot - 1u) : 0u;
    bool lb = (d[0] & 0x80u) != 0;
    if (!lb) {
        memcpy(cap_mbc_hdr[si], d, T3_CSBK_BYTES);
        cap_mbc_ms[si] = now;
        cap_mbc_open[si] = true;
    } else {
        cap_mbc_open[si] = false;
    }
    cap_parse_status(ms, now, d, 10u);
    return true;
}

bool cap_plus_rx_mbc_cont(t3_ms_t *ms, uint32_t now, uint8_t slot, uint8_t cc,
                          const uint8_t d[T3_CSBK_BYTES]) {
    (void)cc;
    uint8_t si = (slot >= 1u && slot <= 2u) ? (uint8_t)(slot - 1u) : 0u;
    if (!cap_mbc_open[si] || (uint32_t)(now - cap_mbc_ms[si]) >= 130u) {
        cap_mbc_open[si] = false;
        return false;
    }
    cap_mbc_open[si] = false;
    uint8_t combined[20];
    memcpy(combined, cap_mbc_hdr[si], 10u);
    memcpy(combined + 10u, d, 10u);
    cap_parse_status(ms, now, combined, 20u);
    return true;
}

bool cap_plus_rx_slc(t3_ms_t *ms, uint32_t now, uint32_t slc) {
    uint8_t slco = (uint8_t)((slc >> 28) & 0x0Fu);
    uint8_t mfid = (uint8_t)((slc >> 20) & 0xFFu);
    if (slco != CAP_SLCO_REST || mfid != CAP_FID_MOTOROLA) return false;

    cap_force_listen_only(ms);
    uint8_t rest_lsn = (uint8_t)((slc >> 8) & 0x1Fu);
    t3_chan_t rest_chan;
    uint8_t rest_slot = 1;
    if (cap_lsn_to_chan(ms, rest_lsn, &rest_chan, &rest_slot)) {
        ms->last_valid_ms = now;
        bool moved = (!ms->confirmed ||
                      ms->tscc.rx_hz != rest_chan.rx_hz ||
                      ms->tscc_slots != 0x3u);
        ms->confirmed = true;
        ms->tscc = rest_chan;
        ms->tscc_slots = 0x3u;

        if (ms->state == T3_HUNT) {
            ms->state = T3_CAMPED;
            cap_queue_tune(ms, rest_chan, rest_slot, false);
        } else if (ms->state == T3_CAMPED && moved) {
            cap_queue_tune(ms, rest_chan, rest_slot, false);
        }
    }
    return true;
}

bool cap_plus_on_call_burst(t3_ms_t *ms, uint32_t now, uint8_t slot, uint8_t cc) {
    (void)cc;
    if (ms->state != T3_CAMPED || !ms->confirmed || ms->tscc_slots != 0x3u) return false;
    if (slot < 1u || slot > 2u) return false;

    ms->st.grants++;
    ms->st.followed++;
    ms->state = T3_FOLLOW;
    ms->last_act_ms = now;
    ms->call.op = CAP_CSBKO_STATUS;
    ms->call.dst = 0;
    ms->call.src = 0;
    ms->call.group = true;
    ms->call.emergency = false;
    ms->call.chan = ms->tscc;
    ms->call.slot = slot;
    ms->call.start_ms = now;
    ms->call.ours = false;
    ms->call.data = false;
    cap_queue_tune(ms, ms->tscc, slot, true);
    return true;
}
