/*
 * Serial ATA 1.0a - Link layer (section 7).
 */
#include "sata_link.h"

#include "sata_codec.h"

/* What the state machine wants on the wire in this Dword slot. */
typedef struct {
    bool     is_data;
    uint32_t v;
} req_t;

void sata_link_init(sata_link_t *link, sata_role_t role)
{
    /* The receive buffer does not need clearing. */
    link->role = role;
    link->state = SATA_L_NO_COMM;
    link->diag = 0;
    link->use_cont = true;
    link->tx_pause = false;
    link->rx_throttle = false;
    link->rx_dmat = false;
    link->tx_buf = NULL;
    link->tx_len = 0;
    link->tx_pos = 0;
    link->tx_result = SATA_TX_NONE;
    link->tx_dmat = false;
    link->rx_len = 0;
    link->rx_failed = false;
    link->rx_pending = false;
    link->rx_verdict = 0;
}

bool sata_link_tx_start(sata_link_t *link, const uint32_t *fis,
                        unsigned ndwords)
{
    if (link->state == SATA_L_NO_COMM || link->tx_result == SATA_TX_BUSY ||
        ndwords == 0 || ndwords > SATA_MAX_FIS_DWORDS)
        return false;
    link->tx_buf = fis;
    link->tx_len = (uint16_t)ndwords;
    link->tx_pos = 0;
    link->tx_dmat = false;
    link->tx_result = SATA_TX_BUSY;
    return true;
}

void sata_link_tx_ack(sata_link_t *link)
{
    if (link->tx_result != SATA_TX_BUSY)
        link->tx_result = SATA_TX_NONE;
}

void sata_link_rx_done(sata_link_t *link, bool ok)
{
    if (!link->rx_pending)
        return;
    link->rx_pending = false;
    link->rx_verdict = ok ? 1 : -1;
}

static void tx_end(sata_link_t *l, sata_tx_result_t result)
{
    if (l->tx_result == SATA_TX_BUSY)
        l->tx_result = result;
}

/* The frame being sent or received is lost; back to L_IDLE. */
static void drop_to_idle(sata_link_t *l)
{
    tx_end(l, SATA_TX_ABORTED);
    l->rx_pending = false;
    l->state = SATA_L_IDLE;
}

static void rx_store(sata_link_t *l, uint32_t d)
{
    d ^= sata_scr_next(&l->rx_scr);
    l->rx_crc = sata_crc_step(l->rx_crc, d);
    if (l->rx_len < SATA_MAX_FRAME_DWORDS)
        l->rx_buf[l->rx_len++] = d;
    else
        l->rx_bad = true;
}

bool sata_link_tx_cancel(sata_link_t *link)
{
    if (link->tx_result != SATA_TX_BUSY)
        return false;
    switch (link->state) {
    case SATA_L_SEND_CHK_RDY:
    case SATA_L_SEND_SOF:
    case SATA_L_SEND_DATA:
    case SATA_L_RCVR_HOLD:
    case SATA_L_SEND_HOLD:
    case SATA_L_SEND_CRC:
    case SATA_L_SEND_EOF:
    case SATA_L_WAIT:
        return false;
    default:
        link->tx_result = SATA_TX_ABORTED;
        return true;
    }
}

bool sata_link_rx_failed(sata_link_t *link)
{
    bool failed = link->rx_failed;
    link->rx_failed = false;
    return failed;
}

void sata_link_abort(sata_link_t *link)
{
    if (link->state != SATA_L_NO_COMM)
        drop_to_idle(link);
}

/* LT7: the CRC is a data Dword and needs a free slot. */
static void send_crc(sata_link_t *l, bool slot, req_t *r)
{
    l->state = SATA_L_SEND_CRC;
    if (slot) {
        r->is_data = true;
        r->v = l->tx_crc ^ sata_scr_next(&l->tx_scr);
        l->state = SATA_L_SEND_EOF;
    }
}

/*
 * Link state machine (7.6). 'slot' tells whether the Dword requested here
 * will go out; when it is false (ALIGN insertion) states that send something
 * exactly once have to wait.
 */
static req_t fsm(sata_link_t *l, bool slot, bool is_data, uint32_t d)
{
    req_t r = { false, SATA_PRIM_SYNC };
    uint32_t p = l->rx_prim;
    const bool tx_busy = l->tx_result == SATA_TX_BUSY;

    switch (l->state) {
    case SATA_L_IDLE:
        /*
         * A frame is only started once the other side is seen idle (or,
         * for the device, asking to transmit: the host backs off, LT1:2).
         * That also guarantees the other side saw our SYNC after the
         * previous frame.
         */
        if (p == SATA_PRIM_X_RDY &&
            !(l->role == SATA_ROLE_DEVICE && tx_busy))
            l->state = SATA_L_RCV_WAIT_FIFO;
        else if (tx_busy && (p == SATA_PRIM_SYNC || p == SATA_PRIM_X_RDY))
            l->state = SATA_L_SEND_CHK_RDY;
        else if (p == SATA_PRIM_PMREQ_P || p == SATA_PRIM_PMREQ_S)
            l->state = SATA_L_PM_DENY;
        break;

    /* ---- transmit (7.6.1.3) ---- */

    case SATA_L_SEND_CHK_RDY:
        r.v = SATA_PRIM_X_RDY;
        if (p == SATA_PRIM_R_RDY)
            l->state = SATA_L_SEND_SOF;
        else if (p == SATA_PRIM_X_RDY && l->role == SATA_ROLE_HOST)
            l->state = SATA_L_RCV_WAIT_FIFO; /* frame stays queued */
        break;

    case SATA_L_SEND_SOF:
        r.v = SATA_PRIM_X_RDY;
        if (p == SATA_PRIM_SYNC) {
            l->diag |= SATA_DIAG_S;
            drop_to_idle(l);
        } else if (slot) {
            r.v = SATA_PRIM_SOF;
            l->tx_scr = SATA_SCR_INIT;
            l->tx_crc = SATA_CRC_INIT;
            l->tx_pos = 0;
            l->state = SATA_L_SEND_DATA;
        }
        break;

    case SATA_L_SEND_DATA:
        r.v = SATA_PRIM_HOLD;
        if (p == SATA_PRIM_SYNC) {
            l->diag |= SATA_DIAG_S;
            drop_to_idle(l);
        } else if (p == SATA_PRIM_DMAT) {
            l->tx_dmat = true;
            send_crc(l, slot, &r);
        } else if (p == SATA_PRIM_HOLD) {
            r.v = SATA_PRIM_HOLDA;
            l->state = SATA_L_RCVR_HOLD;
        } else if (l->tx_pause) {
            l->state = SATA_L_SEND_HOLD;
        } else if (slot) {
            uint32_t w = l->tx_buf[l->tx_pos++];
            l->tx_crc = sata_crc_step(l->tx_crc, w);
            r.is_data = true;
            r.v = w ^ sata_scr_next(&l->tx_scr);
            if (l->tx_pos >= l->tx_len)
                l->state = SATA_L_SEND_CRC;
        }
        break;

    case SATA_L_RCVR_HOLD:
        r.v = SATA_PRIM_HOLDA;
        if (p == SATA_PRIM_SYNC) {
            l->diag |= SATA_DIAG_S;
            drop_to_idle(l);
        } else if (p == SATA_PRIM_DMAT) {
            l->tx_dmat = true;
            l->state = SATA_L_SEND_CRC;
        } else if (p != SATA_PRIM_HOLD) {
            l->state = SATA_L_SEND_DATA;
        }
        break;

    case SATA_L_SEND_HOLD:
        r.v = SATA_PRIM_HOLD;
        if (p == SATA_PRIM_SYNC) {
            l->diag |= SATA_DIAG_S;
            drop_to_idle(l);
        } else if (p == SATA_PRIM_DMAT) {
            l->tx_dmat = true;
            l->state = SATA_L_SEND_CRC;
        } else if (!l->tx_pause) {
            l->state = p == SATA_PRIM_HOLD ? SATA_L_RCVR_HOLD
                                           : SATA_L_SEND_DATA;
        }
        break;

    case SATA_L_SEND_CRC:
        r.v = SATA_PRIM_HOLD;
        if (p == SATA_PRIM_SYNC) {
            l->diag |= SATA_DIAG_S;
            drop_to_idle(l);
        } else {
            send_crc(l, slot, &r);
        }
        break;

    case SATA_L_SEND_EOF:
        r.v = SATA_PRIM_HOLD;
        if (p == SATA_PRIM_SYNC) {
            l->diag |= SATA_DIAG_S;
            drop_to_idle(l);
        } else if (slot) {
            r.v = SATA_PRIM_EOF;
            l->state = SATA_L_WAIT;
        }
        break;

    case SATA_L_WAIT:
        r.v = SATA_PRIM_WTRM;
        if (p == SATA_PRIM_R_OK) {
            tx_end(l, SATA_TX_OK);
            l->state = SATA_L_IDLE;
        } else if (p == SATA_PRIM_R_ERR) {
            l->diag |= SATA_DIAG_H;
            tx_end(l, SATA_TX_R_ERR);
            l->state = SATA_L_IDLE;
        } else if (p == SATA_PRIM_SYNC) {
            l->diag |= SATA_DIAG_S;
            drop_to_idle(l);
        }
        break;

    /* ---- receive (7.6.1.4) ---- */

    case SATA_L_RCV_WAIT_FIFO:
        r.v = SATA_PRIM_SYNC;
        if (p != SATA_PRIM_X_RDY)
            l->state = SATA_L_IDLE;
        else if (!l->rx_pending)
            l->state = SATA_L_RCV_CHK_RDY;
        break;

    case SATA_L_RCV_CHK_RDY:
        r.v = SATA_PRIM_R_RDY;
        if (p == SATA_PRIM_SOF) {
            l->rx_len = 0;
            l->rx_crc = SATA_CRC_INIT;
            l->rx_scr = SATA_SCR_INIT;
            l->rx_bad = false;
            l->rx_verdict = 0;
            l->state = SATA_L_RCV_DATA;
        } else if (p != SATA_PRIM_X_RDY) {
            l->state = SATA_L_IDLE;
        }
        break;

    case SATA_L_RCV_DATA:
    case SATA_L_HOLD:
    case SATA_L_RCV_SEND_HOLD:
        if (is_data) {
            /* Data ends whatever primitive was in effect before it. */
            rx_store(l, d);
            l->rx_prim = p = 0;
        }
        r.v = SATA_PRIM_R_IP;
        if (p == SATA_PRIM_EOF) {
            /* LR6: the CRC of a good frame, CRC Dword included, is zero. */
            if (l->rx_bad || l->rx_len < 2 || l->rx_crc != 0) {
                l->diag |= SATA_DIAG_C;
                l->rx_failed = true;
                l->state = SATA_L_BAD_END;
            } else {
                l->rx_len--;
                l->rx_pending = true;
                l->state = SATA_L_GOOD_CRC;
            }
        } else if (p == SATA_PRIM_SYNC) {
            l->diag |= SATA_DIAG_S;
            l->state = SATA_L_IDLE;
        } else if (p == SATA_PRIM_WTRM) {
            l->diag |= SATA_DIAG_S; /* EOF was lost */
            l->rx_failed = true;
            l->state = SATA_L_BAD_END;
        } else if (l->state == SATA_L_RCV_DATA) {
            if (p == SATA_PRIM_HOLD) {
                r.v = SATA_PRIM_HOLDA;
                l->state = SATA_L_RCV_SEND_HOLD;
            } else if (l->rx_throttle) {
                r.v = SATA_PRIM_HOLD;
                l->state = SATA_L_HOLD;
            } else if (l->rx_dmat) {
                r.v = SATA_PRIM_DMAT;
            }
        } else if (l->state == SATA_L_HOLD) {
            r.v = SATA_PRIM_HOLD;
            if (!l->rx_throttle)
                l->state = p == SATA_PRIM_HOLD ? SATA_L_RCV_SEND_HOLD
                                               : SATA_L_RCV_DATA;
        } else {
            r.v = l->rx_dmat ? SATA_PRIM_DMAT : SATA_PRIM_HOLDA;
            if (p != SATA_PRIM_HOLD)
                l->state = SATA_L_RCV_DATA;
        }
        break;

    case SATA_L_GOOD_CRC:
        r.v = SATA_PRIM_R_IP;
        if (p == SATA_PRIM_SYNC) {
            l->diag |= SATA_DIAG_S;
            l->rx_pending = false;
            l->state = SATA_L_IDLE;
        } else if (l->rx_verdict > 0) {
            l->state = SATA_L_GOOD_END;
        } else if (l->rx_verdict < 0) {
            l->state = SATA_L_BAD_END;
        }
        break;

    case SATA_L_GOOD_END:
        r.v = SATA_PRIM_R_OK;
        if (p == SATA_PRIM_SYNC)
            l->state = SATA_L_IDLE;
        break;

    case SATA_L_BAD_END:
        r.v = SATA_PRIM_R_ERR;
        if (p == SATA_PRIM_SYNC)
            l->state = SATA_L_IDLE;
        break;

    case SATA_L_PM_DENY:
        r.v = SATA_PRIM_PMNAK;
        if (p != SATA_PRIM_PMREQ_P && p != SATA_PRIM_PMREQ_S)
            l->state = SATA_L_IDLE;
        break;

    default:
        break;
    }
    return r;
}

/* Primitives that may be followed by CONT (7.4.5). */
static bool cont_allowed(uint32_t prim)
{
    switch (prim) {
    case SATA_PRIM_HOLD:
    case SATA_PRIM_HOLDA:
    case SATA_PRIM_PMREQ_P:
    case SATA_PRIM_PMREQ_S:
    case SATA_PRIM_R_ERR:
    case SATA_PRIM_R_IP:
    case SATA_PRIM_R_OK:
    case SATA_PRIM_R_RDY:
    case SATA_PRIM_SYNC:
    case SATA_PRIM_WTRM:
    case SATA_PRIM_X_RDY:
        return true;
    default:
        return false;
    }
}

static void out(sata_xcvr_tx_t *tx, uint32_t data, bool isk)
{
    tx->data = data;
    tx->isk = isk;
}

/* Transmit back end: repeated primitive suppression. */
static void tx_backend(sata_link_t *l, sata_xcvr_tx_t *tx, req_t r)
{
    if (r.is_data) {
        if (l->tx_junk) {
            /*
             * The receiver ignores data after CONT, so the primitive has
             * to be sent once more before real data; the data waits a slot.
             */
            out(tx, l->tx_prim, true);
            l->pend = r.v;
            l->pend_valid = true;
        } else {
            out(tx, r.v, false);
        }
        l->tx_prim = 0;
        l->tx_rep = 0;
        l->tx_junk = false;
        return;
    }

    if (r.v == l->tx_prim) {
        if (l->tx_junk) {
            out(tx, sata_scr_next(&l->tx_junk_scr), false);
            return;
        }
        /* A primitive goes out at least twice before CONT. */
        if (l->tx_rep >= 2 && l->use_cont && !l->cont_inhibit &&
            cont_allowed(r.v)) {
            out(tx, SATA_PRIM_CONT, true);
            l->tx_junk = true;
            return;
        }
        l->tx_rep++;
    } else {
        l->tx_prim = r.v;
        l->tx_rep = 1;
        l->tx_junk = false;
    }
    out(tx, r.v, true);
    if (l->cont_inhibit)
        l->cont_inhibit--;
}

/* LS2/LS3 -> L1: communication (re)established. */
static void link_up(sata_link_t *l)
{
    l->state = SATA_L_IDLE;
    l->rx_prim = 0;
    l->rx_cont = false;
    l->align_cnt = 0;
    l->align_left = 2; /* the first two Dwords out are ALIGN (6.7.6) */
    l->tx_prim = 0;
    l->tx_rep = 0;
    l->tx_junk = false;
    l->tx_junk_scr = SATA_SCR_INIT;
    /* No CONT until 10 primitives went out or the peer talks (7.4.5). */
    l->cont_inhibit = 10;
    l->pend_valid = false;
}

void sata_link_tick(sata_link_t *l, bool phy_ready, const sata_xcvr_rx_t *rx,
                    sata_xcvr_tx_t *tx)
{
    if (!phy_ready) {
        if (l->state != SATA_L_NO_COMM) {
            /* LS1: post the error and wait for the Phy */
            l->diag |= SATA_DIAG_N;
            drop_to_idle(l);
            l->state = SATA_L_NO_COMM;
        }
        return;
    }
    if (l->state == SATA_L_NO_COMM)
        link_up(l);

    /*
     * Receive front end: ALIGN is dropped, CONT keeps the previous
     * primitive in effect until the next one (7.4.5, 7.4.6).
     */
    bool is_data = false;
    if (rx->valid) {
        if (rx->err) {
            l->diag |= SATA_DIAG_B;
            /* filler after CONT is thrown away anyway */
            if (!l->rx_cont)
                l->rx_bad = true;
        } else if (rx->isk) {
            if (rx->data == SATA_PRIM_CONT) {
                l->rx_cont = l->rx_prim != 0;
            } else if (rx->data != SATA_PRIM_ALIGN) {
                l->rx_prim = rx->data;
                l->rx_cont = false;
                if (rx->data != SATA_PRIM_SYNC)
                    l->cont_inhibit = 0;
            }
        } else if (!l->rx_cont) {
            is_data = true;
        }
    }

    /*
     * Transmit slot: two ALIGNs at least every 256 Dwords (6.7.6), then a
     * data Dword the back end had to delay.
     */
    bool slot = true;
    if (l->align_left == 0 && l->align_cnt >= 254) {
        l->align_left = 2;
        l->align_cnt = 0;
    }
    if (l->align_left) {
        out(tx, SATA_PRIM_ALIGN, true);
        l->align_left--;
        slot = false;
    } else if (l->pend_valid) {
        out(tx, l->pend, false);
        l->pend_valid = false;
        l->align_cnt++;
        slot = false;
    }

    req_t r = fsm(l, slot, is_data, rx->data);
    if (slot) {
        tx_backend(l, tx, r);
        l->align_cnt++;
    }
}
