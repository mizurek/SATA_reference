/*
 * Serial ATA 1.0a - Phy initialization state machines (6.8.1.1).
 */
#include "sata_phy.h"

void sata_phy_init(sata_phy_t *phy, sata_role_t role)
{
    *phy = (sata_phy_t){
        .role = role,
        .state = role == SATA_ROLE_HOST ? SATA_HP_RESET : SATA_DP_RESET,
        /* six bursts: 6 * (106.7 + 320) ns and 6 * (106.7 + 106.7) ns */
        .t_comreset = 96,
        .t_comwake = 48,
        .t_host_align = 32768,
        .t_dev_align = 4096, /* 54.6 us, extended as allowed by DP6 note 4 */
        .t_lost = 64,
    };
}

static void go(sata_phy_t *phy, sata_phy_state_t state)
{
    bool was_ready = sata_phy_ready(phy);

    phy->state = state;
    if (sata_phy_ready(phy) != was_ready)
        phy->diag |= SATA_DIAG_N;
    phy->timer = 0;
    phy->bad = 0;
    phy->nonalign = 0;
}

void sata_phy_reset(sata_phy_t *phy, bool hold)
{
    phy->reset_hold = hold;
    phy->device_seen = false;
    go(phy, phy->role == SATA_ROLE_HOST ? SATA_HP_RESET : SATA_DP_RESET);
}

void sata_phy_offline(sata_phy_t *phy)
{
    phy->device_seen = false;
    go(phy, SATA_PHY_OFFLINE);
}

static bool rx_is_align(const sata_xcvr_rx_t *rx)
{
    return rx->valid && !rx->err && rx->isk && rx->data == SATA_PRIM_ALIGN;
}

static void tx_dword(sata_xcvr_tx_t *tx, uint32_t data, bool isk)
{
    tx->data = data;
    tx->isk = isk;
    tx->elec_idle = 0;
}

/* Common to HP8 and DP7: PhyRdy drops when the receiver loses the signal. */
static bool ready_lost(sata_phy_t *phy, const sata_xcvr_rx_t *rx)
{
    phy->bad = rx->valid ? 0 : phy->bad + 1;
    if (phy->bad < phy->t_lost)
        return false;
    phy->diag |= SATA_DIAG_I;
    return true;
}

static void host_tick(sata_phy_t *phy, const sata_xcvr_rx_t *rx,
                      sata_xcvr_tx_t *tx)
{
    /*
     * COMINIT in any state makes the host reinitialize communications; the
     * transition into HP2B is implied in every state (6.8.1.1.1).
     */
    if (rx->cominit && phy->state != SATA_HP_RESET &&
        phy->state != SATA_HP_AWAIT_NO_COMINIT &&
        phy->state != SATA_PHY_OFFLINE)
        go(phy, SATA_HP_AWAIT_NO_COMINIT);

    switch (phy->state) {
    case SATA_HP_RESET: /* HP1 */
        tx->comreset = 1;
        if (!phy->reset_hold && phy->timer >= phy->t_comreset)
            go(phy, SATA_HP_AWAIT_COMINIT);
        break;

    case SATA_HP_AWAIT_COMINIT: /* HP2 */
        break;

    case SATA_HP_AWAIT_NO_COMINIT: /* HP2B; HP3 calibration is bypassed */
        phy->device_seen = true;
        if (!rx->cominit)
            go(phy, SATA_HP_COMWAKE);
        break;

    case SATA_HP_COMWAKE: /* HP4 */
        tx->comwake = 1;
        if (phy->timer >= phy->t_comwake)
            go(phy, rx->comwake ? SATA_HP_AWAIT_NO_COMWAKE
                                : SATA_HP_AWAIT_COMWAKE);
        break;

    case SATA_HP_AWAIT_COMWAKE: /* HP5 */
        if (rx->comwake) {
            phy->diag |= SATA_DIAG_W;
            go(phy, SATA_HP_AWAIT_NO_COMWAKE);
        }
        break;

    case SATA_HP_AWAIT_NO_COMWAKE: /* HP5B */
        if (!rx->comwake)
            go(phy, SATA_HP_AWAIT_ALIGN);
        break;

    case SATA_HP_AWAIT_ALIGN: /* HP6 */
        tx_dword(tx, SATA_D10_2_DWORD, false);
        if (rx_is_align(rx))
            go(phy, SATA_HP_SEND_ALIGN); /* HP11: nothing to adjust at Gen1 */
        else if (phy->timer >= phy->t_host_align)
            go(phy, SATA_HP_RESET);
        break;

    case SATA_HP_SEND_ALIGN: /* HP7 */
        tx_dword(tx, SATA_PRIM_ALIGN, true);
        if (rx->valid && !rx->err && rx->isk &&
            (rx->data & 0xFFu) == SATA_K28_3)
            phy->nonalign++;
        else
            phy->nonalign = 0;
        if (phy->nonalign >= 3)
            go(phy, SATA_HP_READY);
        else if (phy->timer >= phy->t_host_align)
            /* The spec waits here forever; starting over cannot hurt. */
            go(phy, SATA_HP_RESET);
        break;

    case SATA_HP_READY: /* HP8 */
        tx->elec_idle = 0;
        if (ready_lost(phy, rx))
            go(phy, SATA_HP_RESET);
        break;

    default: /* offline */
        break;
    }
}

static void device_tick(sata_phy_t *phy, const sata_xcvr_rx_t *rx,
                        sata_xcvr_tx_t *tx)
{
    /* COMRESET is a hard reset in every state (6.8.1.1.2). */
    if (rx->cominit && phy->state != SATA_DP_RESET &&
        phy->state != SATA_PHY_OFFLINE)
        go(phy, SATA_DP_RESET);

    switch (phy->state) {
    case SATA_DP_RESET: /* DP1 */
        if (!rx->cominit)
            go(phy, SATA_DP_COMINIT);
        break;

    case SATA_DP_COMINIT: /* DP2 */
        tx->comreset = 1;
        if (phy->timer >= phy->t_comreset)
            go(phy, SATA_DP_AWAIT_COMWAKE);
        break;

    case SATA_DP_AWAIT_COMWAKE: /* DP3 */
        if (rx->comwake) {
            phy->diag |= SATA_DIAG_W;
            go(phy, SATA_DP_AWAIT_NO_COMWAKE);
        }
        break;

    case SATA_DP_AWAIT_NO_COMWAKE: /* DP3B; DP4 calibration is bypassed */
        if (!rx->comwake)
            go(phy, SATA_DP_COMWAKE);
        break;

    case SATA_DP_COMWAKE: /* DP5 */
        tx->comwake = 1;
        if (phy->timer >= phy->t_comwake)
            go(phy, SATA_DP_SEND_ALIGN);
        break;

    case SATA_DP_SEND_ALIGN: /* DP6 */
        tx_dword(tx, SATA_PRIM_ALIGN, true);
        if (rx_is_align(rx))
            go(phy, SATA_DP_READY);
        else if (phy->timer >= phy->t_dev_align)
            go(phy, SATA_DP_ERROR); /* Gen1 is the lowest speed */
        break;

    case SATA_DP_READY: /* DP7 */
        tx->elec_idle = 0;
        if (ready_lost(phy, rx))
            go(phy, SATA_DP_RESET); /* ask the host to start over */
        break;

    default: /* DP11 error (left by COMRESET only), offline */
        break;
    }
}

bool sata_phy_tick(sata_phy_t *phy, const sata_xcvr_rx_t *rx,
                   sata_xcvr_tx_t *tx)
{
    *tx = (sata_xcvr_tx_t){ .elec_idle = 1 };

    sata_phy_state_t before = phy->state;
    if (phy->role == SATA_ROLE_HOST)
        host_tick(phy, rx, tx);
    else
        device_tick(phy, rx, tx);
    if (phy->state == before)
        phy->timer++;

    return sata_phy_ready(phy);
}
