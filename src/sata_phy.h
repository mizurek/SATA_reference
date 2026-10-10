/*
 * Serial ATA 1.0a - Phy initialization state machines (6.8.1.1).
 *
 * This is the digital part of the Phy layer that sits directly on top of the
 * transceiver: OOB sequencing (COMRESET / COMINIT / COMWAKE), the ALIGN
 * handshake and the PhyRdy indication for the Link layer.
 *
 * Not implemented: interface power management (Partial / Slumber), which is
 * optional, and speed negotiation, of which a Generation 1 only Phy has none.
 */
#ifndef SATA_PHY_H
#define SATA_PHY_H

#include "sata_defs.h"

typedef enum {
    /* host, HP1..HP8 */
    SATA_HP_RESET,
    SATA_HP_AWAIT_COMINIT,
    SATA_HP_AWAIT_NO_COMINIT,
    SATA_HP_COMWAKE,
    SATA_HP_AWAIT_COMWAKE,
    SATA_HP_AWAIT_NO_COMWAKE,
    SATA_HP_AWAIT_ALIGN,
    SATA_HP_SEND_ALIGN,
    SATA_HP_READY,
    /* device, DP1..DP11 */
    SATA_DP_RESET,
    SATA_DP_COMINIT,
    SATA_DP_AWAIT_COMWAKE,
    SATA_DP_AWAIT_NO_COMWAKE,
    SATA_DP_COMWAKE,
    SATA_DP_SEND_ALIGN,
    SATA_DP_READY,
    SATA_DP_ERROR,
    /* both: interface disabled (SControl DET = 4) */
    SATA_PHY_OFFLINE,
} sata_phy_state_t;

typedef struct {
    sata_role_t      role;
    sata_phy_state_t state;
    uint32_t         timer;       /* Dwords spent in the current state        */
    uint32_t         bad;         /* consecutive unusable Dwords while ready  */
    uint8_t          nonalign;    /* consecutive non-ALIGN primitives (HP7)   */
    bool             reset_hold;  /* application holds the interface in reset */
    bool             device_seen; /* host: COMINIT seen since the last reset  */
    uint32_t         diag;        /* SATA_DIAG_* flags, sticky                */

    /* Timing in Dwords (26.67 ns); sata_phy_init() sets the defaults. */
    uint32_t t_comreset;   /* length of a COMRESET / COMINIT transmission     */
    uint32_t t_comwake;    /* length of a COMWAKE transmission                */
    uint32_t t_host_align; /* host: wait for ALIGN, then for the first
                              primitives, before starting over (880 us)       */
    uint32_t t_dev_align;  /* device: send ALIGN this long without an answer
                              before giving up (54.6 us)                      */
    uint32_t t_lost;       /* this many consecutive unusable Dwords while
                              ready drop PhyRdy                               */
} sata_phy_t;

void sata_phy_init(sata_phy_t *phy, sata_role_t role);

/*
 * Hard reset. Host: transmit COMRESET for as long as 'hold' is true (and at
 * least t_comreset), then initialize. Device: restart with COMINIT.
 */
void sata_phy_reset(sata_phy_t *phy, bool hold);

/* Disable the interface until the next sata_phy_reset(). */
void sata_phy_offline(sata_phy_t *phy);

/*
 * Advance by one Dword. Returns PhyRdy. While PhyRdy is false *tx is driven
 * by the Phy; while it is true only the OOB/idle controls are (to inactive)
 * and the Link layer supplies tx->data / tx->isk.
 */
bool sata_phy_tick(sata_phy_t *phy, const sata_xcvr_rx_t *rx,
                   sata_xcvr_tx_t *tx);

static inline bool sata_phy_ready(const sata_phy_t *phy)
{
    return phy->state == SATA_HP_READY || phy->state == SATA_DP_READY;
}

#endif /* SATA_PHY_H */
