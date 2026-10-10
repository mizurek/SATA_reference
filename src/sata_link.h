/*
 * Serial ATA 1.0a - Link layer (section 7).
 *
 * Frame transmission and reception: X_RDY/R_RDY negotiation, SOF/EOF
 * framing, CRC, scrambling, HOLD/HOLDA flow control, R_OK/R_ERR handshake,
 * CONT repeated primitive suppression and ALIGN insertion.
 *
 * The layer is clocked once per Dword by sata_link_tick(). A frame is handed
 * over as a whole: the Transport layer passes a buffer to transmit and
 * inspects the receive buffer when a frame came in with good CRC.
 *
 * Power management requests from the other side are rejected with PMNAK
 * (Partial / Slumber are optional and not implemented).
 */
#ifndef SATA_LINK_H
#define SATA_LINK_H

#include "sata_defs.h"

typedef enum {
    SATA_L_NO_COMM,       /* LS1/LS2 Phy not ready                         */
    SATA_L_IDLE,          /* L1                                            */
    SATA_L_SEND_CHK_RDY,  /* LT1/LT2                                       */
    SATA_L_SEND_SOF,      /* LT3                                           */
    SATA_L_SEND_DATA,     /* LT4                                           */
    SATA_L_RCVR_HOLD,     /* LT5                                           */
    SATA_L_SEND_HOLD,     /* LT6                                           */
    SATA_L_SEND_CRC,      /* LT7                                           */
    SATA_L_SEND_EOF,      /* LT8                                           */
    SATA_L_WAIT,          /* LT9                                           */
    SATA_L_RCV_WAIT_FIFO, /* LR2                                           */
    SATA_L_RCV_CHK_RDY,   /* LR1                                           */
    SATA_L_RCV_DATA,      /* LR3                                           */
    SATA_L_HOLD,          /* LR4                                           */
    SATA_L_RCV_SEND_HOLD, /* LR5 (L_SendHold of the receive diagram)       */
    SATA_L_GOOD_CRC,      /* LR6/LR7                                       */
    SATA_L_GOOD_END,      /* LR8                                           */
    SATA_L_BAD_END,       /* LR9                                           */
    SATA_L_PM_DENY,       /* LPM4                                          */
} sata_link_state_t;

typedef enum {
    SATA_TX_NONE,    /* nothing was requested                               */
    SATA_TX_BUSY,    /* frame is waiting or on its way                      */
    SATA_TX_OK,      /* receiver answered R_OK                              */
    SATA_TX_R_ERR,   /* receiver answered R_ERR                             */
    SATA_TX_ABORTED, /* SYNC from the receiver, Phy not ready or abort      */
} sata_tx_result_t;

typedef struct {
    sata_role_t       role;
    sata_link_state_t state;
    uint32_t          diag; /* SATA_DIAG_* flags, sticky */

    /* Options */
    bool use_cont;    /* suppress repeated primitives with CONT (7.4.5)     */
    bool tx_pause;    /* transmit data not ready: insert HOLD               */
    bool rx_throttle; /* receive side not ready: ask for HOLD               */
    bool rx_dmat;     /* ask the transmitter to terminate (DMAT, 7.4.4)     */

    /* Receive front end */
    uint32_t rx_prim; /* primitive currently in effect, 0 = none            */
    bool     rx_cont; /* CONT received: rx_prim repeats until the next one  */

    /* Transmit back end */
    uint16_t align_cnt;   /* Dwords since the last ALIGN pair               */
    uint8_t  align_left;  /* ALIGNs still to send of the current pair       */
    uint32_t tx_prim;     /* primitive last sent, 0 = data                  */
    uint8_t  tx_rep;      /* how often in a row                             */
    bool     tx_junk;     /* CONT was sent, sending scrambled filler        */
    uint16_t tx_junk_scr;
    uint8_t  cont_inhibit; /* primitives to send before CONT is allowed     */
    bool     pend_valid;   /* data Dword delayed by one slot                */
    uint32_t pend;

    /* Frame being transmitted */
    const uint32_t  *tx_buf;
    uint16_t         tx_len;
    uint16_t         tx_pos;
    uint32_t         tx_crc;
    uint16_t         tx_scr;
    sata_tx_result_t tx_result;
    bool             tx_dmat; /* frame was cut short by DMAT */

    /* Frame being received; rx_buf holds FIS + CRC while receiving */
    uint32_t rx_buf[SATA_MAX_FRAME_DWORDS];
    uint16_t rx_len;
    uint32_t rx_crc;
    uint16_t rx_scr;
    bool     rx_bad;     /* decode error or overflow inside the frame       */
    bool     rx_pending; /* FIS waits for the verdict of the Transport      */
    int8_t   rx_verdict; /* 0 = none yet, >0 good, <0 bad                   */
    bool     rx_failed;  /* a frame was answered with R_ERR by the Link     */
} sata_link_t;

void sata_link_init(sata_link_t *link, sata_role_t role);

/*
 * Advance by one Dword. 'phy_ready' is PhyRdy; when it is true the Dword to
 * transmit is written to tx->data / tx->isk.
 */
void sata_link_tick(sata_link_t *link, bool phy_ready,
                    const sata_xcvr_rx_t *rx, sata_xcvr_tx_t *tx);

/*
 * Queue a FIS ('ndwords' Dwords, without CRC) for transmission. The buffer
 * must stay valid until the result is no longer SATA_TX_BUSY. Returns false
 * if a frame is already queued or the Phy is not ready.
 */
bool sata_link_tx_start(sata_link_t *link, const uint32_t *fis,
                        unsigned ndwords);

/* Result of the last sata_link_tx_start(). */
static inline sata_tx_result_t sata_link_tx_result(const sata_link_t *link)
{
    return link->tx_result;
}

/* Forget the result (back to SATA_TX_NONE) unless a frame is still busy. */
void sata_link_tx_ack(sata_link_t *link);

/* Number of payload Dwords that went out; less than queued after a DMAT. */
static inline unsigned sata_link_tx_sent(const sata_link_t *link)
{
    return link->tx_pos;
}

/*
 * A FIS with good CRC is in rx_buf[0 .. rx_len-1] and the Link is sending
 * R_IP until the Transport layer answers with sata_link_rx_done().
 */
static inline bool sata_link_rx_pending(const sata_link_t *link)
{
    return link->rx_pending;
}

/* Verdict on the pending FIS: answer with R_OK (true) or R_ERR (false). */
void sata_link_rx_done(sata_link_t *link, bool ok);

/*
 * Withdraw a queued frame that has not started yet (the Link is idle or
 * receiving). Returns false if there is none or it is already on its way.
 */
bool sata_link_tx_cancel(sata_link_t *link);

/*
 * True once after a frame was received damaged (CRC or decode error) and
 * answered with R_ERR by the Link layer itself.
 */
bool sata_link_rx_failed(sata_link_t *link);

/*
 * Give up whatever frame is queued or in progress and return to idle, which
 * sends SYNC and so takes the other side back to idle as well. Used by the
 * host to get a software reset through (LT4:7, LR3:9).
 */
void sata_link_abort(sata_link_t *link);

#endif /* SATA_LINK_H */
