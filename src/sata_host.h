/*
 * Serial ATA 1.0a - host port: Phy + Link + host Transport layer (8.6) with
 * the Shadow Register Block (5.2) and a small command interface on top.
 *
 * Everything is driven by sata_host_tick(), called once per Dword with what
 * the transceiver received; it returns what to transmit. Commands are
 * started with sata_host_start() (or one of the helpers) and complete some
 * ticks later; nothing blocks.
 *
 * The transport reacts to what the device sends, as the spec describes it,
 * so one entry point serves the non-data, PIO in/out and DMA in/out
 * protocols alike. First-party DMA Setup and BIST Activate FIS's are
 * answered with R_ERR (not supported).
 */
#ifndef SATA_HOST_H
#define SATA_HOST_H

#include "sata_fis.h"
#include "sata_link.h"
#include "sata_phy.h"

typedef enum {
    SATA_HOST_OK,
    SATA_HOST_BUSY,       /* command or reset in progress                    */
    SATA_HOST_NOT_READY,  /* no link, device busy or a command is active     */
    SATA_HOST_DEV_ERROR,  /* device ended the command with ERR or DF set     */
    SATA_HOST_LINK_ERROR, /* link went down or a frame could not be sent     */
    SATA_HOST_TIMEOUT,    /* no completion in time; device needs a reset     */
    SATA_HOST_ABORTED,    /* cut short by a reset                            */
} sata_host_result_t;

typedef struct {
    sata_phy_t  phy;
    sata_link_t link;

    /* Shadow Register Block */
    uint8_t     status;
    uint8_t     error;
    uint8_t     control;
    sata_regs_t regs;
    bool        irq; /* interrupt pending flag (5.2.2), cleared by a command */

    uint32_t diag; /* SATA_DIAG_T / SATA_DIAG_F, sticky */

    /* Internal state */
    bool                 up;
    uint8_t              tx_kind;
    uint8_t              retries;
    unsigned             fis_len;
    uint8_t              srst;
    bool                 active;
    bool                 cmd_sent;
    bool                 xfer_err;
    bool                 pio_valid;
    sata_fis_pio_setup_t pio;
    uint8_t             *buf;
    uint32_t             buf_len;
    uint32_t             done;
    unsigned             chunk;
    uint32_t             timeout;
    sata_host_result_t   result;
    uint32_t             fis[SATA_MAX_FIS_DWORDS];
} sata_host_t;

void sata_host_init(sata_host_t *h);

/* One Dword clock: rx from the transceiver in, tx to the transceiver out. */
void sata_host_tick(sata_host_t *h, const sata_xcvr_rx_t *rx,
                    sata_xcvr_tx_t *tx);

/* Link is up, the device is not busy and no command is active. */
bool sata_host_ready(const sata_host_t *h);

/*
 * Issue a command. 'cmd' supplies Command, Features and the taskfile
 * registers. 'buf' / 'nbytes' is the data buffer: filled by data-in
 * commands, sent by data-out commands, unused otherwise. 'timeout' is in
 * Dwords (37.5 million a second at Gen1), 0 waits forever.
 * Returns SATA_HOST_BUSY when started, SATA_HOST_NOT_READY otherwise.
 */
sata_host_result_t sata_host_start(sata_host_t *h, const sata_fis_h2d_t *cmd,
                                   void *buf, uint32_t nbytes,
                                   uint32_t timeout);

/* SATA_HOST_BUSY while the command runs, then its outcome. */
sata_host_result_t sata_host_result(const sata_host_t *h);

/* Bytes of data transferred by the current / last command. */
static inline uint32_t sata_host_transferred(const sata_host_t *h)
{
    return h->done;
}

/*
 * Software reset (5.2.1): two Register FIS's toggling SRST; the device
 * answers with its signature. Aborts a command in progress. Done when
 * sata_host_ready() is true again.
 */
void sata_host_soft_reset(sata_host_t *h);

/* Hard reset: COMRESET and a new initialization sequence. */
void sata_host_comreset(sata_host_t *h);

/* SStatus and SError as defined in 10.1 (SError: DIAG field only). */
uint32_t sata_host_sstatus(const sata_host_t *h);
uint32_t sata_host_serror(const sata_host_t *h);
void     sata_host_serror_clear(sata_host_t *h, uint32_t bits);

/* ---- command helpers; all return what sata_host_start() returns -------- */

/* IDENTIFY DEVICE into a 512 byte buffer. */
sata_host_result_t sata_host_identify(sata_host_t *h, uint8_t *id,
                                      uint32_t timeout);
/* READ / WRITE DMA EXT, 'nsect' = 1..65536 sectors of 512 bytes. */
sata_host_result_t sata_host_read_dma(sata_host_t *h, uint64_t lba,
                                      uint32_t nsect, uint8_t *buf,
                                      uint32_t timeout);
sata_host_result_t sata_host_write_dma(sata_host_t *h, uint64_t lba,
                                       uint32_t nsect, const uint8_t *buf,
                                       uint32_t timeout);
/* READ / WRITE SECTOR(S) EXT (PIO protocol). */
sata_host_result_t sata_host_read_pio(sata_host_t *h, uint64_t lba,
                                      uint32_t nsect, uint8_t *buf,
                                      uint32_t timeout);
sata_host_result_t sata_host_write_pio(sata_host_t *h, uint64_t lba,
                                       uint32_t nsect, const uint8_t *buf,
                                       uint32_t timeout);
/* FLUSH CACHE EXT */
sata_host_result_t sata_host_flush(sata_host_t *h, uint32_t timeout);

/* The interesting parts of IDENTIFY DEVICE data. */
typedef struct {
    char     model[41];
    char     serial[21];
    char     firmware[9];
    bool     lba48;
    uint64_t sectors;
} sata_identify_t;

void sata_identify_parse(const uint8_t *id, sata_identify_t *out);

#endif /* SATA_HOST_H */
