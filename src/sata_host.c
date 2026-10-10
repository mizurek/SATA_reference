/*
 * Serial ATA 1.0a - host port: Phy + Link + host Transport layer (8.6).
 */
#include "sata_host.h"

#include <string.h>

enum { TX_NONE, TX_CMD, TX_CTRL, TX_PIO_DATA, TX_DMA_DATA };

/* Software reset sequence */
enum { SRST_NONE, SRST_ASSERT, SRST_NEGATE, SRST_WAIT };

#define MAX_RETRIES 8

void sata_host_init(sata_host_t *h)
{
    memset(h, 0, offsetof(sata_host_t, fis));
    sata_phy_init(&h->phy, SATA_ROLE_HOST);
    sata_link_init(&h->link, SATA_ROLE_HOST);
    h->status = 0x7F; /* no device detected yet (5.2) */
    h->error = 0xFF;
    h->result = SATA_HOST_NOT_READY;
}

bool sata_host_ready(const sata_host_t *h)
{
    return h->up && !h->active && h->srst == SRST_NONE &&
           h->tx_kind == TX_NONE && !(h->status & (ATA_SR_BSY | ATA_SR_DRQ));
}

sata_host_result_t sata_host_result(const sata_host_t *h)
{
    return h->active ? SATA_HOST_BUSY : h->result;
}

static void finish(sata_host_t *h, sata_host_result_t result)
{
    if (!h->active)
        return;
    h->active = false;
    h->pio_valid = false;
    h->result = result;
}

static void send(sata_host_t *h, uint8_t kind)
{
    if (sata_link_tx_start(&h->link, h->fis, h->fis_len)) {
        h->tx_kind = kind;
    } else {
        /* The device asked for data while a frame was still queued. */
        h->diag |= SATA_DIAG_T;
        h->xfer_err = true;
    }
}

static void send_control(sata_host_t *h)
{
    sata_fis_h2d_t f = { .c = false, .control = h->control, .regs = h->regs };
    h->fis_len = sata_fis_build_h2d(h->fis, &f);
    h->retries = 0;
    send(h, TX_CTRL);
}

/* Data FIS of 'n' bytes from the command buffer, zero filled past its end. */
static void send_data(sata_host_t *h, unsigned n, uint8_t kind)
{
    uint32_t left = h->buf ? h->buf_len - h->done : 0;
    unsigned avail = n < left ? n : (unsigned)left;
    unsigned total = 1u + (n + 3u) / 4u;

    h->fis_len = sata_fis_build_data(h->fis, avail ? h->buf + h->done : NULL,
                                     avail);
    while (h->fis_len < total)
        h->fis[h->fis_len++] = 0;
    h->chunk = avail;
    send(h, kind);
}

sata_host_result_t sata_host_start(sata_host_t *h, const sata_fis_h2d_t *cmd,
                                   void *buf, uint32_t nbytes,
                                   uint32_t timeout)
{
    if (!sata_host_ready(h))
        return SATA_HOST_NOT_READY;

    sata_fis_h2d_t f = *cmd;
    f.c = true;
    f.control = h->control;
    h->fis_len = sata_fis_build_h2d(h->fis, &f);
    h->retries = 0;

    /* Writing the Command register sets BSY and clears the interrupt. */
    h->regs = f.regs;
    h->status |= ATA_SR_BSY;
    h->irq = false;

    h->active = true;
    h->cmd_sent = false;
    h->xfer_err = false;
    h->pio_valid = false;
    h->buf = buf;
    h->buf_len = buf ? nbytes : 0;
    h->done = 0;
    h->timeout = timeout;
    send(h, TX_CMD);
    return SATA_HOST_BUSY;
}

void sata_host_soft_reset(sata_host_t *h)
{
    if (!h->up)
        return;
    finish(h, SATA_HOST_ABORTED);
    sata_link_abort(&h->link);
    h->tx_kind = TX_NONE;
    h->pio_valid = false;
    h->status |= ATA_SR_BSY;
    h->irq = false;
    h->control |= ATA_DC_SRST;
    h->srst = SRST_ASSERT;
    send_control(h);
}

void sata_host_comreset(sata_host_t *h)
{
    finish(h, SATA_HOST_ABORTED);
    sata_phy_reset(&h->phy, false);
}

uint32_t sata_host_sstatus(const sata_host_t *h)
{
    if (h->phy.state == SATA_PHY_OFFLINE)
        return 0x004;
    if (sata_phy_ready(&h->phy))
        return 0x113; /* IPM active, Gen1, device present and talking */
    return h->phy.device_seen ? 0x001 : 0x000;
}

uint32_t sata_host_serror(const sata_host_t *h)
{
    return (h->phy.diag | h->link.diag | h->diag) << 16;
}

void sata_host_serror_clear(sata_host_t *h, uint32_t bits)
{
    bits = ~(bits >> 16);
    h->phy.diag &= bits;
    h->link.diag &= bits;
    h->diag &= bits;
}

/* A frame we sent has been answered (HTCM2, HTCR2, HTDA4, HTPS4). */
static void tx_done(sata_host_t *h, sata_tx_result_t res)
{
    uint8_t kind = h->tx_kind;
    h->tx_kind = TX_NONE;
    sata_link_tx_ack(&h->link);

    if (res != SATA_TX_OK) {
        if (kind == TX_CMD || kind == TX_CTRL) {
            /* Register FIS's change no state on failure and are retried. */
            if (h->retries++ < MAX_RETRIES) {
                send(h, kind);
            } else {
                h->srst = SRST_NONE;
                finish(h, SATA_HOST_LINK_ERROR);
            }
        } else {
            /*
             * Data is never retried; the device reports the error with a
             * Register FIS, which ends the command.
             */
            h->xfer_err = true;
        }
        return;
    }

    switch (kind) {
    case TX_CMD:
        h->cmd_sent = true;
        break;
    case TX_CTRL:
        if (h->srst == SRST_ASSERT) {
            h->control &= (uint8_t)~ATA_DC_SRST;
            h->srst = SRST_NEGATE;
            send_control(h);
        } else if (h->srst == SRST_NEGATE) {
            h->srst = SRST_WAIT;
        }
        break;
    case TX_PIO_DATA:
        h->done += h->chunk;
        h->status = h->pio.e_status;
        h->pio_valid = false;
        break;
    case TX_DMA_DATA: {
        /* Less than queued went out if the device sent DMAT. */
        unsigned sent = (sata_link_tx_sent(&h->link) - 1u) * 4u;
        h->done += sent < h->chunk ? sent : h->chunk;
        break;
    }
    default:
        break;
    }
}

static void load_shadow(sata_host_t *h, uint8_t status, uint8_t error,
                        const sata_regs_t *regs, bool irq)
{
    h->status = status;
    h->error = error;
    h->regs = *regs;
    if (irq)
        h->irq = true;
}

/*
 * Decompose a received FIS (HTI2 and the diagrams it branches to). Returns
 * false if the FIS is to be answered with R_ERR.
 */
static bool rx_fis(sata_host_t *h, const uint32_t *fis, unsigned len)
{
    switch (sata_fis_type(fis)) {
    case SATA_FIS_REG_D2H: {
        sata_fis_d2h_t f;
        if (!sata_fis_parse_d2h(fis, len, &f))
            break;
        /*
         * Dropped unless the device owns the registers (8.5.3.3), and while
         * a software reset is on its way: that would be the end of a
         * command the reset is about to kill.
         */
        if ((h->status & (ATA_SR_BSY | ATA_SR_DRQ)) &&
            h->srst != SRST_ASSERT && h->srst != SRST_NEGATE) {
            load_shadow(h, f.status, f.error, &f.regs, f.i);
            h->pio_valid = false;
            if (h->srst == SRST_WAIT)
                h->srst = SRST_NONE;
        }
        return true;
    }

    case SATA_FIS_SET_DEV_BITS: {
        sata_fis_sdb_t f;
        if (!sata_fis_parse_sdb(fis, len, &f))
            break;
        h->error = f.error;
        h->status = (uint8_t)((h->status & 0x88u) | f.status);
        if (f.i && !(h->status & (ATA_SR_BSY | ATA_SR_DRQ)))
            h->irq = true;
        return true;
    }

    case SATA_FIS_PIO_SETUP: {
        sata_fis_pio_setup_t f;
        if (!sata_fis_parse_pio_setup(fis, len, &f) || f.xfer_count == 0 ||
            (f.xfer_count & 1u) || f.xfer_count > SATA_MAX_DATA_BYTES)
            break;
        h->pio = f;
        h->pio_valid = true;
        if (!f.d) {
            /* HTPS2: show DRQ, then send the data the device asked for. */
            load_shadow(h, f.status, f.error, &f.regs, f.i);
            send_data(h, f.xfer_count, TX_PIO_DATA);
        }
        return true;
    }

    case SATA_FIS_DMA_ACTIVATE:
        if (len != SATA_FIS_DMA_ACTIVATE_LEN)
            break;
        if (h->active && h->buf && h->done < h->buf_len) {
            uint32_t left = h->buf_len - h->done;
            send_data(h,
                      left < SATA_MAX_DATA_BYTES ? (unsigned)left
                                                 : SATA_MAX_DATA_BYTES,
                      TX_DMA_DATA);
        } else {
            h->diag |= SATA_DIAG_T; /* nothing to send */
            h->xfer_err = true;
        }
        return true;

    case SATA_FIS_DATA: {
        if (len < 2)
            break;
        unsigned payload = (len - 1u) * 4u;
        uint32_t room = h->buf ? h->buf_len - h->done : 0;

        if (h->pio_valid && h->pio.d) {
            /* HTPS5/6: PIO data in, exactly as announced by PIO Setup. */
            unsigned n = h->pio.xfer_count;
            if (payload != (n + 3u) / 4u * 4u) {
                h->diag |= SATA_DIAG_T;
                return false;
            }
            if (n > room) {
                n = (unsigned)room;
                h->xfer_err = true;
            }
            if (n)
                h->done +=
                    sata_fis_data_payload(fis, len, h->buf + h->done, n);
            load_shadow(h, h->pio.e_status, h->pio.error, &h->pio.regs,
                        h->pio.i);
            h->pio_valid = false;
        } else if (h->active) {
            /* HTDA5: DMA data in */
            unsigned n = payload;
            if (n > room) {
                n = (unsigned)room;
                h->xfer_err = true;
            }
            if (n)
                h->done +=
                    sata_fis_data_payload(fis, len, h->buf + h->done, n);
        }
        return true;
    }

    default:
        /* DMA Setup, BIST Activate and anything unknown */
        h->diag |= SATA_DIAG_F;
        return false;
    }

    h->diag |= SATA_DIAG_T; /* malformed */
    return false;
}

static void transport(sata_host_t *h, bool phy_ready)
{
    if (!phy_ready) {
        if (h->up) {
            h->up = false;
            h->tx_kind = TX_NONE;
            h->srst = SRST_NONE;
            h->control &= (uint8_t)~ATA_DC_SRST;
            finish(h, SATA_HOST_LINK_ERROR);
        }
        /* 5.2: 7Fh until a device is detected, then BSY */
        h->status = h->phy.device_seen ? 0xFF : 0x7F;
        h->error = 0xFF;
        return;
    }
    if (!h->up) {
        h->up = true;
        h->status = 0xFF; /* until the signature FIS arrives */
    }

    sata_tx_result_t res = sata_link_tx_result(&h->link);
    if (h->tx_kind != TX_NONE && res != SATA_TX_BUSY)
        tx_done(h, res);

    if (sata_link_rx_pending(&h->link))
        sata_link_rx_done(&h->link,
                          rx_fis(h, h->link.rx_buf, h->link.rx_len));

    if (!h->active)
        return;

    /* The command is over when the device gave the registers back. */
    if (h->cmd_sent && h->tx_kind == TX_NONE &&
        !(h->status & (ATA_SR_BSY | ATA_SR_DRQ))) {
        if (h->status & (ATA_SR_ERR | ATA_SR_DF))
            finish(h, SATA_HOST_DEV_ERROR);
        else
            finish(h, h->xfer_err ? SATA_HOST_LINK_ERROR : SATA_HOST_OK);
    } else if (h->timeout && --h->timeout == 0) {
        sata_link_abort(&h->link);
        h->tx_kind = TX_NONE;
        finish(h, SATA_HOST_TIMEOUT);
    }
}

void sata_host_tick(sata_host_t *h, const sata_xcvr_rx_t *rx,
                    sata_xcvr_tx_t *tx)
{
    bool ready = sata_phy_tick(&h->phy, rx, tx);
    sata_link_tick(&h->link, ready, rx, tx);
    transport(h, ready);
}

/* ---- command helpers --------------------------------------------------- */

sata_host_result_t sata_host_identify(sata_host_t *h, uint8_t *id,
                                      uint32_t timeout)
{
    sata_fis_h2d_t f = { .command = ATA_CMD_IDENTIFY_DEVICE };
    return sata_host_start(h, &f, id, SATA_SECTOR_SIZE, timeout);
}

static sata_host_result_t rw_ext(sata_host_t *h, uint8_t command, uint64_t lba,
                                 uint32_t nsect, uint8_t *buf,
                                 uint32_t timeout)
{
    if (nsect == 0 || nsect > 65536u)
        return SATA_HOST_NOT_READY;
    sata_fis_h2d_t f = {
        .command = command,
        .regs = { .lba = lba & 0xFFFFFFFFFFFFull,
                  .count = (uint16_t)nsect, /* 0 means 65536 */
                  .device = ATA_DH_LBA },
    };
    return sata_host_start(h, &f, buf, nsect * SATA_SECTOR_SIZE, timeout);
}

sata_host_result_t sata_host_read_dma(sata_host_t *h, uint64_t lba,
                                      uint32_t nsect, uint8_t *buf,
                                      uint32_t timeout)
{
    return rw_ext(h, ATA_CMD_READ_DMA_EXT, lba, nsect, buf, timeout);
}

sata_host_result_t sata_host_write_dma(sata_host_t *h, uint64_t lba,
                                       uint32_t nsect, const uint8_t *buf,
                                       uint32_t timeout)
{
    /* The buffer is only read by a data-out command. */
    return rw_ext(h, ATA_CMD_WRITE_DMA_EXT, lba, nsect, (uint8_t *)buf,
                  timeout);
}

sata_host_result_t sata_host_read_pio(sata_host_t *h, uint64_t lba,
                                      uint32_t nsect, uint8_t *buf,
                                      uint32_t timeout)
{
    return rw_ext(h, ATA_CMD_READ_SECTORS_EXT, lba, nsect, buf, timeout);
}

sata_host_result_t sata_host_write_pio(sata_host_t *h, uint64_t lba,
                                       uint32_t nsect, const uint8_t *buf,
                                       uint32_t timeout)
{
    return rw_ext(h, ATA_CMD_WRITE_SECTORS_EXT, lba, nsect, (uint8_t *)buf,
                  timeout);
}

sata_host_result_t sata_host_flush(sata_host_t *h, uint32_t timeout)
{
    sata_fis_h2d_t f = { .command = ATA_CMD_FLUSH_CACHE_EXT,
                         .regs = { .device = ATA_DH_LBA } };
    return sata_host_start(h, &f, NULL, 0, timeout);
}

static unsigned id_word(const uint8_t *id, unsigned w)
{
    return id[2 * w] | (unsigned)id[2 * w + 1] << 8;
}

/* ATA strings: two characters per word, the first in the high byte. */
static void id_string(const uint8_t *id, unsigned word, unsigned nwords,
                      char *out)
{
    unsigned n = 0;
    for (unsigned w = word; w < word + nwords; w++) {
        out[n++] = (char)id[2 * w + 1];
        out[n++] = (char)id[2 * w];
    }
    while (n && (out[n - 1] == ' ' || out[n - 1] == '\0'))
        n--;
    out[n] = '\0';
}

void sata_identify_parse(const uint8_t *id, sata_identify_t *out)
{
    id_string(id, 10, 10, out->serial);
    id_string(id, 23, 4, out->firmware);
    id_string(id, 27, 20, out->model);
    out->lba48 = (id_word(id, 83) & 0x0400u) != 0;
    out->sectors = 0;
    if (out->lba48) {
        for (unsigned i = 0; i < 4; i++)
            out->sectors |= (uint64_t)id_word(id, 100 + i) << (16 * i);
    } else {
        out->sectors = id_word(id, 60) | (uint64_t)id_word(id, 61) << 16;
    }
}
