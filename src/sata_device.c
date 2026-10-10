/*
 * Serial ATA 1.0a - device port: Phy + Link + device Transport layer (8.7)
 * and the device command layer (section 9).
 */
#include "sata_device.h"

#include <string.h>

enum {
    D_IDLE,         /* DI0                                               */
    D_SRST,         /* DSR0: SRST is set, waiting for it to clear        */
    D_SIGNATURE,    /* DHR2 / DSR2 / DEDD1: sending the signature        */
    D_STATUS,       /* sending the Register FIS that ends a command      */
    D_PIOIN_SETUP,  /* DPIOI1                                            */
    D_PIOIN_DATA,   /* DPIOI2                                            */
    D_PIOOUT_SETUP, /* DPIOO1                                            */
    D_PIOOUT_DATA,  /* DPIOO2: waiting for the Data FIS                  */
    D_DMAIN_DATA,   /* DDMAI1                                            */
    D_DMAOUT_ACT,   /* DDMAO1                                            */
    D_DMAOUT_DATA,  /* DDMAO2: waiting for the Data FIS                  */
};

#define MAX_RETRIES 8
#define DMA_CHUNK   (SATA_MAX_DATA_BYTES / SATA_SECTOR_SIZE)

#define ST_GOOD (ATA_SR_DRDY | ATA_SR_DSC)
#define ST_FAIL (ATA_SR_DRDY | ATA_SR_DSC | ATA_SR_ERR)

void sata_device_init(sata_device_t *dev, const sata_disk_t *disk)
{
    memset(dev, 0, offsetof(sata_device_t, fis));
    sata_phy_init(&dev->phy, SATA_ROLE_DEVICE);
    sata_link_init(&dev->link, SATA_ROLE_DEVICE);
    dev->disk = *disk;
}

/* Transmit the FIS in dev->fis and move on to 'state'. */
static void send(sata_device_t *dev, uint8_t state)
{
    dev->state = state;
    dev->retries = 0;
    dev->tx_pending = sata_link_tx_start(&dev->link, dev->fis, dev->fis_len);
    if (!dev->tx_pending) {
        dev->diag |= SATA_DIAG_T;
        dev->state = D_IDLE;
    }
}

/* End the command with a Register - Device to Host FIS. */
static void send_status(sata_device_t *dev, uint8_t status, uint8_t error)
{
    sata_fis_d2h_t f = { .i = true, .status = status, .error = error,
                         .regs = dev->cmd.regs };
    dev->fis_len = sata_fis_build_d2h(dev->fis, &f);
    send(dev, D_STATUS);
}

/* Reset signature of a device without the PACKET feature set (9.1). */
static void send_signature(sata_device_t *dev, bool irq)
{
    sata_fis_d2h_t f = { .i = irq, .status = ST_GOOD, .error = 0x01,
                         .regs = { .lba = 1, .count = 1 } };
    dev->fis_len = sata_fis_build_d2h(dev->fis, &f);
    send(dev, D_SIGNATURE);
}

static void send_pio_setup(sata_device_t *dev, bool d2h, bool irq,
                           uint8_t e_status, uint8_t state)
{
    sata_fis_pio_setup_t f = {
        .i = irq,
        .d = d2h,
        .status = ST_GOOD | ATA_SR_DRQ,
        .e_status = e_status,
        .xfer_count = SATA_SECTOR_SIZE,
        .regs = dev->cmd.regs,
    };
    dev->fis_len = sata_fis_build_pio_setup(dev->fis, &f);
    send(dev, state);
}

/* ---- command protocols (9.7 - 9.10) ------------------------------------ */

static void pioin_next(sata_device_t *dev)
{
    if (!dev->raw && !dev->disk.read(dev->disk.ctx, dev->lba, 1, dev->data)) {
        send_status(dev, ST_FAIL, ATA_ER_UNC);
        return;
    }
    /* After the last DRQ block the command is over without another FIS. */
    send_pio_setup(dev, true, true, dev->left > 1 ? ATA_SR_BSY : ST_GOOD,
                   D_PIOIN_SETUP);
}

static void pioout_next(sata_device_t *dev)
{
    if (dev->left == 0)
        send_status(dev, ST_GOOD, 0);
    else
        send_pio_setup(dev, false, !dev->first, ATA_SR_BSY, D_PIOOUT_SETUP);
}

static void dmain_next(sata_device_t *dev)
{
    if (dev->left == 0) {
        send_status(dev, ST_GOOD, 0);
        return;
    }
    dev->chunk = dev->left < DMA_CHUNK ? (unsigned)dev->left : DMA_CHUNK;
    if (!dev->disk.read(dev->disk.ctx, dev->lba, dev->chunk, dev->data)) {
        send_status(dev, ST_FAIL, ATA_ER_UNC);
        return;
    }
    dev->fis_len = sata_fis_build_data(dev->fis, dev->data,
                                       dev->chunk * SATA_SECTOR_SIZE);
    send(dev, D_DMAIN_DATA);
}

static void dmaout_next(sata_device_t *dev)
{
    if (dev->left == 0) {
        send_status(dev, ST_GOOD, 0);
        return;
    }
    dev->fis_len = sata_fis_build_dma_activate(dev->fis);
    send(dev, D_DMAOUT_ACT);
}

/* Take LBA and sector count from the command; false if out of range. */
static bool setup_range(sata_device_t *dev, bool ext)
{
    const sata_regs_t *r = &dev->cmd.regs;

    if (!(r->device & ATA_DH_LBA))
        return false; /* CHS addressing is not supported */
    if (ext) {
        dev->lba = r->lba;
        dev->left = r->count ? r->count : 65536u;
    } else {
        dev->lba = (r->lba & 0xFFFFFFu) | (uint64_t)(r->device & 0x0Fu) << 24;
        dev->left = (r->count & 0xFFu) ? (r->count & 0xFFu) : 256u;
    }
    return dev->lba < dev->disk.sectors &&
           dev->left <= dev->disk.sectors - dev->lba;
}

/* DI2: Check_command */
static void exec(sata_device_t *dev)
{
    bool ext = false;

    dev->first = true;
    dev->raw = false;

    switch (dev->cmd.command) {
    case ATA_CMD_READ_SECTORS_EXT:
    case ATA_CMD_WRITE_SECTORS_EXT:
    case ATA_CMD_READ_DMA_EXT:
    case ATA_CMD_WRITE_DMA_EXT:
    case ATA_CMD_READ_VERIFY_EXT:
        ext = true;
        /* fall through */
    case ATA_CMD_READ_SECTORS:
    case ATA_CMD_WRITE_SECTORS:
    case ATA_CMD_READ_DMA:
    case ATA_CMD_WRITE_DMA:
    case ATA_CMD_READ_VERIFY:
        if (!setup_range(dev, ext)) {
            send_status(dev, ST_FAIL, ATA_ER_IDNF | ATA_ER_ABRT);
            break;
        }
        switch (dev->cmd.command) {
        case ATA_CMD_READ_SECTORS:
        case ATA_CMD_READ_SECTORS_EXT:
            pioin_next(dev);
            break;
        case ATA_CMD_WRITE_SECTORS:
        case ATA_CMD_WRITE_SECTORS_EXT:
            pioout_next(dev);
            break;
        case ATA_CMD_READ_DMA:
        case ATA_CMD_READ_DMA_EXT:
            dmain_next(dev);
            break;
        case ATA_CMD_WRITE_DMA:
        case ATA_CMD_WRITE_DMA_EXT:
            dmaout_next(dev);
            break;
        default: /* READ VERIFY: the range is good */
            send_status(dev, ST_GOOD, 0);
            break;
        }
        break;

    case ATA_CMD_IDENTIFY_DEVICE:
        sata_device_identify(&dev->disk, dev->data);
        dev->raw = true;
        dev->left = 1;
        pioin_next(dev);
        break;

    case ATA_CMD_FLUSH_CACHE:
    case ATA_CMD_FLUSH_CACHE_EXT:
        if (dev->disk.flush && !dev->disk.flush(dev->disk.ctx))
            send_status(dev, ST_FAIL | ATA_SR_DF, ATA_ER_ABRT);
        else
            send_status(dev, ST_GOOD, 0);
        break;

    case ATA_CMD_CHECK_POWER_MODE:
        dev->cmd.regs.count = 0xFF; /* active */
        send_status(dev, ST_GOOD, 0);
        break;

    case ATA_CMD_SET_FEATURES:
    case ATA_CMD_INIT_DEV_PARAMS:
    case ATA_CMD_IDLE_IMMEDIATE:
    case ATA_CMD_STANDBY_IMMEDIATE:
        send_status(dev, ST_GOOD, 0);
        break;

    case ATA_CMD_EXEC_DEV_DIAG:
        send_signature(dev, true);
        break;

    default: /* DI3: No_command */
        send_status(dev, ST_FAIL, ATA_ER_ABRT);
        break;
    }
}

/* ---- transport --------------------------------------------------------- */

static void tx_ok(sata_device_t *dev)
{
    switch (dev->state) {
    case D_PIOIN_SETUP:
        dev->fis_len =
            sata_fis_build_data(dev->fis, dev->data, SATA_SECTOR_SIZE);
        send(dev, D_PIOIN_DATA);
        break;
    case D_PIOIN_DATA:
        dev->lba++;
        if (--dev->left)
            pioin_next(dev);
        else
            dev->state = D_IDLE;
        break;
    case D_PIOOUT_SETUP:
        dev->state = D_PIOOUT_DATA;
        break;
    case D_DMAIN_DATA:
        dev->lba += dev->chunk;
        dev->left -= dev->chunk;
        dmain_next(dev);
        break;
    case D_DMAOUT_ACT:
        dev->state = D_DMAOUT_DATA;
        break;
    default: /* status or signature delivered */
        dev->state = D_IDLE;
        break;
    }
}

static void tx_failed(sata_device_t *dev)
{
    if (dev->state == D_PIOIN_DATA || dev->state == D_DMAIN_DATA) {
        /* Data is never sent twice; the command fails (11.4.2.2). */
        send_status(dev, ST_FAIL, ATA_ER_ICRC | ATA_ER_ABRT);
    } else if (dev->retries < MAX_RETRIES &&
               sata_link_tx_start(&dev->link, dev->fis, dev->fis_len)) {
        dev->retries++;
        dev->tx_pending = true;
    } else {
        dev->state = D_IDLE;
    }
}

/* A Data FIS arrived while one was awaited (DPIOO2, DDMAO2). */
static void rx_data(sata_device_t *dev, const uint32_t *fis, unsigned len)
{
    unsigned n = (len - 1u) * 4u / SATA_SECTOR_SIZE;

    if (dev->state == D_PIOOUT_DATA && n > 1)
        n = 1;
    if (n > dev->left)
        n = (unsigned)dev->left;
    if (n == 0) {
        send_status(dev, ST_FAIL, ATA_ER_ABRT);
        return;
    }
    sata_fis_data_payload(fis, len, dev->data, n * SATA_SECTOR_SIZE);
    if (!dev->disk.write(dev->disk.ctx, dev->lba, n, dev->data)) {
        send_status(dev, ST_FAIL | ATA_SR_DF, ATA_ER_ABRT);
        return;
    }
    dev->lba += n;
    dev->left -= n;
    dev->first = false;
    if (dev->state == D_PIOOUT_DATA)
        pioout_next(dev);
    else
        dmaout_next(dev);
}

/* DTI1 / DI1; returns false to answer the FIS with R_ERR. */
static bool rx_fis(sata_device_t *dev, const uint32_t *fis, unsigned len)
{
    switch (sata_fis_type(fis)) {
    case SATA_FIS_REG_H2D: {
        sata_fis_h2d_t f;
        if (!sata_fis_parse_h2d(fis, len, &f)) {
            dev->diag |= SATA_DIAG_T;
            return false;
        }
        if (!f.c) {
            if (f.control & ATA_DC_SRST) {
                /* DSR0: drop everything, even a FIS waiting to go out */
                if (dev->tx_pending && sata_link_tx_cancel(&dev->link))
                    dev->tx_pending = false;
                dev->state = D_SRST;
            } else if (dev->state == D_SRST && !dev->tx_pending) {
                send_signature(dev, false);
            }
        } else if (dev->state == D_IDLE) {
            dev->cmd = f;
            exec(dev);
        }
        /* A command while busy is ignored, as a parallel device would. */
        return true;
    }

    case SATA_FIS_DATA:
        if (len < 2) {
            dev->diag |= SATA_DIAG_T;
            return false;
        }
        if (dev->state == D_PIOOUT_DATA || dev->state == D_DMAOUT_DATA)
            rx_data(dev, fis, len);
        return true;

    default:
        dev->diag |= SATA_DIAG_F;
        return false;
    }
}

static void transport(sata_device_t *dev, bool phy_ready)
{
    if (!phy_ready) {
        dev->up = false;
        dev->tx_pending = false;
        dev->state = D_IDLE;
        return;
    }
    if (!dev->up) {
        /* DHR2: communication is up after power-on or COMRESET */
        dev->up = true;
        send_signature(dev, false);
        return;
    }

    if (dev->tx_pending) {
        sata_tx_result_t res = sata_link_tx_result(&dev->link);
        if (res != SATA_TX_BUSY) {
            dev->tx_pending = false;
            sata_link_tx_ack(&dev->link);
            if (dev->state == D_SRST)
                ; /* result of a FIS overtaken by the reset */
            else if (res == SATA_TX_OK)
                tx_ok(dev);
            else
                tx_failed(dev);
        }
    }

    /* A damaged Data FIS ends a data-out command with an error (8.6.9). */
    if (sata_link_rx_failed(&dev->link) &&
        (dev->state == D_PIOOUT_DATA || dev->state == D_DMAOUT_DATA))
        send_status(dev, ST_FAIL, ATA_ER_ICRC | ATA_ER_ABRT);

    if (sata_link_rx_pending(&dev->link))
        sata_link_rx_done(&dev->link,
                          rx_fis(dev, dev->link.rx_buf, dev->link.rx_len));
}

void sata_device_tick(sata_device_t *dev, const sata_xcvr_rx_t *rx,
                      sata_xcvr_tx_t *tx)
{
    bool ready = sata_phy_tick(&dev->phy, rx, tx);
    sata_link_tick(&dev->link, ready, rx, tx);
    transport(dev, ready);
}

/* ---- IDENTIFY DEVICE data (5.2.5, ATA/ATAPI-5) ------------------------- */

static void id_put(uint8_t *id, unsigned word, unsigned value)
{
    id[2 * word] = (uint8_t)value;
    id[2 * word + 1] = (uint8_t)(value >> 8);
}

/* ATA strings: space padded, first character in the high byte of a word. */
static void id_put_string(uint8_t *id, unsigned word, unsigned nwords,
                          const char *s)
{
    for (unsigned i = 0; i < 2 * nwords; i++) {
        char c = ' ';
        if (s && *s)
            c = *s++;
        id[2 * word + (i ^ 1u)] = (uint8_t)c;
    }
}

void sata_device_identify(const sata_disk_t *disk, uint8_t *id)
{
    uint64_t n = disk->sectors;
    uint32_t n28 = n > 0x0FFFFFFFu ? 0x0FFFFFFFu : (uint32_t)n;

    memset(id, 0, SATA_SECTOR_SIZE);
    id_put(id, 0, 0x0040);  /* ATA device, fixed media */
    id_put(id, 1, 16383);   /* default CHS translation */
    id_put(id, 3, 16);
    id_put(id, 6, 63);
    id_put_string(id, 10, 10, disk->serial);
    id_put_string(id, 23, 4, disk->firmware);
    id_put_string(id, 27, 20, disk->model);
    id_put(id, 47, 0x8000); /* READ/WRITE MULTIPLE not supported */
    id_put(id, 49, 0x0F00); /* IORDY, IORDY disable, LBA, DMA */
    id_put(id, 53, 0x0007); /* words 54-58, 64-70 and 88 are valid */
    id_put(id, 60, n28 & 0xFFFFu);
    id_put(id, 61, n28 >> 16);
    id_put(id, 63, 0x0007); /* Multiword DMA modes 0-2 */
    id_put(id, 64, 0x0003); /* PIO modes 3 and 4 */
    id_put(id, 65, 120);    /* cycle times, ns */
    id_put(id, 66, 120);
    id_put(id, 67, 120);
    id_put(id, 68, 120);
    id_put(id, 80, 0x003E); /* ATA-1 to ATA/ATAPI-5 */
    id_put(id, 83, 0x7400); /* FLUSH CACHE (EXT), 48 bit addressing */
    id_put(id, 84, 0x4000);
    id_put(id, 86, 0x3400);
    id_put(id, 87, 0x4000);
    id_put(id, 88, 0x001F); /* Ultra DMA modes 0-4 */
    id_put(id, 93, 0x0000);
    for (unsigned i = 0; i < 4; i++)
        id_put(id, 100 + i, (unsigned)(n >> (16 * i)) & 0xFFFFu);
}
