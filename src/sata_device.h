/*
 * Serial ATA 1.0a - device port: Phy + Link + device Transport layer (8.7)
 * and the device command layer (section 9) for a simple LBA disk.
 *
 * Command protocols implemented: non-data, PIO data-in, PIO data-out, DMA
 * data-in, DMA data-out, EXECUTE DEVICE DIAGNOSTIC and software reset. The
 * PACKET and queued DMA protocols are not (the device is not ATAPI and does
 * not advertise queuing); unknown commands are aborted.
 *
 * Storage is reached through sata_disk_t callbacks, which are called from
 * sata_device_tick() and must not block.
 */
#ifndef SATA_DEVICE_H
#define SATA_DEVICE_H

#include "sata_fis.h"
#include "sata_link.h"
#include "sata_phy.h"

typedef struct {
    void    *ctx;
    uint64_t sectors; /* capacity in 512 byte sectors */
    /* Transfer 'n' (1..16) sectors; return false on a media error. */
    bool (*read)(void *ctx, uint64_t lba, unsigned n, uint8_t *buf);
    bool (*write)(void *ctx, uint64_t lba, unsigned n, const uint8_t *buf);
    bool (*flush)(void *ctx); /* optional */
    const char *model;        /* up to 40 characters */
    const char *serial;       /* up to 20 characters */
    const char *firmware;     /* up to 8 characters  */
} sata_disk_t;

typedef struct {
    sata_phy_t  phy;
    sata_link_t link;
    sata_disk_t disk;

    uint32_t diag; /* SATA_DIAG_T / SATA_DIAG_F, sticky */

    /* Internal state */
    bool           up;
    uint8_t        state;
    bool           tx_pending;
    uint8_t        retries;
    unsigned       fis_len;
    sata_fis_h2d_t cmd;   /* command being executed            */
    uint64_t       lba;   /* next sector to transfer           */
    uint32_t       left;  /* sectors still to transfer         */
    unsigned       chunk; /* sectors in the current Data FIS   */
    bool           first; /* first DRQ block of the command    */
    bool           raw;   /* data[] is not from the disk       */
    uint32_t       fis[SATA_MAX_FIS_DWORDS];
    uint8_t        data[SATA_MAX_DATA_BYTES];
} sata_device_t;

void sata_device_init(sata_device_t *dev, const sata_disk_t *disk);

/* One Dword clock: rx from the transceiver in, tx to the transceiver out. */
void sata_device_tick(sata_device_t *dev, const sata_xcvr_rx_t *rx,
                      sata_xcvr_tx_t *tx);

/* Fill a 512 byte buffer with the IDENTIFY DEVICE data of 'disk' (5.2.5). */
void sata_device_identify(const sata_disk_t *disk, uint8_t *id);

#endif /* SATA_DEVICE_H */
