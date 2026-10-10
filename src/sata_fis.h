/*
 * Serial ATA 1.0a - Frame Information Structures (8.5).
 *
 * Build functions fill a Dword buffer and return the FIS length in Dwords;
 * parse functions return false if type or length do not match.
 */
#ifndef SATA_FIS_H
#define SATA_FIS_H

#include "sata_defs.h"

/* FIS lengths in Dwords */
#define SATA_FIS_REG_LEN          5u
#define SATA_FIS_SET_DEV_BITS_LEN 2u
#define SATA_FIS_DMA_ACTIVATE_LEN 1u
#define SATA_FIS_DMA_SETUP_LEN    7u
#define SATA_FIS_BIST_LEN         3u
#define SATA_FIS_PIO_SETUP_LEN    5u

/*
 * Taskfile registers shared by the register type FIS's. 'lba' is 48 bits:
 * Sector Number = 7:0, Cylinder Low = 15:8, Cylinder High = 23:16 and their
 * "exp" (previous content) counterparts 31:24, 39:32, 47:40. 'count' is
 * Sector Count with Sector Count (exp) in 15:8.
 */
typedef struct {
    uint64_t lba;
    uint16_t count;
    uint8_t  device; /* Device/Head */
} sata_regs_t;

/* Register - Host to Device (8.5.2) */
typedef struct {
    bool        c;        /* 1 = Command register written, 0 = Device Control */
    uint8_t     command;
    uint16_t    features; /* Features, Features (exp) in 15:8 */
    uint8_t     control;  /* Device Control */
    sata_regs_t regs;
} sata_fis_h2d_t;

/* Register - Device to Host (8.5.3) */
typedef struct {
    bool        i; /* interrupt */
    uint8_t     status;
    uint8_t     error;
    sata_regs_t regs;
} sata_fis_d2h_t;

/* Set Device Bits - Device to Host (8.5.4) */
typedef struct {
    bool    i;
    uint8_t status; /* only bits 6:4 and 2:0 are carried */
    uint8_t error;
} sata_fis_sdb_t;

/* PIO Setup - Device to Host (8.5.8) */
typedef struct {
    bool        i;
    bool        d;        /* 1 = data goes device to host */
    uint8_t     status;   /* Status at the start of the data transfer */
    uint8_t     error;
    uint8_t     e_status; /* Status once the data has been transferred */
    uint16_t    xfer_count; /* bytes in the Data FIS that follows, even */
    sata_regs_t regs;
} sata_fis_pio_setup_t;

/* DMA Setup - bidirectional (8.5.6) */
typedef struct {
    bool     i;
    bool     d; /* 1 = data goes from the sender of this FIS to its receiver */
    uint64_t buffer_id;
    uint32_t buffer_offset;
    uint32_t xfer_count;
} sata_fis_dma_setup_t;

/* BIST Activate - bidirectional (8.5.7) */
#define SATA_BIST_T 0x80u /* far end transmit only            */
#define SATA_BIST_A 0x40u /* ALIGN bypass                     */
#define SATA_BIST_S 0x20u /* scrambling bypass                */
#define SATA_BIST_L 0x10u /* far end retimed loopback         */
#define SATA_BIST_F 0x08u /* far end analog loopback          */
#define SATA_BIST_P 0x04u /* primitive bit                    */
#define SATA_BIST_V 0x01u /* vendor unique                    */
typedef struct {
    uint8_t  pattern; /* SATA_BIST_* */
    uint32_t data[2];
} sata_fis_bist_t;

static inline uint8_t sata_fis_type(const uint32_t *fis)
{
    return (uint8_t)fis[0];
}

unsigned sata_fis_build_h2d(uint32_t *fis, const sata_fis_h2d_t *f);
unsigned sata_fis_build_d2h(uint32_t *fis, const sata_fis_d2h_t *f);
unsigned sata_fis_build_sdb(uint32_t *fis, const sata_fis_sdb_t *f);
unsigned sata_fis_build_pio_setup(uint32_t *fis, const sata_fis_pio_setup_t *f);
unsigned sata_fis_build_dma_activate(uint32_t *fis);
unsigned sata_fis_build_dma_setup(uint32_t *fis, const sata_fis_dma_setup_t *f);
unsigned sata_fis_build_bist(uint32_t *fis, const sata_fis_bist_t *f);
/*
 * Data FIS (8.5.9): 'nbytes' (1..8192, a whole number of 16 bit words) from
 * 'data', the last Dword zero padded.
 */
unsigned sata_fis_build_data(uint32_t *fis, const uint8_t *data,
                             unsigned nbytes);

bool sata_fis_parse_h2d(const uint32_t *fis, unsigned len, sata_fis_h2d_t *f);
bool sata_fis_parse_d2h(const uint32_t *fis, unsigned len, sata_fis_d2h_t *f);
bool sata_fis_parse_sdb(const uint32_t *fis, unsigned len, sata_fis_sdb_t *f);
bool sata_fis_parse_pio_setup(const uint32_t *fis, unsigned len,
                              sata_fis_pio_setup_t *f);
bool sata_fis_parse_dma_setup(const uint32_t *fis, unsigned len,
                              sata_fis_dma_setup_t *f);
bool sata_fis_parse_bist(const uint32_t *fis, unsigned len, sata_fis_bist_t *f);
/* Copy up to 'nbytes' of Data FIS payload out; returns the bytes copied. */
unsigned sata_fis_data_payload(const uint32_t *fis, unsigned len,
                               uint8_t *data, unsigned nbytes);

#endif /* SATA_FIS_H */
