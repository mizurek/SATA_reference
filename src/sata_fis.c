/*
 * Serial ATA 1.0a - Frame Information Structures (8.5).
 */
#include "sata_fis.h"

/* Flag bits in byte 1 of Dword 0 */
#define FL_C 0x80u
#define FL_I 0x40u
#define FL_D 0x20u

static uint32_t hdr(uint8_t type, uint8_t flags, uint8_t b2, uint8_t b3)
{
    return (uint32_t)type | (uint32_t)flags << 8 | (uint32_t)b2 << 16 |
           (uint32_t)b3 << 24;
}

/* Dwords 1..3 of the register type FIS's, less the top byte of 2 and 3. */
static void put_regs(uint32_t *fis, const sata_regs_t *r)
{
    fis[1] = (uint32_t)(r->lba & 0xFFFFFFu) | (uint32_t)r->device << 24;
    fis[2] = (uint32_t)(r->lba >> 24) & 0xFFFFFFu;
    fis[3] = r->count;
}

static void get_regs(const uint32_t *fis, sata_regs_t *r)
{
    r->lba = (uint64_t)(fis[1] & 0xFFFFFFu) |
             (uint64_t)(fis[2] & 0xFFFFFFu) << 24;
    r->device = (uint8_t)(fis[1] >> 24);
    r->count = (uint16_t)fis[3];
}

unsigned sata_fis_build_h2d(uint32_t *fis, const sata_fis_h2d_t *f)
{
    fis[0] = hdr(SATA_FIS_REG_H2D, f->c ? FL_C : 0, f->command,
                 (uint8_t)f->features);
    put_regs(fis, &f->regs);
    fis[2] |= (uint32_t)(f->features >> 8) << 24;
    fis[3] |= (uint32_t)f->control << 24;
    fis[4] = 0;
    return SATA_FIS_REG_LEN;
}

bool sata_fis_parse_h2d(const uint32_t *fis, unsigned len, sata_fis_h2d_t *f)
{
    if (len != SATA_FIS_REG_LEN || sata_fis_type(fis) != SATA_FIS_REG_H2D)
        return false;
    f->c = (fis[0] >> 8 & FL_C) != 0;
    f->command = (uint8_t)(fis[0] >> 16);
    f->features = (uint16_t)(fis[0] >> 24 | (fis[2] >> 24) << 8);
    f->control = (uint8_t)(fis[3] >> 24);
    get_regs(fis, &f->regs);
    return true;
}

unsigned sata_fis_build_d2h(uint32_t *fis, const sata_fis_d2h_t *f)
{
    fis[0] = hdr(SATA_FIS_REG_D2H, f->i ? FL_I : 0, f->status, f->error);
    put_regs(fis, &f->regs);
    fis[4] = 0;
    return SATA_FIS_REG_LEN;
}

bool sata_fis_parse_d2h(const uint32_t *fis, unsigned len, sata_fis_d2h_t *f)
{
    if (len != SATA_FIS_REG_LEN || sata_fis_type(fis) != SATA_FIS_REG_D2H)
        return false;
    f->i = (fis[0] >> 8 & FL_I) != 0;
    f->status = (uint8_t)(fis[0] >> 16);
    f->error = (uint8_t)(fis[0] >> 24);
    get_regs(fis, &f->regs);
    return true;
}

unsigned sata_fis_build_sdb(uint32_t *fis, const sata_fis_sdb_t *f)
{
    fis[0] = hdr(SATA_FIS_SET_DEV_BITS, f->i ? FL_I : 0, f->status & 0x77u,
                 f->error);
    fis[1] = 0;
    return SATA_FIS_SET_DEV_BITS_LEN;
}

bool sata_fis_parse_sdb(const uint32_t *fis, unsigned len, sata_fis_sdb_t *f)
{
    if (len != SATA_FIS_SET_DEV_BITS_LEN ||
        sata_fis_type(fis) != SATA_FIS_SET_DEV_BITS)
        return false;
    f->i = (fis[0] >> 8 & FL_I) != 0;
    f->status = (uint8_t)(fis[0] >> 16) & 0x77u;
    f->error = (uint8_t)(fis[0] >> 24);
    return true;
}

unsigned sata_fis_build_pio_setup(uint32_t *fis, const sata_fis_pio_setup_t *f)
{
    fis[0] = hdr(SATA_FIS_PIO_SETUP, (uint8_t)((f->i ? FL_I : 0) | (f->d ? FL_D : 0)),
                 f->status, f->error);
    put_regs(fis, &f->regs);
    fis[3] |= (uint32_t)f->e_status << 24;
    fis[4] = f->xfer_count;
    return SATA_FIS_PIO_SETUP_LEN;
}

bool sata_fis_parse_pio_setup(const uint32_t *fis, unsigned len,
                              sata_fis_pio_setup_t *f)
{
    if (len != SATA_FIS_PIO_SETUP_LEN ||
        sata_fis_type(fis) != SATA_FIS_PIO_SETUP)
        return false;
    f->i = (fis[0] >> 8 & FL_I) != 0;
    f->d = (fis[0] >> 8 & FL_D) != 0;
    f->status = (uint8_t)(fis[0] >> 16);
    f->error = (uint8_t)(fis[0] >> 24);
    f->e_status = (uint8_t)(fis[3] >> 24);
    f->xfer_count = (uint16_t)fis[4];
    get_regs(fis, &f->regs);
    return true;
}

unsigned sata_fis_build_dma_activate(uint32_t *fis)
{
    fis[0] = hdr(SATA_FIS_DMA_ACTIVATE, 0, 0, 0);
    return SATA_FIS_DMA_ACTIVATE_LEN;
}

unsigned sata_fis_build_dma_setup(uint32_t *fis, const sata_fis_dma_setup_t *f)
{
    fis[0] = hdr(SATA_FIS_DMA_SETUP, (uint8_t)((f->i ? FL_I : 0) | (f->d ? FL_D : 0)), 0,
                 0);
    fis[1] = (uint32_t)f->buffer_id;
    fis[2] = (uint32_t)(f->buffer_id >> 32);
    fis[3] = 0;
    fis[4] = f->buffer_offset;
    fis[5] = f->xfer_count;
    fis[6] = 0;
    return SATA_FIS_DMA_SETUP_LEN;
}

bool sata_fis_parse_dma_setup(const uint32_t *fis, unsigned len,
                              sata_fis_dma_setup_t *f)
{
    if (len != SATA_FIS_DMA_SETUP_LEN ||
        sata_fis_type(fis) != SATA_FIS_DMA_SETUP)
        return false;
    f->i = (fis[0] >> 8 & FL_I) != 0;
    f->d = (fis[0] >> 8 & FL_D) != 0;
    f->buffer_id = (uint64_t)fis[1] | (uint64_t)fis[2] << 32;
    f->buffer_offset = fis[4];
    f->xfer_count = fis[5];
    return true;
}

unsigned sata_fis_build_bist(uint32_t *fis, const sata_fis_bist_t *f)
{
    fis[0] = hdr(SATA_FIS_BIST, 0, f->pattern, 0);
    fis[1] = f->data[0];
    fis[2] = f->data[1];
    return SATA_FIS_BIST_LEN;
}

bool sata_fis_parse_bist(const uint32_t *fis, unsigned len, sata_fis_bist_t *f)
{
    if (len != SATA_FIS_BIST_LEN || sata_fis_type(fis) != SATA_FIS_BIST)
        return false;
    f->pattern = (uint8_t)(fis[0] >> 16);
    f->data[0] = fis[1];
    f->data[1] = fis[2];
    return true;
}

unsigned sata_fis_build_data(uint32_t *fis, const uint8_t *data,
                             unsigned nbytes)
{
    unsigned ndw = (nbytes + 3u) / 4u;

    fis[0] = hdr(SATA_FIS_DATA, 0, 0, 0);
    for (unsigned i = 0; i < ndw; i++)
        fis[1 + i] = 0;
    /* Byte 0 of a Dword is the first one on the wire (7.2.3.1.2). */
    for (unsigned i = 0; i < nbytes; i++)
        fis[1 + i / 4u] |= (uint32_t)data[i] << (8u * (i % 4u));
    return 1u + ndw;
}

unsigned sata_fis_data_payload(const uint32_t *fis, unsigned len,
                               uint8_t *data, unsigned nbytes)
{
    if (len < 2 || sata_fis_type(fis) != SATA_FIS_DATA)
        return 0;
    unsigned avail = (len - 1u) * 4u;
    if (nbytes > avail)
        nbytes = avail;
    for (unsigned i = 0; i < nbytes; i++)
        data[i] = (uint8_t)(fis[1 + i / 4u] >> (8u * (i % 4u)));
    return nbytes;
}
