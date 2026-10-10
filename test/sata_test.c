/*
 * Self test: a host port and a device port (RAM disk) wired back to back
 * through a model of the serial line that does the 8b/10b coding, a few
 * Dwords of delay and, on request, bit errors.
 */
#include <stdio.h>
#include <string.h>

#include "sata_codec.h"
#include "sata_device.h"
#include "sata_host.h"

/* ---- tiny test framework ----------------------------------------------- */

static int failures;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
            failures++;                                                      \
        }                                                                    \
    } while (0)

static uint32_t rng = 0x12345678u;

static uint32_t rnd(void)
{
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return rng;
}

/* ---- RAM disk ---------------------------------------------------------- */

#define DISK_SECTORS 4096u

static uint8_t  disk_mem[DISK_SECTORS * SATA_SECTOR_SIZE];
static uint64_t disk_bad_lba = UINT64_MAX; /* reads of this sector fail */
static unsigned disk_flushes;

static bool disk_read(void *ctx, uint64_t lba, unsigned n, uint8_t *buf)
{
    (void)ctx;
    if (disk_bad_lba >= lba && disk_bad_lba < lba + n)
        return false;
    memcpy(buf, disk_mem + lba * SATA_SECTOR_SIZE, n * SATA_SECTOR_SIZE);
    return true;
}

static bool disk_write(void *ctx, uint64_t lba, unsigned n, const uint8_t *buf)
{
    (void)ctx;
    memcpy(disk_mem + lba * SATA_SECTOR_SIZE, buf, n * SATA_SECTOR_SIZE);
    return true;
}

static bool disk_flush(void *ctx)
{
    (void)ctx;
    disk_flushes++;
    return true;
}

/* ---- the serial line --------------------------------------------------- */

#define WIRE_DELAY 5

typedef struct {
    sata_xcvr_tx_t q[WIRE_DELAY];
    unsigned       pos;
    unsigned       align_gap; /* Dwords since the last ALIGN */
    int            rd_tx, rd_rx;
    bool           idle;
} wire_t;

static unsigned      err_period; /* one bit error every so many Dwords */
static unsigned long bit_errors;
static unsigned long n_cont, n_hold, n_holda;
static unsigned      max_align_gap;

static void wire_init(wire_t *w)
{
    memset(w, 0, sizeof *w);
    for (unsigned i = 0; i < WIRE_DELAY; i++)
        w->q[i].elec_idle = 1;
    w->rd_tx = w->rd_rx = SATA_RD_NEG;
    w->idle = true;
}

static void wire_xfer(wire_t *w, const sata_xcvr_tx_t *in, sata_xcvr_rx_t *out)
{
    sata_xcvr_tx_t t = w->q[w->pos];
    w->q[w->pos] = *in;
    w->pos = (w->pos + 1) % WIRE_DELAY;

    memset(out, 0, sizeof *out);
    out->cominit = t.comreset;
    out->comwake = t.comwake;
    if (t.comreset || t.comwake || t.elec_idle) {
        w->idle = true;
        return;
    }
    if (w->idle) {
        /* the receiver picks the disparity up when it locks */
        w->rd_rx = w->rd_tx;
        w->idle = false;
        w->align_gap = 0;
    }

    /* what is on the line, for the checks at the end */
    if (t.isk && t.data == SATA_PRIM_ALIGN) {
        w->align_gap = 0;
    } else if (++w->align_gap > max_align_gap) {
        max_align_gap = w->align_gap;
    }
    if (t.isk) {
        n_cont += t.data == SATA_PRIM_CONT;
        n_hold += t.data == SATA_PRIM_HOLD;
        n_holda += t.data == SATA_PRIM_HOLDA;
    }

    uint16_t code[4];
    sata_8b10b_encode_dword(t.data, t.isk, &w->rd_tx, code);
    if (err_period && rnd() % err_period == 0) {
        unsigned bit = rnd() % 40;
        code[bit / 10] ^= (uint16_t)(1u << (bit % 10));
        bit_errors++;
    }
    uint32_t data;
    bool isk;
    bool ok = sata_8b10b_decode_dword(code, &w->rd_rx, &data, &isk);
    out->data = data;
    out->isk = isk;
    out->err = !ok;
    out->valid = 1;
}

/* ---- the system under test --------------------------------------------- */

static sata_host_t   host;
static sata_device_t dev;
static wire_t        h2d, d2h;
static sata_xcvr_rx_t h_rx, d_rx;
static unsigned long long ticks;
static bool stress; /* random HOLD flow control on both sides */
static uint32_t link_states; /* Link states either side has been in */

static void step(void)
{
    sata_xcvr_tx_t h_tx, d_tx;

    if (stress) {
        if (rnd() % 61 == 0)
            host.link.tx_pause = !host.link.tx_pause;
        if (rnd() % 67 == 0)
            host.link.rx_throttle = !host.link.rx_throttle;
        if (rnd() % 71 == 0)
            dev.link.tx_pause = !dev.link.tx_pause;
        if (rnd() % 73 == 0)
            dev.link.rx_throttle = !dev.link.rx_throttle;
    } else {
        host.link.tx_pause = host.link.rx_throttle = false;
        dev.link.tx_pause = dev.link.rx_throttle = false;
    }

    sata_host_tick(&host, &h_rx, &h_tx);
    sata_device_tick(&dev, &d_rx, &d_tx);
    wire_xfer(&h2d, &h_tx, &d_rx);
    wire_xfer(&d2h, &d_tx, &h_rx);
    link_states |= 1u << host.link.state | 1u << dev.link.state;
    ticks++;
}

static bool wait_ready(unsigned max)
{
    for (unsigned n = 0; n < max && !sata_host_ready(&host); n++)
        step();
    return sata_host_ready(&host);
}

static sata_host_result_t wait_cmd(sata_host_result_t started)
{
    if (started != SATA_HOST_BUSY)
        return started;
    for (unsigned n = 0;
         n < 20000000u && sata_host_result(&host) == SATA_HOST_BUSY; n++)
        step();
    return sata_host_result(&host);
}

static void fill(uint8_t *buf, size_t n)
{
    for (size_t i = 0; i < n; i++)
        buf[i] = (uint8_t)rnd();
}

/* ---- tests ------------------------------------------------------------- */

static void test_codec(void)
{
    printf("codec: CRC, scrambler, 8b/10b\n");

    /* A.2.4 */
    static const uint32_t scr[8] = {
        0xC2D2768D, 0x1F26B368, 0xA508436C, 0x3452D354,
        0x8A559502, 0xBB1ABE1B, 0xFA56B73D, 0x53F60B1B,
    };
    uint16_t ctx = SATA_SCR_INIT;
    uint32_t v = 0;
    for (int i = 0; i < 32; i++) {
        v = sata_scr_next(&ctx);
        if (i < 8)
            CHECK(v == scr[i]);
    }
    CHECK(v == 0x76F46A1E);

    /* A.3: PIO write command, LBA 1234567h, 2 sectors */
    sata_fis_h2d_t cmd = {
        .c = true,
        .command = ATA_CMD_WRITE_SECTORS,
        .regs = { .lba = 0x234567, .count = 2, .device = 0xE1 },
    };
    uint32_t fis[SATA_FIS_REG_LEN];
    CHECK(sata_fis_build_h2d(fis, &cmd) == 5);
    static const uint32_t plain[5] = { 0x00308027, 0xE1234567, 0, 2, 0 };
    static const uint32_t crcs[5] = { 0x11E353FD, 0x0F656DA7, 0x3D14369C,
                                      0x92D0D681, 0x319FFF6F };
    static const uint32_t scrambled[6] = { 0xC2E2F6AA, 0xFE05F60F, 0xA508436C,
                                           0x3452D356, 0x8A559502,
                                           0x8A854174 };
    uint32_t crc = SATA_CRC_INIT;
    ctx = SATA_SCR_INIT;
    for (int i = 0; i < 5; i++) {
        CHECK(fis[i] == plain[i]);
        crc = sata_crc_step(crc, fis[i]);
        CHECK(crc == crcs[i]);
        CHECK((fis[i] ^ sata_scr_next(&ctx)) == scrambled[i]);
    }
    CHECK(sata_crc(fis, 5) == 0x319FFF6F);
    CHECK((crc ^ sata_scr_next(&ctx)) == scrambled[5]);
    CHECK(sata_crc_step(crc, crc) == 0);

    sata_fis_h2d_t back;
    CHECK(sata_fis_parse_h2d(fis, 5, &back));
    CHECK(back.c && back.command == cmd.command &&
          back.regs.lba == cmd.regs.lba && back.regs.count == 2 &&
          back.regs.device == 0xE1);

    /* 7.2.2.4 / 7.2.2.5 */
    int rd = SATA_RD_NEG;
    CHECK(sata_8b10b_encode(0x4A, false, &rd) == 0x155 && rd < 0); /* D10.2 */
    CHECK(sata_8b10b_encode(0xB5, false, &rd) == 0x2AA && rd < 0); /* D21.5 */
    CHECK(sata_8b10b_encode(0xBC, true, &rd) == 0x0FA && rd > 0);  /* K28.5 */
    CHECK(sata_8b10b_encode(0xEB, false, &rd) == 0x348 && rd < 0); /* D11.7 */
    CHECK(sata_8b10b_encode(0xF1, false, &rd) == 0x237 && rd > 0); /* D17.7 */
    CHECK(sata_8b10b_encode(0x7C, true, &rd) == 0x30C && rd < 0);  /* K28.3 */
    CHECK(sata_8b10b_encode(0x00, false, &rd) == 0x274 && rd < 0); /* D0.0  */
    CHECK(sata_8b10b_encode(0xE7, false, &rd) == 0x38E && rd > 0); /* D7.7  */

    /* ALIGN leaves the disparity as it found it; rd+ form from 6.7.6 */
    uint16_t code[4];
    rd = SATA_RD_POS;
    sata_8b10b_encode_dword(SATA_PRIM_ALIGN, true, &rd, code);
    CHECK(rd == SATA_RD_POS);
    CHECK(code[0] == 0x305 && code[1] == 0x155 && code[2] == 0x155 &&
          code[3] == 0x363);

    /* every character, both disparities, there and back */
    int used[1024] = { 0 };
    for (int start = -1; start <= 1; start += 2) {
        for (unsigned c = 0; c < 258; c++) {
            bool k = c >= 256;
            uint8_t byte = c == 256 ? SATA_K28_3
                         : c == 257 ? SATA_K28_5 : (uint8_t)c;
            int rd_e = start, rd_d = start;
            uint16_t e = sata_8b10b_encode(byte, k, &rd_e);
            uint8_t b2;
            bool k2;
            CHECK(sata_8b10b_decode(e, &rd_d, &b2, &k2));
            CHECK(b2 == byte && k2 == k && rd_d == rd_e);
            used[e] = 1;
            /* at the wrong disparity only a code that is the same in
               both columns is legal */
            rd_e = rd_d = -start;
            bool same = sata_8b10b_encode(byte, k, &rd_e) == e;
            CHECK(sata_8b10b_decode(e, &rd_d, &b2, &k2) == same);
        }
    }
    /* codes nobody generates are violations */
    int legal = 0;
    for (unsigned c = 0; c < 1024; c++) {
        uint8_t b2;
        bool k2;
        rd = SATA_RD_NEG;
        bool neg = sata_8b10b_decode((uint16_t)c, &rd, &b2, &k2);
        rd = SATA_RD_POS;
        bool pos = sata_8b10b_decode((uint16_t)c, &rd, &b2, &k2);
        CHECK((neg || pos) == (used[c] != 0));
        legal += used[c];
    }
    CHECK(legal > 256);
}

static void test_link_up(void)
{
    printf("power-on: OOB sequence, ALIGN handshake, signature\n");
    CHECK(sata_host_sstatus(&host) == 0x000);
    CHECK(host.status == 0x7F);
    CHECK(wait_ready(100000));
    CHECK(sata_host_sstatus(&host) == 0x113);
    CHECK(host.status == (ATA_SR_DRDY | ATA_SR_DSC) && host.error == 0x01);
    CHECK(host.regs.lba == 1 && host.regs.count == 1);
    /* the Phy came up and saw COMWAKE on the way */
    CHECK(sata_host_serror(&host) ==
          (uint32_t)(SATA_DIAG_N | SATA_DIAG_W) << 16);
    sata_host_serror_clear(&host, 0xFFFFFFFFu);
    CHECK(sata_host_serror(&host) == 0);
    printf("  link up after %lu Dwords\n", (unsigned long)ticks);
}

static void test_identify(void)
{
    printf("IDENTIFY DEVICE (PIO data-in)\n");
    uint8_t id[SATA_SECTOR_SIZE];
    sata_identify_t info;

    CHECK(wait_cmd(sata_host_identify(&host, id, 1000000)) == SATA_HOST_OK);
    CHECK(sata_host_transferred(&host) == SATA_SECTOR_SIZE);
    CHECK(host.irq);
    sata_identify_parse(id, &info);
    CHECK(strcmp(info.model, "SATA RAM DISK") == 0);
    CHECK(strcmp(info.serial, "0123456789") == 0);
    CHECK(strcmp(info.firmware, "1.0a") == 0);
    CHECK(info.lba48 && info.sectors == DISK_SECTORS);
    printf("  model \"%s\", %lu sectors\n", info.model,
           (unsigned long)info.sectors);
}

static uint8_t wbuf[256 * SATA_SECTOR_SIZE], rbuf[256 * SATA_SECTOR_SIZE];

static void test_rw(const char *name)
{
    printf("read/write, %s\n", name);

    /* PIO: one Data FIS per sector */
    fill(wbuf, 9 * SATA_SECTOR_SIZE);
    CHECK(wait_cmd(sata_host_write_pio(&host, 10, 9, wbuf, 4000000)) ==
          SATA_HOST_OK);
    CHECK(sata_host_transferred(&host) == 9 * SATA_SECTOR_SIZE);
    CHECK(memcmp(disk_mem + 10 * SATA_SECTOR_SIZE, wbuf,
                 9 * SATA_SECTOR_SIZE) == 0);
    memset(rbuf, 0, sizeof rbuf);
    CHECK(wait_cmd(sata_host_read_pio(&host, 10, 9, rbuf, 4000000)) ==
          SATA_HOST_OK);
    CHECK(sata_host_transferred(&host) == 9 * SATA_SECTOR_SIZE);
    CHECK(memcmp(rbuf, wbuf, 9 * SATA_SECTOR_SIZE) == 0);

    /* DMA: 8 KB per Data FIS, so several of them plus a short one */
    fill(wbuf, 133 * SATA_SECTOR_SIZE);
    CHECK(wait_cmd(sata_host_write_dma(&host, 1000, 133, wbuf, 4000000)) ==
          SATA_HOST_OK);
    CHECK(sata_host_transferred(&host) == 133 * SATA_SECTOR_SIZE);
    CHECK(memcmp(disk_mem + 1000 * SATA_SECTOR_SIZE, wbuf,
                 133 * SATA_SECTOR_SIZE) == 0);
    memset(rbuf, 0, sizeof rbuf);
    CHECK(wait_cmd(sata_host_read_dma(&host, 1000, 133, rbuf, 4000000)) ==
          SATA_HOST_OK);
    CHECK(sata_host_transferred(&host) == 133 * SATA_SECTOR_SIZE);
    CHECK(memcmp(rbuf, wbuf, 133 * SATA_SECTOR_SIZE) == 0);

    /* 28 bit commands */
    sata_fis_h2d_t f = {
        .command = ATA_CMD_READ_DMA,
        .regs = { .lba = 1000, .count = 3, .device = ATA_DH_LBA },
    };
    memset(rbuf, 0, sizeof rbuf);
    CHECK(wait_cmd(sata_host_start(&host, &f, rbuf, 3 * SATA_SECTOR_SIZE,
                                   4000000)) == SATA_HOST_OK);
    CHECK(memcmp(rbuf, wbuf, 3 * SATA_SECTOR_SIZE) == 0);
    f.command = ATA_CMD_READ_SECTORS;
    memset(rbuf, 0, sizeof rbuf);
    CHECK(wait_cmd(sata_host_start(&host, &f, rbuf, 3 * SATA_SECTOR_SIZE,
                                   4000000)) == SATA_HOST_OK);
    CHECK(memcmp(rbuf, wbuf, 3 * SATA_SECTOR_SIZE) == 0);
    CHECK(sata_host_serror(&host) == 0);
}

static void test_nondata_and_errors(void)
{
    printf("non-data commands and device errors\n");

    unsigned before = disk_flushes;
    CHECK(wait_cmd(sata_host_flush(&host, 1000000)) == SATA_HOST_OK);
    CHECK(disk_flushes == before + 1);
    CHECK(host.irq);

    sata_fis_h2d_t f = { .command = ATA_CMD_CHECK_POWER_MODE };
    CHECK(wait_cmd(sata_host_start(&host, &f, NULL, 0, 1000000)) ==
          SATA_HOST_OK);
    CHECK(host.regs.count == 0xFF);

    /* not implemented by the device: command aborted */
    f.command = 0xB0;
    CHECK(wait_cmd(sata_host_start(&host, &f, NULL, 0, 1000000)) ==
          SATA_HOST_DEV_ERROR);
    CHECK((host.status & ATA_SR_ERR) && host.error == ATA_ER_ABRT);

    /* past the end of the disk */
    CHECK(wait_cmd(sata_host_read_dma(&host, DISK_SECTORS - 1, 2, rbuf,
                                      1000000)) == SATA_HOST_DEV_ERROR);
    CHECK(host.error & ATA_ER_IDNF);
    CHECK(wait_cmd(sata_host_write_pio(&host, DISK_SECTORS, 1, wbuf,
                                       1000000)) == SATA_HOST_DEV_ERROR);

    /* media error in the middle of a transfer */
    disk_bad_lba = 2020;
    CHECK(wait_cmd(sata_host_read_dma(&host, 2000, 40, rbuf, 4000000)) ==
          SATA_HOST_DEV_ERROR);
    CHECK(host.error & ATA_ER_UNC);
    CHECK(sata_host_transferred(&host) == 16 * SATA_SECTOR_SIZE);
    CHECK(wait_cmd(sata_host_read_pio(&host, 2018, 4, rbuf, 4000000)) ==
          SATA_HOST_DEV_ERROR);
    CHECK(host.error & ATA_ER_UNC);
    CHECK(sata_host_transferred(&host) == 2 * SATA_SECTOR_SIZE);
    disk_bad_lba = UINT64_MAX;

    /* and the device is fine afterwards */
    CHECK(sata_host_ready(&host));
    CHECK(wait_cmd(sata_host_read_dma(&host, 2000, 40, rbuf, 4000000)) ==
          SATA_HOST_OK);
    CHECK(sata_host_serror(&host) == 0);
}

static void test_resets(void)
{
    printf("software reset and COMRESET\n");

    /* idle */
    sata_host_soft_reset(&host);
    CHECK(!sata_host_ready(&host));
    CHECK(host.status & ATA_SR_BSY);
    CHECK(wait_ready(100000));
    CHECK(host.error == 0x01 && host.regs.lba == 1 && host.regs.count == 1);

    /* in the middle of a data transfer, either direction */
    CHECK(sata_host_read_dma(&host, 0, 200, rbuf, 0) == SATA_HOST_BUSY);
    for (int i = 0; i < 3000; i++)
        step();
    CHECK(sata_host_result(&host) == SATA_HOST_BUSY);
    sata_host_soft_reset(&host);
    CHECK(sata_host_result(&host) == SATA_HOST_ABORTED);
    CHECK(wait_ready(100000));

    CHECK(sata_host_write_dma(&host, 0, 200, wbuf, 0) == SATA_HOST_BUSY);
    for (int i = 0; i < 3000; i++)
        step();
    sata_host_soft_reset(&host);
    CHECK(wait_ready(100000));
    CHECK(sata_host_write_pio(&host, 0, 20, wbuf, 0) == SATA_HOST_BUSY);
    for (int i = 0; i < 1000; i++)
        step();
    sata_host_soft_reset(&host);
    CHECK(wait_ready(100000));
    test_identify();

    /* command timeout: the device keeps the registers, reset recovers */
    CHECK(wait_cmd(sata_host_read_dma(&host, 0, 200, rbuf, 2000)) ==
          SATA_HOST_TIMEOUT);
    CHECK(!sata_host_ready(&host));
    sata_host_soft_reset(&host);
    CHECK(wait_ready(100000));

    /* hard reset */
    sata_host_serror_clear(&host, 0xFFFFFFFFu);
    CHECK(sata_host_read_dma(&host, 0, 200, rbuf, 0) == SATA_HOST_BUSY);
    for (int i = 0; i < 3000; i++)
        step();
    sata_host_comreset(&host);
    CHECK(sata_host_result(&host) == SATA_HOST_ABORTED);
    step();
    CHECK(!sata_host_ready(&host));
    CHECK(wait_ready(100000));
    CHECK(sata_host_serror(&host) & (SATA_DIAG_N << 16));
    CHECK(host.regs.lba == 1 && host.regs.count == 1);
    test_identify();
    sata_host_serror_clear(&host, 0xFFFFFFFFu);
    CHECK(sata_host_serror(&host) == 0);
}

/* What the disk must contain if every command of the noise test is right. */
static uint8_t expect[DISK_SECTORS * SATA_SECTOR_SIZE];

static void test_noise(unsigned period)
{
    printf("random traffic over a line with bit errors\n");

    unsigned long resets = 0, hard_resets = 0, retries = 0;
    unsigned long results[SATA_HOST_ABORTED + 1] = { 0 };
    unsigned long long start = ticks;
    bit_errors = 0;
    memcpy(expect, disk_mem, sizeof expect);
    err_period = period;

    for (int op = 0; op < 400; op++) {
        unsigned n = 1 + rnd() % 64;
        unsigned lba = rnd() % (DISK_SECTORS - n);
        unsigned kind = rnd() % 4;
        size_t bytes = (size_t)n * SATA_SECTOR_SIZE;
        bool write = kind & 1, pio = kind & 2;
        bool done = false;

        if (pio && n > 8)
            n = 8, bytes = (size_t)n * SATA_SECTOR_SIZE;
        if (write) {
            fill(wbuf, bytes);
            memcpy(expect + (size_t)lba * SATA_SECTOR_SIZE, wbuf, bytes);
        }

        for (int attempt = 0; attempt < 200 && !done; attempt++) {
            if (!wait_ready(20000)) {
                /* stuck: what a driver does after a timeout */
                sata_host_soft_reset(&host);
                resets++;
                if (!wait_ready(200000)) {
                    sata_host_comreset(&host);
                    hard_resets++;
                    wait_ready(2000000);
                }
                continue;
            }
            sata_host_result_t r;
            if (write)
                r = pio ? sata_host_write_pio(&host, lba, n, wbuf, 300000)
                        : sata_host_write_dma(&host, lba, n, wbuf, 300000);
            else
                r = pio ? sata_host_read_pio(&host, lba, n, rbuf, 300000)
                        : sata_host_read_dma(&host, lba, n, rbuf, 300000);
            r = wait_cmd(r);
            results[r]++;
            if (r == SATA_HOST_OK) {
                done = true;
                CHECK(sata_host_transferred(&host) == bytes);
                if (!write)
                    CHECK(memcmp(rbuf,
                                 expect + (size_t)lba * SATA_SECTOR_SIZE,
                                 bytes) == 0);
            } else {
                retries++;
            }
        }
        CHECK(done);
        if (!done)
            break;
    }
    err_period = 0;

    CHECK(wait_ready(2000000) || (sata_host_soft_reset(&host),
                                  wait_ready(2000000)));
    CHECK(memcmp(disk_mem, expect, sizeof expect) == 0);
    CHECK(bit_errors > 20);
    CHECK(results[SATA_HOST_OK] == 400);
    printf("  %lu Dwords, %lu bit errors: %lu commands failed and were "
           "repeated\n  (device error %lu, link error %lu, timeout %lu), "
           "%lu soft resets, %lu COMRESETs\n",
           (unsigned long)(ticks - start), bit_errors, retries, results[SATA_HOST_DEV_ERROR],
           results[SATA_HOST_LINK_ERROR], results[SATA_HOST_TIMEOUT], resets,
           hard_resets);
    printf("  SError DIAG = %03Xh\n", sata_host_serror(&host) >> 16);
    sata_host_serror_clear(&host, 0xFFFFFFFFu);
}

int main(void)
{
    static const sata_disk_t disk = {
        .sectors = DISK_SECTORS,
        .read = disk_read,
        .write = disk_write,
        .flush = disk_flush,
        .model = "SATA RAM DISK",
        .serial = "0123456789",
        .firmware = "1.0a",
    };

    test_codec();

    sata_host_init(&host);
    sata_device_init(&dev, &disk);
    wire_init(&h2d);
    wire_init(&d2h);

    test_link_up();
    test_identify();
    test_rw("plain");
    test_nondata_and_errors();

    stress = true;
    test_rw("random HOLD/HOLDA flow control");
    host.link.use_cont = dev.link.use_cont = false;
    test_rw("random HOLD/HOLDA flow control, no CONT");
    stress = false;
    test_rw("no CONT");
    host.link.use_cont = dev.link.use_cont = true;

    test_resets();

    test_noise(30000);
    stress = true;
    test_noise(60000);
    stress = false;
    test_rw("plain, after the noise");

    /* the whole Link state machine was walked, power management aside */
    CHECK(link_states ==
          ((2u << SATA_L_PM_DENY) - 1u) - (1u << SATA_L_PM_DENY));
    /* flow control and CONT really happened; ALIGN came often enough */
    CHECK(n_hold > 1000 && n_holda > 1000 && n_cont > 1000);
    CHECK(max_align_gap > 200 && max_align_gap <= 254);
    printf("on the line: %lu CONT, %lu HOLD, %lu HOLDA, at most %u Dwords "
           "between ALIGNs\n", n_cont, n_hold, n_holda, max_align_gap);

    printf("%s (%d failures, %lu Dwords simulated)\n",
           failures ? "FAILED" : "PASSED", failures, (unsigned long)ticks);
    return failures != 0;
}
