/*
 * Serial ATA 1.0a - CRC, scrambler and 8b/10b code (7.2, 7.5, appendix A).
 */
#include "sata_codec.h"

/* ---- CRC --------------------------------------------------------------- */

/*
 * G(x) = x^32+x^26+x^23+x^22+x^16+x^12+x^11+x^10+x^8+x^7+x^5+x^4+x^2+x+1,
 * calculated on Dword quantities, most significant bit first.
 */
uint32_t sata_crc_step(uint32_t crc, uint32_t dword)
{
    crc ^= dword;
    for (int i = 0; i < 32; i++)
        crc = (crc << 1) ^ ((crc & 0x80000000u) ? 0x04C11DB7u : 0u);
    return crc;
}

uint32_t sata_crc(const uint32_t *dwords, size_t n)
{
    uint32_t crc = SATA_CRC_INIT;
    for (size_t i = 0; i < n; i++)
        crc = sata_crc_step(crc, dwords[i]);
    return crc;
}

/* ---- Scrambler --------------------------------------------------------- */

/*
 * G(x) = x^16+x^15+x^13+x^4+1, 32 bits per step. Bit n of the output is the
 * parity of (context & scr_mask[n]); these are the *M1 (bits 31:16) and *M2
 * (bits 15:0) matrices of A.2.3. Bits 31:16 become the next context.
 */
static const uint16_t scr_mask[32] = {
    0xA011, 0xE033, 0x6077, 0xC0EE, 0x21CD, 0x439A, 0x8734, 0xAE79,
    0xFCE3, 0x59D7, 0xB3AE, 0xC74D, 0x2E8B, 0x5D16, 0xBA2C, 0xD449,
    0x0883, 0x1106, 0x220C, 0x4418, 0x8830, 0xB071, 0xC0F3, 0x21F7,
    0x43EE, 0x87DC, 0xAFA9, 0xFF43, 0x5E97, 0xBD2E, 0xDA4D, 0x148B,
};

static unsigned parity16(uint16_t v)
{
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1u;
}

uint32_t sata_scr_next(uint16_t *ctx)
{
    uint32_t out = 0;
    for (int i = 0; i < 32; i++)
        out |= (uint32_t)parity16(*ctx & scr_mask[i]) << i;
    *ctx = (uint16_t)(out >> 16);
    return out;
}

/* ---- 8b/10b ------------------------------------------------------------ */

/* 5B/6B code, "abcdei" for current rd- (table 16). */
static const uint8_t enc6[32] = {
    0x27, 0x1D, 0x2D, 0x31, 0x35, 0x29, 0x19, 0x38,
    0x39, 0x25, 0x15, 0x34, 0x0D, 0x2C, 0x1C, 0x17,
    0x1B, 0x23, 0x13, 0x32, 0x0B, 0x2A, 0x1A, 0x3A,
    0x33, 0x26, 0x16, 0x36, 0x0E, 0x2E, 0x1E, 0x2B,
};

/* 3B/4B code, "fghj" for current rd- (table 17); index 7 is Dx.P7. */
static const uint8_t enc4[8] = {
    0xB, 0x9, 0x5, 0xC, 0xD, 0xA, 0x6, 0xE,
};
#define ENC4_A7 0x7

#define K28_3_NEG 0x0F3u /* 001111 0011 */
#define K28_5_NEG 0x0FAu /* 001111 1010 */

static int ones(unsigned v)
{
    int n = 0;
    for (; v; v >>= 1)
        n += (int)(v & 1u);
    return n;
}

uint16_t sata_8b10b_encode(uint8_t byte, bool k, int *rd)
{
    if (k) {
        /* Both K28.3 and K28.5 invert the running disparity (7.2.2.5.2). */
        unsigned code = (byte == SATA_K28_5) ? K28_5_NEG : K28_3_NEG;
        if (*rd > 0)
            code ^= 0x3FFu;
        *rd = -*rd;
        return (uint16_t)code;
    }

    unsigned x = byte & 0x1Fu, y = byte >> 5;

    unsigned six = enc6[x];
    bool six_neutral = ones(six) == 3;
    /* D7 is neutral but still alternates (111000 / 000111). */
    if (*rd > 0 && (!six_neutral || x == 7))
        six ^= 0x3Fu;
    int rd6 = six_neutral ? *rd : -*rd;

    unsigned four = enc4[y];
    if (y == 7) {
        /* A7 replaces P7 to avoid a run of five (table 17 note). */
        unsigned ei = six & 0x3u;
        if ((rd6 > 0 && ei == 0x0u) || (rd6 < 0 && ei == 0x3u))
            four = ENC4_A7;
    }
    bool four_neutral = ones(four) == 2;
    /* Dx.3 is neutral but still alternates (1100 / 0011). */
    if (rd6 > 0 && (!four_neutral || y == 3))
        four ^= 0xFu;
    *rd = four_neutral ? rd6 : -rd6;

    return (uint16_t)(six << 4 | four);
}

/* Running disparity after a received sub-block (7.2.2.2). */
static int rd_after(int rd, unsigned sub, int bits)
{
    int n = ones(sub);
    if (2 * n > bits)
        return SATA_RD_POS;
    if (2 * n < bits)
        return SATA_RD_NEG;
    if (bits == 6) {
        if (sub == 0x07u)
            return SATA_RD_POS;
        if (sub == 0x38u)
            return SATA_RD_NEG;
    } else {
        if (sub == 0x3u)
            return SATA_RD_POS;
        if (sub == 0xCu)
            return SATA_RD_NEG;
    }
    return rd;
}

/* Decode tables, one per current running disparity. 0 = not a legal code. */
#define DEC_VALID 0x8000u
#define DEC_K     0x0100u
static uint16_t dec_tab[2][1024];
static bool dec_ready;

static void dec_build(void)
{
    for (int col = 0; col < 2; col++) {
        for (unsigned b = 0; b < 256; b++) {
            int rd = col ? SATA_RD_POS : SATA_RD_NEG;
            dec_tab[col][sata_8b10b_encode((uint8_t)b, false, &rd)] =
                (uint16_t)(DEC_VALID | b);
        }
        static const uint8_t kchars[2] = { SATA_K28_3, SATA_K28_5 };
        for (int i = 0; i < 2; i++) {
            int rd = col ? SATA_RD_POS : SATA_RD_NEG;
            dec_tab[col][sata_8b10b_encode(kchars[i], true, &rd)] =
                (uint16_t)(DEC_VALID | DEC_K | kchars[i]);
        }
    }
    dec_ready = true;
}

bool sata_8b10b_decode(uint16_t code, int *rd, uint8_t *byte, bool *k)
{
    if (!dec_ready)
        dec_build();

    code &= 0x3FFu;
    int col = *rd > 0;
    uint16_t e = dec_tab[col][code];
    bool ok = (e & DEC_VALID) != 0;
    if (!ok)
        e = dec_tab[!col][code];

    *byte = (uint8_t)e;
    *k = (e & DEC_K) != 0;
    *rd = rd_after(rd_after(*rd, code >> 4, 6), code & 0xFu, 4);
    return ok;
}

void sata_8b10b_encode_dword(uint32_t data, bool isk, int *rd, uint16_t code[4])
{
    for (int i = 0; i < 4; i++)
        code[i] = sata_8b10b_encode((uint8_t)(data >> (8 * i)), isk && i == 0,
                                    rd);
}

bool sata_8b10b_decode_dword(const uint16_t code[4], int *rd, uint32_t *data,
                             bool *isk)
{
    bool ok = true;
    *data = 0;
    *isk = false;
    for (int i = 0; i < 4; i++) {
        uint8_t b;
        bool k;
        if (!sata_8b10b_decode(code[i], rd, &b, &k))
            ok = false;
        *data |= (uint32_t)b << (8 * i);
        if (k) {
            /* A control character is only legal in byte 0 (4.1.9). */
            if (i == 0)
                *isk = true;
            else
                ok = false;
        }
    }
    return ok;
}
