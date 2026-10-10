/*
 * Serial ATA 1.0a - CRC, scrambler and 8b/10b code (7.2, 7.5, appendix A).
 */
#ifndef SATA_CODEC_H
#define SATA_CODEC_H

#include "sata_defs.h"

/* ---- CRC (7.5.1) ------------------------------------------------------- */

#define SATA_CRC_INIT 0x52325032u

/* Advance the CRC by one Dword of (unscrambled) FIS contents. */
uint32_t sata_crc_step(uint32_t crc, uint32_t dword);
uint32_t sata_crc(const uint32_t *dwords, size_t n);

/* ---- Scrambler (7.5, A.2) ---------------------------------------------- */

/* Context of the parallel scrambler; first output after reset is C2D2768Dh. */
#define SATA_SCR_INIT 0xF0F6u

/* Return the next 32 bit scrambler value and advance the context. */
uint32_t sata_scr_next(uint16_t *ctx);

/* ---- 8b/10b (7.2) ------------------------------------------------------ */

/*
 * Encoded characters are held as 10 bit values written "abcdeifghj", i.e.
 * bit 9 = a (first on the wire) ... bit 0 = j, the same way the tables in
 * 7.2.2.5 are printed. Running disparity is -1 or +1.
 *
 * Only the control characters used by Serial ATA exist: K28.3 and K28.5.
 */
#define SATA_RD_NEG (-1)
#define SATA_RD_POS (+1)

/* Encode one character and update *rd. */
uint16_t sata_8b10b_encode(uint8_t byte, bool k, int *rd);

/*
 * Decode one character and update *rd (the new running disparity is computed
 * from the received bits whether or not the character is legal, 7.2.4.1).
 * Returns false on a code violation, byte and k are then a best effort.
 */
bool sata_8b10b_decode(uint16_t code, int *rd, uint8_t *byte, bool *k);

/* Dword helpers: code[0] carries byte 0 and goes on the wire first. */
void sata_8b10b_encode_dword(uint32_t data, bool isk, int *rd,
                             uint16_t code[4]);
/* Returns false if any character was a code violation (4.1.9). */
bool sata_8b10b_decode_dword(const uint16_t code[4], int *rd, uint32_t *data,
                             bool *isk);

#endif /* SATA_CODEC_H */
