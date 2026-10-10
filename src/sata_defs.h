/*
 * Serial ATA 1.0a - common definitions.
 *
 * Section numbers in comments refer to "Serial ATA: High Speed Serialized
 * AT Attachment", revision 1.0a.
 */
#ifndef SATA_DEFS_H
#define SATA_DEFS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum { SATA_ROLE_HOST, SATA_ROLE_DEVICE } sata_role_t;

/*
 * Primitives (7.4.3, table 21). A Dword is written 31:0, byte 0 (bits 7:0)
 * is transmitted first and is the K character of a primitive.
 */
#define SATA_PRIM_ALIGN   0x7B4A4ABCu /* D27.3 D10.2 D10.2 K28.5 */
#define SATA_PRIM_CONT    0x9999AA7Cu
#define SATA_PRIM_DMAT    0x3636B57Cu
#define SATA_PRIM_EOF     0xD5D5B57Cu
#define SATA_PRIM_HOLD    0xD5D5AA7Cu
#define SATA_PRIM_HOLDA   0x9595AA7Cu
#define SATA_PRIM_PMACK   0x9595957Cu
#define SATA_PRIM_PMNAK   0xF5F5957Cu
#define SATA_PRIM_PMREQ_P 0x1717B57Cu
#define SATA_PRIM_PMREQ_S 0x7575957Cu
#define SATA_PRIM_R_ERR   0x5656B57Cu
#define SATA_PRIM_R_IP    0x5555B57Cu
#define SATA_PRIM_R_OK    0x3535B57Cu
#define SATA_PRIM_R_RDY   0x4A4A957Cu
#define SATA_PRIM_SOF     0x3737B57Cu
#define SATA_PRIM_SYNC    0xB5B5957Cu
#define SATA_PRIM_WTRM    0x5858B57Cu
#define SATA_PRIM_X_RDY   0x5757B57Cu

#define SATA_K28_3 0x7Cu
#define SATA_K28_5 0xBCu
#define SATA_D10_2_DWORD 0x4A4A4A4Au

/* FIS types (8.5, appendix B) */
#define SATA_FIS_REG_H2D      0x27u
#define SATA_FIS_REG_D2H      0x34u
#define SATA_FIS_DMA_ACTIVATE 0x39u
#define SATA_FIS_DMA_SETUP    0x41u
#define SATA_FIS_DATA         0x46u
#define SATA_FIS_BIST         0x58u
#define SATA_FIS_PIO_SETUP    0x5Fu
#define SATA_FIS_SET_DEV_BITS 0xA1u

/* Sizes */
#define SATA_SECTOR_SIZE     512u
#define SATA_MAX_DATA_BYTES  8192u                        /* 8.5.9.1 */
#define SATA_MAX_DATA_DWORDS (SATA_MAX_DATA_BYTES / 4u)
#define SATA_MAX_FIS_DWORDS  (SATA_MAX_DATA_DWORDS + 1u)  /* + FIS header  */
#define SATA_MAX_FRAME_DWORDS (SATA_MAX_FIS_DWORDS + 1u)  /* + CRC         */

/* ATA Status register */
#define ATA_SR_BSY  0x80u
#define ATA_SR_DRDY 0x40u
#define ATA_SR_DF   0x20u
#define ATA_SR_DSC  0x10u /* also SERV */
#define ATA_SR_DRQ  0x08u
#define ATA_SR_ERR  0x01u

/* ATA Error register */
#define ATA_ER_ICRC 0x80u
#define ATA_ER_UNC  0x40u
#define ATA_ER_IDNF 0x10u
#define ATA_ER_ABRT 0x04u

/* ATA Device Control register */
#define ATA_DC_SRST 0x04u
#define ATA_DC_NIEN 0x02u

/* ATA Device/Head register */
#define ATA_DH_LBA 0x40u

/* ATA commands used by this implementation */
#define ATA_CMD_NOP                 0x00u
#define ATA_CMD_READ_SECTORS        0x20u
#define ATA_CMD_READ_SECTORS_EXT    0x24u
#define ATA_CMD_READ_DMA_EXT        0x25u
#define ATA_CMD_WRITE_SECTORS       0x30u
#define ATA_CMD_WRITE_SECTORS_EXT   0x34u
#define ATA_CMD_WRITE_DMA_EXT       0x35u
#define ATA_CMD_READ_VERIFY         0x40u
#define ATA_CMD_READ_VERIFY_EXT     0x42u
#define ATA_CMD_EXEC_DEV_DIAG       0x90u
#define ATA_CMD_INIT_DEV_PARAMS     0x91u
#define ATA_CMD_READ_DMA            0xC8u
#define ATA_CMD_WRITE_DMA           0xCAu
#define ATA_CMD_STANDBY_IMMEDIATE   0xE0u
#define ATA_CMD_IDLE_IMMEDIATE      0xE1u
#define ATA_CMD_CHECK_POWER_MODE    0xE5u
#define ATA_CMD_FLUSH_CACHE         0xE7u
#define ATA_CMD_FLUSH_CACHE_EXT     0xEAu
#define ATA_CMD_IDENTIFY_DEVICE     0xECu
#define ATA_CMD_SET_FEATURES        0xEFu

/*
 * Diagnostic flags, laid out like the DIAG field of the SError register
 * (10.1.2) shifted down to bit 0.
 */
#define SATA_DIAG_N 0x001u /* PhyRdy changed                    */
#define SATA_DIAG_I 0x002u /* Phy internal error                */
#define SATA_DIAG_W 0x004u /* COMWAKE detected                  */
#define SATA_DIAG_B 0x008u /* 10b to 8b decode error            */
#define SATA_DIAG_D 0x010u /* disparity error                   */
#define SATA_DIAG_C 0x020u /* CRC error                         */
#define SATA_DIAG_H 0x040u /* handshake error (R_ERR received)  */
#define SATA_DIAG_S 0x080u /* link sequence error               */
#define SATA_DIAG_T 0x100u /* transport state transition error  */
#define SATA_DIAG_F 0x200u /* unrecognized FIS type             */

/*
 * Interface to the gigabit transceiver, sampled/driven once per Dword
 * (26.67 ns at 1.5 Gbit/s). This is the only thing the stack needs from the
 * hardware below it.
 *
 * The transceiver (or glue logic) is expected to do serialization, comma
 * alignment to Dword boundaries, OOB burst generation/detection and - unless
 * sata_8b10b_* from sata_codec.h is used in between - 8b/10b coding.
 */
typedef struct {
    uint32_t data;    /* received Dword, byte 0 = first on the wire           */
    uint8_t  isk;     /* byte 0 is a K character (Dword is a primitive)       */
    uint8_t  err;     /* code violation / disparity error inside this Dword   */
    uint8_t  valid;   /* receiver locked and Dword aligned, 'data' is usable  */
    uint8_t  cominit; /* COMINIT (host) / COMRESET (device) being detected    */
    uint8_t  comwake; /* COMWAKE being detected                               */
} sata_xcvr_rx_t;

typedef struct {
    uint32_t data;      /* Dword to transmit                                  */
    uint8_t  isk;       /* byte 0 is a K character                            */
    uint8_t  elec_idle; /* drive the bus idle (zero differential)             */
    uint8_t  comreset;  /* send COMRESET (host) / COMINIT (device) bursts     */
    uint8_t  comwake;   /* send COMWAKE bursts                                */
} sata_xcvr_tx_t;

#endif /* SATA_DEFS_H */
