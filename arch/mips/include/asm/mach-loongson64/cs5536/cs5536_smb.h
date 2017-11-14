/*
 * The header file of cs5536 SMBus control.
 *
 * Copyright (C) 2017 Jiaxun Yang.
 * Author : Jiaxun Yang <jiaxun.yang@flygoat.com>
 */

#ifndef _CS5536_SMB_H
#define _CS5536_SMB_H

#include <linux/types.h>

/*
* SMBus I/O SPACE
*/

extern u32 smb_base;

#define SMB_SDA 0x00
#define SMB_STS 0x01
#define SMB_STS_SLVSTP (0x01 << 7)
#define SMB_STS_SDAST (0x01 << 6)
#define SMB_STS_BER (0x01 << 5)
#define SMB_STS_NEGACK (0x01 << 4)
#define SMB_STS_STASTR (0x01 << 3)
#define SMB_STS_NMATCH (0x01 << 2)
#define SMB_STS_MASTER (0x01 << 1)
#define SMB_STS_XMIT (0x01 << 0)

#define SMB_CTRL_STS 0x02
#define SMB_CSTS_TGSCL (0x01 << 5)
#define SMB_CSTS_TSDA (0x01 << 4)
#define SMB_CSTS_GCMTCH (0x01 << 3)
#define SMB_CSTS_MATCH (0x01 << 2)
#define SMB_CSTS_BB (0x01 << 1)
#define SMB_CSTS_BUSY (0x01 << 0)

#define SMB_CTRL1 0x03
#define SMB_CTRL1_STASTRE (0x01 << 7)
#define SMB_CTRL1_NMINTE (0x01 << 6)
#define SMB_CTRL1_GCMEN (0x01 << 5)
#define SMB_CTRL1_ACK (0x01 << 4)
#define SMB_CTRL1_RSVD (0x01 << 3)
#define SMB_CTRL1_INTEN (0x01 << 2)
#define SMB_CTRL1_STOP (0x01 << 1)
#define SMB_CTRL1_START (0x01 << 0)

#define SMB_ADD 0x04
#define SMB_ADD_SAEN (0x01 << 7)

#define SMB_CTRL2 0x05
#define SMB_CTRL2_ENABLE (0x01 << 0)

#define SMB_CTRL3 0x06

extern int cs5536_smb_wait(void);

extern void cs5536_smb_read_single(int addr, int regNo, char *value);

extern void cs5536_smb_write_single(int addr, int regNo, char value);

#endif /*!_CS5536_MFGPT_H */
