/*
 * Read/Write operation to the CS5536 SMbus
 *
 * Copyright (C) 2017 Jiaxun Yang.
 * Author : Jiaxun Yang, jiaxun.yang@flygoat.com
 *
 * This program is free software; you can redistribute	it and/or modify it
 * under  the terms of	the GNU General	 Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/types.h>
#include <linux/delay.h>
#include <linux/export.h>

#include <asm/io.h>

#include <cs5536/cs5536.h>
#include <cs5536/cs5536_smb.h>

int cs5536_smb_wait(void)
{
	char c;
	int i;

	udelay(1000);
	for (i = 0; i < 20; i++) {
		c = inb(smb_base | SMB_STS);
		if (c & (SMB_STS_BER | SMB_STS_NEGACK))
			return -1;
		if (c & SMB_STS_SDAST)
			return 0;
		udelay(100);
	}
	return -2;
}
EXPORT_SYMBOL(cs5536_smb_wait);

void cs5536_smb_read_single(int addr, int regNo, char *value)
{
	unsigned char c;

	/* Start condition */
	c = inb(smb_base | SMB_CTRL1);
	outb(c | SMB_CTRL1_START, smb_base | SMB_CTRL1);
	cs5536_smb_wait();

	/* Send slave address */
	outb(addr & 0xfe, smb_base | SMB_SDA);
	cs5536_smb_wait();

	/* Acknowledge smbus */
	c = inb(smb_base | SMB_CTRL1);
	outb(c | SMB_CTRL1_ACK, smb_base | SMB_CTRL1);

	/* Send register index */
	outb(regNo, smb_base | SMB_SDA);
	cs5536_smb_wait();

	/* Acknowledge smbus */
	c = inb(smb_base | SMB_CTRL1);
	outb(c | SMB_CTRL1_ACK, smb_base | SMB_CTRL1);

	/* Start condition again */
	c = inb(smb_base | SMB_CTRL1);
	outb(c | SMB_CTRL1_START, smb_base | SMB_CTRL1);
	cs5536_smb_wait();

	/* Send salve address again */
	outb(1 | addr, smb_base | SMB_SDA);
	cs5536_smb_wait();

	/* Acknowledge smbus */
	c = inb(smb_base | SMB_CTRL1);
	outb(c | SMB_CTRL1_ACK, smb_base | SMB_CTRL1);

	/* Read data */
	*value = inb(smb_base | SMB_SDA);

	/* Stop condition */
	outb(SMB_CTRL1_STOP, smb_base | SMB_CTRL1);
	cs5536_smb_wait();
}
EXPORT_SYMBOL(cs5536_smb_read_single);

void cs5536_smb_write_single(int addr, int regNo, char value)
{
	unsigned char c;

	/* Start condition */
	c = inb(smb_base | SMB_CTRL1);
	outb(c | SMB_CTRL1_START, smb_base | SMB_CTRL1);
	cs5536_smb_wait();

	/* Send slave address */
	outb(addr & 0xfe, smb_base | SMB_SDA);
	cs5536_smb_wait();

	/* Send register index */
	outb(regNo, smb_base | SMB_SDA);
	cs5536_smb_wait();

	/* Write data */
	outb(value, smb_base | SMB_SDA);
	cs5536_smb_wait();
	/* Stop condition */
	outb(SMB_CTRL1_STOP, smb_base | SMB_CTRL1);
	cs5536_smb_wait();
}
EXPORT_SYMBOL(cs5536_smb_write_single);
