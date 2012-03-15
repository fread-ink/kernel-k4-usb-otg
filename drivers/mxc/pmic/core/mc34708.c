/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 * 
 * Copyright 2010-2011 Amazon Technologies, Inc. All Rights Reserved.
 * Manish Lachwani (lachwani@lab126.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

/*!
 * @file pmic/core/mc34708.c
 * @brief This file contains MC34708 specific PMIC code. This implementaion
 * may differ for each PMIC chip.
 *
 * @ingroup PMIC_CORE
 */

/*
 * Includes
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/mfd/mc34708/core.h>

#include <asm/mach-types.h>

#include "pmic.h"

#define MXC_PMIC_FRAME_MASK		0x00FFFFFF
#define MXC_PMIC_MAX_REG_NUM		0x3F
#define MXC_PMIC_REG_NUM_SHIFT		0x19
#define MXC_PMIC_WRITE_BIT_SHIFT	31

struct mxc_pmic pmic_drv_data;
static unsigned int events_enabled0;
static unsigned int events_enabled1;

int yoshi_button_green_led = 0;
EXPORT_SYMBOL(yoshi_button_green_led);

int pmic_spi_setup(struct spi_device *spi)
{
	/* Setup the SPI slave i.e.PMIC */
	pmic_drv_data.spi = spi;

	spi->mode = SPI_MODE_0 | SPI_CS_HIGH;
	spi->bits_per_word = 32;
	return spi_setup(spi);
}

int pmic_read(int reg_num, unsigned int *reg_val)
{
	unsigned int frame = 0;
	int ret = 0;

	if (pmic_drv_data.spi != NULL) {
		if (reg_num > MXC_PMIC_MAX_REG_NUM)
			return PMIC_ERROR;

		frame |= reg_num << MXC_PMIC_REG_NUM_SHIFT;

		ret = spi_rw(pmic_drv_data.spi, (u8 *) &frame, 1);

		*reg_val = frame & MXC_PMIC_FRAME_MASK;
	} else {
		pr_err("SPI dev Not set\n");
		return PMIC_ERROR;
	}

	return PMIC_SUCCESS;
}

int pmic_write(int reg_num, const unsigned int reg_val)
{
	unsigned int frame = 0;
	int ret = 0;

	if (pmic_drv_data.spi != NULL) {
		if (reg_num > MXC_PMIC_MAX_REG_NUM)
			return PMIC_ERROR;

		frame |= (1 << MXC_PMIC_WRITE_BIT_SHIFT);

		frame |= reg_num << MXC_PMIC_REG_NUM_SHIFT;

		frame |= reg_val & MXC_PMIC_FRAME_MASK;

		ret = spi_rw(pmic_drv_data.spi, (u8 *) &frame, 1);

		return ret;
	} else {
		pr_err("SPI dev Not set\n");
		return PMIC_ERROR;
	}
}

unsigned int pmic_get_active_events(unsigned int *active_events)
{
	unsigned int count = 0;
	unsigned int status0, status1;
	int bit_set;

	pmic_read(REG_MC34708_INT_STATUS0, &status0);
	pmic_read(REG_MC34708_INT_STATUS1, &status1);
	pmic_write(REG_MC34708_INT_STATUS0, status0);
	pmic_write(REG_MC34708_INT_STATUS1, status1);
	status0 &= events_enabled0;
	status1 &= events_enabled1;

	while (status0) {
		bit_set = ffs(status0) - 1;
		*(active_events + count) = bit_set;
		count++;
		status0 ^= (1 << bit_set);
	}
	while (status1) {
		bit_set = ffs(status1) - 1;
		*(active_events + count) = bit_set + 24;
		count++;
		status1 ^= (1 << bit_set);
	}

	return count;
}
EXPORT_SYMBOL(pmic_get_active_events);

#define EVENT_MASK_0			0x387fff
#define EVENT_MASK_1			0x1177ef

int pmic_event_unmask(type_event event)
{
	unsigned int event_mask = 0;
	unsigned int mask_reg = 0;
	unsigned int event_bit = 0;
	int ret;

	if (event < EVENT_1HZI) {
		mask_reg = REG_INT_MASK0;
		event_mask = EVENT_MASK_0;
		event_bit = (1 << event);
		events_enabled0 |= event_bit;
	} else {
		event -= 24;
		mask_reg = REG_INT_MASK1;
		event_mask = EVENT_MASK_1;
		event_bit = (1 << event);
		events_enabled1 |= event_bit;
	}

	if ((event_bit & event_mask) == 0) {
		pr_debug("Error: unmasking a reserved/unused event\n");
		return PMIC_ERROR;
	}

	ret = pmic_write_reg(mask_reg, 0, event_bit);

	pr_debug("Enable Event : %d\n", event);

	return ret;
}
EXPORT_SYMBOL(pmic_event_unmask);

int pmic_event_mask(type_event event)
{
	unsigned int event_mask = 0;
	unsigned int mask_reg = 0;
	unsigned int event_bit = 0;
	int ret;

	if (event < EVENT_1HZI) {
		mask_reg = REG_INT_MASK0;
		event_mask = EVENT_MASK_0;
		event_bit = (1 << event);
		events_enabled0 &= ~event_bit;
	} else {
		event -= 24;
		mask_reg = REG_INT_MASK1;
		event_mask = EVENT_MASK_1;
		event_bit = (1 << event);
		events_enabled1 &= ~event_bit;
	}

	if ((event_bit & event_mask) == 0) {
		pr_debug("Error: masking a reserved/unused event\n");
		return PMIC_ERROR;
	}

	ret = pmic_write_reg(mask_reg, event_bit, event_bit);

	pr_debug("Disable Event : %d\n", event);

	return ret;
}
EXPORT_SYMBOL(pmic_event_mask);

/* Disable charging even if charger is connected */
void pmic_stop_charging(void)
{
    /* TODO (CEL-31): add charger disable code code */
}
EXPORT_SYMBOL(pmic_stop_charging);

int pmic_check_factory_mode(void) 
{
    /* TODO (CEL-31): check for factory mode */
    return 0;
}
EXPORT_SYMBOL(pmic_check_factory_mode);

/*
 * Defines
 */
#define MC34708_I2C_RETRY_TIMES 10
#define MXC_PMIC_FRAME_MASK		0x00FFFFFF
#define MXC_PMIC_MAX_REG_NUM		0x3F
#define MXC_PMIC_REG_NUM_SHIFT		0x19
#define MXC_PMIC_WRITE_BIT_SHIFT		31

void mc34708_power_off(void);

void *pmic_alloc_data(struct device *dev)
{
	struct mc34708 *mc34708;

	mc34708 = kzalloc(sizeof(struct mc34708), GFP_KERNEL);
	if (mc34708 == NULL)
		return NULL;

	mc34708->dev = dev;

	return (void *)mc34708;
}

int pmic_init_registers(void)
{
	CHECK_ERROR(pmic_write(REG_MC34708_INT_MASK0, 0xFFFFFF));
	CHECK_ERROR(pmic_write(REG_MC34708_INT_STATUS0, 0xFFFFFF));
	CHECK_ERROR(pmic_write(REG_MC34708_INT_STATUS1, 0xFFFFFF));
	CHECK_ERROR(pmic_write(REG_MC34708_INT_MASK1, 0xFFFFFF));

	yoshi_button_green_led = 1;

	return PMIC_SUCCESS;
}


/*!
 * This function returns the PMIC version in system.
 *
 * @param	ver	pointer to the pmic_version_t structure
 *
 * @return	This function returns PMIC version.
 */
void pmic_get_revision(pmic_version_t *ver)
{
	int rev_id = 0;
	int rev1 = 0;
	int rev2 = 0;
	int finid = 0;
	int icid = 0;

	ver->id = PMIC_MC34708;
	pmic_read(REG_IDENTIFICATION, &rev_id);

	rev1 = (rev_id & 0x018) >> 3;
	rev2 = (rev_id & 0x007);
	icid = (rev_id & 0x01C0) >> 6;
	finid = (rev_id & 0x01E00) >> 9;

	ver->revision = ((rev1 * 10) + rev2);
	printk(KERN_INFO "mc34708 Rev %d.%d FinVer %x detected\n", rev1,
	       rev2, finid);
}

void mc34708_power_off(void)
{
	unsigned int value;

	pmic_read_reg(REG_MC34708_POWER_CTL0, &value, 0xffffff);

	value |= 0x000008;

	pmic_write_reg(REG_MC34708_POWER_CTL0, value, 0xffffff);
}
