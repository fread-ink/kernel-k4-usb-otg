/*
 *  linux/drivers/video/eink/fslepdc/fslepdc.h --
 *  eInk frame buffer device HAL broadsheet defs
 *
 *      Copyright (c) 2010-2011 Amazon Technologies, Inc.
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#ifndef _FSLEPDC_H
#define _FSLEPDC_H

#include <linux/mxcfb.h>
#include <linux/mxcfb_epdc_kernel.h>

#define FSLEPDC_BOOTSTRAPPED()      (true == fslepdc_get_bootstrapped_state())
#define BOOTSTRAP_FSLEPDC()         fslepdc_set_bootstrapped_state(true)

// FSL EPDC PMIC API (from fslepdc_pmic.c)
//
extern bool fslepdc_pmic_present(void);

extern bool fslepdc_pmic_init(void);
extern void fslepdc_pmic_done(void);

extern int  fslepdc_pmic_get_temperature(void);
extern int  papyrus_temp;

extern int  fslepdc_pmic_get_vcom_default(void);
extern int  fslepdc_pmic_set_vcom(int vcom);

// FSL EPDC API (from fsledpc_hal.c)
//
extern int  flsepdc_read_temperature(void);

extern void fslepdc_set_bootstrapped_state(bool state);
extern bool fslepdc_get_bootstrapped_state(void);

#endif
