/*
 *  linux/drivers/video/eink/fslepdc/fslepdc_waveform.h --
 *  eInk frame buffer device HAL fslepdc waveform defs
 *
 *      Copyright (c) 2005-2011 Amazon Technologies, Inc.
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#ifndef _FSLEPDC_WAVEFORM_H
#define _FSLEPDC_WAVEFORM_H

extern void fsledpc_set_waveform_modes(void);
extern int  fslepdc_get_waveform_mode(int upd_mode);

extern void fslepdc_override_vcom(int vcom);
extern char *fslepdc_get_vcom_str(void);

#endif // _FSLEPDC_WAVEFORM_H
