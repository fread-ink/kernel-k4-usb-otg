/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
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
 *
 */
#ifndef _MXCFB_EPDC_KERNEL
#define _MXCFB_EPDC_KERNEL

void mxc_epdc_fb_set_waveform_modes(struct mxcfb_waveform_modes *modes,
						struct fb_info *info);
int mxc_epdc_fb_set_temperature(int temperature, struct fb_info *info);
int mxc_epdc_fb_set_auto_update(u32 auto_mode, struct fb_info *info);
int mxc_epdc_fb_send_update(struct mxcfb_update_data *upd_data,
				   struct fb_info *info);
int mxc_epdc_fb_wait_update_complete(u32 update_marker, struct fb_info *info);
int mxc_epdc_fb_set_pwrdown_delay(u32 pwrdown_delay,
					    struct fb_info *info);
int mxc_epdc_get_pwrdown_delay(struct fb_info *info);
int mxc_epdc_fb_set_upd_scheme(u32 upd_scheme, struct fb_info *info);

/* Lab126
 */
#define MXC_EDPC_POWER_STATE_UNINITED	-1
#define MXC_EDPC_POWER_STATE_OFF	0
#define MXC_EDPC_POWER_STATE_ON		1

int  mxc_epdc_power_state(struct fb_info *info);

void mxc_epdc_get_var_fix_info(struct fb_var_screeninfo *var, struct fb_fix_screeninfo *fix);
int  mxc_epdc_fb_set(struct fb_var_screeninfo *var);

bool mxc_epdc_busy(bool ignore_luts_busy);
int mxc_epdc_blit_to_fb(u8 *buffer, struct mxcfb_rect *rect, struct mxcfb_rect *dirty_rect);

void mxc_epdc_set_wv_file(void *wv_file, char *wv_file_name, int wv_file_size);
void mxc_epdc_clr_wv_file(void);

#endif
