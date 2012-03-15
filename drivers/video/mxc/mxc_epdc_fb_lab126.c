/*
 * Copyright 2011 Amazon Technologies, Inc. All Rights Reserved.
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

/* Lab126 functions for mxc_epdc
 */
#if 0 // def __ARM_NEON__
#include <arm_neon.h>
#endif

int mxc_epdc_power_state(struct fb_info *info)
{
	struct mxc_epdc_fb_data *fb_data = info ?
		(struct mxc_epdc_fb_data *)info : g_fb_data;

	if (!fb_data)
	    return MXC_EDPC_POWER_STATE_UNINITED;

	return fb_data->power_state;
}

void mxc_epdc_get_var_fix_info(struct fb_var_screeninfo *var, struct fb_fix_screeninfo *fix)
{
	struct mxc_epdc_fb_data *fb_data = g_fb_data;

	if (fb_data && var != NULL && fix != NULL) {
		*var = fb_data->info.var;
		*fix = fb_data->info.fix;
	}
}

int mxc_epdc_fb_set(struct fb_var_screeninfo *var) 
{
	int result = -EINVAL;
	struct mxc_epdc_fb_data *fb_data = g_fb_data;

	if (fb_data && var) {		
		var->activate = FB_ACTIVATE_FORCE | FB_ACTIVATE_NOW | FB_ACTIVATE_ALL;
		result = fb_set_var(&(fb_data->info), var);
	}
	
	return result;
}

int mxc_epdc_blit_to_fb(u8 *buffer, struct mxcfb_rect *rect, struct mxcfb_rect *dirty_rect)
{
	int is_DU = 0;
	int du_possible = 1;
	struct mxc_epdc_fb_data *fb_data = g_fb_data;

	if (!fb_data)
	    return 0;

	if (buffer && rect) {
		struct fb_info *info = &fb_data->info;
		u8 *fb = info->screen_base;
		int i, y;

		/*
		 * Simplify the blit by precomputing various values.
		 */
		u32 bpp			= info->var.bits_per_pixel;
		
		u32 xres_fb		= info->var.xres_virtual;
		u32 xres_buffer 	= rect->width;
		
		u32 rowbytes_fb		= xres_fb * (bpp / 8);
		u32 rowbytes_buffer	= xres_buffer * (bpp / 8);
		
		u32 ystart		= rect->top;
		u32 yend		= rect->height;
		u32 xstart		= rect->left  * (bpp / 8);
		unsigned long flags;

		/*
		 * Blit from the buffer to the fb a row at a time.
		 */
		local_irq_save(flags);
#if 0 /* __ARM_NEON__ */
        /* Disabling neon optimizations until we figure out what's wrong */
		uint8_t z = 1;
		uint8x8_t inc_vec = vld1_u8(&z);
		uint32_t cmp[2];
		for (i = 0, y = ystart; i < yend; i++, y++) {
			if (du_possible) {
				int x;
				for (x=0 ; x <rowbytes_buffer ; x +=8) {
					int base_offset = (rowbytes_buffer * i) + x;
					uint8x8_t fb_vec = vld1_u8(&fb[(rowbytes_fb * y) + xstart + x]);
					uint8x8_t buf_vec = vld1_u8(&buffer[base_offset]);
					uint8x8_t cmp_vec = vceq_u8(fb_vec, buf_vec);
					vst1_u32(cmp, cmp_vec);
					if ( !cmp[0] && !cmp[1] ) {
						uint8x8_t zcmp_vec = vshr_n_u8(vadd_u8(buf_vec, inc_vec),1); // Shift right by 1
						vst1_u32(cmp, zcmp_vec);
						if ( !cmp[0] && !cmp[1] ) {
							is_DU = 1;
						}
						else {
							is_DU = 0;
							du_possible = 0;
							goto out;
						}
					}
				}
			}
#else
		for (i = 0, y = ystart; i < yend; i++, y++) {
			int x;

			if (du_possible) {
				for (x = 0; x < rowbytes_buffer; x++) {
				        if (fb[(rowbytes_fb * y) + xstart + x]
						!= buffer[rowbytes_buffer * i + x]) {
						if (((buffer[rowbytes_buffer * i + x] == 0) ||
						(buffer[rowbytes_buffer * i + x] == 0xff)))
							is_DU = 1;
						else {
							is_DU = 0;
							du_possible = 0;
							goto out;
						}
					}
				}
			}
#endif
out:
			memcpy(&fb[(rowbytes_fb * y) + xstart],
				&buffer[rowbytes_buffer * i], rowbytes_buffer);
		}
		local_irq_restore(flags);
	}

	return is_DU;
}

void mxc_epdc_set_wv_file(void *wv_file, char *wv_file_name, int wv_file_size)
{
	struct mxc_epdc_fb_data *fb_data = g_fb_data;
	
	if (!fb_data)
	    return;

	if (wv_file && wv_file_name && wv_file_size) {
		fb_data->wv_file = (struct mxcfb_waveform_data_file *)wv_file;
		fb_data->wv_file_name = wv_file_name;
		fb_data->wv_file_size = wv_file_size;
		
	}
}

void mxc_epdc_clr_wv_file(void)
{
	struct mxc_epdc_fb_data *fb_data = g_fb_data;

	if (!fb_data)
	    return;

	fb_data->wv_file = NULL;
	fb_data->wv_file_name = NULL;
	fb_data->wv_file_size = 0;
}

EXPORT_SYMBOL(mxc_epdc_power_state);
EXPORT_SYMBOL(mxc_epdc_get_var_fix_info);
EXPORT_SYMBOL(mxc_epdc_fb_set);
EXPORT_SYMBOL(mxc_epdc_blit_to_fb);
EXPORT_SYMBOL(mxc_epdc_set_wv_file);
EXPORT_SYMBOL(mxc_epdc_clr_wv_file);

#include <linux/einkwf.h>

#define WF_PATH_LEN 256

static u8   *mxc_epdc_waveform_proxy = NULL;
static char *mxc_epdc_waveform_name  = NULL;

static void set_waveform_modes(struct mxc_epdc_fb_data *fb_data)
{
    struct mxcfb_waveform_modes waveform_modes;

    einkwf_panel_set_update_modes();
    
    // Tell the EPDC what the waveform update modes are, based on
    // what we've read back from the waveform itself.  These are
    // what is used when we tell the EPDC to use WF_UPD_MODE_AUTO. 
    //
    waveform_modes.mode_init = einkwf_panel_get_waveform_mode(WF_UPD_MODE_INIT);
    waveform_modes.mode_du   = einkwf_panel_get_waveform_mode(WF_UPD_MODE_MU);
    waveform_modes.mode_gc4  = einkwf_panel_get_waveform_mode(WF_UPD_MODE_GU);
    waveform_modes.mode_gc8  = einkwf_panel_get_waveform_mode(WF_UPD_MODE_GC);
    waveform_modes.mode_gc16 = einkwf_panel_get_waveform_mode(WF_UPD_MODE_GC);
    waveform_modes.mode_gc32 = einkwf_panel_get_waveform_mode(WF_UPD_MODE_GC);
    waveform_modes.mode_gl16 = einkwf_panel_get_waveform_mode(WF_UPD_MODE_GL);
    
    mxc_epdc_fb_set_waveform_modes(&waveform_modes, (struct fb_info *) fb_data);
}

#define mV_to_uV(mV)        ((mV) * 1000)
#define uV_to_mV(uV)        ((uV) / 1000)
#define V_to_uV(V)          (mV_to_uV((V) * 1000))
#define uV_to_V(uV)         (uV_to_mV(uV) / 1000)

static void set_vcom(struct mxc_epdc_fb_data *fb_data, int vcom)
{
    int ret;
    
    // Wake up the display pmic
    mutex_lock(&fb_data->power_mutex);

    ret = regulator_enable(fb_data->display_regulator);
    if (IS_ERR((void *)ret)) {
	dev_err(fb_data->dev, "Unable to enable DISPLAY regulator."
		"err = 0x%x\n", ret);
	mutex_unlock(&fb_data->power_mutex);
	return;
    }
    ret = regulator_enable(fb_data->vcom_regulator);
    if (IS_ERR((void *)ret)) {
	dev_err(fb_data->dev, "Unable to enable VCOM regulator."
		"err = 0x%x\n", ret);
	mutex_unlock(&fb_data->power_mutex);
	return;
    }

    // Note:  The set-voltage operation takes a min and max value in uV.  But we really
    //        only use the max value to set VCOM with.
    //
    if ( 0 != regulator_set_voltage(fb_data->vcom_regulator, 0, mV_to_uV(vcom) ) )
    {
	dev_err(fb_data->dev, "unable to set VCOM = %d\n", vcom);
    }
    else
    {
        dev_dbg(fb_data->dev, "VCOM = %d\n", uV_to_mV(regulator_get_voltage(fb_data->vcom_regulator)));
    }
    

    /* Disable power to the EPD panel */
    regulator_disable(fb_data->vcom_regulator);
    regulator_disable(fb_data->display_regulator);
    
    mutex_unlock(&fb_data->power_mutex);
}

void mxc_epdc_waveform_init(struct mxc_epdc_fb_data *fb_data) 
{
    int waveform_proxy_size = 0;
    struct fb_var_screeninfo tmpvar;

    if (wf_to_use  == NULL) {
	wf_to_use = kzalloc(WF_PATH_LEN, GFP_KERNEL);
    }

    if ( !einkwf_panel_flash_present() )
    {
        // We'll be using the built-in waveform in this case...
        //
        strcpy(wf_to_use, EINK_WF_USE_BUILTIN_WAVEFORM);
	printk(KERN_ERR "no panel present, use builtin waveform\n");

    }

    // Read in the waveform and waveform proxy files.
    //
    mxc_epdc_waveform_proxy = einkwf_panel_get_waveform(wf_to_use, &waveform_proxy_size);
        
    // Upon success, pass the waveform proxy file to the EPDC.  Otherwise,
    // log an error that it couldn't be used.
    //
    if ( mxc_epdc_waveform_proxy )
    {
	mxc_epdc_waveform_name = einkwf_panel_get_waveform_version_string(eink_waveform_filename);

	pr_debug("Setting wv file: %p %s (sz=%d)\n", mxc_epdc_waveform_proxy, mxc_epdc_waveform_name, waveform_proxy_size);

	fb_data->wv_file = (struct mxcfb_waveform_data_file *) mxc_epdc_waveform_proxy;
	fb_data->wv_file_name = mxc_epdc_waveform_name;
	fb_data->wv_file_size = waveform_proxy_size;
    }
    else {
	printk(KERN_ERR "Couldn't find waveform, using builtin\n");
    }

    // Set vcom value
    //
    set_vcom(fb_data, einkwf_panel_get_vcom());

    // Set the waveform modes from the waveform.
    //
    set_waveform_modes(fb_data);

    // Say that we want to switch into Y8 mode:  This is where the INIT waveform
    // update occurs.
    //
    tmpvar = fb_data->info.var;
    tmpvar.bits_per_pixel = 8;
    tmpvar.grayscale = GRAYSCALE_8BIT;
    tmpvar.activate = FB_ACTIVATE_FORCE | FB_ACTIVATE_NOW | FB_ACTIVATE_ALL;
    fb_set_var(&(fb_data->info), &tmpvar);

    // Say that we want to switch to portrait mode. 
    //
    tmpvar.rotate = FB_ROTATE_CW;
    tmpvar.activate = FB_ACTIVATE_FORCE | FB_ACTIVATE_NOW | FB_ACTIVATE_ALL;
    fb_set_var(&(fb_data->info), &tmpvar);   
}

void mxc_epdc_waveform_done(void)
{
    if ( mxc_epdc_waveform_proxy )
    {
        einkwf_panel_waveform_free(mxc_epdc_waveform_proxy);
        mxc_epdc_clr_wv_file();
        
        mxc_epdc_waveform_proxy = NULL;
    }
}
