/*
 *  linux/drivers/video/eink/fslepdc/fslepdc.c
 *  --eInk frame buffer device HAL FSL EPDC
 *
 *  Copyright (c) 2010-2011 Amazon Technologies, Inc.
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include "../hal/einkfb_hal.h"

#include "fslepdc.h"
#include "fslepdc_waveform.h"

// Default EPDC resolution is 600x800@8bpp.
//
#define XRES_6                  600
#define YRES_6                  800
#define BPP                     EINKFB_8BPP
#define FSLEPDC_SIZE            BPP_SIZE((XRES_6*YRES_6), BPP)

#define FSLEPDC_SU_TIMEOUT      (HZ * 2) // How long to keep trying to send an update before giving up.
#define FSLEPDC_SU_WAIT         (HZ / 2) // How long to wait before trying to send an update again.

#define FSLEPDC_SU_DELAY        (HZ / 2) // Delay until we until we try again.
#define FSLEPDC_SU_RETRIES      3        // Give up.

#define FSLEPDC_UPDATE_RETRY    false
#define FSLEPDC_UPDATE_NEW      true

#define FSLEPDC_ROTATE_UR_0     FB_ROTATE_UR
#define FSLEPDC_ROTATE_UR_90    FB_ROTATE_CW
#define FSLEPDC_ROTATE_UR_180   FB_ROTATE_UD
#define FSLEPDC_ROTATE_UR_270   FB_ROTATE_CCW

#define FSLEPDC_ROTATE_UD_0     FB_ROTATE_UD
#define FSLEPDC_ROTATE_UD_90    FB_ROTATE_CCW
#define FSLEPDC_ROTATE_UD_180   FB_ROTATE_UR
#define FSLEPDC_ROTATE_UD_270   FB_ROTATE_CW

#define FSLEPDC_ROTATE_0        (fslepdc_rotate ? fslepdc_rotate[FB_ROTATE_UR]  : FB_ROTATE_UR)
#define FSLEPDC_ROTATE_90       (fslepdc_rotate ? fslepdc_rotate[FB_ROTATE_CW]  : FB_ROTATE_CW)
#define FSLEPDC_ROTATE_180      (fslepdc_rotate ? fslepdc_rotate[FB_ROTATE_UD]  : FB_ROTATE_UD)
#define FSLEPDC_ROTATE_270      (fslepdc_rotate ? fslepdc_rotate[FB_ROTATE_CCW] : FB_ROTATE_CCW)

#define FSLEPDC_ORIENTATION(r)                      \
    (fslepdc_orientation ? fslepdc_orientation[(r)] \
                         : fslepdc_orientations[(r)])

// TODO: Retrieve width and height based on panel id
// Portrait - HardCoded.
#define FSLEPDC_DEFAULT_WIDTH   91
#define FSLEPDC_DEFAULT_HEIGHT  121

static int fslepdc_removed = 0;
static int fslepdc_repair_count = 0;
static u32 fslepdc_repair_x1;
static u32 fslepdc_repair_y1;
static u32 fslepdc_repair_x2;
static u32 fslepdc_repair_y2;

static struct fb_var_screeninfo fslepdc_var =
{
    .xres                       = XRES_6,
    .yres                       = YRES_6,
    .xres_virtual               = XRES_6,
    .yres_virtual               = YRES_6,
    .bits_per_pixel             = BPP,
    .grayscale                  = 1,
    .activate                   = FB_ACTIVATE_TEST,
    .height                     = -1,
    .width                      = -1
};

static struct fb_fix_screeninfo fslepdc_fix =
{
    .id                         = EINKFB_NAME,
    .smem_len                   = FSLEPDC_SIZE,
    .type                       = FB_TYPE_PACKED_PIXELS,
    .visual                     = FB_VISUAL_STATIC_PSEUDOCOLOR,
    .xpanstep                   = 0,
    .ypanstep                   = 0,
    .ywrapstep                  = 0,
    .line_length                = BPP_SIZE(XRES_6, BPP),
    .accel                      = FB_ACCEL_NONE
};

static orientation_t fslepdc_orientations[8] =
{
    // Upright orientations.
    //
    orientation_landscape,
    orientation_portrait,
    orientation_landscape_upside_down,
    orientation_portrait_upside_down,
    
    // Upside down orientations.
    //
    orientation_landscape_upside_down,
    orientation_portrait_upside_down,
    orientation_landscape,
    orientation_portrait
};

static int fslepdc_rotations[8] =
{
    // Upright rotatations.
    //
    FSLEPDC_ROTATE_UR_0,
    FSLEPDC_ROTATE_UR_90,
    FSLEPDC_ROTATE_UR_180,
    FSLEPDC_ROTATE_UR_270,
    
    // Upside down rotatations.
    //
    FSLEPDC_ROTATE_UD_0,
    FSLEPDC_ROTATE_UD_90,
    FSLEPDC_ROTATE_UD_180,
    FSLEPDC_ROTATE_UD_270,
};

static orientation_t *fslepdc_orientation_ur = &fslepdc_orientations[0];
static orientation_t *fslepdc_orientation_ud = &fslepdc_orientations[4];
static orientation_t *fslepdc_orientation    = NULL;

static int *fslepdc_rotate_ur = &fslepdc_rotations[0];
static int *fslepdc_rotate_ud = &fslepdc_rotations[4];
static int *fslepdc_rotate    = NULL;

static char fslepdc_wf_path[EINK_WF_PATH_LEN] = EINK_WF_UNKNOWN_PATH;
static char *fslepdc_wf_to_use = NULL;

static einkfb_power_level fslepdc_power_level = einkfb_power_level_init;
static int fslepdc_temperature = EINKFB_TEMP_INVALID;

static u8   *fslepdc_waveform_proxy = NULL;
static char *fslepdc_waveform_name  = NULL;

static u32  fslepdc_last_update_marker = 0;
static int  fslepdc_bootstrap = 0;

struct mxcfb_update_data fslepdc_send_update_retry_data;
static int  fslepdc_send_update_retry_counter = 0;
static void fslepdc_send_update_retry(struct work_struct *work);
static DECLARE_DELAYED_WORK(fslepdc_send_update_work, fslepdc_send_update_retry);

#ifdef MODULE
module_param_named(waveform_to_use, fslepdc_wf_to_use, charp, S_IRUGO);
MODULE_PARM_DESC(waveform_to_use, "/path/to/waveform_file or built-in");
module_param_named(bootstrap, fslepdc_bootstrap, int, S_IRUGO);
MODULE_PARM_DESC(bootstrap, "non-zero to bootstrap");
#endif //  MODULE

#include "fslepdc_waveform.c"

static void fslepdc_init_screen_var_fix_info(struct fb_var_screeninfo *var, struct fb_fix_screeninfo *fix)
{
    if ( var && fix )
    {
        memset(var, 0, sizeof(struct fb_var_screeninfo));
        memset(fix, 0, sizeof(struct fb_fix_screeninfo));
    }
}

static void fslepdc_set_orientation(void)
{
        fslepdc_orientation = fslepdc_orientation_ur;
        fslepdc_rotate = fslepdc_rotate_ur;    
}

static bool fslepdc_sw_init(struct fb_var_screeninfo *var, struct fb_fix_screeninfo *fix)
{
    // Note:  We're doing this backwards here at the moment.  That is, we're getting
    //        the panel resolution information from the EPDC driver.  We should be
    //        reading the waveform from the panel, using the information from the
    //        waveform header to tell us what kind of panel is actually connected
    //        to the device, and then using that information to tell the EPDC
    //        driver how to run the panel.
    //
    // If we're not boot-strapped, get the screen into the right state.
    //
    if ( !FSLEPDC_BOOTSTRAPPED() )
    {
        struct fb_var_screeninfo temp_var;
        struct fb_fix_screeninfo temp_fix;
        int waveform_proxy_size = 0;
        
        fslepdc_init_screen_var_fix_info(&temp_var, &temp_fix);

        // Read in the waveform and waveform proxy files.
        //
        fslepdc_waveform_proxy = einkwf_panel_get_waveform(fslepdc_wf_to_use, &waveform_proxy_size);
        
        // Upon success, pass the waveform proxy file to the EPDC.  Otherwise,
        // log an error that it couldn't be used.
        //
        if ( fslepdc_waveform_proxy )
        {
	    einkwf_set_buffer(fslepdc_waveform_proxy);
	    fslepdc_waveform_name = einkwf_get_waveform_version_string(eink_waveform_filename);

            mxc_epdc_set_wv_file((void *)fslepdc_waveform_proxy, fslepdc_waveform_name, waveform_proxy_size);
        }
        else
            einkfb_print_warn("waveform_to_use \"%s\" couldn't be set\n", fslepdc_wf_to_use);

        // Before bringing the display up for the first time, ensure that the VCOM value from
        // the panel is set.
        //
        fslepdc_pmic_set_vcom(einkwf_panel_get_vcom());
        
        // Set the orientation for the display.
        //
        fslepdc_set_orientation();
    
        // Set the waveform modes from the waveform.
        //
        fsledpc_set_waveform_modes();
        
        // Tell the EPDC driver to use the snapshot update scheme since it's more compatible
        // with how Broadsheet/ISIS works.
        //
        //mxc_epdc_fb_set_upd_scheme(UPDATE_SCHEME_SNAPSHOT, NULL);

        // Get the default variable and fixed screen_info from the EPDC.
        //
        mxc_epdc_get_var_fix_info(&temp_var, &temp_fix);

        // Say that we want to switch into Y8-inverted mode:  This is where the INIT waveform
        // update occurs.
        //
        temp_var.bits_per_pixel = BPP;
        temp_var.grayscale = GRAYSCALE_8BIT_INVERTED;
        mxc_epdc_fb_set(&temp_var);
    
        // Say that we want to switch to portrait mode. 
        //
        temp_var.rotate = FSLEPDC_ROTATE_90;
        mxc_epdc_fb_set(&temp_var);
    
        // Now, return to the eInk HAL how things are actually set up.
        //
        // Note:  We should also be setting the width and height fields, which are the width
        //        and height of the panel in millimeters for DPI computations.  But we need
        //        to be getting resolution of the panel from the waveform so that we can
        //        set those values correctly.
        //
        mxc_epdc_get_var_fix_info(&temp_var, &temp_fix);
    
        fslepdc_var.xres = temp_var.xres;
        fslepdc_var.yres = temp_var.yres;
        fslepdc_var.xres_virtual =  temp_var.xres;
        fslepdc_var.yres_virtual =  temp_var.yres;
        fslepdc_var.bits_per_pixel = temp_var.bits_per_pixel;
        fslepdc_var.width  = FSLEPDC_DEFAULT_WIDTH;
        fslepdc_var.height = FSLEPDC_DEFAULT_HEIGHT;
        fslepdc_fix.smem_len = BPP_SIZE((fslepdc_var.xres * fslepdc_var.yres), fslepdc_var.bits_per_pixel);
        fslepdc_fix.line_length = BPP_SIZE(fslepdc_var.xres, fslepdc_var.bits_per_pixel);
    }

    *var = fslepdc_var;
    *fix = fslepdc_fix;
    
    return ( EINKFB_SUCCESS );
}

static void fslepdc_free_waveform_proxy(void)
{
    if ( fslepdc_waveform_proxy )
    {
	einkwf_set_buffer(NULL);
        einkwf_panel_waveform_free(fslepdc_waveform_proxy);
        mxc_epdc_clr_wv_file();
        
        fslepdc_waveform_proxy = NULL;
    }
}

static void fslepdc_sw_done(void) 
{
    // Ensure that any scheduled work is completed.
    //
    flush_scheduled_work();
    
    // Stop using the PMIC.
    //
    fslepdc_pmic_done();
    
    // Free the memory for the waveform proxy.
    //
    fslepdc_free_waveform_proxy();
}

static bool fslepdc_set_display_orientation(orientation_t orientation) 
{
    struct fb_var_screeninfo temp_var;
    struct fb_fix_screeninfo temp_fix;
    
    int rotate = FSLEPDC_ROTATE_90;
    bool result = false;

    fslepdc_repair_count = 0;
    
    fslepdc_init_screen_var_fix_info(&temp_var, &temp_fix);

    switch ( orientation )
    {
        case orientation_portrait:
        default:
            rotate = FSLEPDC_ROTATE_90;
        break;
        
        case orientation_portrait_upside_down:
            rotate = FSLEPDC_ROTATE_270;
        break;
        
        case orientation_landscape:
            rotate = FSLEPDC_ROTATE_0;
        break;
        
        case orientation_landscape_upside_down:
            rotate = FSLEPDC_ROTATE_180;
        break;
    }

    mxc_epdc_get_var_fix_info(&temp_var, &temp_fix);
    
    if ( rotate != temp_var.rotate )
    {
        temp_var.rotate = rotate;
        result = 0 == mxc_epdc_fb_set(&temp_var);
    }

    return ( result );
}

static orientation_t fslepdc_get_display_orientation(void) 
{
    struct fb_var_screeninfo temp_var;
    struct fb_fix_screeninfo temp_fix;
    
    fslepdc_init_screen_var_fix_info(&temp_var, &temp_fix);
    mxc_epdc_get_var_fix_info(&temp_var, &temp_fix);
    
    return ( FSLEPDC_ORIENTATION(temp_var.rotate) );
}

int fslepdc_read_temperature(void)
{
    int temp = fslepdc_temperature;
    
    // If the override temperature is not in range...
    //
    if ( !IN_RANGE(fslepdc_temperature, EINKFB_TEMP_MIN, EINKFB_TEMP_MAX) )
    {
        // ...then get the temperature from the PMIC in the
        // specified way.
        //
        temp = fslepdc_pmic_get_temperature();
    }
    
    return ( temp );
}

static bool fslepdc_using_builtin_waveform(void)
{
    return (strcmp(fslepdc_wf_to_use, EINK_WF_USE_BUILTIN_WAVEFORM) == 0);
}

static void fslepdc_sync(void)
{
    if ( fslepdc_last_update_marker )
    {
        mxc_epdc_fb_wait_update_complete(fslepdc_last_update_marker, NULL);
    }
}

static unsigned long fslepdc_set_ld_img_start;
static unsigned long fslepdc_ld_img_start;
static unsigned long fslepdc_upd_data_start;

static unsigned long fslepdc_image_start_time;
static unsigned long fslepdc_image_processing_time;
static unsigned long fslepdc_image_loading_time;
static unsigned long fslepdc_image_display_time;
static unsigned long fslepdc_image_stop_time;

#define FSLEPDC_IMAGE_TIMING_START   0
#define FSLEPDC_IMAGE_TIMING_PROC    1
#define FSLEPDC_IMAGE_TIMING_LOAD    2
#define FSLEPDC_IMAGE_TIMING_DISP    3
#define FSLEPDC_IMAGE_TIMING_STOP    4
#define FSLEPDC_NUM_IMAGE_TIMINGS    (FSLEPDC_IMAGE_TIMING_STOP + 1)
static unsigned long fslepdc_image_timings[FSLEPDC_NUM_IMAGE_TIMINGS];

static unsigned long *fslepdc_get_img_timings(int *num_timings)
{
    unsigned long *timings = NULL;

    if ( num_timings )
    {
        fslepdc_image_timings[FSLEPDC_IMAGE_TIMING_START] = fslepdc_image_start_time;
        fslepdc_image_timings[FSLEPDC_IMAGE_TIMING_PROC]  = fslepdc_image_processing_time;
        fslepdc_image_timings[FSLEPDC_IMAGE_TIMING_LOAD]  = fslepdc_image_loading_time;
        fslepdc_image_timings[FSLEPDC_IMAGE_TIMING_DISP]  = fslepdc_image_display_time;
        fslepdc_image_timings[FSLEPDC_IMAGE_TIMING_STOP]  = fslepdc_image_stop_time;

        *num_timings = FSLEPDC_NUM_IMAGE_TIMINGS;
        timings = fslepdc_image_timings;
    }

    return ( timings );
}

static bool fslepdc_send_update(struct mxcfb_update_data *update_data, bool retry)
{
    bool result = false;
    
    if ( update_data )
    {
        unsigned long start_time, stop_time;
        int send_update_err;
        
        // If this isn't a retry...
        //
        if ( !retry )
        {
            // ...cancel any pending retries.
            //
            cancel_delayed_work(&fslepdc_send_update_work);
            
            // But accumulate any pending retry with the new data.
            //
            if ( fslepdc_send_update_retry_counter )
            {
                struct mxcfb_rect old_retry = fslepdc_send_update_retry_data.update_region,
                                  new_retry,
                                  update    = update_data->update_region;
                u32               old_retry_right,
                                  old_retry_bot,
                                  new_retry_right,
                                  new_retry_bot,
                                  update_right,
                                  update_bot;
                
                // First, accumulate the update region.
                //
                old_retry_right  = (old_retry.left + old_retry.width)  - 1;
                old_retry_bot    = (old_retry.top  + old_retry.height) - 1;
                update_right     = (update.left    + update.width)     - 1;
                update_bot       = (update.top     + update.height)    - 1;
                
                new_retry.left   = min(old_retry.left,  update.left);
                new_retry.top    = min(old_retry.top,   update.top);
                new_retry_right  = max(old_retry_right, update_right);
                new_retry_bot    = max(old_retry_bot,   update_bot);
                
                new_retry.width  = (new_retry_right - new_retry.left)  + 1;
                new_retry.height = (new_retry_bot   - new_retry.top)   + 1;
                
                fslepdc_send_update_retry_data.update_region = new_retry;
                
                // Since it's a retry, go for the highest fidelity possible.
                //
                fslepdc_send_update_retry_data.waveform_mode = fslepdc_get_waveform_mode(WF_UPD_MODE_GC);
                fslepdc_send_update_retry_data.update_mode   = UPDATE_MODE_FULL;
                
                // Use the latest marker and temperature.
                //
                fslepdc_send_update_retry_data.update_marker = update_data->update_marker;
                fslepdc_send_update_retry_data.temp          = update_data->temp;
                
                // Copy the retry data back for this attempt.
                //
                *update_data = fslepdc_send_update_retry_data;
            }
        }
        
        // We can get errors sending updates to EPDC if it's not ready to do
        // an update yet.  So, back off and retry a few times here first
        // before scheduling a retry.
        //
        start_time = jiffies; stop_time = start_time + FSLEPDC_SU_TIMEOUT;    

        do
        {
            send_update_err = mxc_epdc_fb_send_update(update_data, NULL);
            
            if ( 0 != send_update_err )
            {
                einkfb_print_error("EPDC_send_update_error=%d:\n", send_update_err);
                schedule_timeout_uninterruptible(FSLEPDC_SU_WAIT);
            }
        }
        while ( (0 != send_update_err) && time_before_eq(jiffies, stop_time) );
        
        if ( time_after(jiffies, stop_time) )
        {
             einkfb_print_crit("EDPC_send_update_timed_out=true:\n");
        }
        else
        {
            char temp_string[16];
            
            if ( TEMP_USE_AMBIENT == update_data->temp )
                strcpy(temp_string, "ambient");
            else
                sprintf(temp_string, "%d", update_data->temp);
            
            einkfb_debug("update_data:\n");
            einkfb_debug("  rect x: %d\n", update_data->update_region.left);
            einkfb_debug("  rect y: %d\n", update_data->update_region.top);
            einkfb_debug("  rect w: %d\n", update_data->update_region.width);
            einkfb_debug("  rect h: %d\n", update_data->update_region.height);
            einkfb_debug("  wfmode: %d\n", update_data->waveform_mode);
            einkfb_debug("  update: %s\n", update_data->update_mode ? "flashing" : "non-flashing");
            einkfb_debug("  marker: %d\n", update_data->update_marker);
            einkfb_debug("  temp:   %s\n", temp_string);
            
            fslepdc_send_update_retry_counter = 0;
            result = true;
        }

        // If our attempt to send an update failed, try it again later.
        //
        if ( !result )
        {
            if ( FSLEPDC_SU_RETRIES > ++fslepdc_send_update_retry_counter )
            {
                // If this isn't a retry, use the current update data.
                //
                if ( !retry )
                    fslepdc_send_update_retry_data = *update_data;
                
                schedule_delayed_work(&fslepdc_send_update_work, FSLEPDC_SU_DELAY);
            }
            else
            {
                einkfb_print_crit("Updates are failing...\n");
            }
        }
    }
    
    return ( result );
}

static void fslepdc_send_update_retry(struct work_struct *work)
{
    if (fslepdc_removed)
	return;

    einkfb_debug("Retrying update, count = %d\n", fslepdc_send_update_retry_counter);
    fslepdc_send_update(&fslepdc_send_update_retry_data, FSLEPDC_UPDATE_RETRY);
}

static void fsledpc_init_update_data(struct mxcfb_update_data *update_data)
{
    if ( update_data )
        memset(update_data, 0, sizeof(struct mxcfb_update_data));
}

static void fslepdc_repair_worker(struct work_struct *unused)
{
	struct mxcfb_update_data update_data;

	if (fslepdc_repair_count <= 1) {
		/* Too early  - no need to repair for one update */
		return;
	}

	memset(&update_data, 0, sizeof(struct mxcfb_update_data));

	update_data.update_region.left = fslepdc_repair_x1;
	update_data.update_region.top = fslepdc_repair_y1;
	update_data.update_region.width = fslepdc_repair_x2 - fslepdc_repair_x1;
	update_data.update_region.height = fslepdc_repair_y2 - fslepdc_repair_y1;

	update_data.waveform_mode = WF_UPD_MODE_AUTO; 

	update_data.update_mode = UPDATE_MODE_PARTIAL;
	fslepdc_last_update_marker++;
	update_data.update_marker = fslepdc_last_update_marker;
	update_data.temp = fslepdc_using_builtin_waveform() ? 
	    TEMP_USE_AMBIENT : fslepdc_read_temperature();

	fslepdc_repair_count = 0;

	fslepdc_send_update(&update_data, FSLEPDC_UPDATE_NEW);
}
DECLARE_DELAYED_WORK(fslepdc_repair_work, fslepdc_repair_worker);

#define is_MU_skip (-1)

static void fslepdc_update_area(update_area_t *update_area)
{
    int is_MU = 0;

    if ( einkfb_power_level_on == fslepdc_power_level )
    {
        fx_type update_mode = update_area->which_fx;
        u8 *data = update_area->buffer;

        if ( fx_display_sync == update_mode )
        {
            fslepdc_sync();
            data = NULL;
        }
    
        if ( data || UPDATE_MODE_BUFFER_DISPLAY(update_mode) )
        {
            bool    skip_buffer_display = false,
                    skip_buffer_load    = false,
                    flashing_update     = false, 
                    area_update         = false;
            u32     waveform_mode       = fslepdc_get_waveform_mode(WF_UPD_MODE_GC);
            
            struct mxcfb_update_data update_data;
            struct mxcfb_rect dirty_rect;
            struct einkfb_info info;

	    cancel_rearming_delayed_work(&fslepdc_repair_work);
            
            fslepdc_set_ld_img_start = jiffies;
            
            fsledpc_init_update_data(&update_data);
            einkfb_get_info(&info);
        
            update_data.update_region.left   = update_area->x1;
            update_data.update_region.top    = update_area->y1;
            update_data.update_region.width  = update_area->x2 - update_area->x1;
            update_data.update_region.height = update_area->y2 - update_area->y1; ;
            
            if ( (info.xres == update_data.update_region.width) && (info.yres == update_data.update_region.height) )
                area_update = false;
            else
                area_update = true;
            
            switch ( update_mode )
            {
                // Just load up the hardware's buffer; don't display it.
                //
                case fx_buffer_load:
                    skip_buffer_display = true;
                break;
        
                // Just display what's already in the hardware buffer; don't reload it.
                //
                case fx_buffer_display_partial:
                case fx_buffer_display_full:
                    skip_buffer_load = true;
                goto set_update_mode;
        
                // Regardless of what gets put into the hardware's buffer,
                // only update the black and white pixels.
                //
                case fx_update_fast:
                  waveform_mode = fslepdc_get_waveform_mode(WF_UPD_MODE_PU);
                  is_MU = is_MU_skip;
                goto set_update_mode;
        
                // Regardless of what gets put into the hardware's buffer,
                // use white transition update.
                //
                case fx_update_white_trans:
                  waveform_mode = fslepdc_get_waveform_mode(WF_UPD_MODE_GL);
                  is_MU = is_MU_skip;
                goto set_update_mode;
                    
                // Regardless of what gets put into the hardware's buffer,
                // refresh all pixels as cleanly as possible.
                //
                case fx_update_slow:
                  waveform_mode = fslepdc_get_waveform_mode(WF_UPD_MODE_GC);
                  is_MU = is_MU_skip;
                /* goto set_update_mode; */
                    
                set_update_mode:
                default:
                    
                    // Normalize to either flashing or non-flashing.
                    //
                    update_mode     = area_update     ? UPDATE_AREA_MODE(update_mode)
                                                      : UPDATE_MODE(update_mode);
                    
                    // Simplify that.
                    //
                    flashing_update = UPDATE_FULL(update_mode);

                    // Don't use DU/MU for flashing updates
                    is_MU = flashing_update ? is_MU_skip : is_MU;

                    // Convert to the MXC EPDC's scheme.
                    //
                    update_mode     = flashing_update ? UPDATE_MODE_FULL
                                                      : UPDATE_MODE_PARTIAL;
                break;
            }
    
            // Process and load the image data if we should.
            //
            fslepdc_ld_img_start = jiffies;
           
	    if (!skip_buffer_load)
	    {
                // Check to see whether we can force an MU or not.
                //
                if (mxc_epdc_blit_to_fb(data, &update_data.update_region, &dirty_rect))
                {
                        // In the fx_update_fast & fx_update_slow cases, we want very
                        // specific waveform modes.  So, in those cases, we don't
                        // want to force an MU, even if we can.
                        //
                        if (0 == is_MU)
                                is_MU = 1;

                }
            }

            // Skip the forced MU when we should.
            //
            if (is_MU_skip == is_MU)
                is_MU = 0;

            // Update the display in the specified way if we should.
            //
            fslepdc_upd_data_start = jiffies;

            if ( !skip_buffer_display )
            {
                // If this is a flashing or a full-screen update, wait until
                // the last update completes before sending a new one.
                //
                if ( flashing_update || !area_update ) {
			if (fslepdc_repair_x1 && fslepdc_repair_y1 && 
				((fslepdc_repair_y2 - fslepdc_repair_y1) <= 300) &&
				((fslepdc_repair_x2 - fslepdc_repair_x1) <= 300) )
			{
				/* Do Nothing */
			}
			else {
				if ( (fslepdc_var.xres == (fslepdc_repair_x2 - fslepdc_repair_x1)) &&
					(fslepdc_var.yres == (fslepdc_repair_y2 - fslepdc_repair_y1)) ){
						/* Do Nothing */
				}
				else 
					fslepdc_sync();
			}
		}
                
                // Set up to perform the specified update type, marking it pseudo-uniquely.  Also,
                // if we're using the built-in waveform, which is 25C-only, don't even bother
                // using the cached temperature since it won't be used anyway.
                //
    
                update_data.update_mode   = update_mode;
                update_data.temp          = fslepdc_using_builtin_waveform() ? TEMP_USE_AMBIENT
                                                                             : fslepdc_read_temperature();

                // Send the update itself.
                //
		if (is_MU)
			update_data.waveform_mode = WF_UPD_MODE_MU;
		else
			update_data.waveform_mode = waveform_mode;

		fslepdc_last_update_marker++;
		update_data.update_marker = fslepdc_last_update_marker;
			
		/* repair logic */
		if (!fslepdc_repair_count ) {
			fslepdc_repair_x1 = update_data.update_region.left;
			fslepdc_repair_y1 = update_data.update_region.top;
			fslepdc_repair_x2 = fslepdc_repair_x1 + update_data.update_region.width;
			fslepdc_repair_y2 = fslepdc_repair_y1 + update_data.update_region.height;

			fslepdc_repair_count++;
		}
		else {
			fslepdc_repair_x1 = min(update_data.update_region.left, fslepdc_repair_x1);
			fslepdc_repair_y1 = min(update_data.update_region.top, fslepdc_repair_y1);
			fslepdc_repair_x2 = max((update_data.update_region.left +
					update_data.update_region.width), fslepdc_repair_x2);

			fslepdc_repair_y2 = max((update_data.update_region.top +
					update_data.update_region.height), fslepdc_repair_y2);
			fslepdc_repair_count++;
		}

		fslepdc_send_update(&update_data, FSLEPDC_UPDATE_NEW);
		if (UPDATE_MODE_FULL == update_data.update_mode) {
			fslepdc_repair_count = 0;
		}
		else {
			schedule_delayed_work(&fslepdc_repair_work, msecs_to_jiffies(1000));
		}
	}

	fslepdc_image_stop_time = jiffies;
    
	fslepdc_image_start_time = jiffies_to_msecs(fslepdc_set_ld_img_start - info.jif_on);
	fslepdc_image_processing_time = jiffies_to_msecs(fslepdc_ld_img_start - fslepdc_set_ld_img_start);
	fslepdc_image_loading_time = jiffies_to_msecs(fslepdc_upd_data_start - fslepdc_ld_img_start);
	fslepdc_image_display_time = jiffies_to_msecs(fslepdc_image_stop_time  - fslepdc_upd_data_start);
	fslepdc_image_stop_time = jiffies_to_msecs(fslepdc_image_stop_time  - info.jif_on);
        }
    }
}

static void fslepdc_update_display_buffer(fx_type update_mode, u8 *buffer)
{
    update_area_t update_area;
    struct einkfb_info info;
    einkfb_get_info(&info);
    
    update_area.x1 = 0;
    update_area.y1 = 0;
    update_area.x2 = info.xres;
    update_area.y2 = info.yres;
    
    update_area.which_fx = update_mode;
    update_area.buffer = (NULL == buffer) ? info.start : buffer;
    
    fslepdc_update_area(&update_area);
}

static void fslepdc_update_display(fx_type update_mode) 
{
    fslepdc_update_display_buffer(update_mode, NULL);
}

static void fslepdc_blank_screen(void)
{
    struct einkfb_info info;
    einkfb_get_info(&info);
    
    memset(info.buf, EINKFB_WHITE, BPP_SIZE((info.xres * info.yres), info.bpp));
    fslepdc_update_display_buffer(fx_update_slow, info.buf);
}

static void fslepdc_set_power_level(einkfb_power_level power_level)
{
    switch ( power_level )
    {
        // If we're not already powerd on, refresh the cached temperature.
        //
        case einkfb_power_level_on:
        case einkfb_power_level_standby:
        case einkfb_power_level_sleep:
            fslepdc_power_level = power_level;
        break;
        
        // Blank the screen and return to standby.
        //
        case einkfb_power_level_blank:
            fslepdc_set_power_level(einkfb_power_level_on);
            fslepdc_blank_screen();
            fslepdc_set_power_level(einkfb_power_level_standby);
        break;
        
        // Blank screen and go to sleep.
        //
        case einkfb_power_level_off:
            fslepdc_set_power_level(einkfb_power_level_on);
            fslepdc_blank_screen();
            fslepdc_set_power_level(einkfb_power_level_sleep);
        break;

        // Prevent the compiler from complaining.
        //
        default:
        break;
    }
}

static einkfb_power_level fslepdc_get_power_level(void)
{
    return ( fslepdc_power_level );
}

// /sys/devices/platform/eink_fb.0/override_upd_mode (read/write)
//
static ssize_t show_override_upd_mode(FB_DSHOW_PARAMS)
{
    return ( sprintf(buf, "%d\n", fslepdc_get_override_upd_mode()) );
}

static ssize_t store_override_upd_mode(FB_DSTOR_PARAMS)
{
    int result = -EINVAL, upd_mode;
    
    if ( sscanf(buf, "%d", &upd_mode) )
    {
        fslepdc_set_override_upd_mode(upd_mode);
        result = count;
    }
    
    return ( result );
}

// /sys/devices/platform/eink_fb.0/vcom (read/write)
//
static int read_vcom(char *page, unsigned long off, int count)
{
    int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off ) {
        result = sprintf(page, "%s\n", fslepdc_get_vcom_str());
    }

    return ( result );
}

static int write_vcom(char *page, unsigned long off, int count)
{
    int result = -EINVAL, vcom = 0;
    
    if ( sscanf(page, "%d", &vcom) )
    {
        fslepdc_override_vcom(vcom);
        
        result = count;
    }
    
    return ( result );
}

static ssize_t show_vcom(FB_DSHOW_PARAMS)
{
    return ( EINKFB_PROC_SYSFS_RW_LOCK(buf, NULL, 0, 0, 0, 0, read_vcom) );
}

static ssize_t store_vcom(FB_DSTOR_PARAMS)
{
    return ( EINKFB_PROC_SYSFS_RW_LOCK((char *)buf, NULL, 0, count, 0, 0, write_vcom) );
}

// /sys/devices/platform/eink_fb.0/image_timings (read-only)
//
static ssize_t show_image_timings(FB_DSHOW_PARAMS)
{
    int i, len, num_timings;
    unsigned long *timings = fslepdc_get_img_timings(&num_timings);
    
    for ( i = 0, len = 0; i < num_timings; i++ )
        len += sprintf(&buf[len], "%ld%s", timings[i], (i == (num_timings - 1)) ? "" : " " );
        
    return ( len );
}

static DEVICE_ATTR(override_upd_mode,   DEVICE_MODE_RW, show_override_upd_mode, store_override_upd_mode);
static DEVICE_ATTR(vcom,                DEVICE_MODE_RW, show_vcom,              store_vcom);
static DEVICE_ATTR(image_timings,       DEVICE_MODE_R,  show_image_timings,     NULL);

static void fslepdc_create_proc_entries(void)
{
    struct einkfb_info info;
    einkfb_get_info(&info);
    
    FB_DEVICE_CREATE_FILE(&info.dev->dev, &dev_attr_override_upd_mode);
    FB_DEVICE_CREATE_FILE(&info.dev->dev, &dev_attr_image_timings);

    if ( einkwf_panel_supports_vcom() )
          FB_DEVICE_CREATE_FILE(&info.dev->dev, &dev_attr_vcom);    
}

static void fslepdc_remove_proc_entries(void)
{
    struct einkfb_info info;
    einkfb_get_info(&info);

    if ( einkwf_panel_supports_vcom() )
        device_remove_file(&info.dev->dev, &dev_attr_vcom);
    
    device_remove_file(&info.dev->dev, &dev_attr_image_timings);
    device_remove_file(&info.dev->dev, &dev_attr_override_upd_mode);
}

static char *fslepdc_waveform_version_io(char *path)
{
    char *result = NULL;
    
    // If we're requesting that the in-use waveform version be returned...
    //
    if ( EINKFB_READ_WFV(path) )
    {
        // ...then do that now.
        //
	einkwf_set_buffer(fslepdc_waveform_proxy);
        result = einkwf_get_waveform_version_string(eink_waveform_version_string);
    }
    else
    {
        // ...otherwise, read in the waveform from the passed-in path.
        //
        einkfb_debug("waveform path = %s\n", path);
    }
    
    return ( result );
}

static char *fslepdc_waveform_file_io(char *path)
{
    char *result = NULL;
    
    // If we're requesting that the last-written waveform file path be returned...
    //
    if ( EINKFB_READ_WFF(path) )
    {
        // ...then do that now.
        //
        result = fslepdc_wf_path;
    }
    else
    {
        char *waveform_filename = einkwf_panel_get_waveform_version_string(eink_waveform_filename);
        int  str_len = strlen(path) + strlen(waveform_filename);
        
        // ...otherwise, write out the waveform file to the passed-in path.
        //
        if ( EINK_WF_PATH_LEN < str_len )
            einkfb_print_warn("waveform file path too long (%d chars)\n", str_len);
        else
        {
            // First, construct the full path name to the waveform file we need
            // for writing it out.
            //
            strcpy(fslepdc_wf_path, path);
            strcat(fslepdc_wf_path, waveform_filename);
            
            // Now, attempt to write it out.
            //
            if ( EINKFB_FAILURE == einkwf_write_waveform_to_file(fslepdc_wf_path) )
                einkfb_print_warn("failed to write out waveform file to %s\n",
                    fslepdc_wf_path);
        }
    }
    
    return ( result );
}

static int fslepdc_temperature_io(int temperature)
{
    int result = EINKFB_TEMP_INVALID;
    
    // If we're requesting a read from the controller...
    //
    if ( EINKFB_READ_TEMP(temperature) )
    {
        // ...ensure that we're actually reading it back
        // from the controller.
        //
        fslepdc_temperature = EINKFB_TEMP_INVALID;
        result = fslepdc_read_temperature();
    }
    else
    {
        // ...otherwise, set the override temperature.
        //
        fslepdc_temperature = temperature;
    }
     
    return ( result );
}

static einkfb_hal_ops_t fslepdc_hal_ops =
{
    .hal_sw_init                 = fslepdc_sw_init,
    .hal_sw_done                 = fslepdc_sw_done,
    
    .hal_create_proc_entries     = fslepdc_create_proc_entries,
    .hal_remove_proc_entries     = fslepdc_remove_proc_entries,

    .hal_update_display          = fslepdc_update_display,
    .hal_update_area             = fslepdc_update_area,

    .hal_set_power_level         = fslepdc_set_power_level,
    .hal_get_power_level         = fslepdc_get_power_level,
    
    .hal_set_display_orientation = fslepdc_set_display_orientation,
    .hal_get_display_orientation = fslepdc_get_display_orientation,

    .hal_waveform_version_io     = fslepdc_waveform_version_io,
    .hal_waveform_file_io        = fslepdc_waveform_file_io,
    .hal_temperature_io          = fslepdc_temperature_io
};

// Boolean interface to the fslepdc_bootstrap module param to make its use
// simpler to read.
//
void fslepdc_set_bootstrapped_state(bool state)
{
    fslepdc_bootstrap = (int)state;
}

bool fslepdc_get_bootstrapped_state(void)
{
    return ( fslepdc_bootstrap ? true : false );
}

static int fslepdc_hal_init(void)
{
    // set up the waveform module parameter
    if (fslepdc_wf_to_use == NULL) {
	fslepdc_wf_to_use = kmalloc(EINK_WF_PATH_LEN, GFP_KERNEL);
	memset(fslepdc_wf_to_use, 0, EINK_WF_PATH_LEN);
    }

    // Preflight what's needed to bring up the FSLEPDC and the display, regardless
    // of whether we're bootstrapped or not.
    //
    // Note:  If some of the hardware doesn't come up, we can keep around the other
    //        hardware as the I/O is independent of each other.  The exit routine
    //        will still make the appropriates call to exit any of the hardware
    //        that did come up.
    //
    // If there's no PMIC available, there will be no video.  So, there's no reason
    // to do anything but go into bootstrapping mode in that case.
    //
    if ( !fslepdc_pmic_init() )
        BOOTSTRAP_FSLEPDC();

    // Verify that the epdc driver is available
    //
    if ( mxc_epdc_power_state(NULL) == MXC_EDPC_POWER_STATE_UNINITED )
        BOOTSTRAP_FSLEPDC();	
    
    // If there's no panel flash, a panel may or may not be available.
    //
    if ( !einkwf_panel_flash_present() )
    {
        // We'll be using the built-in waveform in this case...
        //
        strcpy(fslepdc_wf_to_use, EINK_WF_USE_BUILTIN_WAVEFORM);
        
        // ...so there's no reason to return that.  And, since we have no idea
        // what the panel is, we can't possibly accept a waveform to use.
        //
        fslepdc_hal_ops.hal_waveform_file_io        = NULL;
    }
        
    // If we're in bootstrapping mode (either by choice or circumstance), don't allow any
    // display updates or power management.
    //
    if ( FSLEPDC_BOOTSTRAPPED() )
    {
        fslepdc_hal_ops.hal_update_display          = NULL;
        fslepdc_hal_ops.hal_update_area             = NULL;
        
        fslepdc_hal_ops.hal_set_power_level         = NULL;
        fslepdc_hal_ops.hal_get_power_level         = NULL;
        
        fslepdc_hal_ops.hal_set_display_orientation = NULL;
        fslepdc_hal_ops.hal_get_display_orientation = NULL;
        
        // Also, disable other I/O as appropriate.
        //
        if ( !fslepdc_pmic_present() )
            fslepdc_hal_ops.hal_temperature_io      = NULL;
            
        if ( !einkwf_panel_flash_present() )
        {
            fslepdc_hal_ops.hal_waveform_version_io = NULL;
            fslepdc_hal_ops.hal_waveform_file_io    = NULL;
        }
    }

    return ( einkfb_hal_ops_init(&fslepdc_hal_ops) );
}

module_init(fslepdc_hal_init);

#ifdef MODULE
static void fslepdc_hal_exit(void)
{
    fslepdc_repair_count = 0;
    fslepdc_removed = 1;
    cancel_rearming_delayed_work(&fslepdc_send_update_work);
    cancel_rearming_delayed_work(&fslepdc_repair_work);
    einkfb_hal_ops_done();
}

module_exit(fslepdc_hal_exit);

MODULE_AUTHOR("Lab126");
MODULE_LICENSE("GPL");
#endif // MODULE

