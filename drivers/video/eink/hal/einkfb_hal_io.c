/*
 *  linux/drivers/video/eink/hal/einkfb_hal_io.c -- eInk frame buffer device HAL I/O
 *
 *      Copyright (c) 2005-2011 Amazon Technologies, Inc.
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include "einkfb_hal.h"

#if PRAGMAS
    #pragma mark Definitions/Globals
    #pragma mark -
#endif

struct load_buffer_t
{
    u8 *src, *dst;
};
typedef struct load_buffer_t load_buffer_t;

typedef u8 (*change_area_data_t)(u8 data, int i);

static display_paused_t einkfb_display_paused = display_paused;
static einkfb_ioctl_hook_t einkfb_ioctl_hook = NULL;
static unsigned long ioctl_time[2] = { 0, 0 };
static update_area_t einkfb_update_area;
static EINKFB_MUTEX(update_area_lock);
static EINKFB_MUTEX(ioctl_lock);

#if PRAGMAS
    #pragma mark -
    #pragma mark Local Utilities
    #pragma mark -
#endif

static void einkfb_load_buffer(int x, int y, int rowbytes, int bytes, void *data)
{
    load_buffer_t *load_buffer = (load_buffer_t *)data;
    load_buffer->dst[bytes] = load_buffer->src[(rowbytes * y) + x];
}

static int einkfb_validate_area_data(update_area_t *update_area)
{
    int xstart = update_area->x1, xend = update_area->x2,
        ystart = update_area->y1, yend = update_area->y2,
        
        xres = xend - xstart,
        yres = yend - ystart,
        
        buffer_size = 0;
    
    struct einkfb_info info;
    einkfb_get_info(&info);
    
    if ( einkfb_bounds_are_acceptable(xstart, xres, ystart, yres) )
        buffer_size = BPP_SIZE((xres * yres), info.bpp);

    // Fix the bounds to the appropriate byte alignment if the passed-in bounds
    // aren't byte aligned and we're just doing an area update from the
    // framebuffer itself.
    //
    if ( (0 == buffer_size) && (NOT_BYTE_ALIGNED_X() || NOT_BYTE_ALIGNED_Y()) && (NULL == update_area->buffer) )
    {
        if ( einkfb_align_bounds((rect_t *)update_area) )
        {
            // Recompute buffer size based on the re-aligned bounds to allow even
            // zero-sized update-areas to come through.  The actual buffer size
            // is immaterial because we do a blit from the newly-computed
            // coordinates.
            //
            xstart = update_area->x1, xend = update_area->x2;
            ystart = update_area->y1, yend = update_area->y2;
            xres   = xend - xstart;
            yres   = yend - ystart;
            
            buffer_size = BPP_SIZE((xres * yres), info.bpp);
        }
    }

    return ( buffer_size );
}

static void einkfb_area_update_lock_entry(void)
{
    einkfb_inc_lock_count();
    einkfb_down(&update_area_lock);
}

static void einkfb_area_update_lock_exit(void)
{
    up(&update_area_lock);
    einkfb_dec_lock_count();
}

static update_area_t *einkfb_set_area_data(unsigned long flag, update_area_t *update_area)
{
    update_area_t *result = NULL;
    
    if ( update_area )
    {
        result = &einkfb_update_area;
        
        // If debugging is enabled, output the passed-in update_area data.
        //
        einkfb_debug("x1       = %d\n",      update_area->x1);
        einkfb_debug("y1       = %d\n",      update_area->y1);
        einkfb_debug("x2       = %d\n",      update_area->x2);
        einkfb_debug("y2       = %d\n",      update_area->y2);
        einkfb_debug("which_fx = %d\n",      update_area->which_fx);
        einkfb_debug("buffer   = 0x%08lX\n", (unsigned long)update_area->buffer);
        
        if ( result )
        {
            int success = einkfb_memcpy(EINKFB_IOCTL_FROM_USER, flag, result, update_area, sizeof(update_area_t));
            unsigned char *buffer = NULL;

            if ( EINKFB_SUCCESS == success )
            {
                int buffer_size = einkfb_validate_area_data(result);
                
                success = EINKFB_FAILURE;
                result->buffer = NULL;

                if ( buffer_size )
                {
                    struct einkfb_info info;
                    einkfb_get_info(&info);
                    
                    buffer = info.buf;
    
                    if ( update_area->buffer )
                        success = einkfb_memcpy(EINKFB_IOCTL_FROM_USER, flag, buffer, update_area->buffer, buffer_size);
                    else
                    {
                        load_buffer_t load_buffer;

                        load_buffer.src = info.start;
                        load_buffer.dst = buffer;
                        
                        einkfb_blit(result->x1, result->x2, result->y1, result->y2, einkfb_load_buffer, (void *)&load_buffer);
                        success = EINKFB_SUCCESS;
                    }
                }
            }

            if ( EINKFB_SUCCESS == success )
                result->buffer = buffer;
            else
                result = NULL;
        }
     }
    
    return ( result );
}

static void einkfb_change_area_data(update_area_t *update_area, change_area_data_t change_area_data)
{
    int xstart = update_area->x1, xend = update_area->x2,
        ystart = update_area->y1, yend = update_area->y2,
        
        xres = xend - xstart,
        yres = yend - ystart,
        
        buffer_size = 0,
        i;
        
    u8  *buffer = update_area->buffer;
        
    struct einkfb_info info;
    einkfb_get_info(&info);
    
    buffer_size = BPP_SIZE((xres * yres), info.bpp);
    
    for ( i = 0; i < buffer_size; i++ )
    {
        buffer[i] = change_area_data(buffer[i], i);
        EINKFB_SCHEDULE_BLIT(i);
    }
}

static u8 invert_area_data(u8 data, int i)
{
    return ( ~data );
}

static void einkfb_invert_area_data(update_area_t *update_area)
{
    einkfb_change_area_data(update_area, invert_area_data);
}

static void einkfb_posterize_area_data(update_area_t *update_area)
{
    einkfb_posterize_to_1bpp_begin();
    
    einkfb_change_area_data(update_area, einkfb_posterize_to_1bpp);
    
    einkfb_posterize_to_1bpp_end();
}

static void einkfb_posterize_invert_area_data(update_area_t *update_area)
{
    einkfb_posterize_to_1bpp_begin();
    
    einkfb_change_area_data(update_area, einkfb_posterize_to_1bpp);
     
    einkfb_posterize_to_1bpp_end();
    
    einkfb_change_area_data(update_area, invert_area_data);
}

static void einkfb_contrast_area_data(update_area_t *update_area)
{
    einkfb_contrast_begin();
    
    einkfb_change_area_data(update_area, einkfb_apply_contrast);
    
    einkfb_contrast_end();
}

static void einkfb_pause_display(display_paused_t which)
{
    if ( einkfb_display_paused != which )
        einkfb_display_paused = which;
}

#if PRAGMAS
    #pragma mark -
    #pragma mark External Interfaces
    #pragma mark -
#endif

static char unknown_cmd_string[32];

char *einkfb_get_cmd_string(unsigned int cmd)
{
    char *cmd_string = NULL;
    
    switch ( cmd )
    {
        // Supported by HAL.
        //
        case FBIO_EINK_UPDATE_DISPLAY:
            cmd_string = "update_display";
        break;
        
        case FBIO_EINK_UPDATE_DISPLAY_AREA:
            cmd_string = "update_display_area";
        break;
        
        case FBIO_EINK_RESTORE_DISPLAY:
            cmd_string = "restore_display";
        break;
        
        case FBIO_EINK_SET_REBOOT_BEHAVIOR:
            cmd_string = "set_reboot_behavior";
        break;

        case FBIO_EINK_GET_REBOOT_BEHAVIOR:
            cmd_string = "get_reboot_behavior";
        break;

        case FBIO_EINK_SET_DISPLAY_ORIENTATION:
            cmd_string = "set_display_orientation";
        break;
        
        case FBIO_EINK_GET_DISPLAY_ORIENTATION:
            cmd_string = "get_display_orientation";
        break;

        case FBIO_EINK_SET_SLEEP_BEHAVIOR:
            cmd_string = "set_sleep_behavior";
        break;
        
        case FBIO_EINK_GET_SLEEP_BEHAVIOR:
            cmd_string = "get_sleep_behavior";
        break;

        case FBIO_EINK_SET_CONTRAST:
            cmd_string = "set_contrast";
        break;

        case FBIO_EINK_SET_PAUSE_RESUME:
            cmd_string = "set_pause_resume";
        break;

        case FBIO_EINK_GET_PAUSE_RESUME:
            cmd_string = "get_pause_resume";
        break;

        case FBIO_WAITFORVSYNC:
            cmd_string = "waitforvsync";
        break;

        // Supported by Shim.
        //
        case FBIO_EINK_UPDATE_DISPLAY_FX:
            cmd_string = "update_dislay_fx";
        break;
        
        case FBIO_EINK_SPLASH_SCREEN:
            cmd_string = "splash_screen";
        break;
        
        case FBIO_EINK_SPLASH_SCREEN_SLEEP:
            cmd_string = "splash_screen_sleep";
        break;
        
        case FBIO_EINK_OFF_CLEAR_SCREEN:
            cmd_string = "off_clear_screen";
        break;
        
        case FBIO_EINK_CLEAR_SCREEN:
            cmd_string = "clear_screen";
        break;
        
        case FBIO_EINK_POWER_OVERRIDE:
            cmd_string = "power_override";
        break;
        
        case FBIO_EINK_PROGRESSBAR:
            cmd_string = "progressbar";
        break;
        
        case FBIO_EINK_PROGRESSBAR_SET_XY:
            cmd_string = "progressbar_set_xy";
        break;
        
        case FBIO_EINK_PROGRESSBAR_BADGE:
            cmd_string = "progressbar_badge";
        break;
        
        case FBIO_EINK_PROGRESSBAR_BACKGROUND:
            cmd_string = "progressbar_background";
        break;
        
        // Unknown and/or unsupported.
        //
        default:
            sprintf(unknown_cmd_string, "unknown cmd = 0x%08X", cmd);
            cmd_string = unknown_cmd_string;
        break;
    }
    
    return ( cmd_string );
}

unsigned long einkfb_get_ioctl_time1(void)
{
    return ( ioctl_time[0] );
}

unsigned long einkfb_get_ioctl_time2(void)
{
    return ( ioctl_time[1] );
}

#define IOCTL_FLAG(f, l)                    \
    if ( EINKFB_IOCTL_PROC == f )           \
        l = EINKFB_IOCTL_KERN
#define IOCTL_LOCK_ENTRY(f)                 \
    if ( EINKFB_IOCTL_KERN != f )           \
    {                                       \
        einkfb_inc_lock_count();            \
        einkfb_down(&ioctl_lock);           \
    }
#define IOCTL_LOCK_EXIT(f)                  \
    if ( EINKFB_IOCTL_KERN != f )           \
    {                                       \
        up(&ioctl_lock);                    \
        einkfb_dec_lock_count();            \
    }

#define IOCTL_TIMING "ioctl_timing"

int einkfb_fsync(struct file *file, struct dentry *dentry, int datasync)
{
    return ( EINKFB_IOCTL(FBIO_WAITFORVSYNC, 0) );
}

display_paused_t einkfb_get_display_paused(void)
{
    return ( einkfb_display_paused );
}

int einkfb_ioctl_dispatch(unsigned long flag, struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    bool done = !EINKFB_IOCTL_DONE, bad_arg = false;
    unsigned long start_time = jiffies;
    orientation_t old_orientation;
    int result = EINKFB_SUCCESS;
    struct einkfb_info hal_info;

    unsigned long local_flag = flag;
    unsigned int  local_cmd  = cmd;
    unsigned long local_arg  = arg;
    
    einkfb_debug_ioctl("%s(0x%08lX)\n", einkfb_get_cmd_string(cmd), arg);
    EINKFB_PRINT_PERF_REL(IOCTL_TIMING, 0UL, einkfb_get_cmd_string(cmd));

    IOCTL_FLAG(flag, local_flag);
    IOCTL_LOCK_ENTRY(flag);
    
    einkfb_get_info(&hal_info);

    // If there's a hook, give it the pre-command call.
    //
    if ( einkfb_ioctl_hook )
        done = (*einkfb_ioctl_hook)(einkfb_ioctl_hook_pre, local_flag, &local_cmd, &local_arg);
    
    // Process the command if it hasn't already been handled.
    //
    if ( !done )
    {
        update_area_t *update_area = NULL;
        
        switch ( local_cmd )
        {
            case FBIO_EINK_UPDATE_DISPLAY:
                einkfb_update_display(local_arg);
            break;
            
            case FBIO_EINK_UPDATE_DISPLAY_AREA:
            {
                bool failure = false;
                
                einkfb_area_update_lock_entry();
                update_area = einkfb_set_area_data(local_flag, (update_area_t *)local_arg);

                if ( update_area )
                {
                    fx_type saved_fx = update_area->which_fx;
                    bool flash_area = false;

                    // If there's a hook, give it the beginning in-situ call.
                    //
                    if ( einkfb_ioctl_hook )
                        (*einkfb_ioctl_hook)(einkfb_ioctl_hook_insitu_begin, local_flag, &local_cmd,
                            (unsigned long *)&update_area);

                    // Update the display with the area data, preserving and
                    // normalizing which_fx as we go.
                    //
                    switch ( update_area->which_fx )
                    {
                        case fx_flash:
                            saved_fx = fx_update_full;
                            flash_area = true;

                        case fx_invert:
                            einkfb_invert_area_data(update_area);
                            saved_fx = fx_update_partial;
                        break;

                        case fx_update_white_trans:
                            // Don't expect white transition update here
                            einkfb_print_warn("got update_type=fx_update_white_trans in ioctl dispatch for FBIO_EINK_UPDATE_DISPLAY_AREA\n");
                            saved_fx = fx_update_white_trans;
                            flash_area = true;
                        break;

                        case fx_update_fast:
                            einkfb_posterize_area_data(update_area);
                        break;

                        case fx_update_fast_invert:
                            saved_fx = fx_update_fast;
                            einkfb_posterize_invert_area_data(update_area);
                        break;

                        default:
                            if ( hal_info.contrast )
                                einkfb_contrast_area_data(update_area);
                        break;
                    }

                    update_area->which_fx = saved_fx;
                    einkfb_update_display_area(update_area);

                    if ( flash_area )
                    {
                        // Make sure that the first update has completed before starting
                        // the second one since we're trying to make the display blink.
                        //
                        einkfb_update_display_sync();
                        
                        // Now, do the invert and then update the display again.
                        //
                        update_area->which_fx = fx_update_partial;
                        einkfb_invert_area_data(update_area);

                        einkfb_update_display_area(update_area);
                    }

                    update_area->which_fx = saved_fx;

                    // If there's a hook, give it the ending in-situ call.
                    //
                    if ( einkfb_ioctl_hook )
                        (*einkfb_ioctl_hook)(einkfb_ioctl_hook_insitu_end, local_flag, &local_cmd,
                            (unsigned long *)&update_area);
                }
                else
                    failure = true;
                
                einkfb_area_update_lock_exit();
                
                if ( failure )
                    goto failure;
            }
            break;
            
            case FBIO_EINK_RESTORE_DISPLAY:
            {
                fx_type update_mode = UPDATE_MODE(local_arg);
                
                // On restores, force a slow (highest fidelity) update.
                //
                if ( fx_update_full == update_mode )
                    update_mode = fx_update_slow;

                einkfb_restore_display(update_mode);
            }
            break;
            
            case FBIO_EINK_SET_REBOOT_BEHAVIOR:
                einkfb_set_reboot_behavior((reboot_behavior_t)local_arg);
            break;
            
            case FBIO_EINK_GET_REBOOT_BEHAVIOR:
                if ( local_arg )
                {
                    reboot_behavior_t reboot_behavior = einkfb_get_reboot_behavior();

                    einkfb_memcpy(EINKFB_IOCTL_TO_USER, local_flag, (reboot_behavior_t *)local_arg,
                        &reboot_behavior, sizeof(reboot_behavior_t));
                }
                else
                    goto failure;
            break;
            
            case FBIO_EINK_SET_DISPLAY_ORIENTATION:
                old_orientation = einkfb_get_display_orientation();
                
                if ( einkfb_set_display_orientation((orientation_t)local_arg) )
                {
                    orientation_t new_orientation = einkfb_get_display_orientation();
                    
                    if ( !ORIENTATION_SAME(old_orientation, new_orientation) )
                    {
                        if ( !info )
                            info = hal_info.fbinfo;

                        if ( info )
                            einkfb_set_res_info(info, info->var.yres, info->var.xres);
                    }
                }
            break;
            
            case FBIO_EINK_GET_DISPLAY_ORIENTATION:
                if ( local_arg )
                {
                    old_orientation = einkfb_get_display_orientation();
                    
                    einkfb_memcpy(EINKFB_IOCTL_TO_USER, local_flag, (orientation_t *)local_arg,
                        &old_orientation, sizeof(orientation_t));
                }
            break;
            
            case FBIO_EINK_SET_SLEEP_BEHAVIOR:
                einkfb_set_sleep_behavior((sleep_behavior_t)local_arg);
            break;
            
            case FBIO_EINK_GET_SLEEP_BEHAVIOR:
                if ( local_arg )
                {
                    sleep_behavior_t sleep_behavior = einkfb_get_sleep_behavior();

                    einkfb_memcpy(EINKFB_IOCTL_TO_USER, local_flag, (sleep_behavior_t *)local_arg,
                        &sleep_behavior, sizeof(sleep_behavior_t));
                }
                else
                    goto failure;
            break;
            
            case FBIO_EINK_SET_CONTRAST:
                einkfb_set_contrast((contrast_t)local_arg);
            break;

            case FBIO_EINK_SET_PAUSE_RESUME:
                einkfb_pause_display((display_paused_t)local_arg);
            break;
            
            case FBIO_EINK_GET_PAUSE_RESUME:
                if ( local_arg )
                {
                    display_paused_t display_paused = einkfb_get_display_paused();

                    einkfb_memcpy(EINKFB_IOCTL_TO_USER, local_flag, (display_paused_t *)local_arg,
                        &display_paused, sizeof(display_paused_t));
                }
                else
                    goto failure;
            break;

            case FBIO_WAITFORVSYNC:
                einkfb_update_display_sync();
            break;

            failure:
                bad_arg = true;
            default:
                result = EINKFB_IOCTL_FAILURE;
            break;
        }
    }

    // If there's a hook and we haven't determined that we've received a bad argument, give it
    // the post-command call.  Use the originally passed-in cmd & arg here instead of local copies
    // in case they were changed in the pre-command processing.
    //
    if ( !bad_arg && einkfb_ioctl_hook )
    {
        done = (*einkfb_ioctl_hook)(einkfb_ioctl_hook_post, local_flag, &cmd, &arg);
        
        // If the hook processed the command, don't pass along the HAL's failure to do so.
        //
        if ( done && (EINKFB_IOCTL_FAILURE == result) )
            result = EINKFB_SUCCESS;
    }
    
    // It's useful to keep track of the last two ioctl times because area-update "animation"
    // often involves a draw update and an erase update coupled together.
    //
    ioctl_time[1] = ioctl_time[0];
    ioctl_time[0] = (unsigned long)jiffies_to_msecs(jiffies - start_time);
    einkfb_debug_ioctl("result = %d\n", result);
    
    EINKFB_PRINT_PERF_ABS(IOCTL_TIMING, ioctl_time[0], einkfb_get_cmd_string(cmd));
    IOCTL_LOCK_EXIT(flag);    
    
    return ( result );
}

int einkfb_ioctl(FB_IOCTL_PARAMS)
{
    return ( einkfb_ioctl_dispatch(EINKFB_IOCTL_USER, info, cmd, arg) );
}

void einkfb_set_ioctl_hook(einkfb_ioctl_hook_t ioctl_hook)
{
    // Need to make a queue of these if this becomes truly useful.
    //
    if ( ioctl_hook )
        einkfb_ioctl_hook = ioctl_hook;
    else
        einkfb_ioctl_hook = NULL;
}

EXPORT_SYMBOL(einkfb_get_cmd_string);
EXPORT_SYMBOL(einkfb_ioctl_dispatch);
EXPORT_SYMBOL(einkfb_ioctl);
EXPORT_SYMBOL(einkfb_set_ioctl_hook);
