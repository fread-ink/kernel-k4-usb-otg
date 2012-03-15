/*
 *  linux/drivers/video/eink/legacy/einkfb_shim_mario.c -- eInk framebuffer device platform compatibility
 *
 *      Copyright (C) 2005-2010 Amazon Technologies
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */

#include "../hal/einkfb_hal.h"
#include "einkfb_shim.h"

#include "einksp_mario.h"

#if PRAGMAS
	#pragma mark Definitions/Globals
	#pragma mark -
#endif

static einkfb_dma_addr_t kernelbuffer_phys = INIT_EINKFB_DMA_ADDR_T();
static int show_logo = 0;

module_param_named(show_logo, show_logo, int, S_IRUGO);
MODULE_PARM_DESC(show_logo, "non-zero to show logo");

#if PRAGMAS
	#pragma mark -
	#pragma mark Allocation
	#pragma mark -
#endif

unsigned char *einkfb_shim_alloc_kernelbuffer(unsigned long size, struct einkfb_info *info)
{
	unsigned char *result = NULL;
	
	if ( info->dma )
	{
		result = EINKFB_MALLOC_MOD(size, &kernelbuffer_phys);
		
		if ( result )
		einkfb_memset(result, EINKFB_WHITE, size);
	}
	
	return ( result );
}

void einkfb_shim_free_kernelbuffer(unsigned char *buffer, struct einkfb_info *info)
{
	if ( info->dma )
		EINKFB_FREE_MOD(buffer, &kernelbuffer_phys);
}

einkfb_dma_addr_t *einkfb_shim_get_kernelbuffer_phys(void)
{
	return ( &kernelbuffer_phys );
}

#if PRAGMAS
	#pragma mark -
	#pragma mark Power
	#pragma mark -
#endif

static int einkfb_shim_suspend_resume_hook(bool which)
{
	int result = EINKFB_FAILURE;
	
	switch ( which )
	{
		case EINKFB_SUSPEND:
			result = EINKFB_BLANK(FB_BLANK_HSYNC_SUSPEND);
		break;
		
		case EINKFB_RESUME:
			result = EINKFB_BLANK(FB_BLANK_UNBLANK);
		break;
	}
	
	return ( result );
}

static ssize_t store_test_suspend_resume(FB_DSTOR_PARAMS)
{
	int result = -EINVAL, suspend_resume;
	
	if ( sscanf(buf, "%d", &suspend_resume) )
	{
		if ( suspend_resume )
			einkfb_shim_suspend_resume_hook(EINKFB_SUSPEND);
		else
			einkfb_shim_suspend_resume_hook(EINKFB_RESUME);
		
		result = count;
	}

	return ( result );
}

void einkfb_shim_sleep_screen(unsigned int cmd, splash_screen_type which_screen)
{
	// We may already be sleeping, so, to put up this screen during sleep,
	// we need override the current power level.
	//
	// Because we may need to wake up the screen in this case, whatever 
	// is in the framebuffer before we make this call might show up on
	// on the screen.  To prevent that, we clear whatever is in the
	// in all of the buffers.
	//
	// So, we may momentarily wake to a blank screen before the splash
	// screen itself comes up.
	//
	if ( FBIO_EINK_SPLASH_SCREEN_SLEEP == cmd )
	{
		CLEAR_BUFFERS_FOR_WAKE();
		POWER_OVERRIDE_BEGIN();
	}
	
	splash_screen_dispatch(which_screen);
	
	// Ensure that we're back to sleep when requested.
	//
	if ( FBIO_EINK_SPLASH_SCREEN_SLEEP == cmd )
	{	
		einkfb_shim_suspend_resume_hook(EINKFB_SUSPEND);
		POWER_OVERRIDE_END();
	}
}

void einkfb_shim_power_op_complete(void)
{
}

void einkfb_shim_power_off_screen(void)
{
	POWER_OVERRIDE_BEGIN();
	CLEAR_SCREEN();
	
	einkfb_shim_suspend_resume_hook(EINKFB_SUSPEND);
	POWER_OVERRIDE_END();
}

bool einkfb_shim_override_power_lockout(unsigned int cmd, unsigned long flag)
{
	return ( false );
}

bool einkfb_shim_enforce_power_lockout(void)
{
	return ( false );
}

char *einkfb_shim_get_power_string(void)
{
	return ( "not applicable" );
}

#if PRAGMAS
	#pragma mark -
	#pragma mark Splash Screen
	#pragma mark -
#endif

bool einkfb_shim_platform_splash_screen_dispatch(splash_screen_type which_screen, int yres)
{
	system_screen_t system_screen = INIT_SYSTEM_SCREEN_T();
	bool handled = true;
	
	switch ( which_screen )
	{
		case splash_screen_usb_recovery_util:
			system_screen.picture_header_len = PICTURE_USB_RECOVERY_UTIL_HEADER_LEN(yres);
			system_screen.picture_header = PICTURE_USB_RECOVERY_UTIL_HEADER(yres);
			system_screen.header_offset = USB_RECOVERY_UTIL_OFFSET_HEADER(yres);
			system_screen.header_width = USB_RECOVERY_UTIL_WIDTH_HEADER(yres);
									
			system_screen.picture_footer_len = PICTURE_USB_RECOVERY_UTIL_FOOTER_LEN(yres);
			system_screen.picture_footer = PICTURE_USB_RECOVERY_UTIL_FOOTER(yres);
			system_screen.footer_offset = USB_RECOVERY_UTIL_OFFSET_FOOTER(yres);
			system_screen.footer_width = USB_RECOVERY_UTIL_WIDTH_FOOTER(yres);

			system_screen.picture_body_len = PICTURE_USB_RECOVERY_UTIL_BODY_LEN(yres);
			system_screen.picture_body = PICTURE_USB_RECOVERY_UTIL_BODY(yres);
			system_screen.body_offset = USB_RECOVERY_UTIL_OFFSET_BODY(yres);
			system_screen.body_width = USB_RECOVERY_UTIL_WIDTH_BODY(yres);
			
			system_screen.which_screen = which_screen;
			system_screen.to_screen = update_screen;
			system_screen.which_fx = fx_update_full;
			
			display_system_screen(&system_screen);
		break;
				
		case splash_screen_lowbatt:
			system_screen.picture_header_len = PICTURE_LOWBATT_HEADER_LEN(yres);
			system_screen.picture_header = PICTURE_LOWBATT_HEADER(yres);
			system_screen.header_offset = LOWBATT_OFFSET_HEADER(yres);
			system_screen.header_width = LOWBATT_WIDTH_HEADER(yres);
			
			system_screen.picture_footer_len = PICTURE_LOWBATT_FOOTER_LEN(yres);
			system_screen.picture_footer = PICTURE_LOWBATT_FOOTER(yres);
			system_screen.footer_offset = LOWBATT_OFFSET_FOOTER(yres);
			system_screen.footer_width = LOWBATT_WIDTH_FOOTER(yres);
			
			system_screen.picture_body_len = PICTURE_LOWBATT_BODY_LEN(yres);
			system_screen.picture_body = PICTURE_LOWBATT_BODY(yres);
			system_screen.body_offset = LOWBATT_OFFSET_BODY(yres);
			system_screen.body_width = LOWBATT_WIDTH_BODY(yres);
			
			system_screen.which_screen = which_screen;
			system_screen.to_screen = update_screen;
			system_screen.which_fx = fx_update_full;
			
			display_system_screen(&system_screen);
		break;
		
		case splash_screen_repair_needed:
			system_screen.picture_header_len = PICTURE_REPAIR_HEADER_LEN(yres);
			system_screen.picture_header = PICTURE_REPAIR_HEADER(yres);
			system_screen.header_offset = REPAIR_OFFSET_HEADER(yres);
			system_screen.header_width = REPAIR_WIDTH_HEADER(yres);
			
			system_screen.picture_footer_len = PICTURE_REPAIR_FOOTER_LEN(yres);
			system_screen.picture_footer = PICTURE_REPAIR_FOOTER(yres);
			system_screen.footer_offset = REPAIR_OFFSET_FOOTER(yres);
			system_screen.footer_width = REPAIR_WIDTH_FOOTER(yres);
			
			system_screen.picture_body_len = PICTURE_REPAIR_BODY_LEN(yres);
			system_screen.picture_body = PICTURE_REPAIR_BODY(yres);
			system_screen.body_offset = REPAIR_OFFSET_BODY(yres);
			system_screen.body_width = REPAIR_WIDTH_BODY(yres);
			
			system_screen.which_screen = which_screen;
			system_screen.to_screen = update_screen;
			system_screen.which_fx = fx_update_full;
			
			display_system_screen(&system_screen);
		break;
		
		default:
			handled = false;
		break;
	}

	return ( handled );
}

#if PRAGMAS
	#pragma mark -
	#pragma mark Main
	#pragma mark -
#endif

static DEVICE_ATTR(test_suspend_resume, DEVICE_MODE_W, NULL, store_test_suspend_resume);

void einkfb_shim_platform_init(struct einkfb_info *info)
{
	// Hook into the suspend/resume mechanism.
	//
	FB_DEVICE_CREATE_FILE(&info->dev->dev, &dev_attr_test_suspend_resume);
	einkfb_set_suspend_resume_hook(einkfb_shim_suspend_resume_hook);
 
	// Say that we should put up a splash screen on reboot.
	//
	EINKFB_IOCTL(FBIO_EINK_SET_REBOOT_BEHAVIOR, reboot_screen_splash);
	
	// Bring up the logo if we should.
	//
	splash_screen_up = splash_screen_invalid;
	
	if ( show_logo )
		splash_screen_dispatch(splash_screen_boot);
}

void einkfb_shim_platform_done(struct einkfb_info *info)
{
	// Remove the hook into the suspend/resume mechanism.
	//
	device_remove_file(&info->dev->dev, &dev_attr_test_suspend_resume);
	einkfb_set_suspend_resume_hook(NULL);
}
