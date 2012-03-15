/*
 *  linux/drivers/video/eink/legacy/einkfb_shim.h -- eInk framebuffer device compatibility utility
 *
 *      Copyright (C) 2005-2010 Amazon Technologies, Inc.
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 *
 */
 
#ifndef _EINKFB_SHIM_H
#define _EINKFB_SHIM_H

#define FRAMEWORK_OR_DIAGS_RUNNING()			(FRAMEWORK_RUNNING() || running_diags)

#define CLEAR_BUFFERS_FORCE_BUFFERS_NOT_EQUAL           true
#define CLEAR_BUFFERS_SPLASH_SCREEN_SLEEP               false

#define CLEAR_BUFFERS()                                 clear_buffers(CLEAR_BUFFERS_FORCE_BUFFERS_NOT_EQUAL)
#define CLEAR_BUFFERS_FOR_WAKE()                        clear_buffers(CLEAR_BUFFERS_SPLASH_SCREEN_SLEEP)

#define CLEAR_SCREEN_ALL_BUFFERS                        true
#define CLEAR_SCREEN_KERNEL_BUFFER_ONLY                 false

#define CLEAR_SCREEN()                                  clear_screen(CLEAR_SCREEN_ALL_BUFFERS)
#define CLEAR_SCREEN_KERNEL_BUFFER()                    clear_screen(CLEAR_SCREEN_KERNEL_BUFFER_ONLY)

enum update_type
{
	update_overlay = 0,	// Hardware composite on top of existing framebuffer.
	update_overlay_mask,	// Hardware composite on top of existing framebuffer using mask.
	update_partial,		// Software composite *into* existing framebuffer.
	update_full,		// Fully replace existing framebuffer.
	update_full_refresh,	// Fully replace existing framebuffer with manual refresh.
	update_none		// Buffer update only.
};
typedef enum update_type update_type;

enum update_which
{
	update_buffer = 0,	// Just update the buffer, not the screen.
	update_screen		// Update both the buffer and the screen.
};
typedef enum update_which update_which;

struct picture_info_type
{
	int		x, y;
	update_type	update;
	update_which	to_screen;
	int		headerless,
			xres, yres,
			bpp;
};
typedef struct picture_info_type picture_info_type;

struct system_screen_t
{
	u8			*picture_header,
				*picture_footer,
				*picture_body;
		
	int			picture_header_len,
				picture_footer_len,
				picture_body_len,
		
				header_width,
				footer_width,
				body_width,
				
				header_offset,
				footer_offset,
				body_offset;
		
	splash_screen_type	which_screen;
	update_which		to_screen;
	fx_type			which_fx;
};
typedef struct system_screen_t system_screen_t;

#define INIT_SYSTEM_SCREEN_T() { NULL, NULL, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }

#define splash_screen_activity_begin			1
#define splash_screen_activity_end			0

#define begin_splash_screen_activity(s)			splash_screen_activity(splash_screen_activity_begin, s)
#define end_splash_screen_activity()			splash_screen_activity(splash_screen_activity_end, splash_screen_up)

extern splash_screen_type splash_screen_up;
extern bool power_level_initiated_call;
extern int running_diags;

// From einkfb_shim.c
//
extern void clear_buffers(bool force_buffers_not_equal);
extern void clear_screen(bool clear_framebuffer);

extern void splash_screen_activity(int which_activity, splash_screen_type which_screen);
extern void splash_screen_dispatch(splash_screen_type which_screen);

extern void display_system_screen(system_screen_t *system_screen);

// From einkfb_shim_<plaform>.c
//
extern unsigned char *einkfb_shim_alloc_kernelbuffer(unsigned long size, struct einkfb_info *info);
extern void einkfb_shim_free_kernelbuffer(unsigned char *buffer, struct einkfb_info *info);
extern einkfb_dma_addr_t *einkfb_shim_get_kernelbuffer_phys(void);

extern void einkfb_shim_sleep_screen(unsigned int cmd, splash_screen_type which_screen);
extern void einkfb_shim_power_op_complete(void);
extern void einkfb_shim_power_off_screen(void);

extern bool einkfb_shim_override_power_lockout(unsigned int cmd, unsigned long flag);
extern bool einkfb_shim_enforce_power_lockout(void);
extern char *einkfb_shim_get_power_string(void);

extern bool einkfb_shim_platform_splash_screen_dispatch(splash_screen_type which_screen, int yres);

extern void einkfb_shim_platform_init(struct einkfb_info *info);
extern void einkfb_shim_platform_done(struct einkfb_info *info);

#endif // _EINKFB_SHIM_H


