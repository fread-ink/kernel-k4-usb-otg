/*
 * mwan.h  --  WAN hardware control driver
 *
 * Copyright 2005-2010 Amazon Technologies, Inc.  All rights reserved.
 *
 */

#ifndef __MWAN_H__
#define __MWAN_H__

#include <wan_types.h>

typedef enum {
	WAN_INVALID = -1,
	WAN_OFF,			// 0
	WAN_ON,				// 1
	WAN_OFF_KILL			// 2
} wan_status_t;

extern __weak void wan_set_power_status(wan_status_t);
extern __weak wan_status_t wan_get_power_status(void);

extern int wan_get_modem_type(void);

typedef void (*wan_usb_wake_callback_t)(void);

extern void wan_set_usb_wake_callback(wan_usb_wake_callback_t callback);

#endif // __MWAN_H__

