/*
 * Copyright 2006-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/mutex.h>

//#if 0
#define DEBUG 	1
#define VERBOSE	1
//#endif

#ifdef DEBUG
#define DBG(fmt, args...) printk("[%s]  " fmt "\n", \
		__FUNCTION__, ## args)

#else
#define DBG(fmt, args...)	do{}while(0)
#endif

#ifdef VERBOSE
#define VDBG		DBG
#else
#define VDBG(stuff...)	do{}while(0)
#endif

#ifdef VERBOSE
#define MPC_LOC printk("Current Location [%s]:[%d]\n", __FILE__, __LINE__)
#else
#define MPC_LOC do{}while(0)
#endif

struct otg_fsm {
	/* Input */
	int vbus_vld;
	int id;
	int gadget_available;
	int host_available;

	struct otg_fsm_ops *ops;
	struct otg_transceiver *transceiver;

	struct mutex state_mutex;
};

struct otg_fsm_ops {
	int (*enable_port) (struct otg_fsm *fsm, int on);
	int (*start_host) (struct otg_fsm * fsm, int on);
	int (*start_gadget) (struct otg_fsm * fsm, int on);
};

int otg_statemachine(struct otg_fsm *fsm);
int otg_set_state(struct otg_fsm *fsm, enum usb_otg_state new_state);

