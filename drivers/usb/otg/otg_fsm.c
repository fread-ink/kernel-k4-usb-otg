/*
 * Copyright 2006-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2010-2012 Christian Hoff
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <asm/types.h>
#include <linux/kernel.h>
#include <linux/usb/otg.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/usb.h>
// #include <linux/usb_gadget.h>

#include "otg_fsm.h"

const char *state_string(enum usb_otg_state state)
{
	switch (state) {
	case OTG_STATE_IDLE:
		return "idle";
	case OTG_STATE_HOST:
		return "host";
	case OTG_STATE_GADGET:
		return "gadget";
	default:
		return "UNDEFINED";
	}
}

/* Called when leaving a state.  Do state clean up jobs here */
int otg_leave_state(struct otg_fsm *fsm, enum usb_otg_state old_state)
{
	switch (old_state) {
	case OTG_STATE_IDLE:
		return fsm->ops->enable_port(fsm, 1);
	case OTG_STATE_GADGET:
		return fsm->ops->start_gadget(fsm, 0);
	case OTG_STATE_HOST:
		return fsm->ops->start_host(fsm, 0);
	default:
		return -EINVAL;
	}
}

/* Called when entering a state */
int otg_enter_state(struct otg_fsm *fsm, enum usb_otg_state new_state)
{
	int retval;

	enum usb_otg_state old_state = fsm->transceiver->state;
	fsm->transceiver->state = new_state;

	switch (new_state) {
	case OTG_STATE_IDLE:
		retval = fsm->ops->enable_port(fsm, 0);
		break;
	case OTG_STATE_GADGET:
		retval = fsm->ops->start_gadget(fsm, 1);
		break;
	case OTG_STATE_HOST:
		retval = fsm->ops->start_host(fsm, 1);
		break;
	default:
		printk(KERN_ERR "Unknown state: %d", new_state);
		retval = -EINVAL;
		break;
	}

	if (unlikely(retval != 0))
		fsm->transceiver->state = old_state;

	return retval;
}

static int otg_try_enter_state(struct otg_fsm *fsm, enum usb_otg_state new_state) {
	int retval, fallback_retval;

	if ((retval = otg_enter_state(fsm, new_state))) {
		/* Failure during state switch. Restore old state */
		if ((fallback_retval = otg_enter_state(fsm, fsm->transceiver->state))) {

			printk(KERN_ERR "%s: ERROR: Could not restore state %s. The FSM is now corrupted."
					" New state=%s, retval=%d, orig retval=%d\n", __func__,
					state_string(fsm->transceiver->state), state_string(new_state),
					fallback_retval, retval);

			return fallback_retval;
		}
	}

	return 0;
}

/* Called when new state is being set */
int otg_set_state(struct otg_fsm *fsm, enum usb_otg_state new_state)
{
	int retval;

	if (fsm->transceiver->state == new_state)
		return 0;

	VDBG("chg state to %s", state_string(new_state));

	if ((retval = otg_leave_state(fsm, fsm->transceiver->state))) {
		printk(KERN_ERR "%s: ERROR: Could not leave state %s. The FSM is now corrupted."
						" retval=%d", __func__, state_string(fsm->transceiver->state), retval);

		return retval;
	}

	if ((retval = otg_enter_state(fsm, new_state))) {
		printk(KERN_ERR "%s: ERROR: Could not enter state %s. The FSM is now corrupted."
				" retval=%d", __func__, state_string(new_state), retval);

		return retval;
	}

	return 0;
}

/* State change judgement */
int otg_statemachine(struct otg_fsm *fsm)
{
	enum usb_otg_state state;
	int retval = 0;

	mutex_lock(&fsm->state_mutex);

	state = fsm->transceiver->state;
	/* State machine state change judgement */

	VDBG("top: curr state=%s, vbus_vld=%d, id=%d, gadget=%d, host=%d",
			state_string(state), fsm->vbus_vld, fsm->id,
			fsm->gadget_available, fsm->host_available);

	switch (state) {
	case OTG_STATE_IDLE:
		/* Check if a voltage has been applied on VBUS and power on the port only
		 * if there are approx. 5V available on VBUS 
		 */
		if (fsm->vbus_vld && (fsm->gadget_available || fsm->host_available)) {
			retval = otg_leave_state(fsm, OTG_STATE_IDLE);

			if (retval) {
				printk(KERN_ERR "%s: Could not leave idle state.\n", __func__);
				return retval;
			}

			DBG("re-awakening port");

			/*
			 * For now, we don't support HNP and SRP. This means that whether 
			 * we are running in host/gadget mode is solely determined 
			 * by the state of the ID pin.
			 * ID pin = 1 -> B device -> gadget mode
			 * ID pin = 0 -> A device -> host mode
			 */
			if (fsm->id) {
				retval = otg_try_enter_state(fsm, OTG_STATE_GADGET);
			} else {
				retval = otg_try_enter_state(fsm, OTG_STATE_HOST);
			}
		}
		break;
	case OTG_STATE_HOST:
		if (!fsm->host_available || !fsm->vbus_vld) {
			retval = otg_set_state(fsm, OTG_STATE_IDLE);
		} else if (fsm->id) {
			retval = otg_set_state(fsm, OTG_STATE_GADGET);
		}

		break;
	case OTG_STATE_GADGET:
		if (!fsm->gadget_available || !fsm->vbus_vld) {
			retval = otg_set_state(fsm, OTG_STATE_IDLE);
		} else if (!fsm->id) {
			retval = otg_set_state(fsm, OTG_STATE_HOST);
		}

		break;
	default:
		break;
	}

	mutex_unlock(&fsm->state_mutex);

	return retval;
}
