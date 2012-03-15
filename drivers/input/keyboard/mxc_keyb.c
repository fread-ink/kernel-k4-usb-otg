/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2010-2011 Amazon Technologies, Inc. All Rights Reserved.
 * Manish Lachwani (lachwani@lab126.com).
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mxc_keyb.c
 *
 * @brief Driver for the Freescale Semiconductor MXC keypad port.
 *
 * The keypad driver is designed as a standard Input driver which interacts
 * with low level keypad port hardware. Upon opening, the Keypad driver
 * initializes the keypad port. When the keypad interrupt happens the driver
 * calles keypad polling timer and scans the keypad matrix for key
 * press/release. If all key press/release happened it comes out of timer and
 * waits for key press interrupt. The scancode for key press and release events
 * are passed to Input subsytem.
 *
 * @ingroup keypad
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/kd.h>
#include <linux/fs.h>
#include <linux/kbd_kern.h>
#include <linux/ioctl.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <mach/boardid.h>
#include <mach/hardware.h>
#include <asm/mach/keypad.h>

// Logging
static unsigned int logging_mask;
#define LLOG_G_LOG_MASK logging_mask
#define LLOG_KERNEL_COMP "mxc_keyb"
#include <llog.h>

#include <mach/boardid.h>


/*
 * Module header file
 */
#include "mxc_keyb.h"

/*!
 * This structure holds the keypad private data structure.
 */
static struct keypad_priv kpp_dev;

/*! Input device structure. */
static struct input_dev *mxckbd_dev = NULL;

/*! Proc entry structure and global variable. */
#define KEYPAD_PROC_FILE "keypad"
#define PROC_CMD_LEN 50
static struct proc_dir_entry *proc_entry;
static bool keypad_lock = false;  // Is the keypad locked or not

/*! KPP clock handle. */
static struct clk *kpp_clk;

/*! This static variable indicates whether a key event is pressed/released. */
static unsigned short KPress;

/*! This static variable is used for the ALT key state machine. */
enum AltState ALT_state = AltStateStart;

/*! ALT stickiness timeout in milliseconds. */
static atomic_t alt_sticky_timeout = ATOMIC_INIT(2000);

static unsigned long sticky_alt_jiffies;

/*! cur_rcmap and prev_rcmap array is used to detect key press and release. */
static unsigned short *cur_rcmap;	/* max 64 bits (8x8 matrix) */
static unsigned short *prev_rcmap;

/*!
 * Debounce polling period(10ms) in system ticks.
 */
static unsigned short KScanRate = (10 * HZ) / 1000;

static struct keypad_data *keypad;

static void mxc_kp_tq(struct work_struct *);

static ssize_t
show_alt_timer(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t
store_alt_timer(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static DEVICE_ATTR(alt_timer, 0644, show_alt_timer, store_alt_timer);

/*!
 * These arrays are used to store press and release scancodes.
 */
static short **press_scancode;
static short **release_scancode;

static const unsigned short *mxckpd_keycodes;
static unsigned short mxckpd_keycodes_size;

/*!
 * Keycode press counters
 */
#define MAX_CODES 256
static int keycode_count[MAX_CODES];


#define press_left_code     30
#define press_right_code    29
#define press_up_code       28
#define press_down_code     27

#define rel_left_code       158
#define rel_right_code      157
#define rel_up_code         156
#define rel_down_code       155

// Module parameters

#define REPEAT_DELAY_DEFAULT 1000
static int repeat_delay = REPEAT_DELAY_DEFAULT;
module_param(repeat_delay, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(repeat_delay, "Delay between key press and start of repeat functionality, in msec (default is 1000)");

#define REPEAT_PERIOD_DEFAULT 200
static int repeat_period = REPEAT_PERIOD_DEFAULT;
module_param(repeat_period, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(repeat_period, "Delay between repeat-key events, in msec (default is 200)");

static int debug = 0;
module_param(debug, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging; 0=off, any other value=on (default is off)");


/*
 * Set the log mask depending on the debug level
 */
static void set_debug_log_mask(int debug_level)
{
   logging_mask = LLOG_LEVEL_MASK_INFO | LLOG_LEVEL_MASK_EXCEPTIONS;
   switch (debug_level) {
      case 10: logging_mask |= LLOG_LEVEL_DEBUG9;
      case  9: logging_mask |= LLOG_LEVEL_DEBUG8;
      case  8: logging_mask |= LLOG_LEVEL_DEBUG7;
      case  7: logging_mask |= LLOG_LEVEL_DEBUG6;
      case  6: logging_mask |= LLOG_LEVEL_DEBUG5;
      case  5: logging_mask |= LLOG_LEVEL_DEBUG4;
      case  4: logging_mask |= LLOG_LEVEL_DEBUG3;
      case  3: logging_mask |= LLOG_LEVEL_DEBUG2;
      case  2: logging_mask |= LLOG_LEVEL_DEBUG1;
      case  1: logging_mask |= LLOG_LEVEL_DEBUG0;

      case  0:
      default: ; // No logging if debug_level <= 0 or > 10
   }
}


/*!
 * These functions are used to configure and the GPIO pins for keypad to
 * activate and deactivate it.
 */
extern int gpio_keypad_active(void);
extern int gpio_keypad_inactive(void);


/**
* This is a wrapper for input_event.  In addition to calling
* input_event, this function also sends a uevent for any
* processes listening in the userspace
*/
void report_event(struct input_dev *dev, unsigned int type, unsigned int code, int value )
{
        // Send key input event
	input_event( dev, type, code, value );

	// Send uevent so userspace power daemon can monitor
	// keyboard activity.  NOTE - the keycode is not sent through
	// this medium; only the fact that a key event has happened.
	// NOTE: only sending uevent on key down.
	if (value == 1) {
	   if( kobject_uevent_atomic( &dev->dev.kobj, KOBJ_CHANGE ) )
	   {
		LLOG_ERROR("uevent", "", "Keyboard failed to send uevent\n" );
	   }
	   // Increment keycode counter
	   if ((code >= 0) && (code < MAX_CODES)) {
	      keycode_count[code]++;
	   }
	}
}

/*
 * Send the repeat key events
 */

static void input_repeat_key(unsigned long data)
{
        struct input_dev *dev = (void *) data;

        if (!test_bit(dev->repeat_key, dev->key))
                return;

        report_event(dev, EV_KEY, dev->repeat_key, 2);

        // Make sure we have the most recent repeat period - this way
        // it could be changed dynamically during a repeat sequence
	mxckbd_dev->rep[REP_PERIOD] = repeat_period;
        if (dev->rep[REP_PERIOD]) {
           mod_timer(&dev->timer, jiffies + msecs_to_jiffies(dev->rep[REP_PERIOD]));
	}
}


/*!
 * For now, do mapping for top-row keys which are numeric when ALT is pressed.
 */
unsigned int alt_convert(unsigned int keycode) {
	switch (keycode) {
	case KEY_Q:
	   return KEY_1;
	   break;
	case KEY_W:
	   return KEY_2;
	   break;
	case KEY_E:
	   return KEY_3;
	   break;
	case KEY_R:
	   return KEY_4;
	   break;
	case KEY_T:
	   return KEY_5;
	   break;
	case KEY_Y:
	   return KEY_6;
	   break;
	case KEY_U:
	   return KEY_7;
	   break;
	case KEY_I:
	   return KEY_8;
	   break;
	case KEY_O:
	   return KEY_9;
	   break;
	case KEY_P:
	   return KEY_0;
	   break;
	default:
	   return keycode;
	   break;
	}
}


/*!
 * This function is called to scan the keypad matrix to find out the key press
 * and key release events. Make scancode and break scancode are generated for
 * key press and key release events.
 *
 * The following scanning sequence are done for
 * keypad row and column scanning,
 * -# Write 1's to KPDR[15:8], setting column data to 1's
 * -# Configure columns as totem pole outputs(for quick discharging of keypad
 * capacitance)
 * -# Configure columns as open-drain
 * -# Write a single column to 0, others to 1.
 * -# Sample row inputs and save data. Multiple key presses can be detected on
 * a single column.
 * -# Repeat steps the above steps for remaining columns.
 * -# Return all columns to 0 in preparation for standby mode.
 * -# Clear KPKD and KPKR status bit(s) by writing to a 1,
 *    Set the KPKR synchronizer chain by writing "1" to KRSS register,
 *    Clear the KPKD synchronizer chain by writing "1" to KDSC register
 *
 * @result    Number of key pressed/released.
 */
static int mxc_kpp_scan_matrix(void)
{
	unsigned short reg_val;
	int col, row;
	short scancode = 0;
	unsigned int keycode = 0;
	unsigned int alt_keycode = 0;
	int keycnt = 0;		/* How many keys are still pressed */

	/*
	 * wmb() linux kernel function which guarantees orderings in write
	 * operations
	 */
	wmb();

	/* save cur keypad matrix to prev */

	memcpy(prev_rcmap, cur_rcmap, kpp_dev.kpp_rows * sizeof(prev_rcmap[0]));
	memset(cur_rcmap, 0, kpp_dev.kpp_rows * sizeof(cur_rcmap[0]));

	for (col = 0; col < kpp_dev.kpp_cols; col++) {	/* Col */
	   /* 2. Write 1.s to KPDR[15:8] setting column data to 1.s */
	   reg_val = __raw_readw(KPDR);
	   reg_val |= 0xff00;
	   __raw_writew(reg_val, KPDR);

	   /*
	   * 3. Configure columns as totem pole outputs(for quick
	   * discharging of keypad capacitance)
	   */
	   reg_val = __raw_readw(KPCR);
	   reg_val &= 0x00ff;
	   __raw_writew(reg_val, KPCR);

	   udelay(2);

	   /*
	   * 4. Configure columns as open-drain
	   */
	   reg_val = __raw_readw(KPCR);
	   reg_val |= ((1 << kpp_dev.kpp_cols) - 1) << 8;
	   __raw_writew(reg_val, KPCR);

	   /*
	   * 5. Write a single column to 0, others to 1.
	   * 6. Sample row inputs and save data. Multiple key presses
	   * can be detected on a single column.
	   * 7. Repeat steps 2 - 6 for remaining columns.
	   */

	   /* Col bit starts at 8th bit in KPDR */
	   reg_val = __raw_readw(KPDR);
	   reg_val &= ~(1 << (8 + col));
	   __raw_writew(reg_val, KPDR);

	   /* Delay added to avoid propagating the 0 from column to row
	   * when scanning. */

           udelay(5);

	   /* Read row input */
	   reg_val = __raw_readw(KPDR);
	   for (row = 0; row < kpp_dev.kpp_rows; row++) {	/* sample row */
	      if (TEST_BIT(reg_val, row) == 0) {
	         cur_rcmap[row] = BITSET(cur_rcmap[row], col);
	         keycnt++;
	      }
	   }
	}

	/*
	 * 8. Return all columns to 0 in preparation for standby mode.
	 * 9. Clear KPKD and KPKR status bit(s) by writing to a .1.,
	 * set the KPKR synchronizer chain by writing "1" to KRSS register,
	 * clear the KPKD synchronizer chain by writing "1" to KDSC register
	 */
	reg_val = 0x00;
	__raw_writew(reg_val, KPDR);
	reg_val = __raw_readw(KPDR);
	reg_val = __raw_readw(KPSR);
	reg_val |= KBD_STAT_KPKD | KBD_STAT_KPKR | KBD_STAT_KRSS |
	           KBD_STAT_KDSC;
	__raw_writew(reg_val, KPSR);

	/* Check key press status change */

	/*
	 * prev_rcmap array will contain the previous status of the keypad
	 * matrix.  cur_rcmap array will contains the present status of the
	 * keypad matrix. If a bit is set in the array, that (row, col) bit is
	 * pressed, else it is not pressed.
	 *
	 * XORing these two variables will give us the change in bit for
	 * particular row and column.  If a bit is set in XOR output, then that
	 * (row, col) has a change of status from the previous state.  From
	 * the diff variable the key press and key release of row and column
	 * are found out.
	 *
	 * If the key press is determined then scancode for key pressed
	 * can be generated using the following statement:
	 *    scancode = ((row * 8) + col);
	 *
	 * If the key release is determined then scancode for key release
	 * can be generated using the following statement:
	 *    scancode = ((row * 8) + col) + MXC_KEYRELEASE;
	 */
	for (row = 0; row < kpp_dev.kpp_rows; row++) {
	   unsigned char diff;

	   /*
	    * Calculate the change in the keypad row status
	    */
	   diff = prev_rcmap[row] ^ cur_rcmap[row];

	   for (col = 0; col < kpp_dev.kpp_cols; col++) {
	      if ((diff >> col) & 0x1) {
	         /* There is a status change on col */
		 if ((prev_rcmap[row] & BITSET(0, col)) == 0) {
		    /*
		     * Previous state is 0, so now a key is pressed
		     */
		    scancode = ((row * kpp_dev.kpp_cols) + col);
		    KPress = 1;
		    kpp_dev.iKeyState = KStateUp;
		    LLOG_DEBUG0("Press   (%d, %d) scan=%d Kpress=%d\n",
			      row, col, scancode, KPress);
		    press_scancode[row][col] = (short)scancode;
		 }
	         else {
	            /*
		     * Previous state is not 0, so now a key is released
		     */
		    scancode = (row*kpp_dev.kpp_cols) + col + MXC_KEYRELEASE;
		    KPress = 0;
		    kpp_dev.iKeyState = KStateDown;
		    LLOG_DEBUG0("Release (%d, %d) scan=%d Kpress=%d\n",
		              row, col, scancode, KPress);
		    release_scancode[row][col] = (short)scancode;
	            keycnt++;
	         }
	      }
	   }
	}

	/*
	 * This switch case statement is the implementation of state machine
	 * of debounce logic for key press/release.  The explaination of
	 * state machine is as follows:
	 *
	 * KStateUp State:
	 * This is in intial state of the state machine this state it checks
	 * for any key presses.  The key press can be checked using the
	 * variable KPress. If KPress is set, then key press is identified
	 * and switches the to KStateFirstDown state for key press to debounce.
	 *
	 * KStateFirstDown:
	 * After debounce delay(10ms), if the KPress is still set then pass
	 * scancode generated to input device and change the state to
	 * KStateDown, else key press debounce is not satisfied so change
	 * the state to KStateUp.
	 *
	 * KStateDown:
	 * In this state it checks for any key release.  If KPress variable
	 * is cleared, then key release is indicated and so, switch the
	 * state to KStateFirstUp else to state KStateDown.
	 *
	 * KStateFirstUp:
	 * After debounce delay(10ms), if the KPress is still reset then pass
	 * the key release scancode to input device and change the state to
	 * KStateUp else key release is not satisfied so change the state to
	 * KStateDown.
	 */
	switch (kpp_dev.iKeyState) {
	case KStateUp:
	   if (KPress) {
	      /* First Down (must debounce). */
	      kpp_dev.iKeyState = KStateFirstDown;
	   }
	   else {
	      /* Still UP.(NO Changes) */
	      kpp_dev.iKeyState = KStateUp;
	   }
	   break;

	case KStateFirstDown:
	   if (KPress) {
	      for (row = 0; row < kpp_dev.kpp_rows; row++) {
	         for (col = 0; col < kpp_dev.kpp_cols; col++) {
	            if ((press_scancode[row][col] != -1)) {
	               unsigned long end_sticky = msecs_to_jiffies(atomic_read(&alt_sticky_timeout)) + sticky_alt_jiffies;

	               /* Still Down, so add scancode */
	               scancode = press_scancode[row][col];
	               // Make sure we have the latest repeat delay
	               mxckbd_dev->rep[REP_DELAY] = repeat_delay;
	               // Get the keycode from the mapping
                       keycode = mxckpd_keycodes[scancode];
                       alt_keycode = alt_convert(keycode);

                       //Did the timer expire and we are still in the "Sticky ALT" state?
                       if (ALT_state == AltStateSticky && end_sticky != sticky_alt_jiffies && time_after(jiffies, end_sticky)) {
                          ALT_state = AltStateStart;
                       }

                       // ALT key state machine - press
                       switch (ALT_state) {
                       case AltStateStart:
                          if (keycode == KEY_LEFTALT) {
                             // First press of ALT; no codes sent; change state
                             ALT_state = AltStateFirstDown;
                          }
                          else {
	                     // Not ALT; send the appropriate keycode
	                     report_event(mxckbd_dev, EV_KEY, keycode, 1);
                          }
                       break;
                       case AltStateFirstDown:
	                  if (keycode == alt_keycode) {
	                     // Not a special numeric ALT+key
	                     // Send the ALT key we had previously detected
	                     report_event(mxckbd_dev, EV_KEY, KEY_LEFTALT, 1);
	                  }
			  keycode = alt_keycode;
	                  // Send the appropriate keycode
	                  report_event(mxckbd_dev, EV_KEY, keycode, 1);
                          // Go to next state
                          ALT_state = AltStateDown;
                       break;
                       case AltStateDown:
	                  if (keycode != alt_keycode) {
	                     // This is a special numeric ALT+key
	                     // Send a fake ALT release before sending
	                     report_event(mxckbd_dev, EV_KEY, KEY_LEFTALT, 0);
	                     // Send the appropriate keycode
			     keycode = alt_keycode;
	                     report_event(mxckbd_dev, EV_KEY, keycode, 1);
	                  }
                          else {
	                     // Send a fake ALT down in case the
	                     // ALT number processing cleared it
	                     report_event(mxckbd_dev, EV_KEY, KEY_LEFTALT, 1);
	                     // Send the keycode
	                     report_event(mxckbd_dev, EV_KEY, keycode, 1);
                          }
                          // We stay in the same state; only ALT release
                          // can take us out of this state
                       break;
                       case AltStateSticky:
	                  if (keycode == KEY_LEFTALT) {
                             // ALT was pushed, released, and pushed again
                             ALT_state = AltStateFirstDown;
                          }
                          else {
	                     if (keycode == alt_keycode) {
	                        // Not a special numeric ALT+key
	                        // Send the ALT key we had previously detected
	                        report_event(mxckbd_dev, EV_KEY, KEY_LEFTALT, 1);
	                        // Send the appropriate keycode and release both
	                        report_event(mxckbd_dev, EV_KEY, keycode, 1);
	                        report_event(mxckbd_dev, EV_KEY, keycode, 0);
	                        report_event(mxckbd_dev, EV_KEY, KEY_LEFTALT, 0);
	                     }
                             else {
	                        // Send the appropriate numeric keycode
	                        // and release it (this avoids a new state)
			        keycode = alt_keycode;
	                        report_event(mxckbd_dev, EV_KEY, keycode, 1);
	                        report_event(mxckbd_dev, EV_KEY, keycode, 0);
	                     }
                             // Go back to Start - end of stickyness
                             ALT_state = AltStateStart;
                          }
                       break;
                       default:
                             // Invalid state; reset
                             ALT_state = AltStateStart;
	                     // Send the keycode
	                     report_event(mxckbd_dev, EV_KEY, keycode, 1);
                       break;
                       }
	               kpp_dev.iKeyState = KStateDown;
	               press_scancode[row][col] = -1;
	            }
	         }
	      }
	   }
	   else {
	      /* Just a bounce */
	      kpp_dev.iKeyState = KStateUp;
	   }
	   break;

	case KStateDown:
	   if (KPress) {
	      /* Still down (no change) */
	      kpp_dev.iKeyState = KStateDown;
	   }
	   else {
	      /* First Up. Must debounce */
	      kpp_dev.iKeyState = KStateFirstUp;
	   }
	   break;

	case KStateFirstUp:
	   if (KPress) {
	      /* Just a bounce */
	      kpp_dev.iKeyState = KStateDown;
	   }
	   else {
	      for (row = 0; row < kpp_dev.kpp_rows; row++) {
	         for (col = 0; col < kpp_dev.kpp_cols; col++) {
	            if ((release_scancode[row][col] != -1)) {
	               scancode = release_scancode[row][col]- MXC_KEYRELEASE;
	               // Get the keycode from the mapping
	               keycode = mxckpd_keycodes [scancode];
	               alt_keycode = alt_convert(keycode);

                       // ALT key state machine - release
                       switch (ALT_state) {
                       case AltStateStart:
                          // Send the key release event; no state change
	                  report_event(mxckbd_dev, EV_KEY, keycode, 0);
	                  report_event(mxckbd_dev, EV_KEY, alt_keycode, 0);
                       break;
                       case AltStateFirstDown:
	                  if (keycode == KEY_LEFTALT) {
	                     // Going into ALT sticky state; send nothing
	                     ALT_state = AltStateSticky;
			     sticky_alt_jiffies = jiffies;
	                  }
                          else {
	                     // Send the appropriate release keycode
	                     report_event(mxckbd_dev, EV_KEY, keycode, 0);
                             // Go to next state
                             ALT_state = AltStateDown;
	                  }
                       break;
                       case AltStateDown:
	                  if (keycode == KEY_LEFTALT) {
	                     // Released ALT key
	                     report_event(mxckbd_dev, EV_KEY, KEY_LEFTALT, 0);
	                     ALT_state = AltStateStart;
	                  }
	                  else if (keycode != alt_keycode) {
	                     // This is a special numeric ALT+key
	                     // Send a fake ALT release before sending
	                     report_event(mxckbd_dev, EV_KEY, KEY_LEFTALT, 0);
	                     // Send the appropriate keycode
			     keycode = alt_keycode;
	                     report_event(mxckbd_dev, EV_KEY, keycode, 0);
	                  }
                          else {
	                     // Send the keycode release
	                     report_event(mxckbd_dev, EV_KEY, keycode, 0);
                          }
                          // We stay in the same state; only ALT release
                          // can take us out of this state
                       break;
                       case AltStateSticky:
                          // We only get to this state if the sequence is:
                          // key down - ALT down - ALT up - key up
                          // We do no special ALT treatment; just ignore it
	                  // Send the keycode release
	                  report_event(mxckbd_dev, EV_KEY, keycode, 0);
                          // Go back to Start
                          ALT_state = AltStateStart;
                       break;
                       default:
                             // Invalid state; reset
                             ALT_state = AltStateStart;
	                     // Send the keycode
	                     report_event(mxckbd_dev, EV_KEY, keycode, 0);
                       break;
                       }
	               kpp_dev.iKeyState = KStateUp;
	               release_scancode[row][col] = -1;
	            }
	         }
	      }
	   }
	   break;

	default:
	   return -EBADRQC;
	   break;
	}

	return keycnt;
}

static void mxc_kp_tq(struct work_struct *work);

/*!
 * This function is called to start the timer for scanning the keypad if there
 * is any key press. Currently this interval is  set to 10 ms. When there are
 * no keys pressed on the keypad we return back, waiting for a keypad key
 * press interrupt.
 *
 * @param data  Opaque data passed back by kernel. Not used.
 */
static void mxc_kpp_handle_timer(unsigned long data)
{
	mxc_kp_tq((struct work_struct *)data);
}

static void mxc_kp_tq(struct work_struct *work)
{
	unsigned short reg_val;
	int i;

	if (keypad_lock) {
		LLOG_DEBUG1("mxc_kp_tq: returning because keypad is locked\n");
		return;
	}

	if (mxc_kpp_scan_matrix() == 0) {
		/*
		 * Stop scanning and wait for interrupt.
		 * Enable press interrupt and disable release interrupt.
		 */
		__raw_writew(0x00FF, KPDR);
		reg_val = __raw_readw(KPSR);
		reg_val |= (KBD_STAT_KPKR | KBD_STAT_KPKD);
		reg_val |= KBD_STAT_KRSS | KBD_STAT_KDSC;
		__raw_writew(reg_val, KPSR);
		reg_val |= KBD_STAT_KDIE;
		reg_val &= ~KBD_STAT_KRIE;
		__raw_writew(reg_val, KPSR);

		/*
		 * No more keys pressed... make sure unwanted key codes are
		 * not given upstairs
		 */
		for (i = 0; i < kpp_dev.kpp_rows; i++) {
			memset(press_scancode[i], -1,
			       sizeof(press_scancode[0][0]) * kpp_dev.kpp_cols);
			memset(release_scancode[i], -1,
			       sizeof(release_scancode[0][0]) *
			       kpp_dev.kpp_cols);

		}
		return;
	}

	/*
	 * There are still some keys pressed, continue to scan.
	 * We shall scan again in 10 ms. This has to be tuned according
	 * to the requirement.
	 */
	kpp_dev.poll_timer.expires = jiffies + KScanRate;
	kpp_dev.poll_timer.function = mxc_kpp_handle_timer;
	//add_timer(&kpp_dev.poll_timer);
	mod_timer(&kpp_dev.poll_timer, kpp_dev.poll_timer.expires);
}

/*!
 * This function is the keypad Interrupt handler.
 * This function checks for keypad status register (KPSR) for key press
 * and interrupt. If key press interrupt has occurred, then the key
 * press interrupt in the KPSR are disabled.
 * It then calls mxc_kpp_scan_matrix to check for any key pressed/released.
 * If any key is found to be pressed, then a timer is set to call
 * mxc_kpp_scan_matrix function for every 10 ms.
 *
 * @param   irq      The Interrupt number
 * @param   dev_id   Driver private data
 *
 * @result    The function returns \b IRQ_RETVAL(1) if interrupt was handled,
 *            returns \b IRQ_RETVAL(0) if the interrupt was not handled.
 *            \b IRQ_RETVAL is defined in include/linux/interrupt.h.
 */
static irqreturn_t mxc_kpp_interrupt(int irq, void *dev_id)
{
	unsigned short reg_val;

	/* Delete the polling timer */
	//del_timer(&kpp_dev.poll_timer);
	reg_val = __raw_readw(KPSR);

	/* Check if it is key press interrupt */
	if (reg_val & KBD_STAT_KPKD) {
	   /*
	    * Disable key press(KDIE status bit) interrupt
	    */
	   reg_val &= ~KBD_STAT_KDIE;
	   __raw_writew(reg_val, KPSR);
#ifdef CONFIG_PM
	}
	else if (reg_val & KBD_STAT_KPKR) {
	   /*
	    * Disable key release(KRIE status bit) interrupt - only caused
	    * by _suspend setting the bit IF a key is down while the system
	    * is being suspended.
	    */
	   reg_val &= ~KBD_STAT_KRIE;
	   __raw_writew(reg_val, KPSR);
#endif
	}
	else {
	   /* spurious interrupt */
	   return IRQ_RETVAL(0);
	}
	/*
	 * Check if any keys are pressed, if so start polling.
	 */
	mxc_kpp_handle_timer(0);
	return IRQ_HANDLED;
}

/*!
 * This function is called when the keypad driver is opened.
 * Since keypad initialization is done in __init, nothing is done in open.
 *
 * @param    dev    Pointer to device inode
 *
 * @result    The function always return 0
 */
static int mxc_kpp_open(struct input_dev *dev)
{
	return 0;
}

/*!
 * This function is called close the keypad device.
 * Nothing is done in this function, since every thing is taken care in
 * __exit function.
 *
 * @param    dev    Pointer to device inode
 *
 */
static void mxc_kpp_close(struct input_dev *dev)
{
}


/*!
 * This function locks the keypad
 */
static void lock_keypad(void)
{
	LLOG_DEBUG3("Entering lock_keypad\n");
	del_timer_sync(&kpp_dev.poll_timer);
	del_timer_sync(&mxckbd_dev->timer);
	disable_irq(keypad->irq);
	cancel_work_sync(&kpp_dev.tq);
	keypad_lock = true;
	clk_disable(kpp_clk);
	gpio_keypad_inactive();
}


/*!
 * This function unlocks the keypad
 */
static void unlock_keypad(void)
{
	LLOG_DEBUG3("Entering unlock_keypad\n");
	del_timer_sync(&kpp_dev.poll_timer);
	del_timer_sync(&mxckbd_dev->timer);
	gpio_keypad_active();
	clk_enable(kpp_clk);
	enable_irq(keypad->irq);
       	keypad_lock = false;

	/*
	 * Check keyboard status and process any key changes that
	 * happened during lock
	 */
	schedule_work (&kpp_dev.tq);
}


#ifdef CONFIG_PM
/*!
 * This function puts the Keypad controller in low-power mode/state.
 * If Keypad is enabled as a wake source(i.e. it can resume the system
 * from suspend mode), the Keypad controller doesn't enter low-power state.
 *
 * @param   pdev  the device structure used to give information on Keypad
 *                to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int mxc_kpp_suspend(struct platform_device *pdev, pm_message_t state)
{
	unsigned short reg_val;

	/*
	 * NOTE: in the Lab126 platform architecture, we always go to
	 * the "locked" state before going to suspend; therefore, all
	 * keyboard locking actions are in the lock_keypad function, above.
	 */

	if (device_may_wakeup(&pdev->dev)) {
		reg_val = __raw_readw(KPSR);
		if ((reg_val & KBD_STAT_KDIE) == 0) {
			/* if no depress interrupt enable the release interrupt */
			reg_val |= KBD_STAT_KRIE;
			__raw_writew(reg_val, KPSR);
		}
		enable_irq_wake(keypad->irq);
	}
	return 0;
}

/*!
 * This function brings the Keypad controller back from low-power state.
 * If Keypad is enabled as a wake source(i.e. it can resume the system
 * from suspend mode), the Keypad controller doesn't enter low-power state.
 *
 * @param   pdev  the device structure used to give information on Keypad
 *                to resume
 *
 * @return  The function always returns 0.
 */
static int mxc_kpp_resume(struct platform_device *pdev)
{
	/*
	 * NOTE: in the Lab126 platform architecture, we always execute
	 * unlock_keypad after mxc_kpp_resume; therefore, all
	 * keyboard unlocking actions are in the unlock_keypad function, above.
	 */
	LLOG_DEBUG3("Entering mxc_kpp_resume\n");
	if (device_may_wakeup(&pdev->dev)) {
		/* the irq routine already cleared KRIE if it was set */
		disable_irq_wake(keypad->irq);
	}
	return 0;
}

#else
#define mxc_kpp_suspend  NULL
#define mxc_kpp_resume   NULL
#endif				/* CONFIG_PM */

static ssize_t
show_alt_timer(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&alt_sticky_timeout));
}

static ssize_t
store_alt_timer(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int		i;

	if (sscanf(buf, "%d", &i) != 1)
		return -EINVAL;

	atomic_set(&alt_sticky_timeout, i <= 0 ? 0 : i);
	return count;
}

/*!
 * This function returns the current state of the keyboard
 */
int keypad_proc_read( char *page, char **start, off_t off,
                      int count, int *eof, void *data )
{
   int len;

   if (off > 0) {
     *eof = 1;
     return 0;
   }
   if (keypad_lock) {
      len = sprintf(page, "keypad is locked\n");
   }
   else {
      len = sprintf(page, "keypad is unlocked\n");
   }
   return len;
}

extern char *mxc_get_task_comm(char *buf, struct task_struct *tsk);

/** Prints out info about the current process
 *
 * @param str		The prefix to tack on to the info
 */
static void printk_process_info(const char *str)
{
	char comm[TASK_COMM_LEN];
	mxc_get_task_comm(comm, current);
	printk(KERN_ERR " %s pid:%d, name:'%s'\n",
		str, current->pid, comm);
}

/*!
 * This function executes keypad control commands based on the input string.
 *    unlock = unlock the keypad
 *    lock   = lock the keypad
 */
ssize_t keypad_proc_write( struct file *filp, const char __user *buff,
                           unsigned long len, void *data )

{
   char command[PROC_CMD_LEN];
   int  keycode = 0;

   if (len > PROC_CMD_LEN) {
      LLOG_ERROR("proc_len", "", "keypad command is too long!\n");
      return -ENOSPC;
   }

   if (copy_from_user(command, buff, len)) {
      return -EFAULT;
   }

   if ( !strncmp(command, "unlock", 6) ) {
      // Requested to unlock keypad
      if (!keypad_lock) {
         // Keypad was already unlocked; do nothing
         LLOG_INFO("unlocked1", "status=unlocked", "\n");
      }
      else {
         // Unlock keypad
         unlock_keypad();
         LLOG_INFO("unlocked2", "status=unlocked", "\n");
      }
   }
   else if ( !strncmp(command, "lock", 4) ) {
      // Requested to lock keypad
      if (keypad_lock) {
         // Keypad was already locked; do nothing
         LLOG_INFO("locked1", "status=locked", "\n");
      }
      else {
         // Lock keypad
         lock_keypad();
         LLOG_INFO("locked2", "status=locked", "\n");
      }
   }
   else if ( !strncmp(command, "sendalt", 7) ) {
      // Requested to send an ALT+keycode sequence
      if (keypad_lock) {
         // Keypad is locked; do nothing
         LLOG_WARN("locked3a", "", "The keypad is locked!  Unlock before sending keycodes.\n");
      }
      else {
         // Read keycode and send
         sscanf(command, "sendalt %d", &keycode);
         LLOG_INFO("send_alt_plus_key", "", "Sending ALT plus keycode %d\n", keycode);
	 report_event(mxckbd_dev, EV_KEY, KEY_LEFTALT, 1);
	 report_event(mxckbd_dev, EV_KEY, keycode, 1);
	 report_event(mxckbd_dev, EV_KEY, keycode, 0);
	 report_event(mxckbd_dev, EV_KEY, KEY_LEFTALT, 0);
      }
   }
   else if ( !strncmp(command, "sendshift", 9) ) {
      // Requested to send an SHIFT+keycode sequence
      if (keypad_lock) {
         // Keypad is locked; do nothing
         LLOG_WARN("locked3b", "", "The keypad is locked!  Unlock before sending keycodes.\n");
      }
      else {
         // Read keycode and send
         sscanf(command, "sendshift %d", &keycode);
         LLOG_INFO("send_shift_plus_key", "", "Sending SHIFT plus keycode %d\n", keycode);
	 report_event(mxckbd_dev, EV_KEY, KEY_LEFTSHIFT, 1);
	 report_event(mxckbd_dev, EV_KEY, keycode, 1);
	 report_event(mxckbd_dev, EV_KEY, keycode, 0);
	 report_event(mxckbd_dev, EV_KEY, KEY_LEFTSHIFT, 0);
      }
   }
   else if ( !strncmp(command, "send", 4) ) {
      // Requested to send a keycode
      if (keypad_lock) {
         // Keypad is locked; do nothing
         LLOG_WARN("locked3", "", "The keypad is locked!  Unlock before sending keycodes.\n");
      }
      else {
         // Read keycode and send
         sscanf(command, "send %d", &keycode);
         LLOG_INFO("send_key", "", "Sending keycode %d\n", keycode);
	 report_event(mxckbd_dev, EV_KEY, keycode, 1);
	 report_event(mxckbd_dev, EV_KEY, keycode, 0);
      }
   }
   else if ( !strncmp(command, "keycnt", 6) ) {
	   printk_process_info("rogue process wrote 'keycnt' to /proc/keypad: ");
   }
   else if ( !strncmp(command, "dokeycnt", 8) ) {
      // Requested the key-pressed count for selected keycodes
      LLOG_INFO("keycnt_home", "HOME_keycount=%d", "\n", keycode_count[KEY_KPSLASH]);
      LLOG_INFO("keycnt_menu", "MENU_keycount=%d", "\n", keycode_count[KEY_MENU]);
      LLOG_INFO("keycnt_back", "BACK_keycount=%d", "\n", keycode_count[KEY_HIRAGANA]);
      LLOG_INFO("keycnt_next_left", "NEXT_LEFT_keycount=%d", "\n", keycode_count[KEY_PAGEUP]);
      LLOG_INFO("keycnt_next_right", "NEXT_RIGHT_keycount=%d", "\n", keycode_count[KEY_YEN]);
      LLOG_INFO("keycnt_prev", "PREV_keycount=%d", "\n", keycode_count[KEY_PAGEDOWN]);
      LLOG_INFO("keycnt_font", "FONT_keycount=%d", "\n", keycode_count[KEY_KATAKANA]);
      LLOG_INFO("keycnt_sym", "SYM_keycount=%d", "\n", keycode_count[KEY_MUHENKAN]);
   }
   else if ( !strncmp(command, "debug", 5) ) {
      // Requested to set debug level
      sscanf(command, "debug %d", &debug);
      LLOG_INFO("debug", "", "Setting debug level to %d\n", debug);
      set_debug_log_mask(debug);
   }
   else {
      LLOG_ERROR("proc_cmd", "", "Unrecognized keypad command\n");
   }

   return len;
}


/*!
 * This function is called to free the allocated memory for local arrays
 */
static void mxc_kpp_free_allocated(void)
{

	int i;

	if (press_scancode) {
		for (i = 0; i < kpp_dev.kpp_rows; i++) {
			if (press_scancode[i])
				kfree(press_scancode[i]);
		}
		kfree(press_scancode);
	}

	if (release_scancode) {
		for (i = 0; i < kpp_dev.kpp_rows; i++) {
			if (release_scancode[i])
				kfree(release_scancode[i]);
		}
		kfree(release_scancode);
	}

	if (cur_rcmap)
		kfree(cur_rcmap);

	if (prev_rcmap)
		kfree(prev_rcmap);

	if (mxckbd_dev)
		input_free_device(mxckbd_dev);
}

/*!
 * This function is called during the driver binding process.
 *
 * @param   pdev  the device structure used to store device specific
 *                information that is used by the suspend, resume and remove
 *                functions.
 *
 * @return  The function returns 0 on successful registration. Otherwise returns
 *          specific error code.
 */
static int mxc_kpp_probe(struct platform_device *pdev)
{
	int i, irq;
	int retval;
	unsigned int reg_val;
	int junk;

        /* Create proc entry */
	proc_entry = create_proc_entry( KEYPAD_PROC_FILE, 0644, NULL );
    	if (proc_entry == NULL) {
      		LLOG_ERROR("create_proc", "", "Keyboard could not create proc entry\n");
      		return -ENOMEM;
    	}
	else {
      		proc_entry->read_proc = keypad_proc_read;
      		proc_entry->write_proc = keypad_proc_write;
    	}

	keypad = (struct keypad_data *)pdev->dev.platform_data;

	kpp_dev.kpp_cols = keypad->colmax;
	kpp_dev.kpp_rows = keypad->rowmax;
	keypad_lock = true;

	/*
	 * Request for IRQ number for keypad port. The Interrupt handler
	 * function (mxc_kpp_interrupt) is called when ever interrupt occurs on
	 * keypad port.
	 */
	irq = platform_get_irq(pdev, 0);
	keypad->irq = irq;
	retval = request_irq(irq, mxc_kpp_interrupt, 0, MOD_NAME, MOD_NAME);
	if (retval) {
		LLOG_ERROR("req_irq", "", "request_irq(%d) returned error %d\n", irq, retval);
		goto del_proc_entry;
	}

	/* Enable keypad clock */
	kpp_clk = clk_get(&pdev->dev, "kpp_clk");
	clk_enable(kpp_clk);

	/* IOMUX configuration for keypad */
	retval = gpio_keypad_active();
	if (retval) {
		LLOG_ERROR("set_gpio", "", "gpio_keypad_active returned error %d\n", retval);
		goto free_irq;
	}

	/* Configure keypad */

	/* Enable number of rows in keypad (KPCR[7:0])
	 * Configure keypad columns as open-drain (KPCR[15:8])
	 *
	 * Configure the rows/cols in KPP
	 * LSB nibble in KPP is for 8 rows
	 * MSB nibble in KPP is for 8 cols
	 */
	reg_val = __raw_readw(KPCR);
	reg_val |= (1 << keypad->rowmax) - 1;	/* LSB */
	reg_val |= ((1 << keypad->colmax) - 1) << 8;	/* MSB */
	__raw_writew(reg_val, KPCR);

	/* Write 0's to KPDR[15:8] */
	reg_val = __raw_readw(KPDR);
	reg_val &= 0x00ff;
	__raw_writew(reg_val, KPDR);

	/* Configure columns as output, rows as input (KDDR[15:0]) */
	reg_val = __raw_readw(KDDR);
	reg_val |= 0xff00;
	reg_val &= 0xff00;
	__raw_writew(reg_val, KDDR);

	reg_val = __raw_readw(KPSR);
	reg_val &= ~(KBD_STAT_KPKR | KBD_STAT_KPKD);
	reg_val |= KBD_STAT_KPKD;
	reg_val |= KBD_STAT_KRSS | KBD_STAT_KDSC;
	__raw_writew(reg_val, KPSR);
	reg_val |= KBD_STAT_KDIE;
	reg_val &= ~KBD_STAT_KRIE;
	__raw_writew(reg_val, KPSR);

	mxckpd_keycodes_size = keypad->rowmax * keypad->colmax;
	mxckpd_keycodes = keypad->matrix;

	if ((keypad->matrix == (void *)0)
	    || (mxckpd_keycodes_size == 0)) {
		retval = -ENODEV;
		goto free_irq;
	}

	mxckbd_dev = input_allocate_device();
	if (!mxckbd_dev) {
		LLOG_ERROR("mxckbd_dev", "", "Not enough memory for input device\n");
		retval = -ENOMEM;
		goto free_irq;
	}

	mxckbd_dev->keycode = &mxckpd_keycodes;
	mxckbd_dev->keycodesize = sizeof(unsigned char);
	mxckbd_dev->keycodemax = mxckpd_keycodes_size;

        // Parameters related to key-repeat functionality
	mxckbd_dev->timer.data = (long) mxckbd_dev;
	mxckbd_dev->timer.function = input_repeat_key;
	mxckbd_dev->rep[REP_DELAY] = repeat_delay;
	mxckbd_dev->rep[REP_PERIOD] = repeat_period;

	mxckbd_dev->name = "mxckpd";
	mxckbd_dev->id.bustype = BUS_HOST;
	mxckbd_dev->open = mxc_kpp_open;
	mxckbd_dev->close = mxc_kpp_close;

	/* allocate required memory */
	press_scancode = kmalloc(kpp_dev.kpp_rows * sizeof(press_scancode[0]),
				 GFP_KERNEL);
	release_scancode =
	    kmalloc(kpp_dev.kpp_rows * sizeof(release_scancode[0]), GFP_KERNEL);

	if (!press_scancode || !release_scancode) {
		retval = -ENOMEM;
		goto kpp_free;
	}

	for (i = 0; i < kpp_dev.kpp_rows; i++) {
		press_scancode[i] = kmalloc(kpp_dev.kpp_cols
					    * sizeof(press_scancode[0][0]),
					    GFP_KERNEL);
		release_scancode[i] =
		    kmalloc(kpp_dev.kpp_cols * sizeof(release_scancode[0][0]),
			    GFP_KERNEL);

		if (!press_scancode[i] || !release_scancode[i]) {
			retval = -ENOMEM;
			goto kpp_free;
		}
	}

	cur_rcmap =
	    kmalloc(kpp_dev.kpp_rows * sizeof(cur_rcmap[0]), GFP_KERNEL);
	prev_rcmap =
	    kmalloc(kpp_dev.kpp_rows * sizeof(prev_rcmap[0]), GFP_KERNEL);

	if (!cur_rcmap || !prev_rcmap) {
		retval = -ENOMEM;
		goto kpp_free;
	}

	__set_bit(EV_KEY, mxckbd_dev->evbit);
	__set_bit(EV_REP, mxckbd_dev->evbit);

	for (i = 0; i < mxckpd_keycodes_size; i++)
		__set_bit(mxckpd_keycodes[i], mxckbd_dev->keybit);

	for (i = 0; i < kpp_dev.kpp_rows; i++) {
		memset(press_scancode[i], -1,
		       sizeof(press_scancode[0][0]) * kpp_dev.kpp_cols);
		memset(release_scancode[i], -1,
		       sizeof(release_scancode[0][0]) * kpp_dev.kpp_cols);
	}
	memset(cur_rcmap, 0, kpp_dev.kpp_rows * sizeof(cur_rcmap[0]));
	memset(prev_rcmap, 0, kpp_dev.kpp_rows * sizeof(prev_rcmap[0]));

	// Initialize key press counters
	for (i = 0; i < MAX_CODES; i++) {
		keycode_count[i] = 0;
	}

	keypad_lock = false;
	/* Initialize the polling timer and the work item */
	init_timer(&kpp_dev.poll_timer);

	INIT_WORK (&kpp_dev.tq, mxc_kp_tq);

	retval = input_register_device(mxckbd_dev);
	if (retval < 0) {
		goto kpp_free;
	}


	/* By default, devices should wakeup if they can */
	/* HOWEVER, the keypad is NOT set as "should wakeup" in our platform */
	device_init_wakeup(&pdev->dev, 0);

	junk = device_create_file(&pdev->dev, &dev_attr_alt_timer);
	return 0; // Success!

kpp_free:
	mxc_kpp_free_allocated();
	input_free_device(mxckbd_dev);
free_irq:
	free_irq(irq, MOD_NAME);
	clk_disable(kpp_clk);
	clk_put(kpp_clk);
del_proc_entry:
	remove_proc_entry(KEYPAD_PROC_FILE, NULL);


	return retval;
}

/*!
 * Dissociates the driver from the kpp device.
 *
 * @param   pdev  the device structure used to give information on which SDHC
 *                to remove
 *
 * @return  The function always returns 0.
 */
static int mxc_kpp_remove(struct platform_device *pdev)
{
	unsigned short reg_val;

	device_remove_file(&pdev->dev, &dev_attr_alt_timer);

	/*
	 * Clear the KPKD status flag (write 1 to it) and synchronizer chain.
	 * Set KDIE control bit, clear KRIE control bit (avoid false release
	 * events. Disable the keypad GPIO pins.
	 */
	__raw_writew(0x00, KPCR);
	__raw_writew(0x00, KPDR);
	__raw_writew(0x00, KDDR);

	reg_val = __raw_readw(KPSR);
	reg_val |= KBD_STAT_KPKD;
	reg_val &= ~KBD_STAT_KRSS;
	reg_val |= KBD_STAT_KDIE;
	reg_val &= ~KBD_STAT_KRIE;
	__raw_writew(reg_val, KPSR);

	gpio_keypad_inactive();
	clk_disable(kpp_clk);
	clk_put(kpp_clk);

	KPress = 0;

	del_timer_sync(&kpp_dev.poll_timer);
	del_timer_sync(&mxckbd_dev->timer);

	free_irq(keypad->irq, MOD_NAME);
	remove_proc_entry(KEYPAD_PROC_FILE, NULL);
	input_unregister_device(mxckbd_dev);

	mxc_kpp_free_allocated();

	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mxc_kpd_driver = {
	.driver = {
		   .name = "mxc_keypad",
		   .bus = &platform_bus_type,
		   },
	.suspend = mxc_kpp_suspend,
	.resume = mxc_kpp_resume,
	.probe = mxc_kpp_probe,
	.remove = mxc_kpp_remove
};

/*!
 * This function is called for module initialization.
 * It registers keypad char driver and requests for KPP irq number. This
 * function does the initialization of the keypad device.
 *
 * The following steps are used for keypad configuration,\n
 * -# Enable number of rows in the keypad control register (KPCR[7:0}).\n
 * -# Write 0's to KPDR[15:8]\n
 * -# Configure keypad columns as open-drain (KPCR[15:8])\n
 * -# Configure columns as output, rows as input (KDDR[15:0])\n
 * -# Clear the KPKD status flag (write 1 to it) and synchronizer chain\n
 * -# Set KDIE control bit, clear KRIE control bit\n
 * In this function the keypad queue initialization is done.
 * The keypad IOMUX configuration are done here.*

 *
 * @return      0 on success and a non-zero value on failure.
 */
static int __init mxc_kpp_init(void)
{
	LLOG_INIT();
	set_debug_log_mask(debug);
	LLOG_INFO("drv", "", "Keypad driver loaded\n");
	platform_driver_register(&mxc_kpd_driver);
	return 0;
}

/*!
 * This function is called whenever the module is removed from the kernel. It
 * unregisters the keypad driver from kernel and frees the irq number.
 * This function puts the keypad to standby mode. The keypad interrupts are
 * disabled. It calls gpio_keypad_inactive function to switch gpio
 * configuration into default state.
 *
 */
static void __exit mxc_kpp_cleanup(void)
{
	platform_driver_unregister(&mxc_kpd_driver);
}

module_init(mxc_kpp_init);
module_exit(mxc_kpp_cleanup);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC Keypad Controller Driver");
MODULE_LICENSE("GPL");
