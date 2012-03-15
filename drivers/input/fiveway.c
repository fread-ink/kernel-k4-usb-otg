/*
** 5-Way Switch driver routine for MX50 Yoshi/Tequila
** Copyright (C) 2010-2011 Amazon Technologies, Inc. All Rights Reserved.
** Manish Lachwani (lachwani@lab126.com).
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/syscalls.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/irq.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/delay.h>

#include <mach/pmic_power.h>
#include <mach/gpio.h>
#include <mach/boardid.h>

// Logging
static unsigned int logging_mask;
#define LLOG_G_LOG_MASK logging_mask
#define LLOG_KERNEL_COMP "fiveway"
#include <llog.h>

#include "fiveway.h"

// Input device structure
static struct input_dev *fiveway_dev = NULL;

// Global variables

static char line_state[NUM_OF_LINES];

// Workqueue
static struct delayed_work fiveway_up_debounce_tq_d;
static struct delayed_work fiveway_left_debounce_tq_d;
static struct delayed_work fiveway_down_debounce_tq_d;
static struct delayed_work fiveway_right_debounce_tq_d;
static struct delayed_work fiveway_select_debounce_tq_d;

static struct delayed_work fiveway_repeat_up_tq_d;
static struct delayed_work fiveway_repeat_select_tq_d;
static struct delayed_work fiveway_repeat_right_tq_d;
static struct delayed_work fiveway_repeat_left_tq_d;
static struct delayed_work fiveway_repeat_down_tq_d;

static void fiveway_up_debounce_wk(struct work_struct *);
static void fiveway_down_debounce_wk(struct work_struct *);
static void fiveway_right_debounce_wk(struct work_struct *);
static void fiveway_left_debounce_wk(struct work_struct *);
static void fiveway_select_debounce_wk(struct work_struct *);

static void fiveway_repeat_up_work(struct work_struct *);
static void fiveway_repeat_left_work(struct work_struct *);
static void fiveway_repeat_select_work(struct work_struct *);
static void fiveway_repeat_right_work(struct work_struct *);
static void fiveway_repeat_down_work(struct work_struct *);

DEFINE_ATOMIC_SPINLOCK(fiveway_spinlock);

static int fiveway_tequila = 0;

// Proc entry structures and global variables.

#define FIVEWAY_PROC_FIVEWAY "fiveway"
#define PROC_FIVEWAY_CMD_LEN 50
static struct proc_dir_entry *proc_fiveway_entry;
static int fiveway_lock = 0;  // 0=unlocked; non-zero=locked

#define FIVEWAY_PROC_FIVEWAY_WE "fiveway_wake"
static struct proc_dir_entry *proc_fiveway_we_entry;
static int fiveway_wake_enable = 0;  // 0=disabled; 1=enabled

/* Make the following /sys configurable */
static int fiveway_hold_timeout = 200; /* 200ms */
static int fiveway_repeat_timeout = 60; /* 60 ms */

// Module parameters

#define FIVEWAY_REV_DEFAULT 0
static int fiveway_rev = FIVEWAY_REV_DEFAULT;
module_param(fiveway_rev, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(fiveway_rev, "5-Way hardware revision; 0=Mario, 1=Turing prototype build, 2=Turing v1 (default is 0)");

static int debounce_timeout = 60; /* 60ms */
module_param(debounce_timeout, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(debounce_timeout, "Time for debouncing the fiveway switch, in msec; MUST be a multiple of 10 (default is 60)");

static int SELECT_hold_timeout = 500; /* 500 ms */
module_param(SELECT_hold_timeout, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(SELECT_hold_timeout, "Time before holding SELECT generates HOLD event or starts auto-repeat mode, in msec; MUST be a multiple of 10 (default is 1000)");

static int debug = 0;
module_param(debug, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(debug, "Enable debugging; 0=off, any other value=on (default is off)");

/*
 * Set the log mask depending on the debug level
 */
static void set_debug_log_mask(int debug_level)
{
   logging_mask = LLOG_LEVEL_MASK_EXCEPTIONS;
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

static atomic_t fiveway_is_locked = ATOMIC_INIT(0);

/*
 * Report the fiveway event.  In addition to calling
 * input_event, this function sends a uevent for any
 * processes listening in the userspace
 */
static void report_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
   if (atomic_read(&fiveway_is_locked))
       return;

   input_event( dev, type, code, value );

   // Send uevent so userspace power daemon can monitor
   // fiveway activity
   // NOTE: only sending uevent on key down as per Manish

   if (value == 1) {
      if( kobject_uevent_atomic( &dev->dev.kobj, KOBJ_CHANGE ) )
      {
          LLOG_ERROR("uevent", "", "Fiveway driver failed to send uevent\n" );
      }
   }
}

#define AUTO_RPT(dbgstr, keycode, time, line) \
      LLOG_DEBUG0("Got 5-Way Switch repeat timeout: %s\n", dbgstr); \
      report_event(fiveway_dev, EV_KEY, keycode, 2); \
      line_state[line] = LINE_HELD;

extern bool fiveway_datain(int);

static void fiveway_repeat_up_work(struct work_struct *work)
{
   LLOG_DEBUG0("Got auto-repeat timeout\n");

   if (fiveway_lock == 1)
	return;

   // Send appropriate event, depending on button held
   if ( fiveway_datain(UP_LINE) == 0) {
      AUTO_RPT("UP", FIVEWAY_KEYCODE_UP, fiveway_repeat_timeout, UP_LINE);
      schedule_delayed_work(&fiveway_repeat_up_tq_d, ((fiveway_repeat_timeout * HZ) / 1000));
   }
}

static void fiveway_repeat_select_work(struct work_struct *work)
{
   LLOG_DEBUG0("Got auto-repeat timeout\n");

   /* Do not send a select auto repeat */
   return;

   // Send appropriate event, depending on button held
   if ( fiveway_datain(SELECT_LINE) == 0) {
      AUTO_RPT("SELECT", FIVEWAY_KEYCODE_SELECT, fiveway_repeat_timeout, SELECT_LINE);
      schedule_delayed_work(&fiveway_repeat_select_tq_d, ((fiveway_repeat_timeout * HZ) / 1000));
   }
}

static void fiveway_repeat_right_work(struct work_struct *work)
{
   LLOG_DEBUG0("Got auto-repeat timeout\n");

   if (fiveway_lock == 1)
        return;

   // Send appropriate event, depending on button held
   if ( fiveway_datain(RIGHT_LINE) == 0) {
      AUTO_RPT("RIGHT", FIVEWAY_KEYCODE_RIGHT, fiveway_repeat_timeout, RIGHT_LINE);
      schedule_delayed_work(&fiveway_repeat_right_tq_d, ((fiveway_repeat_timeout * HZ) / 1000));
   }
}

static void fiveway_repeat_left_work(struct work_struct *work)
{
   LLOG_DEBUG0("Got auto-repeat timeout\n");

   if (fiveway_lock == 1)
        return;

   // Send appropriate event, depending on button held
   if ( fiveway_datain(LEFT_LINE) == 0) {
      AUTO_RPT("LEFT", FIVEWAY_KEYCODE_LEFT, fiveway_repeat_timeout, LEFT_LINE);
      schedule_delayed_work(&fiveway_repeat_left_tq_d, ((fiveway_repeat_timeout * HZ) / 1000));
   }
}

static void fiveway_repeat_down_work(struct work_struct *work)
{
   LLOG_DEBUG0("Got auto-repeat timeout\n");

   if (fiveway_lock == 1)
        return;

   // Send appropriate event, depending on button held
   if ( fiveway_datain(DOWN_LINE) == 0) {
      AUTO_RPT("DOWN", FIVEWAY_KEYCODE_DOWN, fiveway_repeat_timeout, DOWN_LINE);
      schedule_delayed_work(&fiveway_repeat_down_tq_d, ((fiveway_repeat_timeout * HZ) / 1000));
   }
}

static void enable_other_irqs(int irq, int enable);

/* Called from the IRQ handler */
#define PUSHED1(dbgstr, keycode, time, irq, line) \
      LLOG_DEBUG0("Got 5-Way Switch signal push: %s\n", dbgstr); \
      line_state[line] = LINE_ASSERTED; \
      report_event(fiveway_dev, EV_KEY, keycode, 1); \
      if (fiveway_tequila) \
          set_irq_type(irq, IRQF_TRIGGER_FALLING); \
      else \
          set_irq_type(irq, IRQF_TRIGGER_RISING); \

#define PUSHED(dbgstr, keycode, time, irq, line) \
      LLOG_DEBUG0("Got 5-Way Switch signal push: %s\n", dbgstr); \
      timeout = time; \
      if (fiveway_tequila) \
         set_irq_type(irq, IRQF_TRIGGER_FALLING); \
      else \
          set_irq_type(irq, IRQF_TRIGGER_RISING); \
      line_state[line] = LINE_ASSERTED;

#define RELEASED(dbgstr, keycode, line, irq) \
      LLOG_DEBUG0("Got 5-Way Switch signal release: %s\n", dbgstr); \
      report_event(fiveway_dev, EV_KEY, keycode, 0); \
      if (fiveway_tequila) \
         set_irq_type(irq, IRQF_TRIGGER_RISING); \
      else \
         set_irq_type(irq, IRQF_TRIGGER_FALLING);

#define RELEASED1(dbgstr, keycode, line, irq) \
      LLOG_DEBUG0("Got 5-Way Switch signal release: %s\n", dbgstr); \
      report_event(fiveway_dev, EV_KEY, keycode, 0); \
      line_state[line] = LINE_DEASSERTED; \
      if (fiveway_tequila) \
         set_irq_type(irq, IRQF_TRIGGER_RISING); \
      else \
         set_irq_type(irq, IRQF_TRIGGER_FALLING);

static int disabled_up_irq = 0;
static int disabled_select_irq = 0;
static int disabled_right_irq = 0;
static int disabled_left_irq = 0;
static int disabled_down_irq = 0;

static int ignore_up_line = 0;
static int ignore_select_line = 0;
static int ignore_right_line = 0;
static int ignore_down_line = 0;
static int ignore_left_line = 0;

static int disabled_up = 0;
static int disabled_select = 0;
static int disabled_right = 0;
static int disabled_left = 0;
static int disabled_down = 0;

static void fiveway_up_debounce_wk(struct work_struct *work)
{
   int timeout = 0;
   int flag = 0;

   LLOG_DEBUG1("Debounce period expired\n");
   // Send appropriate event, depending on button held
   if (!ignore_up_line && fiveway_datain(UP_LINE) == 0 ) {
      PUSHED("UP", FIVEWAY_KEYCODE_UP, fiveway_hold_timeout,
             FIVEWAY_UP_IRQ, UP_LINE);
      schedule_delayed_work(&fiveway_repeat_up_tq_d, ((timeout * HZ) / 1000));
   }

   if (( fiveway_datain(UP_LINE) == 1 ) &&
       ( line_state[UP_LINE] != LINE_DEASSERTED )) {
      cancel_delayed_work_sync(&fiveway_repeat_up_tq_d);
      RELEASED1("UP", FIVEWAY_KEYCODE_UP, UP_LINE, FIVEWAY_UP_IRQ);
      ignore_up_line = 0;
      flag = 1;
   }

   if (disabled_up == 1) {
	disabled_up = 0;
	enable_irq(FIVEWAY_UP_IRQ);
	if (flag == 1 && !timeout)
		enable_other_irqs(FIVEWAY_UP_IRQ, 1);
   }
}

static void fiveway_down_debounce_wk(struct work_struct *work)
{
   int timeout = 0;
   int flag = 0;

   LLOG_DEBUG1("Debounce period expired\n");
   // Send appropriate event, depending on button held
   if (!ignore_down_line &&  fiveway_datain(DOWN_LINE) == 0 ) {
      PUSHED("DOWN", FIVEWAY_KEYCODE_DOWN, fiveway_hold_timeout,
             FIVEWAY_DOWN_IRQ, DOWN_LINE);
      schedule_delayed_work(&fiveway_repeat_down_tq_d, ((timeout * HZ) / 1000));
   }

   if (( fiveway_datain(DOWN_LINE) == 1 ) &&
       ( line_state[DOWN_LINE] != LINE_DEASSERTED )) {
      cancel_delayed_work_sync(&fiveway_repeat_down_tq_d);
      RELEASED1("DOWN", FIVEWAY_KEYCODE_DOWN, DOWN_LINE, FIVEWAY_DOWN_IRQ);
      ignore_down_line = 0;
      flag = 1;
   }

   if (disabled_down == 1) {
	disabled_down = 0;
        enable_irq(FIVEWAY_DOWN_IRQ);
	if (flag == 1 && !timeout)
		enable_other_irqs(FIVEWAY_DOWN_IRQ, 1);
   }

}

static void fiveway_right_debounce_wk(struct work_struct *work)
{
   int timeout = 0;
   int flag = 0;

   LLOG_DEBUG1("Debounce period expired\n");
   // Send appropriate event, depending on button held

   if (!ignore_right_line &&  fiveway_datain(RIGHT_LINE) == 0 ) {
      PUSHED("RIGHT", FIVEWAY_KEYCODE_RIGHT, fiveway_hold_timeout,
             FIVEWAY_RIGHT_IRQ, RIGHT_LINE);
      schedule_delayed_work(&fiveway_repeat_right_tq_d, ((timeout * HZ) / 1000));
   }

   if (( fiveway_datain(RIGHT_LINE) == 1 ) &&
       ( line_state[RIGHT_LINE] != LINE_DEASSERTED )) {
      cancel_delayed_work_sync(&fiveway_repeat_right_tq_d);
      RELEASED1("RIGHT", FIVEWAY_KEYCODE_RIGHT, RIGHT_LINE, FIVEWAY_RIGHT_IRQ);
      ignore_right_line = 0;
      flag = 1;
   }

   if (disabled_right == 1) {
        disabled_right = 0;
        enable_irq(FIVEWAY_RIGHT_IRQ);
	if (flag == 1 && !timeout)
                enable_other_irqs(FIVEWAY_RIGHT_IRQ, 1);
   }
}

static void fiveway_left_debounce_wk(struct work_struct *work)
{
   int timeout = 0;
   int flag = 0;

   LLOG_DEBUG1("Debounce period expired\n");
   // Send appropriate event, depending on button held

   if (!ignore_left_line &&  fiveway_datain(LEFT_LINE) == 0 ) {
      PUSHED("LEFT", FIVEWAY_KEYCODE_LEFT, fiveway_hold_timeout,
             FIVEWAY_LEFT_IRQ, LEFT_LINE);
      schedule_delayed_work(&fiveway_repeat_left_tq_d, ((timeout * HZ) / 1000));
   }

   if (( fiveway_datain(LEFT_LINE) == 1 ) &&
       ( line_state[LEFT_LINE] != LINE_DEASSERTED )) {
      cancel_delayed_work_sync(&fiveway_repeat_left_tq_d);
      RELEASED1("LEFT", FIVEWAY_KEYCODE_LEFT, LEFT_LINE, FIVEWAY_LEFT_IRQ);
      ignore_left_line = 0;
      flag = 1;
   }
   
   if (disabled_left == 1) {
        disabled_left = 0;
        enable_irq(FIVEWAY_LEFT_IRQ);
	if (flag == 1 && !timeout)
                enable_other_irqs(FIVEWAY_LEFT_IRQ, 1);
   }

}

static void fiveway_select_debounce_wk(struct work_struct *work)
{
   int timeout = 0;
   int flag = 0;

   LLOG_DEBUG1("Debounce period expired\n");
   // Send appropriate event, depending on button held

   if (!ignore_select_line &&  fiveway_datain(SELECT_LINE) == 0 ) {
      PUSHED("SELECT", FIVEWAY_KEYCODE_SELECT, SELECT_hold_timeout,
             FIVEWAY_SELECT_IRQ, SELECT_LINE);
      schedule_delayed_work(&fiveway_repeat_select_tq_d, ((timeout * HZ) / 1000));
   }

   if (( fiveway_datain(SELECT_LINE) == 1 ) &&
       ( line_state[SELECT_LINE] != LINE_DEASSERTED )) {
      cancel_delayed_work_sync(&fiveway_repeat_select_tq_d);
      RELEASED1("SELECT", FIVEWAY_KEYCODE_SELECT, SELECT_LINE, FIVEWAY_SELECT_IRQ);
      ignore_select_line = 0;
      flag = 1;
   }

   if (disabled_select == 1) {
        disabled_select = 0;
        enable_irq(FIVEWAY_SELECT_IRQ);
	if (flag == 1 && !timeout)
                enable_other_irqs(FIVEWAY_SELECT_IRQ, 1);
   }

}

static int fiveway_irq_disabled(int irq)
{
	if ( (irq == FIVEWAY_UP_IRQ) && disabled_up_irq)
		return 1;

	if ( (irq == FIVEWAY_DOWN_IRQ) && disabled_down_irq)
		return 1;

	if ( (irq == FIVEWAY_LEFT_IRQ) && disabled_left_irq)
		return 1;

	if ( (irq == FIVEWAY_RIGHT_IRQ) && disabled_right_irq)
		return 1;

	if ( (irq == FIVEWAY_SELECT_IRQ) && disabled_select_irq)
		return 1;

	return 0;
}

static void enable_other_irqs(int irq, int enable)
{
	unsigned long flags;

	if (fiveway_irq_disabled(irq))
		return;

	atomic_spin_lock_irqsave(&fiveway_spinlock, flags);

	if (enable == 0) {	
		if (irq != FIVEWAY_UP_IRQ) {
			disabled_up_irq = 1;
		}

		if (irq != FIVEWAY_DOWN_IRQ) {
			disabled_down_irq = 1;
		}

		if (irq != FIVEWAY_LEFT_IRQ) {
			disabled_left_irq = 1;
		}

		if (irq != FIVEWAY_RIGHT_IRQ) {
			disabled_right_irq = 1;
		}

		if (irq != FIVEWAY_SELECT_IRQ) {
			disabled_select_irq = 1;
		}
	}
	else {
		if (irq != FIVEWAY_UP_IRQ) {
			if (fiveway_datain(UP_LINE) == 0)
				ignore_up_line = 1;

			disabled_up_irq = 0;
		}	

		if (irq != FIVEWAY_DOWN_IRQ) {
			if (fiveway_datain(DOWN_LINE) == 0)
				ignore_down_line = 1;

			disabled_down_irq = 0;
		}

		if (irq != FIVEWAY_LEFT_IRQ) {
			if (fiveway_datain(LEFT_LINE) == 0)
				ignore_left_line = 1;

			disabled_left_irq = 0;
		}

		if (irq != FIVEWAY_RIGHT_IRQ) {
			if (fiveway_datain(RIGHT_LINE) == 0)
				ignore_right_line = 1;

			disabled_right_irq = 0;
		}

		if (irq != FIVEWAY_SELECT_IRQ) {
			if (fiveway_datain(SELECT_LINE) == 0)
				ignore_select_line = 1;

			disabled_select_irq = 0;
		}
	}
	atomic_spin_unlock_irqrestore(&fiveway_spinlock, flags);
}

static int fiveway_initialized = 0;
static int fiveway_select_wake = 0;
static void select_worker(struct work_struct *work);
static DECLARE_DELAYED_WORK(select_work, select_worker);

static void select_worker(struct work_struct *work)
{
	if (fiveway_datain(SELECT_LINE) == 0) {
		printk(KERN_INFO "Fiveway wakeup\n");
		fiveway_select_wake = 0;
		kobject_uevent(&fiveway_dev->dev.kobj, KOBJ_ADD);
	}
}

static irqreturn_t fiveway_irq (int irq, void *data, struct pt_regs *r)
{
  if (fiveway_irq_disabled(irq))
	return IRQ_HANDLED;

  if (!fiveway_initialized)
        return IRQ_HANDLED;

  if (fiveway_datain(SELECT_LINE) == 0) {
	fiveway_select_wake = 1;
	schedule_delayed_work(&select_work, msecs_to_jiffies(900));
  }
  else {
	if (fiveway_select_wake == 1) {
		fiveway_select_wake = 0;
		/* if released, then cancel pending work */
		cancel_delayed_work(&select_work);
	}
  }

  if ( !disabled_up_irq && (fiveway_datain(UP_LINE) == 0) && (irq == FIVEWAY_UP_IRQ))  {
      if (ignore_up_line && (line_state[UP_LINE] == LINE_ASSERTED)) {
             goto out;
      } else {
		 ignore_up_line = 0;
                 PUSHED1("UP", FIVEWAY_KEYCODE_UP, fiveway_hold_timeout,
                            FIVEWAY_UP_IRQ, UP_LINE);
                 enable_other_irqs(irq, 0);
      }
   }
   else if ( !disabled_down_irq && (fiveway_datain(DOWN_LINE) == 0) && (irq == FIVEWAY_DOWN_IRQ)) {
      if (ignore_down_line && (line_state[DOWN_LINE] == LINE_ASSERTED)) {
	   goto out;
      } else {
		 ignore_down_line = 0;
                  PUSHED1("DOWN", FIVEWAY_KEYCODE_DOWN, fiveway_hold_timeout,
                     FIVEWAY_DOWN_IRQ, DOWN_LINE);
                  enable_other_irqs(irq, 0);
      }
   }
   else if ( !disabled_left_irq && (fiveway_datain(LEFT_LINE) == 0) && (irq == FIVEWAY_LEFT_IRQ)) {
      if (ignore_left_line && (line_state[LEFT_LINE] == LINE_ASSERTED)) {
	   goto out;
      } else {
		  ignore_left_line = 0;
                  PUSHED1("LEFT", FIVEWAY_KEYCODE_LEFT, fiveway_hold_timeout,
                     FIVEWAY_LEFT_IRQ, LEFT_LINE);
                  enable_other_irqs(irq, 0);
      }
   }
   else if ( !disabled_right_irq && (fiveway_datain(RIGHT_LINE) == 0) && (irq == FIVEWAY_RIGHT_IRQ)) {
      if (ignore_right_line && (line_state[RIGHT_LINE] == LINE_ASSERTED)) {
		goto out;
      } else {
		 ignore_right_line = 0;
                 PUSHED1("RIGHT", FIVEWAY_KEYCODE_RIGHT, fiveway_hold_timeout,
                    FIVEWAY_RIGHT_IRQ, RIGHT_LINE);
                 enable_other_irqs(irq, 0);
      }
   }
   else if ( !disabled_select_irq && (fiveway_datain(SELECT_LINE) == 0) && (irq == FIVEWAY_SELECT_IRQ)) {
      if (ignore_select_line && (line_state[SELECT_LINE] == LINE_ASSERTED)) {
		 goto out;
      } else {
		 ignore_select_line = 0;
                PUSHED1("SELECT", FIVEWAY_KEYCODE_SELECT, SELECT_hold_timeout,
                  FIVEWAY_SELECT_IRQ, SELECT_LINE)
                enable_other_irqs(irq, 0);
      }
   }

   if (( fiveway_datain(UP_LINE) == 1 ) &&
       ( line_state[UP_LINE] != LINE_DEASSERTED ) && (irq == FIVEWAY_UP_IRQ)) {
      cancel_delayed_work(&fiveway_repeat_up_tq_d);
      RELEASED("UP", FIVEWAY_KEYCODE_UP, UP_LINE, FIVEWAY_UP_IRQ);
      enable_other_irqs(irq, 1);
      ignore_up_line = 0;
   }
   else if (( fiveway_datain(DOWN_LINE) == 1 ) &&
            ( line_state[DOWN_LINE] != LINE_DEASSERTED ) && (irq == FIVEWAY_DOWN_IRQ)) {
      cancel_delayed_work(&fiveway_repeat_down_tq_d);
      RELEASED("DOWN", FIVEWAY_KEYCODE_DOWN, DOWN_LINE, FIVEWAY_DOWN_IRQ);
      enable_other_irqs(irq, 1);
      ignore_down_line = 0;
   }
   else if (( fiveway_datain(LEFT_LINE) == 1 ) &&
            ( line_state[LEFT_LINE] != LINE_DEASSERTED ) && (irq == FIVEWAY_LEFT_IRQ)) {
      cancel_delayed_work(&fiveway_repeat_left_tq_d);
      RELEASED("LEFT", FIVEWAY_KEYCODE_LEFT, LEFT_LINE, FIVEWAY_LEFT_IRQ);
      enable_other_irqs(irq, 1);
      ignore_left_line = 0;
   }
   else if (( fiveway_datain(RIGHT_LINE) == 1 ) &&
            ( line_state[RIGHT_LINE] != LINE_DEASSERTED ) && (irq == FIVEWAY_RIGHT_IRQ)) {
       cancel_delayed_work(&fiveway_repeat_right_tq_d);
       RELEASED("RIGHT", FIVEWAY_KEYCODE_RIGHT, RIGHT_LINE, FIVEWAY_RIGHT_IRQ);
       enable_other_irqs(irq, 1);
       ignore_right_line = 0;
   }
   else if (( fiveway_datain(SELECT_LINE) == 1 ) &&
            ( line_state[SELECT_LINE] != LINE_DEASSERTED ) && (irq == FIVEWAY_SELECT_IRQ))  {
       cancel_delayed_work(&fiveway_repeat_select_tq_d);
       RELEASED("SELECT", FIVEWAY_KEYCODE_SELECT, SELECT_LINE, FIVEWAY_SELECT_IRQ);
       enable_other_irqs(irq, 1);
      ignore_select_line = 0;
   }

   // Start the debounce delayed work item, to avoid spurious signals
   LLOG_DEBUG1("Starting debounce delayed work item...\n");

   disable_irq_nosync(irq);

   if (irq == FIVEWAY_UP_IRQ) {
	schedule_delayed_work(&fiveway_up_debounce_tq_d, ((debounce_timeout * HZ) / 1000));
	disabled_up = 1;
   }

   if (irq == FIVEWAY_DOWN_IRQ) {
	schedule_delayed_work(&fiveway_down_debounce_tq_d, ((debounce_timeout * HZ) / 1000));
	disabled_down = 1;
   }

   if (irq == FIVEWAY_LEFT_IRQ) {
 	schedule_delayed_work(&fiveway_left_debounce_tq_d, ((debounce_timeout * HZ) / 1000));
	disabled_left = 1;
   }

   if (irq == FIVEWAY_RIGHT_IRQ) {
	schedule_delayed_work(&fiveway_right_debounce_tq_d, ((debounce_timeout * HZ) / 1000));
	disabled_right = 1;
   }

   if (irq == FIVEWAY_SELECT_IRQ) {
	schedule_delayed_work(&fiveway_select_debounce_tq_d, ((debounce_timeout * HZ) / 1000));
	disabled_select = 1;
   }

out:
   return IRQ_HANDLED;
}

/*!
 * This function puts the 5-Way driver in off state
 */
void fiveway_off(void)
{
   LLOG_DEBUG0("Stopping work items; disabling IRQs and pullups...\n");
   atomic_set(&fiveway_is_locked, 1);

   disable_irq(FIVEWAY_RIGHT_IRQ);
   disable_irq(FIVEWAY_UP_IRQ);
   disable_irq(FIVEWAY_DOWN_IRQ);
   disable_irq(FIVEWAY_LEFT_IRQ);
   if( ! fiveway_wake_enable )
      disable_irq(FIVEWAY_SELECT_IRQ);
}


/*!
 * Suspend function
 * @param   pdev  the device structure used to give information on 5-Way
 *                to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int fiveway_suspend(struct platform_device *pdev, pm_message_t state)
{
   /*
    * NOTE: in the Lab126 platform architecture, we always go to
    * the "locked" state before going to suspend; therefore, all
    * fiveway locking actions are in the fiveway_off function, above.
    */

   return 0;
}


/*!
 * This function brings the 5-Way driver back from low-power state,
 * by re-enabling the line interrupts.
 *
 * @return  The function always returns 0.
 */
void fiveway_on(void)
{
   LLOG_DEBUG0("Enabling GPIOs and IRQs...\n");

   if (fiveway_datain(UP_LINE) == 0) {
	line_state[UP_LINE] = LINE_ASSERTED;
	set_irq_type(FIVEWAY_UP_IRQ, !fiveway_tequila ? IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING);
   }
   else {
	line_state[UP_LINE] = LINE_DEASSERTED;
	set_irq_type(FIVEWAY_UP_IRQ, !fiveway_tequila ? IRQF_TRIGGER_FALLING:IRQF_TRIGGER_RISING);
   }

   if (fiveway_datain(DOWN_LINE) == 0) {
        line_state[DOWN_LINE] = LINE_ASSERTED;
        set_irq_type(FIVEWAY_DOWN_IRQ, !fiveway_tequila ? IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING);
   }
   else {
        line_state[DOWN_LINE] = LINE_DEASSERTED;
        set_irq_type(FIVEWAY_DOWN_IRQ, !fiveway_tequila ? IRQF_TRIGGER_FALLING:IRQF_TRIGGER_RISING);
   }

  if (fiveway_datain(LEFT_LINE) == 0) {
        line_state[LEFT_LINE] = LINE_ASSERTED;
        set_irq_type(FIVEWAY_LEFT_IRQ, !fiveway_tequila ? IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING); 
   }
   else {
        line_state[LEFT_LINE] = LINE_DEASSERTED;
        set_irq_type(FIVEWAY_LEFT_IRQ, !fiveway_tequila ? IRQF_TRIGGER_FALLING:IRQF_TRIGGER_RISING);
   }

  if (fiveway_datain(RIGHT_LINE) == 0) {
        line_state[RIGHT_LINE] = LINE_ASSERTED;
        set_irq_type(FIVEWAY_RIGHT_IRQ, !fiveway_tequila ? IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING);
   }
   else {
        line_state[RIGHT_LINE] = LINE_DEASSERTED;
        set_irq_type(FIVEWAY_RIGHT_IRQ, !fiveway_tequila ? IRQF_TRIGGER_FALLING:IRQF_TRIGGER_RISING);
   }

   if (fiveway_datain(SELECT_LINE) == 0) {
        line_state[SELECT_LINE] = LINE_ASSERTED;
        set_irq_type(FIVEWAY_SELECT_IRQ, !fiveway_tequila ? IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING);
   }
   else {
        line_state[SELECT_LINE] = LINE_DEASSERTED;
        set_irq_type(FIVEWAY_SELECT_IRQ, !fiveway_tequila ? IRQF_TRIGGER_FALLING:IRQF_TRIGGER_RISING);
   }

   enable_irq(FIVEWAY_UP_IRQ);
   enable_irq(FIVEWAY_DOWN_IRQ);
   enable_irq(FIVEWAY_LEFT_IRQ);
   enable_irq(FIVEWAY_RIGHT_IRQ);
   if( ! fiveway_wake_enable )
      enable_irq(FIVEWAY_SELECT_IRQ);

   atomic_set(&fiveway_is_locked, 0);
   return;
}


/*!
 * This function brings the 5-Way driver back from low-power state,
 * by re-enabling the line interrupts.
 *
 * @param   pdev  the device structure used to give information on Keypad
 *                to resume
 *
 * @return  The function always returns 0.
 */
static int fiveway_resume(struct platform_device *pdev)
{
   /*
    * NOTE: in the Lab126 platform architecture, we resume functionality
    * by going to the "unlocked" state, not automatically on system resume.
    * Therefore, the fiveway unlocking actions are in the fiveway_on function,
    * above.
    */

   return 0;
}


/*!
 * This function is called when the fiveway driver is opened.
 * Since fiveway initialization is done in __init, nothing is done in open.
 *
 * @param    dev    Pointer to device inode
 *
 * @result    The function always returns 0
 */
static int fiveway_open(struct input_dev *dev)
{
        return 0;
}


/*!
 * This function is called close the fiveway device.
 * Nothing is done in this function, since every thing is taken care in
 * __exit function.
 *
 * @param    dev    Pointer to device inode
 *
 */
static void fiveway_close(struct input_dev *dev)
{
}


/*
 * This function returns the current state of the fiveway device
 */
int fiveway_proc_fiveway_read( char *page, char **start, off_t off,
                               int count, int *eof, void *data )
{
   int len;

   if (off > 0) {
     *eof = 1;
     return 0;
   }
   if (fiveway_lock == 0) {
      len = sprintf(page, "fiveway is unlocked\n");
   }
   else {
      len = sprintf(page, "fiveway is locked\n");
   }

   return len;
}


/*
 * This function executes fiveway control commands based on the input string.
 *    unlock = unlock the fiveway
 *    lock   = lock the fiveway
 *    send   = send a fiveway event, simulating a line detection
 *    debug  = set the debug level for logging messages (0=off)
 */
int fiveway_proc_fiveway_write( struct file *filp, const char __user *buff,
                                unsigned long len, void *data )

{
   char command[PROC_FIVEWAY_CMD_LEN];
   int  keycode;

   if (len > PROC_FIVEWAY_CMD_LEN) {
      LLOG_ERROR("proc_len", "", "Fiveway command is too long!\n");
      return -ENOSPC;
   }

   if (copy_from_user(command, buff, len)) {
      return -EFAULT;
   }

   if ( !strncmp(command, "unlock", 6) ) {
      // Requested to unlock fiveway
      if (fiveway_lock == 0) {
         // Fiveway was already unlocked; do nothing
         LLOG_INFO("unlock1", "status=unlocked", "\n");
      }
      else {
         // Unlock fiveway
         fiveway_on();
         fiveway_lock = 0;
         LLOG_INFO("unlock2", "status=unlocked", "\n");
      }
   }
   else if ( !strncmp(command, "lock", 4) ) {
      // Requested to lock fiveway
      if (fiveway_lock != 0) {
         // Fiveway was already locked; do nothing
         LLOG_INFO("locked1", "status=locked", "\n");
      }
      else {
         // Lock fiveway
	 fiveway_lock = 1;
         fiveway_off();
         LLOG_INFO("locked2", "status=locked", "\n");
      }
   }
   else if ( !strncmp(command, "send", 4) ) {
      // Requested to send a keycode
      if (fiveway_lock != 0) {
         // Keypad is locked; do nothing
         LLOG_WARN("locked3", "", "The fiveway device is locked!  Unlock before sending keycodes.\n");
      }
      else {
         // Read keycode and send
         sscanf(command, "send %d", &keycode);
         LLOG_INFO("send_key", "", "Sending keycode %d\n", keycode);
         report_event(fiveway_dev, EV_KEY, keycode, 1);
         report_event(fiveway_dev, EV_KEY, keycode, 0);
      }
   }
   else if ( !strncmp(command, "debug", 5) ) {
      // Requested to set debug level
      sscanf(command, "debug %d", &debug);
      LLOG_INFO("debug", "", "Setting debug level to %d\n", debug);
      set_debug_log_mask(debug);
   }
   else {
      LLOG_ERROR("proc_cmd", "", "Unrecognized fiveway command\n");
   }

   return len;
}


int
fiveway_proc_fiveway_we_read(
	char *page,
	char **start,
	off_t off,
	int len,
	int *eof,
	void *data)
{
	*eof = 1;

	return sprintf(page, "%d\n", fiveway_wake_enable);
}


int
fiveway_proc_fiveway_we_write(
	struct file *filp,
	const char __user *buf,
	unsigned long len,
	void *data)
{
	int enable;
	char ch;

	if (copy_from_user(&ch, buf, sizeof(ch))) {
		return -EFAULT;
	}

	enable = ch != '0';

	if (fiveway_wake_enable != enable) {
		LLOG_INFO("wake_enable", "", "enable=%d\n", enable);

		if (atomic_read(&fiveway_is_locked) ) {
			if( enable )
			   enable_irq(FIVEWAY_SELECT_IRQ);
			else
			   disable_irq(FIVEWAY_SELECT_IRQ);
		}
		set_irq_wake(FIVEWAY_SELECT_IRQ, fiveway_wake_enable = enable);
	}

        return len;
}


static int show_fiveway_dbg_state(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *curr = buf;

	curr += sprintf(curr, "fiveway interrupt state: left-%d, right-%d, up-%d, down-%d, select-%d\n",
			disabled_left_irq, disabled_right_irq, disabled_up_irq, disabled_down_irq, disabled_select_irq);

	curr+= sprintf(curr, "fiveway ignore state: left-%d, right-%d, up-%d, down-%d, select-%d\n",
			ignore_left_line, ignore_right_line, ignore_up_line, ignore_down_line, ignore_select_line);

	curr+= sprintf(curr, "fiveway pin state: left-%d, right-%d, up-%d, down-%d, select-%d\n",
			fiveway_datain(LEFT_LINE), fiveway_datain(RIGHT_LINE), fiveway_datain(UP_LINE),
			fiveway_datain(DOWN_LINE), fiveway_datain(SELECT_LINE));

	curr+= sprintf(curr, "fiveway assert/de-assert state: left-%d, right-%d, up-%d, down-%d, select-%d\n",
			line_state[LEFT_LINE], line_state[RIGHT_LINE], line_state[UP_LINE], line_state[DOWN_LINE],
			line_state[SELECT_LINE]);

	curr += sprintf(curr, "\n");
	
	return curr - buf;
}
static DEVICE_ATTR(fiveway_dbg_state, 0666, show_fiveway_dbg_state, NULL);

static int show_fiveway_repeat_time(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *curr = buf;

	curr += sprintf(curr, "%d\n", fiveway_repeat_timeout);
	curr += sprintf(curr, "\n");

	return curr - buf;
}

static int store_fiveway_repeat_time(struct device *_dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int value;

	if ( (sscanf(buf, "%d", &value) > 0) ) {
		fiveway_repeat_timeout = value;
		return strlen(buf);
	}

	return -EINVAL;
}
static DEVICE_ATTR (fiveway_repeat_time, S_IRUGO|S_IWUSR, show_fiveway_repeat_time, store_fiveway_repeat_time);

static int show_fiveway_hold_time(struct device *dev, struct device_attribute *attr, char *buf)
{
	char *curr = buf;

	curr += sprintf(curr, "%d\n", fiveway_hold_timeout);
	curr += sprintf(curr, "\n");

	return curr - buf;
}

static int store_fiveway_hold_time(struct device *_dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int value;

	if ( (sscanf(buf, "%d", &value) > 0) ) {
		fiveway_hold_timeout = value;
		return strlen(buf);
	}

	return -EINVAL;
}
static DEVICE_ATTR (fiveway_hold_time, S_IRUGO|S_IWUSR, show_fiveway_hold_time, store_fiveway_hold_time);

static int store_fiveway_hold_fake_event(struct device *_dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	kobject_uevent(&fiveway_dev->dev.kobj, KOBJ_ADD);
	return strlen(buf);
}
	
static DEVICE_ATTR (fiveway_hold_fake_event, S_IRUGO|S_IWUSR, NULL, store_fiveway_hold_fake_event);

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
static int __devinit fiveway_probe(struct platform_device *pdev)
{
   int err = 0;
   int i;

   LLOG_INFO("probe0", "", "Starting...\n");

   /* Create proc entry for "fiveway" */
   proc_fiveway_entry = create_proc_entry(FIVEWAY_PROC_FIVEWAY, S_IWUSR | S_IRUGO, NULL);
   if (proc_fiveway_entry == NULL) {
      LLOG_ERROR("probe1", "", "Fiveway could not create proc entry\n");
      err = -ENOMEM;
      goto exit_probe;
   } else {
      proc_fiveway_entry->read_proc = fiveway_proc_fiveway_read;
      proc_fiveway_entry->write_proc = fiveway_proc_fiveway_write;
   }

   /* Create proc entry for "fiveway_we" */
   proc_fiveway_we_entry = create_proc_entry(FIVEWAY_PROC_FIVEWAY_WE, S_IWUSR | S_IRUGO, NULL);
   if (proc_fiveway_we_entry == NULL) {
      LLOG_ERROR("probe1", "", "Fiveway could not create proc wake enable entry\n");
      err = -ENOMEM;
      remove_proc_entry(FIVEWAY_PROC_FIVEWAY, NULL);
      goto exit_probe;
   } else {
      proc_fiveway_we_entry->read_proc = fiveway_proc_fiveway_we_read;
      proc_fiveway_we_entry->write_proc = fiveway_proc_fiveway_we_write;
   }

   if (mx50_board_is(BOARD_ID_TEQUILA))
      fiveway_tequila = 1;

   // Set up the necessary GPIO lines for the 5-Way inputs
   err = gpio_fiveway_active();
   if (err) {
      LLOG_ERROR("gpio_fail", "", "Failed to set GPIO lines\n");
      goto del_proc_entries;
   }

   // Initialize the line state array
   for (i = 0; i < NUM_OF_LINES; i++) {
      line_state[i] = LINE_DEASSERTED;
   }

   // Set IRQ for fiveway GPIO lines; trigger on falling edges
   // and register the IRQ service routines

   // UP IRQ
   set_irq_type(FIVEWAY_UP_IRQ, fiveway_tequila ? IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING);
   err = request_irq(FIVEWAY_UP_IRQ, (irq_handler_t) fiveway_irq, 0, "fiveway", NULL);
   if (err != 0) {
      LLOG_ERROR("up_irq", "", "UP IRQ line = 0x%X; request status = %d\n", FIVEWAY_UP_IRQ, err);
      goto release_gpios;
   }

   // DOWN IRQ
   set_irq_type(FIVEWAY_DOWN_IRQ, fiveway_tequila ? IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING);
   err = request_irq(FIVEWAY_DOWN_IRQ, (irq_handler_t) fiveway_irq, 0, "fiveway", NULL);
   if (err != 0) {
      LLOG_ERROR("down_irq", "", "DOWN IRQ line = 0x%X; request status = %d\n", FIVEWAY_DOWN_IRQ, err);
      goto release_up_irq;
   }

   // LEFT IRQ
   set_irq_type(FIVEWAY_LEFT_IRQ, fiveway_tequila ? IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING);
   err = request_irq(FIVEWAY_LEFT_IRQ, (irq_handler_t) fiveway_irq, 0, "fiveway", NULL);
   if (err != 0) {
      LLOG_ERROR("left_irq", "", "LEFT IRQ line = 0x%X; request status = %d\n", FIVEWAY_LEFT_IRQ, err);
      goto release_down_irq;
   }

   // RIGHT IRQ
   set_irq_type(FIVEWAY_RIGHT_IRQ, fiveway_tequila ? IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING);
   err = request_irq(FIVEWAY_RIGHT_IRQ, (irq_handler_t) fiveway_irq, 0, "fiveway", NULL);
   if (err != 0) {
      LLOG_ERROR("right_irq", "", "RIGHT IRQ line = 0x%X; request status = %d\n", FIVEWAY_RIGHT_IRQ, err);
      goto release_left_irq;
   }

   // SELECT IRQ
   set_irq_type(FIVEWAY_SELECT_IRQ, fiveway_tequila ? IRQF_TRIGGER_RISING:IRQF_TRIGGER_FALLING);
   err = request_irq(FIVEWAY_SELECT_IRQ, (irq_handler_t) fiveway_irq, 0, "fiveway", NULL);
   if (err != 0) {
      LLOG_ERROR("select_irq", "", "SELECT IRQ line = 0x%X; request status = %d\n", FIVEWAY_SELECT_IRQ, err);
      goto release_right_irq;
   }

   LLOG_INFO("probe_done", "", "GPIOs and IRQs have been set up\n");

   // Set up the device file
   fiveway_dev = input_allocate_device();
   if (!fiveway_dev) {
      LLOG_ERROR("allocate_dev", "", "Not enough memory for input device\n");
      err = -ENOMEM;
      goto release_select_irq;
   }
   fiveway_dev->name = "fiveway";
   fiveway_dev->id.bustype = BUS_HOST;
   fiveway_dev->open = fiveway_open;
   fiveway_dev->close = fiveway_close;
   fiveway_dev->evbit[0] = BIT(EV_KEY);
   // Set up the keycodes sent by each of the 5-Way lines
   set_bit(FIVEWAY_KEYCODE_UP, fiveway_dev->keybit);
   set_bit(FIVEWAY_KEYCODE_DOWN, fiveway_dev->keybit);
   set_bit(FIVEWAY_KEYCODE_LEFT, fiveway_dev->keybit);
   set_bit(FIVEWAY_KEYCODE_RIGHT, fiveway_dev->keybit);
   set_bit(FIVEWAY_KEYCODE_SELECT, fiveway_dev->keybit);

   // Initialize work items
   INIT_DELAYED_WORK(&fiveway_up_debounce_tq_d, fiveway_up_debounce_wk);
   INIT_DELAYED_WORK(&fiveway_select_debounce_tq_d, fiveway_select_debounce_wk);
   INIT_DELAYED_WORK(&fiveway_right_debounce_tq_d, fiveway_right_debounce_wk);
   INIT_DELAYED_WORK(&fiveway_left_debounce_tq_d, fiveway_left_debounce_wk);
   INIT_DELAYED_WORK(&fiveway_down_debounce_tq_d, fiveway_down_debounce_wk);
   INIT_DELAYED_WORK(&fiveway_repeat_up_tq_d, fiveway_repeat_up_work);
   INIT_DELAYED_WORK(&fiveway_repeat_select_tq_d, fiveway_repeat_select_work);
   INIT_DELAYED_WORK(&fiveway_repeat_right_tq_d, fiveway_repeat_right_work);
   INIT_DELAYED_WORK(&fiveway_repeat_left_tq_d, fiveway_repeat_left_work);
   INIT_DELAYED_WORK(&fiveway_repeat_down_tq_d, fiveway_repeat_down_work);
   
   err = input_register_device(fiveway_dev);
   if (err) {
      LLOG_ERROR("reg_dev", "", "Failed to register device\n");
      goto free_input_dev;
   }

   if (device_create_file(&pdev->dev, &dev_attr_fiveway_dbg_state) < 0)
	printk(KERN_ERR "Error - could not create fiveway_dbg_state file\n");

   if (device_create_file(&pdev->dev, &dev_attr_fiveway_repeat_time) < 0)
	printk(KERN_ERR "Error - could not create fiveway_debounce_time file\n");

   if (device_create_file(&pdev->dev, &dev_attr_fiveway_hold_time) < 0)
	printk(KERN_ERR "Error - could not create fiveway_hold_time file\n");

   if (device_create_file(&pdev->dev, &dev_attr_fiveway_hold_fake_event) < 0)
	 printk(KERN_ERR "Error - could not create fiveway_hold_fake_event file\n");

   fiveway_initialized = 1;

   return 0;  // Success!


free_input_dev:
   input_free_device(fiveway_dev);
release_select_irq:
   free_irq(FIVEWAY_SELECT_IRQ, NULL);
release_right_irq:
   free_irq(FIVEWAY_RIGHT_IRQ, NULL);
release_left_irq:
   free_irq(FIVEWAY_LEFT_IRQ, NULL);
release_down_irq:
   free_irq(FIVEWAY_DOWN_IRQ, NULL);
release_up_irq:
   free_irq(FIVEWAY_UP_IRQ, NULL);
release_gpios:
   gpio_fiveway_inactive();
del_proc_entries:
   remove_proc_entry(FIVEWAY_PROC_FIVEWAY, NULL);
   remove_proc_entry(FIVEWAY_PROC_FIVEWAY_WE, NULL);
exit_probe:
   return err;
}


/*!
 * Dissociates the driver from the fiveway device.
 *
 * @param   pdev  the device structure used to give information on which SDHC
 *                to remove
 *
 * @return  The function always returns 0.
 */
static int __devexit fiveway_remove(struct platform_device *pdev)
{
   // Release IRQs and GPIO pins; disable pullups on output GPIOs
   free_irq(FIVEWAY_UP_IRQ, NULL);
   free_irq(FIVEWAY_DOWN_IRQ, NULL);
   free_irq(FIVEWAY_LEFT_IRQ, NULL);
   free_irq(FIVEWAY_RIGHT_IRQ, NULL);
   free_irq(FIVEWAY_SELECT_IRQ, NULL);
   gpio_fiveway_inactive();

   device_remove_file(&pdev->dev, &dev_attr_fiveway_dbg_state);
   device_remove_file(&pdev->dev, &dev_attr_fiveway_repeat_time);
   device_remove_file(&pdev->dev, &dev_attr_fiveway_hold_time);
   device_remove_file(&pdev->dev, &dev_attr_fiveway_hold_fake_event);

   // Stop any pending delayed work items
   cancel_delayed_work_sync(&fiveway_up_debounce_tq_d);
   cancel_delayed_work_sync(&fiveway_right_debounce_tq_d);
   cancel_delayed_work_sync(&fiveway_left_debounce_tq_d);
   cancel_delayed_work_sync(&fiveway_select_debounce_tq_d);
   cancel_delayed_work_sync(&fiveway_down_debounce_tq_d);
   cancel_delayed_work_sync(&fiveway_repeat_up_tq_d);
   cancel_delayed_work_sync(&fiveway_repeat_left_tq_d);
   cancel_delayed_work_sync(&fiveway_repeat_right_tq_d);
   cancel_delayed_work_sync(&fiveway_repeat_select_tq_d);
   cancel_delayed_work_sync(&fiveway_repeat_down_tq_d);

   remove_proc_entry(FIVEWAY_PROC_FIVEWAY, NULL);
   remove_proc_entry(FIVEWAY_PROC_FIVEWAY_WE, NULL);
   input_unregister_device(fiveway_dev);

   LLOG_INFO("exit", "", "IRQs released and work functions stopped.\n");

   return 0;
}


/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver fiveway_driver = {
        .driver = {
                   .name = "fiveway",
                   .owner  = THIS_MODULE,
                   },
        .suspend = fiveway_suspend,
        .resume = fiveway_resume,
        .probe = fiveway_probe,
        .remove = fiveway_remove,
};

static void fiveway_device_release (struct device *dev)
{
	/* Do Nothing */
}

static struct platform_device fiveway_device = {
	.name = "fiveway",
	.id = -1,
	.dev = {
		.release = fiveway_device_release,
	       },
};

/*!
 * This function is called for module initialization.
 * It registers the fiveway char driver.
 *
 * @return      0 on success and a non-zero value on failure.
 */
static int __init fiveway_init(void)
{
	LLOG_INIT();
	set_debug_log_mask(debug);
	platform_device_register(&fiveway_device);
        platform_driver_register(&fiveway_driver);
        LLOG_INFO("drv", "", "Fiveway driver loaded\n");
        return 0;
}

/*!
 * This function is called whenever the module is removed from the kernel. It
 * unregisters the fiveway driver from kernel.
 */
static void __exit fiveway_exit(void)
{
        platform_driver_unregister(&fiveway_driver);
	platform_device_unregister(&fiveway_device);
}

module_init(fiveway_init);
module_exit(fiveway_exit);

MODULE_AUTHOR("Lab126");
MODULE_DESCRIPTION("5-Way Input Device Driver");
MODULE_LICENSE("GPL");
