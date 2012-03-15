/*
 * Volume Keys driver routine for Yoshi.
 * Copyright (C) 2010-2011 Amazon Technologies, Inc. All Rights Reserved.
 * Manish Lachwani (lachwani@lab126.com).
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/syscalls.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <mach/irqs.h>
#include <asm/segment.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/delay.h>

#include <mach/pmic_power.h>
#include <mach/gpio.h>

static unsigned int logging_mask;
#define LLOG_G_LOG_MASK logging_mask
#define LLOG_KERNEL_COMP "volume"

#include <llog.h>
#include "volume.h"

static struct input_dev *volume_dev = NULL;
static char line_state[NUM_OF_LINES];

static struct delayed_work volume_debounce_tq_d;
static struct delayed_work volume_auto_repeat_tq_d;
static void volume_debounce_wk(struct work_struct *);
static void volume_auto_repeat_wk(struct work_struct *);

#define VOLUME_PROC_FILE "volume"
#define PROC_CMD_LEN 50
static struct proc_dir_entry *proc_entry;


#define DEBOUNCE_TIMEOUT_DEFAULT 60
static int debounce_timeout = DEBOUNCE_TIMEOUT_DEFAULT;
module_param(debounce_timeout, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(debounce_timeout, "Time for debouncing the volume switch, in msec; MUST be a multiple of 10 (default is 60)");

#define HOLD_TIMEOUT_DEFAULT 500
static int volume_hold_timeout = HOLD_TIMEOUT_DEFAULT;
module_param(volume_hold_timeout, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(volume_hold_timeout, "Time before holding volume key starts auto-repeat mode, in msec; MUST be a multiple of 10 (default is 500; 0 = no auto-repeat)");

#define AUTO_REPEAT_TIMEOUT_DEFAULT 300
static int volume_auto_repeat_time = AUTO_REPEAT_TIMEOUT_DEFAULT;
module_param(volume_auto_repeat_time, int, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(volume_auto_repeat_time, "Time between volume key auto-repeat events; MUST be a multiple of 10 (default is 300; 0 = no auto-repeat)");

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
      default: ;
   }
}


/*
 * Report the volume event.  In addition to calling
 * input_event, this function sends a uevent for any
 * processes listening in the userspace
 */
static void report_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
   input_event( dev, type, code, value );

   if (value == 1) {
      schedule();
      if( kobject_uevent( &dev->dev.kobj, KOBJ_CHANGE ) )
      {
          LLOG_ERROR("uevent", "", "Volume driver failed to send uevent\n" );
      }
   }
}

#define AUTO_RPT(dbgstr, keycode, time, line) \
      LLOG_DEBUG0("Got Volume Key repeat timeout: %s\n", dbgstr); \
      report_event(volume_dev, EV_KEY, keycode, 2); \
      timeout = time; \
      line_state[line] = LINE_HELD; \
      done = false;

extern int volume_datain(int);

static void volume_auto_repeat_wk(struct work_struct *work)
{
   int timeout = 0;
   bool done = true;

   LLOG_DEBUG0("Got auto-repeat timeout\n");
   if ( volume_datain(VOL_UP_LINE) == 0 ) {
      AUTO_RPT("VOL_UP", VOL_UP_KEYCODE, volume_auto_repeat_time, VOL_UP_LINE);
   }
   if ( volume_datain(VOL_DOWN_LINE) == 0 ) {
      AUTO_RPT("VOL_DOWN", VOL_DOWN_KEYCODE, volume_auto_repeat_time, VOL_DOWN_LINE);
   }
   if (done) {
      return;
   }

   schedule_delayed_work(&volume_auto_repeat_tq_d, ((timeout * HZ) / 1000));
}


#define PUSHED(dbgstr, keycode, time, irq, line) \
      LLOG_DEBUG0("Got Volume signal push: %s\n", dbgstr); \
      report_event(volume_dev, EV_KEY, keycode, 1); \
      timeout = time; \
      set_irq_type(irq, IRQF_TRIGGER_FALLING); \
      line_state[line] = LINE_ASSERTED;

#define RELEASED(dbgstr, keycode, line, irq) \
      LLOG_DEBUG0("Got Volume signal release: %s\n", dbgstr); \
      report_event(volume_dev, EV_KEY, keycode, 0); \
      line_state[line] = LINE_DEASSERTED; \
      set_irq_type(irq, IRQF_TRIGGER_RISING);


static void volume_debounce_wk(struct work_struct *work)
{
   int timeout = 0;

   LLOG_DEBUG1("Debounce period expired\n");
   if ( volume_datain(VOL_UP_LINE) == 0 ) {
      PUSHED("VOL_UP", VOL_UP_KEYCODE, volume_hold_timeout,
             VOL_UP_IRQ, VOL_UP_LINE);
   }
   if ( volume_datain(VOL_DOWN_LINE) == 0 ) {
      PUSHED("VOL_DOWN", VOL_DOWN_KEYCODE, volume_hold_timeout,
             VOL_DOWN_IRQ, VOL_DOWN_LINE);
   }

   if (( volume_datain(VOL_UP_LINE) == 1 ) &&
       ( line_state[VOL_UP_LINE] != LINE_DEASSERTED )) {
      RELEASED("VOL_UP", VOL_UP_KEYCODE, VOL_UP_LINE, VOL_UP_IRQ);
   }
   if (( volume_datain(VOL_DOWN_LINE) == 1 ) &&
       ( line_state[VOL_DOWN_LINE] != LINE_DEASSERTED )) {
      RELEASED("VOL_DOWN", VOL_DOWN_KEYCODE, VOL_DOWN_LINE, VOL_DOWN_IRQ);
   }

   if (timeout > 0) {
      schedule_delayed_work(&volume_auto_repeat_tq_d, ((timeout * HZ) / 1000));
   }
}


static irqreturn_t volume_irq (int irq, void *data, struct pt_regs *r)
{
   if (delayed_work_pending(&volume_debounce_tq_d)) {
      LLOG_DEBUG0("Ignoring Volume Switch signal while debouncing switch\n");
      return IRQ_HANDLED;
   }

   LLOG_DEBUG1("Starting debounce delayed work item...\n");
   schedule_delayed_work(&volume_debounce_tq_d, ((debounce_timeout * HZ) / 1000));

   return IRQ_HANDLED;
}


/*!
 * This function puts the Volume key driver in low-power mode/state,
 * by disabling interrupts from the input lines.
 */
void volume_off(void)
{
   LLOG_DEBUG0("Stopping work items; disabling IRQs and pullups...\n");

   disable_irq(VOL_UP_IRQ);
   disable_irq(VOL_DOWN_IRQ);
   gpio_volume_inactive();

   cancel_delayed_work_sync(&volume_debounce_tq_d);
   cancel_delayed_work_sync(&volume_auto_repeat_tq_d);
}


/*!
 * Suspend function
 * @param   pdev  the device structure used to give information on Volume drv
 *                to suspend
 * @param   state the power state the device is entering
 *
 * @return  The function always returns 0.
 */
static int volume_suspend(struct platform_device *pdev, pm_message_t state)
{
   volume_off();

   return 0;
}


/*!
 * This function brings the Volume key driver back from low-power state,
 * by re-enabling the line interrupts.
 *
 * @return  The function always returns 0.
 */
void volume_on(void)
{
   LLOG_DEBUG0("Enabling GPIOs and IRQs...\n");

   gpio_volume_active();
   enable_irq(VOL_UP_IRQ);
   enable_irq(VOL_DOWN_IRQ);
}


/*!
 * This function brings the Volume key driver back from low-power state,
 * by re-enabling the line interrupts.
 *
 * @param   pdev  the device structure used to give information on Keypad
 *                to resume
 *
 * @return  The function always returns 0.
 */
static int volume_resume(struct platform_device *pdev)
{
   volume_on();

   return 0;
}

/*!
 * This function is called when the volume driver is opened.
 * Since volume initialization is done in __init, nothing is done in open.
 *
 * @param    dev    Pointer to device inode
 *
 * @result    The function always returns 0
 */
static int volume_open(struct input_dev *dev)
{
        return 0;
}

/*!
 * This function is called close the volume device.
 * Nothing is done in this function, since every thing is taken care in
 * __exit function.
 *
 * @param    dev    Pointer to device inode
 *
 */
static void volume_close(struct input_dev *dev)
{
}

/*
 * This function returns the current state of the volume device
 */
int volume_proc_read( char *page, char **start, off_t off,
                      int count, int *eof, void *data )
{
   int len;

   if (off > 0) {
     *eof = 1;
     return 0;
   }
   len = sprintf(page, "volume key driver is active\n");
   return len;
}


/*
 * This function executes volume control commands based on the input string.
 *    unlock = unlock the volume
 *    lock   = lock the volume
 *    send   = send a volume event, simulating a line detection
 *    debug  = set the debug level for logging messages (0=off)
 */
ssize_t volume_proc_write( struct file *filp, const char __user *buff,
                            unsigned long len, void *data )

{
   char command[PROC_CMD_LEN];
   int  keycode;

   if (len > PROC_CMD_LEN) {
      LLOG_ERROR("proc_len", "", "Volume command is too long!\n");
      return -ENOSPC;
   }

   if (copy_from_user(command, buff, len)) {
      return -EFAULT;
   }

   if ( !strncmp(command, "send", 4) ) {
      sscanf(command, "send %d", &keycode);
      LLOG_INFO("send_key", "", "Sending keycode %d\n", keycode);
      report_event(volume_dev, EV_KEY, keycode, 1);
      report_event(volume_dev, EV_KEY, keycode, 0);
   }
   else if ( !strncmp(command, "debug", 5) ) {
      sscanf(command, "debug %d", &debug);
      LLOG_INFO("debug", "", "Setting debug level to %d\n", debug);
      set_debug_log_mask(debug);
   }
   else {
      LLOG_ERROR("proc_cmd", "", "Unrecognized volume command\n");
   }

   return len;
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
static int __devinit volume_probe(struct platform_device *pdev)
{
   int err = 0;
   int i;

   LLOG_INFO("probe0", "", "Starting...\n");

   /* Create proc entry */
   proc_entry = create_proc_entry(VOLUME_PROC_FILE, 0644, NULL);
   if (proc_entry == NULL) {
      LLOG_ERROR("probe1", "", "Volume could not create proc entry\n");
      err = -ENOMEM;
      goto exit_probe;
   } else {
      proc_entry->read_proc = volume_proc_read;
      proc_entry->write_proc = volume_proc_write;
   }

   err = gpio_volume_active();
   if (err) {
      LLOG_ERROR("gpio_fail", "", "Failed to set GPIO lines\n");
      goto del_proc_entry;
   }

   for (i = 0; i < NUM_OF_LINES; i++) {
      line_state[i] = LINE_DEASSERTED;
   }

#define SET_IRQ(irq) \
   set_irq_type(irq, IRQF_TRIGGER_RISING); \
   err = request_irq(irq, (irq_handler_t) volume_irq, 0, "volume", NULL);

   SET_IRQ(VOL_UP_IRQ);
   if (err != 0) {
      LLOG_ERROR("vol_up_irq", "", "VOL_UP IRQ line = 0x%X; request status = %d\n", VOL_UP_IRQ, err);
      goto release_gpios;
   }
   SET_IRQ(VOL_DOWN_IRQ);
   if (err != 0) {
      LLOG_ERROR("vol_down_irq", "", "VOL_DOWN IRQ line = 0x%X; request status = %d\n", VOL_DOWN_IRQ, err);
      goto release_vol_up_irq;
   }

   LLOG_INFO("probe_done", "", "GPIOs and IRQs have been set up\n");

   volume_dev = input_allocate_device();
   if (!volume_dev) {
      LLOG_ERROR("allocate_dev", "", "Not enough memory for input device\n");
      err = -ENOMEM;
      goto release_vol_down_irq;
   }
   volume_dev->name = "volume";
   volume_dev->id.bustype = BUS_HOST;
   volume_dev->open = volume_open;
   volume_dev->close = volume_close;
   volume_dev->evbit[0] = BIT(EV_KEY);

   set_bit(VOL_UP_KEYCODE, volume_dev->keybit);
   set_bit(VOL_DOWN_KEYCODE, volume_dev->keybit);

   INIT_DELAYED_WORK(&volume_debounce_tq_d, volume_debounce_wk);
   INIT_DELAYED_WORK(&volume_auto_repeat_tq_d, volume_auto_repeat_wk);

   err = input_register_device(volume_dev);
   if (err) {
      LLOG_ERROR("reg_dev", "", "Failed to register device\n");
      goto free_input_dev;
   }
   return 0; 


free_input_dev:
   input_free_device(volume_dev);
release_vol_down_irq:
   free_irq(VOL_DOWN_IRQ, NULL);
release_vol_up_irq:
   free_irq(VOL_UP_IRQ, NULL);
release_gpios:
   gpio_volume_inactive();
del_proc_entry:
   remove_proc_entry(VOLUME_PROC_FILE, NULL);
exit_probe:
   return err;
}

/*!
 * Dissociates the driver from the volume device.
 *
 * @param   pdev  the device structure used to give information on which SDHC
 *                to remove
 *
 * @return  The function always returns 0.
 */
static int __devexit volume_remove(struct platform_device *pdev)
{
   free_irq(VOL_UP_IRQ, NULL);
   free_irq(VOL_DOWN_IRQ, NULL);
   gpio_volume_inactive();

   cancel_delayed_work_sync(&volume_debounce_tq_d);
   cancel_delayed_work_sync(&volume_auto_repeat_tq_d);

   remove_proc_entry(VOLUME_PROC_FILE, NULL);
   input_unregister_device(volume_dev);

   LLOG_INFO("exit", "", "IRQs released and work functions stopped.\n");

   return 0;
}


/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver volume_driver = {
        .driver = {
                   .name = "volume",
                   .owner  = THIS_MODULE,
                   },
        .suspend = volume_suspend,
        .resume = volume_resume,
        .probe = volume_probe,
        .remove = __devexit_p(volume_remove),
};

/*!
 * This function is called for module initialization.
 * It registers the volume char driver.
 *
 * @return      0 on success and a non-zero value on failure.
 */
static int __init volume_init(void)
{
	LLOG_INIT();
	set_debug_log_mask(debug);
        platform_driver_register(&volume_driver);
        volume_probe(NULL);
        LLOG_INFO("drv", "", "Volume key driver loaded\n");
        return 0;
}

/*!
 * This function is called whenever the module is removed from the kernel. It
 * unregisters the volume driver from kernel.
 */
static void __exit volume_exit(void)
{
        volume_remove(NULL);
        platform_driver_unregister(&volume_driver);
}

module_init(volume_init);
module_exit(volume_exit);

MODULE_AUTHOR("Lab126");
MODULE_DESCRIPTION("Volume Key Input Device Driver");
MODULE_LICENSE("GPL");
