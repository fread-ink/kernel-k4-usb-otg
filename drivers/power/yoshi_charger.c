#include <linux/module.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/pmic_external.h>
#include <linux/pmic_adc.h>
#include <linux/pmic_light.h>
#include <linux/mutex.h>
#include <mach/boardid.h>

#include <mach/charger.h>

#define	DRIVER_AUTHOR	"Manish Lachwani, Christian Hoff"

#define CHRG_TIMER_THRESHOLD_TEQ       1
#define CHARGER_BOOT_THRESHOLD	4
#define FULL_BATTERY_CAPACITY	100
#define FULL_BATTERY_CURR_LIMIT	20
#define BATT_VOLTAGE_SCALE	4687	/* Scale the values from the ADC based on manual */
#define BATT_CURRENT_SCALE	5865	/* Scale the current read from the ADC */
#define CHARGE_VOLTAGE_SCALE	11700	/* Scale the charger voltage read from A/D */
#define BPSNS_MASK		0x3	/* BPSNS Mask in PMIC reg 13 */
#define BPSNS_VOLTAGE_THRESH	3600	/* 3.6V BPSNS threshold */
#define PMIC_REG_PC_0		13	/* PMIC Register 13 */
#define BPSNS_OFFSET		16	/* BPSNS OFFSET in Register 13 */
#define GREEN_LED_THRESHOLD	95	/* Turn Charge LED to Green */
#define BATTCYCL_THRESHOLD	4105	/* Restart charging if not done automatically */
#define LOW_TEMP_CHARGING	0x2	/* 240mA charging if temp < 10C */
#define CHRGLEDEN		18 	/* offset 18 in REG_CHARGE */
#define CHRGCYCLB		22	/* BATTCYCL to restart charging at 95% full */
#define CHRGRESTART		20	/* Restart charging */
#define BPON_THRESHOLD		3800
#define BPON_THRESHOLD_HI	3810
#define CHARGING_BPON		0x3	/* 320mA charging */
#define CHRGRAW_THRESHOLD	6000
#define CHGDETS  		0x40	/* Sense bit for CHGDET */
#define BVALIDS			0x10000	/* Sense bit for BVALID */

#define CHGCURRS		            0x800	/* Bit #11 */
#define CHGCURRS_THRESHOLD	        4100	/* CHGCURRS Threshold */
#define LOW_VOLTAGE_THRESHOLD		3400	/* 3.4V */
#define CRITICAL_VOLTAGE_THRESHOLD	3100	/* 3.1V */

#define BIT_CHG_CURR_LSH	3
#define BIT_CHG_CURR_WID	4

#define BIT_CHG_CURRS_LSH 	11
#define BIT_CHG_CURRS_WID	1

#define BAT_TH_CHECK_DIS_LSH	9
#define BAT_TH_CHECK_DIS_WID	1

#define AUTO_CHG_DIS_LSH	21
#define AUTO_CHG_DIS_WID	1

#define VI_PROGRAM_EN_LSH	23
#define VI_PROGRAM_EN_WID	1

/* Battery current amplification from A/D */
#define BAT_CURRENT_UNIT_UA    5870

/*
 *	The spec sheet is not clear about these two bits. The name and description shifted for a row.
 *	See P143 mc13892 user guild.
 * */
#define CHGFAULTM 0x000300

#define CHARGER_MISC 10000

#define DEBUG 1

#ifdef DEBUG
#define DBG(fmt, args...) printk(KERN_DEBUG "[%s] " fmt "\n", __func__, ## args)
#else
#define DBG(fmt, args...)	do {} while (0)
#endif

struct charger_callback {
	struct list_head list;
	void (*func) (void *);
	void *param;
};

static struct charger_callback chg_callbacks[N_CHARGER_EVENTS];

extern void gpio_chrgled_active(int enable);
extern int yoshi_battery_valid(void);
extern int pmic_check_factory_mode(void);

static void callback_charger_detect(void *param);
static void callback_bvalid_detect(void *param);
static void callback_bpon_event(void *param);
static void callback_lobatli_event(void *param);
static void callback_lobathi_event(void *param);
static void callback_faulti(void *param);
static void callback_se1b(void *param);
static void charger_statemachine(void);
static void charger_event_emit(enum charger_event event);

static void pmic_enable_lobat_charging(int enable);
static void pmic_restart_charging(void);
static int read_supply_voltage(void);
static int read_charger_voltage(void);
static int charger_check_full_battery(void);
static void pmic_reset_charger_timer(void);
static void do_lobat_work(struct work_struct *work);
static inline void do_charger_statemachine_work(struct work_struct *work);
static void do_charger_misc_work(struct work_struct *work);

extern int yoshi_voltage;
extern int yoshi_battery_current;
extern int yoshi_battery_id;
extern int yoshi_battery_capacity;
extern int yoshi_reduce_charging;
extern unsigned int yoshi_battery_error_flags;

static int batt_voltage = 0;
static int full_battery = 0;
static int lobat_charging = -1; /* unknown at beginning */
static int crit_boot_flag = 0;
static int have_critical_battery = 0;
static int have_low_battery = 0;
static unsigned short charge_current = -1; /* unknown at beginning */
static int max_charge_current = 0;
static int disable_chrgraw = 0;		/* Is charging disabled due to chrgraw? */
static int charger_timer_expiry_counter = 0;
static int charger_timer_expiry_threshold = 1;

DEFINE_MUTEX(chg_state_mutex);
static int chg_connected = -1; /* unknown at beginning */

DECLARE_DELAYED_WORK(charger_misc_work, do_charger_misc_work);
DECLARE_WORK(charger_statemachine_work, do_charger_statemachine_work);
DECLARE_WORK(lobat_work, do_lobat_work);

enum chg_setting {
	BAT_TH_CHECK_DIS,
	AUTO_CHG_DIS,
	VI_PROGRAM_EN
};

enum led_status {
	UNDEFINED_LED,
	NO_LED,
	GREEN_LED,
	YELLOW_LED
};

enum bp_detection_threshold {
	BP_DETECTION_THRESHOLD_UNDEFINED = -1,
	BP_DETECTION_THRESHOLD_MIN = 0,
	BP_DETECTION_THRESHOLD_LOW,
	BP_DETECTION_THRESHOLD_HIGH,
	BP_DETECTION_THRESHOLD_MAX,
};


static pmic_event_callback_t charger_detect = {
	.param = (void * ) NULL,
	.func = callback_charger_detect
};

static pmic_event_callback_t bvalid_detect = {
	.param = (void * ) NULL,
	.func = callback_bvalid_detect
};

static pmic_event_callback_t bpon_event = {
	.param = (void * ) NULL,
	.func = callback_bpon_event
};

static pmic_event_callback_t lobatli_event = {
	.param = (void *) NULL,
	.func = callback_lobatli_event
};

static pmic_event_callback_t lobathi_event = {
	.param = (void *) NULL,
	.func = callback_lobathi_event
};

static pmic_event_callback_t chgfaulti_event = {
	.param = (void *) NULL,
	.func = callback_faulti
};

static pmic_event_callback_t chrgse1b_event = {
	.param = (void *) NULL,
	.func = callback_se1b
};


/*
 *! The charger timer is configured for 120 minutes in APLite.
 *  The charger is automatically turned off after 120 minutes of
 *  charging. Once this occurs, the APLite generates a CHGFAULT
 *  with the sense bits correctly set to "10"
 */
static void callback_faulti(void *param)
{
	int sense_0 = 0;
	/* Bits 9 and 8 should be "10" to detect charger timer expiry */
	int charger_timer_mask = 0x200;

	pmic_read_reg(REG_INT_SENSE0, &sense_0, 0xffffff);

	sense_0 &= CHGFAULTM;

	switch(sense_0) /*see mc13892 data sheet*/
	{	case 0x100:
			printk(KERN_INFO "arcotg_udc: I pmic:faulti:sense_0=0x%x:charger path over voltage/dissipation in pass device.\n", sense_0);
			break;
		case 0x200:
			printk(KERN_INFO "arcotg_udc: I pmic:faulti:sense_0=0x%x:battery dies/charger times out.\n", sense_0);
			break;
		case 0x300:
			printk(KERN_INFO "arcotg_udc: I pmic:faulti:sense_0=0x%x:battery out of temperature.\n", sense_0);
			break;
		default:
			break;
	}
	/* Restart charging if the charger timer has expired */
	if (sense_0 & charger_timer_mask) {

	    /*
	     * According to the IEEE 1725 spec, the max charger timer
	     * should not exceed 6 hours or 360 minutes
	     */
	    if (charger_timer_expiry_counter >= charger_timer_expiry_threshold) {

	    	pr_debug("%s: charge timer expired!  Do not restart charging\n", __func__);
	    	charger_timer_expiry_counter++;

	    } else {
	    	if (is_charger_connected()) {
	    		pmic_restart_charging();
	    		charger_timer_expiry_counter++;
	    	}
	    }
	}
}

static void callback_bvalid_detect(void *param) {
	schedule_work(&charger_statemachine_work);
}

static void callback_charger_detect(void *param) {
	schedule_work(&charger_statemachine_work);
}

static void callback_bpon_event(void *param)
{
	DBG(KERN_DEBUG "callback_bpon_event\n");
	pmic_enable_lobat_charging(0);
}

/*
 * CHRGSE1B wall charger detect
 */
static void callback_se1b(void *param)
{
	schedule_work(&charger_statemachine_work);
}

/*
 * LOBATLI event callback from Atlas
 */
static void callback_lobatli_event(void *param)
{
	schedule_work(&lobat_work);
}

/*
 * LOBATHI event callback from Atlas
 */
static void callback_lobathi_event(void *param)
{
	schedule_work(&lobat_work);
}

static void pmic_enable_yellow_led(int enable)
{
	int flag = enable ? 1 : 0;

	pmic_write_reg(REG_CHARGE, (flag << CHRGLEDEN), (1 << CHRGLEDEN));
	gpio_chrgled_active(flag);
}

static void pmic_enable_green_led(int enable)
{
	if (enable) {
		mc13892_bklit_set_current(LIT_KEY, 0x7);
		mc13892_bklit_set_ramp(LIT_KEY, 0);
		mc13892_bklit_set_dutycycle(LIT_KEY, 0x3f);
	}
	else {
		mc13892_bklit_set_current(LIT_KEY, 0);
		mc13892_bklit_set_dutycycle(LIT_KEY, 0);
	}
}

static void update_leds(void) {
	static enum led_status led_status = UNDEFINED_LED;
	enum led_status new_led_status;

	if (chg_connected) {
		if (full_battery)
			new_led_status = GREEN_LED;
		else if (charge_current)
			new_led_status = YELLOW_LED;
		else
			new_led_status = NO_LED;
	} else {
		new_led_status = NO_LED;
	}

	if (led_status != new_led_status) {
		pmic_enable_green_led(new_led_status == GREEN_LED);
		pmic_enable_yellow_led(new_led_status == YELLOW_LED);

		led_status = new_led_status;
	}
}

int charger_sess_valid(void)
{
	int sense_0 = 0;
	int ret = 0; /* Default: no charger */

	pmic_read_reg(REG_INT_SENSE0, &sense_0, 0xffffff);
	if (sense_0 & BVALIDS)
		ret = 1;

	return ret;
}

/*
 * Is a USB charger connected?
 */
int charger_connected(void)
{
	int sense_0 = 0;
	int ret = 0; /* Default: no charger */

	pmic_read_reg(REG_INT_SENSE0, &sense_0, 0xffffff);
	if (sense_0 & CHGDETS)
		ret = 1;

	pr_debug("%s: %d\n", __func__, ret);

	return ret;
}

static int pmic_set_chg_current(unsigned short curr)
{
	unsigned int mask;
	unsigned int value;

	DBG("Setting charge current to %d", curr);

	/* If we have no battery, charging must be shut off unless in factory mode */
	if (!yoshi_battery_valid() && !pmic_check_factory_mode()) {
	    DBG("%s: no battery, disable charging\n", __func__);
	    curr = 0;
	}

	value = BITFVAL(BIT_CHG_CURR, curr);
	mask = BITFMASK(BIT_CHG_CURR);
	CHECK_ERROR(pmic_write_reg(REG_CHARGE, value, mask));

	update_leds();

	return 0;
}

static int charger_adjust_charge_current(void) {
	unsigned short new_charge_current;
	int sense_chgcurrs;
	int retval;

	if (!chg_connected) {
		new_charge_current = 0;
	} else {

		if (!full_battery) {
			pmic_read_reg(REG_INT_SENSE0, &sense_chgcurrs, CHGCURRS);
			full_battery = (charger_check_full_battery() ||
					(!sense_chgcurrs && (batt_voltage > CHGCURRS_THRESHOLD)));
		} else if (full_battery) {
			full_battery = (batt_voltage >= BATTCYCL_THRESHOLD);
		}

		disable_chrgraw = (read_charger_voltage() >= CHRGRAW_THRESHOLD);

		if ((charger_timer_expiry_counter > charger_timer_expiry_threshold)
				|| yoshi_battery_error_flags
				|| disable_chrgraw
				|| full_battery) {

			new_charge_current = 0;
		} else if (yoshi_reduce_charging) {
			new_charge_current = max(max_charge_current, LOW_TEMP_CHARGING);
		} else {
			new_charge_current = max_charge_current;
		}
	}

	if (new_charge_current != charge_current) {
		if ((retval = pmic_set_chg_current(new_charge_current)))
			return retval;

		if ((new_charge_current > 0) && !charge_current) {
			pmic_restart_charging();
			pmic_reset_charger_timer();
		}

		charge_current = new_charge_current;

		printk(KERN_INFO "Charge current changed to %u\n"
				"full battery=%d, disable chrgraw=%d, charger timer expiry counter=%d"
				", battery error flags=0x%x\n", charge_current, full_battery, disable_chrgraw,
				charger_timer_expiry_counter, yoshi_battery_error_flags);
	}

	return 0;
}

static int pmic_set_chg_misc(enum chg_setting type, unsigned short flag)
{
	unsigned int reg_value = 0;
	unsigned int mask = 0;

	switch (type) {
	case BAT_TH_CHECK_DIS:
		reg_value = BITFVAL(BAT_TH_CHECK_DIS, flag);
		mask = BITFMASK(BAT_TH_CHECK_DIS);
		break;
	case AUTO_CHG_DIS:
		reg_value = BITFVAL(AUTO_CHG_DIS, flag);
		mask = BITFMASK(AUTO_CHG_DIS);
		break;
	case VI_PROGRAM_EN:
		reg_value = BITFVAL(VI_PROGRAM_EN, flag);
		mask = BITFMASK(VI_PROGRAM_EN);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(REG_CHARGE, reg_value, mask));
	return 0;
}


static int charger_check_full_battery(void)
{
	return (yoshi_battery_capacity == FULL_BATTERY_CAPACITY) &&
		(yoshi_battery_current <= FULL_BATTERY_CURR_LIMIT);
}

static void pmic_restart_charging(void)
{
    if (yoshi_battery_valid()) {
    	/* Restart the charging process */
    	pmic_write_reg(REG_CHARGE, (1 << CHRGRESTART), (1 << CHRGRESTART));
    }
}

static void pmic_enable_lobat_charging(int enable)
{
	if (enable == lobat_charging)
		return;

	if (enable) {
		/* Keep auto charging disabled */
		pmic_set_chg_misc(AUTO_CHG_DIS, 1);
	} else {
		/* Keep auto charging enabled */
		pmic_set_chg_misc(AUTO_CHG_DIS, 0);
	}

	/* Turn off TRICKLE CHARGE
	 * The mc13892 doc says that TRICKLE charge should be used in case of a dead battery.
	 * However, the old code for the Kindle always disables the TRCIKLE CHARGE regardless
	 * of whether we are in a low battery condition or not.
	 */
	pmic_write_reg(REG_CHARGE, (0 << 7), (1 << 7));

	pmic_restart_charging();

	lobat_charging = enable;
}

static void charger_set_bp_detection_threshold(enum bp_detection_threshold new_threshold) {
	static enum bp_detection_threshold threshold = BP_DETECTION_THRESHOLD_UNDEFINED;

	if (new_threshold == threshold)
		return;

	pmic_write_reg(PMIC_REG_PC_0, new_threshold << BPSNS_OFFSET,
						BPSNS_MASK << BPSNS_OFFSET);

	threshold = new_threshold;
}

static void pmic_reset_charger_timer(void) {
	charger_timer_expiry_counter = 0;
	pmic_write_reg(REG_MEM_A, (0 << 4), (1 << 4));
}

/*!
 * Read the PMIC A/D for Charger Voltage, i.e. CHGRAW
 */
static int read_charger_voltage(void)
{
	unsigned short bp = 0;

	if (pmic_adc_convert(CHARGE_VOLTAGE, &bp)) {
		printk(KERN_ERR "arcotg_udc: E pmic:could not read CHARGER_VOLTAGE from adc:\n");
		return -1;
	}

	return bp*CHARGE_VOLTAGE_SCALE/1000; /* Return in mV */
}

/*!
 * The battery driver has already computed the voltage. So, do not redo
 * this calculation as it unnecessary generates PMIC/SPI traffic every
 * 10 seconds. Reuse the value unless battery error flags are set.
 */
static int read_supply_voltage(void)
{
	unsigned short bp = 0;

	// force read the voltage if it is not initialized.
	if (yoshi_voltage > 0)
		return yoshi_voltage;

	if (pmic_adc_convert(APPLICATION_SUPPLY, &bp)) {
		printk(KERN_ERR "arcotg_udc: E pmic:could not read APPLICATION_SUPPLY from adc:\n");
		return -1;
	}

	/*
	 * MC13892: Multiply the adc value by BATT_VOLTAGE_SCALE to get
	 * the battery value. This is from the Atlas Lite manual
	 */
	return bp*BATT_VOLTAGE_SCALE/1000; /* Return in mV */
}

/*!
 * Read the PMIC A/D for accurate LOBAT readings
 */
static int read_lobat_supply_voltage(void)
{
	unsigned short bp = 0;
	int err = 0;

	err = pmic_adc_convert(APPLICATION_SUPPLY, &bp);

	if (err) {
		printk(KERN_ERR "arcotg_udc: E pmic:could not read APPLICATION_SUPPLY from adc: %d:\n", err);
		return err;
	}

	/*
	 * MC13892: Multiply the adc value by BATT_VOLTAGE_SCALE to get
	 * the battery value. This is from the Atlas Lite manual
	 */
	return bp*BATT_VOLTAGE_SCALE/1000; /* Return in mV */
}

int charger_get_battery_current(int *curr)
{
	t_channel channel;
	unsigned short result[8];
	unsigned short current_raw;

	channel = BATTERY_CURRENT;
	CHECK_ERROR(pmic_adc_convert(channel, result));
	current_raw = result[0];

	if (current_raw & 0x200) /* check sign bit */
		/* current flowing from battery */
		*curr = (0x1FF - (current_raw & 0x1FF)) * BAT_CURRENT_UNIT_UA * (-1);
	else
		/* current flowing to battery */
		*curr = current_raw & 0x1FF * BAT_CURRENT_UNIT_UA;

	return 0;
}
EXPORT_SYMBOL(charger_get_battery_current);

static void check_low_battery(void) {
	int batt_voltage = read_lobat_supply_voltage();
	int have_critical_battery_new;
	int have_low_battery_new;

	if (batt_voltage < 0)
		return;

	have_critical_battery_new = (batt_voltage <= CRITICAL_VOLTAGE_THRESHOLD);
	have_low_battery_new = (batt_voltage <= LOW_VOLTAGE_THRESHOLD);

	if ((have_low_battery_new != have_low_battery)
			|| (have_critical_battery_new != have_critical_battery)) {

		have_critical_battery = have_critical_battery_new;
		have_low_battery = have_low_battery_new;

		charger_event_emit(CHARGER_LOBAT_EVENT);
	}

	return;
}

static void do_lobat_work(struct work_struct *work) {
	check_low_battery();
}

int charger_have_critical_battery(void) {
	return have_critical_battery;
}
EXPORT_SYMBOL(charger_have_critical_battery);

int charger_have_low_battery(void) {
	return have_low_battery;
}
EXPORT_SYMBOL(charger_have_low_battery);

void charger_event_subscribe(enum charger_event event, void (*callback_func)(void *), void *param) {
	struct charger_callback *callback = kmalloc(sizeof(struct charger_callback), GFP_KERNEL);
	callback->func = callback_func;
	callback->param = param;

	list_add(&callback->list, &chg_callbacks[event].list);
}
EXPORT_SYMBOL(charger_event_subscribe);

int charger_event_unsubscribe(enum charger_event event, void (*callback_func)(void *param)) {
	struct list_head *curr, *tmp;
	struct charger_callback *callback;

	list_for_each_safe(curr, tmp, &chg_callbacks[event].list) {
		callback = list_entry(curr, struct charger_callback, list);
		if (callback->func == callback_func) {
			list_del(curr);
			kfree(callback);

			return 0;
		}
	}

	return -EINVAL;
}
EXPORT_SYMBOL(charger_event_unsubscribe);

int charger_set_current_limit(unsigned int mA) {
	/*
	 * TODO: What is passed in here are not really Milli-Amperes, but
	 * actually the raw values that we are going to write to REG_CHARGE
	 * in order to achieve the desired charge current.
	 * Actually pass in Milli-Amperes here and map them to the correct
	 * values for REG_CHARGER using a lookup table or perhaps a formula.
	 */
	max_charge_current = mA;

	DBG("current limit set to %d\n", max_charge_current);

	return charger_adjust_charge_current();
}
EXPORT_SYMBOL(charger_set_current_limit);

int charger_get_charge_current(void) {
	return charge_current;
}
EXPORT_SYMBOL(charger_get_charge_current);

static inline void do_charger_statemachine_work(struct work_struct *work) {
	charger_statemachine();
}

static void charger_update_connect_status(void) {
	int chg_conn_new = (charger_connected() || charger_sess_valid());

	if (chg_connected != chg_conn_new) {
		chg_connected = chg_conn_new;

		max_charge_current = 0;

		charger_event_emit(CHARGER_CONNECT_EVENT);
	}
}

static void charger_statemachine(void) {
	mutex_lock(&chg_state_mutex);

	charger_update_connect_status();

	batt_voltage = read_supply_voltage();

	if (crit_boot_flag  && (yoshi_battery_capacity > CHARGER_BOOT_THRESHOLD)) {
		crit_boot_flag = 0;
		pmic_reset_charger_timer();
	}

	check_low_battery();

	/*
	 * If a charger is connected, then set the bp detection threshold to
	 * a minimum value for higher charge current. However, if no charger
	 * is attached, we want to detect a low battery condition very fast,
	 * so we need to raise the threshold again.
	 */
	if (chg_connected &&(batt_voltage > BPSNS_VOLTAGE_THRESH)) {
		charger_set_bp_detection_threshold(BP_DETECTION_THRESHOLD_MIN);
	} else {
		charger_set_bp_detection_threshold(BP_DETECTION_THRESHOLD_HIGH);
	}


	if (!lobat_charging && (batt_voltage <= BPON_THRESHOLD)) {
		/* Voltage has dropped below BPON threshold */
		pmic_enable_lobat_charging(1);
	} else if (lobat_charging && (batt_voltage > BPON_THRESHOLD_HI)) {
		/* The voltage has risen above BPON threshold */
		pmic_enable_lobat_charging(0);
	}

	charger_adjust_charge_current();

	update_leds();

	mutex_unlock(&chg_state_mutex);
}

enum charger_status is_charger_connected(void) {
	return chg_connected;
}
EXPORT_SYMBOL(is_charger_connected);

static void charger_event_emit(enum charger_event event) {
	struct list_head *curr;
	struct charger_callback *callback;

	DBG("Emitting charger event. event=%d, connected=%d", (int) event,
			is_charger_connected());

	list_for_each(curr, &chg_callbacks[event].list) {
		callback = list_entry(curr, struct charger_callback, list);
		callback->func(callback->param);
	}
}

/*
 * Do all the miscellaneous activities like lobat checking and led setting
 */
static void do_charger_misc_work(struct work_struct *work)
{
	charger_statemachine();

	schedule_delayed_work(&charger_misc_work, msecs_to_jiffies(CHARGER_MISC));
}

static int try_pmic_event_subscribe(type_event event, pmic_event_callback_t callback) {
	int ret = pmic_event_subscribe(event, callback);
	if (ret)
		printk(KERN_ERR "%s: Error %d subscribing to event %d\n", __func__, ret, event);

	return ret;
}

static int try_pmic_event_unsubscribe(type_event event, pmic_event_callback_t callback) {
	int ret = pmic_event_unsubscribe(event, callback);
	if (ret)
		printk(KERN_ERR "%s: Error %d unsubscribing from event %d\n", __func__, ret, event);

	return ret;
}

static int __init yoshi_charger_init(void) {
	unsigned int reg_memory_a;
	int i;

	for (i = 0; i < N_CHARGER_EVENTS; i++)
		INIT_LIST_HEAD(&chg_callbacks[i].list);

	/* If bit is already set, then it is a fall through */
	pmic_read_reg(REG_MEM_A, &reg_memory_a, (0x1f << 0));
	if (reg_memory_a & 0x10)
		printk(KERN_ERR "boot: I def:Booting out of critical battery::\n");

	if (mx50_board_is(BOARD_ID_TEQUILA)) {
		charger_timer_expiry_threshold = CHRG_TIMER_THRESHOLD_TEQ;
	}

	/* Battery thermistor check disable */
	pmic_set_chg_misc(BAT_TH_CHECK_DIS, 1);

	/* Allow programming of charge voltage and current */
	pmic_set_chg_misc(VI_PROGRAM_EN, 1);

	/* PLIM: bits 15 and 16 */
	pmic_write_reg(REG_CHARGE, (0x2 << 15), (0x3 << 15));

	/* PLIMDIS: bit 17 */
	pmic_write_reg(REG_CHARGE, (0 << 17), (1 << 17));

	/* Keep auto charging enabled */
	pmic_set_chg_misc(AUTO_CHG_DIS, 0);

	/* Turn off CYCLB disable */
	pmic_write_reg(REG_CHARGE, (0 << CHRGCYCLB), (1 << CHRGCYCLB));

	charger_statemachine();

	if (is_charger_connected()) {
		printk(KERN_INFO "arcotg_udc: I pmic:chargerInit::\n");
		/*
		 * The module is just loaded, check capacity. If <= 4%,
		 * then we are surely in recovery utils
		 */
		if (yoshi_battery_capacity <= CHARGER_BOOT_THRESHOLD) {
			/* Turn on bit #4 */
			pmic_write_reg(REG_MEM_A, (1 << 4), (1 << 4));
			crit_boot_flag = 1;
		}
	}

	CHECK_ERROR(try_pmic_event_subscribe(EVENT_CHGDETI, charger_detect));
	CHECK_ERROR(try_pmic_event_subscribe(EVENT_BVALIDI, bvalid_detect));
	CHECK_ERROR(try_pmic_event_subscribe(EVENT_LOBATLI, lobatli_event));
	CHECK_ERROR(try_pmic_event_subscribe(EVENT_LOBATHI, lobathi_event));
	CHECK_ERROR(try_pmic_event_subscribe(EVENT_SE1I, chrgse1b_event));
	CHECK_ERROR(try_pmic_event_subscribe(EVENT_CHGFAULTI, chgfaulti_event));
	CHECK_ERROR(try_pmic_event_subscribe(EVENT_BPONI, bpon_event));

	schedule_delayed_work(&charger_misc_work, msecs_to_jiffies(CHARGER_MISC));

	return 0;
}

static void __exit yoshi_charger_exit(void) {
	int i;
	struct list_head *curr, *tmp;
	struct charger_callback *callback;

	cancel_delayed_work_sync(&charger_misc_work);

	try_pmic_event_unsubscribe(EVENT_CHGDETI, charger_detect);
	try_pmic_event_unsubscribe(EVENT_BVALIDI, bvalid_detect);
	try_pmic_event_unsubscribe(EVENT_LOBATLI, lobatli_event);
	try_pmic_event_unsubscribe(EVENT_LOBATHI, lobathi_event);
	try_pmic_event_unsubscribe(EVENT_SE1I, chrgse1b_event);
	try_pmic_event_unsubscribe(EVENT_CHGFAULTI, chgfaulti_event);
	try_pmic_event_unsubscribe(EVENT_BPONI, bpon_event);

	for (i = 0; i < N_CHARGER_EVENTS; i++) {
		list_for_each_safe(curr, tmp, &chg_callbacks[i].list){
			callback = list_entry(curr, struct charger_callback, list);
			list_del(curr);
			kfree(callback);
		}
	}
}

module_init(yoshi_charger_init);
module_exit(yoshi_charger_exit);

MODULE_DESCRIPTION("MX50 Yoshi Charger Driver");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
