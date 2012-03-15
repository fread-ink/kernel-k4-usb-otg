/*
 * Copyright 2009 Amazon Technologies, Inc. All Rights Reserved.
 * Manish Lachwani (lachwani@lab126.com)
 */

#include <linux/pmic_external.h>
#include <asm/pmic_power.h>
#include <linux/module.h> 

/*
 * Register specific defines
 */
/*
 * Reg Power Control 1
 */

#define MC13892_PWRCTRL_USER_OFF_SPI_LSH	3
#define MC13892_PWRCTRL_USER_OFF_SPI_WID	1
#define MC13892_PWRCTRL_USER_OFF_SPI_ENABLE	1
#define MC13892_PWRCTRL_PCT_LSH			0
#define MC13892_PWRCTRL_PCT_WID			8
#define MC13892_PWRCTRL_PC_COUNT_LSH		8
#define MC13892_PWRCTRL_PC_COUNT_WID		4
#define MC13892_PWRCTRL_PC_MAX_CNT_LSH		12
#define MC13892_PWRCTRL_PC_MAX_CNT_WID		4
#define MC13892_PWRCTRL_MEM_TMR_LSH		16
#define MC13892_PWRCTRL_MEM_TMR_WID		4
#define MC13892_PWRCTRL_MEM_ALLON_LSH		20
#define MC13892_PWRCTRL_MEM_ALLON_WID		1
#define MC13892_PWRCTRL_MEM_ALLON_ENABLE        1
#define MC13892_PWRCTRL_MEM_ALLON_DISABLE       0

/*
 * Reg Power Control 2
 */
#define MC13892_AUTO_RESTART_LSH	0
#define MC13892_AUTO_RESTART_WID	1
#define MC13892_EN_BT_ON1B_LSH		1
#define MC13892_EN_BT_ON1B_WID		1
#define MC13892_EN_BT_ON2B_LSH		2
#define MC13892_EN_BT_ON2B_WID		1
#define MC13892_EN_BT_ON3B_LSH		3
#define MC13892_EN_BT_ON3B_WID		1
#define MC13892_DEB_BT_ON1B_LSH		4
#define MC13892_DEB_BT_ON1B_WID		2
#define MC13892_DEB_BT_ON2B_LSH		6
#define MC13892_DEB_BT_ON2B_WID		2
#define MC13892_DEB_BT_ON3B_LSH		8
#define MC13892_DEB_BT_ON3B_WID		2

/*!
 * This function sets user power off in power control register and thus powers
 * off the device
 */
void pmic_power_off(void)
{
	unsigned int mask, value;

	mask = BITFMASK(MC13892_PWRCTRL_USER_OFF_SPI);
	value = BITFVAL(MC13892_PWRCTRL_USER_OFF_SPI,
			MC13892_PWRCTRL_USER_OFF_SPI_ENABLE);
	
	pmic_write_reg(REG_POWER_CTL0, value, mask);
}

/*!
 * This function enables auto reset after a system reset.
 *
 * @param	en		if true, the auto reset is enabled
 * 
 * @return	This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_set_auto_reset_en(bool en)
{
	unsigned int reg_val = 0, reg_mask = 0;

	reg_val = BITFVAL(MC13892_AUTO_RESTART, en);
	reg_mask = BITFMASK(MC13892_AUTO_RESTART);

	CHECK_ERROR(pmic_write_reg(REG_POWER_CTL2, reg_val, reg_mask));
	return PMIC_SUCCESS;
}

/*!
 * This function configures a system reset on a button.
 *
 * @param	bt		type of button.
 * @param	sys_rst		if true, enable the system reset on this button
 * @param	deb_time	sets the debounce time on this button pin
 *
 * @return	This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_set_conf_button(t_button bt, bool sys_rst, int deb_time)
{
	int max_val = 0;
	unsigned int reg_val = 0, reg_mask = 0;

	max_val = (1 << MC13892_DEB_BT_ON1B_WID) - 1;
	if (deb_time > max_val)
		return PMIC_PARAMETER_ERROR;

	switch (bt) {
	case BT_ON1B:
		reg_val = BITFVAL(MC13892_EN_BT_ON1B, sys_rst) |
			BITFVAL(MC13892_DEB_BT_ON1B, deb_time);
		reg_mask = BITFMASK(MC13892_EN_BT_ON1B) |
			BITFMASK(MC13892_DEB_BT_ON1B);
		break;
	case BT_ON2B:
		reg_val = BITFVAL(MC13892_EN_BT_ON2B, sys_rst) |
			BITFVAL(MC13892_DEB_BT_ON2B, deb_time);
		reg_mask = BITFMASK(MC13892_EN_BT_ON2B) |
			BITFMASK(MC13892_DEB_BT_ON2B);
		break;
	case BT_ON3B:
		reg_val = BITFVAL(MC13892_EN_BT_ON3B, sys_rst) |
			BITFVAL(MC13892_DEB_BT_ON3B, deb_time);
		reg_mask = BITFMASK(MC13892_EN_BT_ON3B) |
			BITFMASK(MC13892_DEB_BT_ON3B);
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	CHECK_ERROR(pmic_write_reg(REG_POWER_CTL2, reg_val, reg_mask));
	return PMIC_SUCCESS;
}

/*!
 * This function is used to un/subscribe on power event IT.
 *
 * @param	event		type of event.
 * @param	callback	event callback function.
 * @param	sub		define if Un/subscribe event.
 *
 * @return	This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_event(t_pwr_int event, void *callback, bool sub)
{
	pmic_event_callback_t power_callback;
	type_event power_event;

	power_callback.func = callback;
	power_callback.param = NULL;
	switch (event) {
	case PWR_IT_BPONI:
		power_event = EVENT_BPONI;
		break;
	case PWR_IT_LOBATLI:
		power_event = EVENT_LOBATLI;
		break;
	case PWR_IT_LOBATHI:
		power_event = EVENT_LOBATHI;
		break;
	case PWR_IT_ONOFD1I:
		power_event = EVENT_ONOFD1I;
		break;
	case PWR_IT_ONOFD2I:
		power_event = EVENT_ONOFD2I;
		break;
	case PWR_IT_ONOFD3I:
		power_event = EVENT_ONOFD3I;
		break;
	case PWR_IT_SYSRSTI:
		power_event = EVENT_SYSRSTI;
		break;
	case PWR_IT_PCI:
		power_event = EVENT_PCI;
		break;
	case PWR_IT_WARMI:
		power_event = EVENT_WARMI;
		break;
	default:
		return PMIC_PARAMETER_ERROR;
	}

	if (sub == true)
		CHECK_ERROR(pmic_event_subscribe(power_event, power_callback));
	else
		CHECK_ERROR(pmic_event_unsubscribe(power_event, power_callback));

	return PMIC_SUCCESS;
}

/*!
 * This function is used to subscribe on power event IT.
 *
 * @param	event		type of event.
 * @param	callback	event callback function.
 *
 * @return	This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_event_sub(t_pwr_int event, void *callback)
{
	return pmic_power_event(event, callback, true);
}

/*!
 * This function is used to unsubscribe on power event IT.
 *
 * @param	event		type of event.
 * @param	callback	event callback function.
 * 
 * @return	This function returns 0 if successful.
 */
PMIC_STATUS pmic_power_event_unsub(t_pwr_int event, void *callback)
{
	return pmic_power_event(event, callback, false);
}

EXPORT_SYMBOL(pmic_power_set_conf_button);
EXPORT_SYMBOL(pmic_power_event_sub);
EXPORT_SYMBOL(pmic_power_event_unsub);

