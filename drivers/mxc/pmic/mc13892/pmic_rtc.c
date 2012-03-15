/*
 * Copyright 2010-2011 Amazon Technologies, Inc. All Rights Reserved.
 * Manish Lachwani (lachwani@lab126.com)
 */

#include <linux/pmic_external.h>
#include <linux/time.h>

#define	MC13892_RTCTIME_TIME_LSH	0
#define	MC13892_RTCTIME_TIME_WID	17

#define	MC13892_RTCALARM_TIME_LSH	0
#define	MC13892_RTCALARM_TIME_WID	17

#define MC13892_RTCDAY_DAY_LSH		0
#define	MC13892_RTCDAY_DAY_WID		15

#define	MC13892_RTCALARM_DAY_LSH	0
#define	MC13892_RTCALARM_DAY_WID	15

/*!
 * This function set the real time clock of PMIC
 *
 * @param	pmic_time	value of date and time
 * 
 * @return			This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_set_time(struct timeval *pmic_time)
{
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0;
	unsigned int mask, value;

	tod_reg_val = pmic_time->tv_sec % 86400;
	day_reg_val = pmic_time->tv_sec / 86400;

	mask = BITFMASK(MC13892_RTCTIME_TIME);
	value = BITFVAL(MC13892_RTCTIME_TIME, tod_reg_val);
	CHECK_ERROR(pmic_write_reg(REG_RTC_TIME, value, mask));

	mask = BITFMASK(MC13892_RTCDAY_DAY);
	value = BITFVAL(MC13892_RTCDAY_DAY, day_reg_val);
	CHECK_ERROR(pmic_write_reg(REG_RTC_DAY, value, mask));

	return PMIC_SUCCESS;
}

/*!
 * This function get the real time clock of PMIC
 *
 * @param	pmic_time	return value of date and time
 *
 * @return	This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_get_time(struct timeval * pmic_time)
{
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0;
	unsigned int mask, value;

	mask = BITFMASK(MC13892_RTCTIME_TIME);
	CHECK_ERROR(pmic_read_reg(REG_RTC_TIME, &value, mask));
	tod_reg_val = BITFEXT(value, MC13892_RTCTIME_TIME);

	mask = BITFMASK(MC13892_RTCDAY_DAY);
	CHECK_ERROR(pmic_read_reg(REG_RTC_DAY, &value, mask));
	day_reg_val = BITFEXT(value, MC13892_RTCDAY_DAY);

	pmic_time->tv_sec = (unsigned long)((unsigned long)(tod_reg_val &
						            0x0001FFFF) +
					    (unsigned long)(day_reg_val *
							    86400));

	return PMIC_SUCCESS;
}

/*!
 * This function set the real time clock of PMIC
 *
 * @param        pmic_time      value of date and time
 *
 * @return       This function returns PMIC_SUCCESS if successful.
 */
PMIC_STATUS pmic_rtc_set_time_alarm(struct timeval * pmic_time)
{
	unsigned int tod_reg_val = 0;
	unsigned int day_reg_val = 0;
	unsigned int mask, value;

	tod_reg_val = pmic_time->tv_sec % 86400;
	day_reg_val = pmic_time->tv_sec / 86400;

	mask = BITFMASK(MC13892_RTCALARM_TIME);
	value = BITFVAL(MC13892_RTCALARM_TIME, tod_reg_val);
	CHECK_ERROR(pmic_write_reg(REG_RTC_ALARM, value, mask));

	mask = BITFMASK(MC13892_RTCALARM_DAY);
	value = BITFVAL(MC13892_RTCALARM_DAY, day_reg_val);
	CHECK_ERROR(pmic_write_reg(REG_RTC_DAY_ALARM, value, mask));

	return PMIC_SUCCESS;
}
