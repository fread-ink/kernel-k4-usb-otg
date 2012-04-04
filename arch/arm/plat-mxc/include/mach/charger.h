/*
 * charger.h
 *
 * (C) Copyright 2010 Christian Hoff.  All rights reserved.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef CHARGER_H_
#define CHARGER_H_

enum charger_status {
	CHARGER_STATE_UNDEFINED = 0,
	CHARGER_STATE_DISCONNECTED = 0,
	CHARGER_STATE_CONNECTED
};

#define N_CHARGER_EVENTS 2
enum charger_event {
	CHARGER_CONNECT_EVENT = 0,
	CHARGER_LOBAT_EVENT
};

void charger_event_subscribe(enum charger_event event, void (*callback_func)(void *param),
		void *param);
int charger_event_unsubscribe(enum charger_event event, void (*callback_func)(void *param));

int charger_set_current_limit(unsigned int mA);
int charger_get_charge_current(void);
int charger_have_critical_battery(void);
int charger_have_low_battery(void);

int charger_get_battery_current(int *curr);

enum charger_status is_charger_connected(void);

#endif /* CHARGER_H_ */
