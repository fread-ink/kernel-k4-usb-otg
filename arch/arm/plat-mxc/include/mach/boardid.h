/*
 * boardid.h
 *
 * (C) Copyright 2010 Amazon Technologies, Inc.  All rights reserved.
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

#ifndef __MACH_BOARDID_H__
#define __MACH_BOARDID_H__

#include <boardid.h>

/* boardid.c */
void mx50_init_boardid(void);
int mx50_board_is(char *id);
int mx50_board_rev_greater(char *id);
int mx50_board_rev_greater_eq(char *id);

extern char mx50_serial_number[];
extern char mx50_mac_address[];

#endif
