/*
 * boardid.h
 *
 * Copyright 2010-2011 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef __BOARDID_H__
#define __BOARDID_H__

#define BOARD_ID_N              3
#define BOARD_ID_REV_N          5
#define BOARD_ID_REV_LEN        BOARD_ID_REV_N - BOARD_ID_N

#define BOARD_ID_TEQUILA              "003"
#define BOARD_ID_WHITNEY              "005"
#define BOARD_ID_WHITNEY_PROTO        "00501"
#define BOARD_ID_WHITNEY_WFO          "006"
#define BOARD_ID_WHITNEY_WFO_PROTO    "00600"
#define BOARD_ID_TEQUILA_EVT2         "00309"
#define BOARD_ID_WHITNEY_EVT2         "00516"
#define BOARD_ID_WHITNEY_EVT3         "00520"
#define BOARD_ID_WHITNEY_WFO_EVT2     "00605"
#define BOARD_ID_WHITNEY_WFO_EVT3     "00606"

#define BOARD_IS_(id, b, n)	(strncmp((id), (b), (n)) == 0)
#define BOARD_REV_GRT_(id, b) (strncmp((id+BOARD_ID_N), (b+BOARD_ID_N), BOARD_ID_REV_LEN) > 0)
#define BOARD_REV_GRT_EQ_(id, b) (strncmp((id+BOARD_ID_N), (b+BOARD_ID_N), BOARD_ID_REV_LEN) >= 0)

#define BOARD_REV_GREATER(id, b) (BOARD_IS_((id), (b), BOARD_ID_N) && BOARD_REV_GRT_((id), (b)))
#define BOARD_REV_GREATER_EQ(id, b) (BOARD_IS_((id), (b), BOARD_ID_N) && BOARD_REV_GRT_EQ_((id), (b)))

#define BOARD_IS_WHITNEY(id)       BOARD_IS_((id), BOARD_ID_WHITNEY,       BOARD_ID_N)
#define BOARD_IS_WHITNEY_WFO(id)   BOARD_IS_((id), BOARD_ID_WHITNEY_WFO,   BOARD_ID_N)
#define BOARD_IS_WHITNEY_PROTO(id) BOARD_IS_((id), BOARD_ID_WHITNEY_PROTO, BOARD_ID_REV_N)
#define BOARD_IS_WHITNEY_WFO_PROTO(id) BOARD_IS_((id), BOARD_ID_WHITNEY_WFO_PROTO, BOARD_ID_REV_N)

#define BOARD_IS_TEQUILA(id)       BOARD_IS_((id), BOARD_ID_TEQUILA,       BOARD_ID_N)
#define BOARD_IS_TEQUILA_EVT2(id)  BOARD_IS_((id), BOARD_ID_TEQUILA_EVT2,  BOARD_ID_REV_N)
#define BOARD_IS_WHITNEY_EVT2(id)  BOARD_IS_((id), BOARD_ID_WHITNEY_EVT2,  BOARD_ID_REV_N)
#define BOARD_IS_WHITNEY_WFO_EVT2(id)  BOARD_IS_((id), BOARD_ID_WHITNEY_WFO_EVT2,  BOARD_ID_REV_N)


#endif
