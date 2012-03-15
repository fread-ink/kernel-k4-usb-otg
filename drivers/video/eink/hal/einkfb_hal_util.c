/*
 *  linux/drivers/video/eink/hal/einkfb_hal_util.c -- eInk frame buffer device HAL utilities
 *
 *      Copyright (C) 2005-2010 Amazon Technologies, Inc.
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include "einkfb_hal.h"

#if PRAGMAS
    #pragma mark Definitions & Globals
    #pragma mark -
#endif

extern einkfb_power_level EINKFB_GET_POWER_LEVEL(void);
extern void EINKFB_SET_POWER_LEVEL(einkfb_power_level power_level);

// 1bpp -> 2bpp
//
// 0 0 0 0 -> 00 00 00 00   0x0 -> 0x00
// 0 0 0 1 -> 00 00 00 11   0x1 -> 0x03
// 0 0 1 0 -> 00 00 11 00   0x2 -> 0x0C
// 0 0 1 1 -> 00 00 11 11   0x3 -> 0x0F
// 0 1 0 0 -> 00 11 00 00   0x4 -> 0x30
// 0 1 0 1 -> 00 11 00 11   0x5 -> 0x33
// 0 1 1 0 -> 00 11 11 00   0x6 -> 0x3C
// 0 1 1 1 -> 00 11 11 11   0x7 -> 0x3F
// 1 0 0 0 -> 11 00 00 00   0x8 -> 0xC0
// 1 0 0 1 -> 11 00 00 11   0x9 -> 0xC3
// 1 0 1 0 -> 11 00 11 00   0xA -> 0xCC
// 1 0 1 1 -> 11 00 11 11   0xB -> 0xCF
// 1 1 0 0 -> 11 11 00 00   0xC -> 0xF0
// 1 1 0 1 -> 11 11 00 11   0xD -> 0xF3
// 1 1 1 0 -> 11 11 11 00   0xE -> 0xFC
// 1 1 1 1 -> 11 11 11 11   0xF -> 0xFF
//
static u8 stretch_nybble_table_1_to_2bpp[16] =
{
    0x00, 0x03, 0x0C, 0x0F, 0x30, 0x33, 0x3C, 0x3F,
    0xC0, 0xC3, 0xCC, 0xCF, 0xF0, 0xF3, 0xFC, 0xFF
};

// 2bpp -> 4bpp
//
// 00 00 -> 0000 0000       0x0 -> 0x00
// 00 01 -> 0000 0101       0x1 -> 0x05
// 00 10 -> 0000 1010       0x2 -> 0x0A
// 00 11 -> 0000 1111       0x3 -> 0x0F
// 01 00 -> 0101 0000       0x4 -> 0x50
// 01 01 -> 0101 0101       0x5 -> 0x55
// 01 10 -> 0101 1010       0x6 -> 0x5A
// 01 11 -> 0101 1111       0x7 -> 0x5F
// 10 00 -> 1010 0000       0x8 -> 0xA0
// 10 01 -> 1010 0101       0x9 -> 0xA5
// 10 10 -> 1010 1010       0xA -> 0xAA
// 10 11 -> 1010 1111       0xB -> 0xAF
// 11 00 -> 1111 0000       0xC -> 0xF0
// 11 01 -> 1111 0101       0xD -> 0xF5
// 11 10 -> 1111 1010       0xE -> 0xFA
// 11 11 -> 1111 1111       0xF -> 0xFF
//
static u8 stretch_nybble_table_2_to_4bpp[16] =
{
    0x00, 0x05, 0x0A, 0x0F, 0x50, 0x55, 0x5A, 0x5F,
    0xA0, 0xA5, 0xAA, 0xAF, 0xF0, 0xF5, 0xFA, 0xFF
};

// 4bpp -> 8bpp
//
// 0000 -> 00000000         0x0 -> 0x00
// 0001 -> 00010001         0x1 -> 0x11
// 0010 -> 00100010         0x2 -> 0x22
// 0011 -> 00110011         0x3 -> 0x33
// 0100 -> 01000100         0x4 -> 0x44
// 0101 -> 01010101         0x5 -> 0x55
// 0110 -> 01100110         0x6 -> 0x66
// 0111 -> 01110111         0x7 -> 0x77
// 1000 -> 10001000         0x8 -> 0x88
// 1001 -> 10011001         0x9 -> 0x99
// 1010 -> 10101010         0xA -> 0xAA
// 1011 -> 10111011         0xB -> 0xBB
// 1100 -> 11001100         0xC -> 0xCC
// 1101 -> 11011101         0xD -> 0xDD
// 1110 -> 11101110         0xE -> 0xEE
// 1111 -> 11111111         0xF -> 0xFF
//
static u8 stretch_nybble_table_4_to_8bpp[16] =
{
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
};

// 1bpp
//
//  0   0 0 0 0 0 0 0 0     0x00
//  1   1 1 1 1 1 1 1 1     0xFF
//
static u8 gray_table_1bpp[2] =
{
    0x00, 0xFF
};

// 2bpp
//
//  0   00 00 00 00         0x00
//  1   01 01 01 01         0x55
//  2   10 10 10 10         0xAA
//  3   11 11 11 11         0xFF
//
static u8 gray_table_2bpp[4] =
{
    0x00, 0x55, 0xAA, 0xFF
};

// 4bpp
//
//  0   0000 0000           0x00
//  1   0001 0001           0x11
//  2   0010 0010           0x22
//  3   0011 0011           0x33
//  4   0100 0100           0x44
//  5   0101 0101           0x55
//  6   0110 0110           0x66
//  7   0111 0111           0x77    
//  8   1000 1000           0x88
//  9   1001 1001           0x99
// 10   1010 1010           0xAA
// 11   1011 1011           0xBB
// 12   1100 1100           0xCC
// 13   1101 1101           0xDD
// 14   1110 1110           0xEE
// 15   1111 1111           0xFF
//
static u8 gray_table_4bpp[16] =
{
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
};

// 2bpp -> 1bpp
//
static u8 posterize_table_2bpp[256] =
{
    0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x03, 0x03, 0x0C, 0x0C, 0x0F, 0x0F, 0x0C, 0x0C, 0x0F, 0x0F, // 00..0F
    0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x03, 0x03, 0x0C, 0x0C, 0x0F, 0x0F, 0x0C, 0x0C, 0x0F, 0x0F, // 10..1F
    0x30, 0x30, 0x33, 0x33, 0x30, 0x30, 0x33, 0x33, 0x3C, 0x3C, 0x3F, 0x3F, 0x3C, 0x3C, 0x3F, 0x3F, // 20..2F
    0x30, 0x30, 0x33, 0x33, 0x30, 0x30, 0x33, 0x33, 0x3C, 0x3C, 0x3F, 0x3F, 0x3C, 0x3C, 0x3F, 0x3F, // 30..3F
    0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x03, 0x03, 0x0C, 0x0C, 0x0F, 0x0F, 0x0C, 0x0C, 0x0F, 0x0F, // 40..4F
    0x00, 0x00, 0x03, 0x03, 0x00, 0x00, 0x03, 0x03, 0x0C, 0x0C, 0x0F, 0x0F, 0x0C, 0x0C, 0x0F, 0x0F, // 50..5F
    0x30, 0x30, 0x33, 0x33, 0x30, 0x30, 0x33, 0x33, 0x3C, 0x3C, 0x3F, 0x3F, 0x3C, 0x3C, 0x3F, 0x3F, // 60..6F
    0x30, 0x30, 0x33, 0x33, 0x30, 0x30, 0x33, 0x33, 0x3C, 0x3C, 0x3F, 0x3F, 0x3C, 0x3C, 0x3F, 0x3F, // 70..7F
    0xC0, 0xC0, 0xC3, 0xC3, 0xC0, 0xC0, 0xC3, 0xC3, 0xCC, 0xCC, 0xCF, 0xCF, 0xCC, 0xCC, 0xCF, 0xCF, // 80..8F
    0xC0, 0xC0, 0xC3, 0xC3, 0xC0, 0xC0, 0xC3, 0xC3, 0xCC, 0xCC, 0xCF, 0xCF, 0xCC, 0xCC, 0xCF, 0xCF, // 90..9F
    0xF0, 0xF0, 0xF3, 0xF3, 0xF0, 0xF0, 0xF3, 0xF3, 0xFC, 0xFC, 0xFF, 0xFF, 0xFC, 0xFC, 0xFF, 0xFF, // A0..AF
    0xF0, 0xF0, 0xF3, 0xF3, 0xF0, 0xF0, 0xF3, 0xF3, 0xFC, 0xFC, 0xFF, 0xFF, 0xFC, 0xFC, 0xFF, 0xFF, // B0..BF
    0xC0, 0xC0, 0xC3, 0xC3, 0xC0, 0xC0, 0xC3, 0xC3, 0xCC, 0xCC, 0xCF, 0xCF, 0xCC, 0xCC, 0xCF, 0xCF, // C0..CF
    0xC0, 0xC0, 0xC3, 0xC3, 0xC0, 0xC0, 0xC3, 0xC3, 0xCC, 0xCC, 0xCF, 0xCF, 0xCC, 0xCC, 0xCF, 0xCF, // D0..DF
    0xF0, 0xF0, 0xF3, 0xF3, 0xF0, 0xF0, 0xF3, 0xF3, 0xFC, 0xFC, 0xFF, 0xFF, 0xFC, 0xFC, 0xFF, 0xFF, // E0..FF
    0xF0, 0xF0, 0xF3, 0xF3, 0xF0, 0xF0, 0xF3, 0xF3, 0xFC, 0xFC, 0xFF, 0xFF, 0xFC, 0xFC, 0xFF, 0xFF  // F0..FF
};

// 4bpp -> 1bpp
//
static u8 posterize_table_4bpp[256] =
{    
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 00..0F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 10..1F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 20..2F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 30..3F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 40..4F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 50..5F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 60..6F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 70..7F
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 80..8F
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 90..9F
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // A0..AF
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // B0..BF
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // C0..CF
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // D0..DF
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // E0..EF
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  // F0..FF
};

// 8bpp -> 1bpp
//
static u8 posterize_table_8bpp[256] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 00..0F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 10..1F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 20..2F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 30..3F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 40..4F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 50..5F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 60..6F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 70..7F
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 80..8F
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 90..9F
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // A0..AF
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // B0..BF
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // C0..CF
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // D0..DF
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // E0..EF
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  // F0..FF
};

// 8/4bpp -> 2bpp lighter (0-9, 10-15, -, -)
//
static u8 contrast_table_lightest[256] = 
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, // 00..0F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, // 10..1F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, // 20..2F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, // 30..3F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, // 40..4F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, // 50..5F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, // 60..6F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, // 70..7F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, // 80..8F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, // 90..9F
    0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, // A0..6F
    0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, // B0..BF
    0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, // C0..CF
    0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, // D0..DF
    0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, // E0..EF
    0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, // F0..FF
};

// 8/4bpp -> 2bpp lighter (0-5, 6-10, 11-15, -)
//
static u8 contrast_table_lighter[256] = 
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, // 00..0F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, // 10..1F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, // 20..2F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, // 30..3F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, // 40..4F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, // 50..5F
    0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, // 60..6F
    0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, // 70..7F
    0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, // 80..8F
    0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, // 90..9F
    0x50, 0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, // A0..AF
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, // B0..BF
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, // C0..CF
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, // D0..DF
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, // E0..EF
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA  // E0..FF
};

// 8/4bpp -> 2bpp light (0-4, 5-9, 10-14, 15)
//
static u8 contrast_table_light[256] = 
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0F, // 00..0F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0F, // 10..1F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0F, // 20..2F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0F, // 30..3F
    0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0F, // 40..4F
    0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, // 50..5F
    0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, // 60..6F
    0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, // 70..7F
    0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, // 80..8F
    0x50, 0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, // 90..9F
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, // A0..AF
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, // B0..BF
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, // C0..CF
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, // D0..DF
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, // E0..EF
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF  // F0..FF
};

// 8/4bpp -> 2bpp medium (0-3, 4-7, 8-11, 12-15)
//
static u8 contrast_table_medium[256] = 
{
    0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0F, 0x0F, 0x0F, 0x0F, // 00..0F
    0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0F, 0x0F, 0x0F, 0x0F, // 10..1F
    0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0F, 0x0F, 0x0F, 0x0F, // 20..2F
    0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0F, 0x0F, 0x0F, 0x0F, // 30..3F
    0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, 0x5F, 0x5F, 0x5F, // 40..4F
    0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, 0x5F, 0x5F, 0x5F, // 50..5F
    0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, 0x5F, 0x5F, 0x5F, // 60..6F
    0x50, 0x50, 0x50, 0x50, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, 0x5F, 0x5F, 0x5F, // 70..7F
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, // 80..8F
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, // 90..9F
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, // A0..AF
    0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, // B0..BF
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF5, 0xF5, 0xF5, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, // C0..CF
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF5, 0xF5, 0xF5, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, // D0..DF
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF5, 0xF5, 0xF5, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, // E0..EF
    0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF5, 0xF5, 0xF5, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF  // F0..FF
};

// 8/4bpp -> 2bpp dark (0, 1-5, 6-10, 11-15)
//
static u8 contrast_table_dark[256] =
{
    0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 00..0F
    0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, // 10..1F
    0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, // 20..2F
    0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, // 30..3F
    0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, // 40..4F
    0x50, 0x55, 0x55, 0x55, 0x55, 0x55, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5F, 0x5F, 0x5F, 0x5F, 0x5F, // 50..5F
    0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 60..6F
    0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 70..7F
    0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 80..8F
    0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 90..9F
    0xA0, 0xA5, 0xA5, 0xA5, 0xA5, 0xA5, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // A0..AF
    0xF0, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // B0..BF
    0xF0, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // C0..CF
    0xF0, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // D0..DF
    0xF0, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // E0..EF
    0xF0, 0xF5, 0xF5, 0xF5, 0xF5, 0xF5, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  // F0..FF
};

// 8/4bpp -> 2bpp dark (0, -, 1-10, 11-15)
//
static u8 contrast_table_darker[256] =
{
    0x00, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0A, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 00..0F
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 10..1F
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 20..2F
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 30..3F
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 40..4F
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 50..5F
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 60..6F
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 70..7F
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 80..8F
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // 90..9F
    0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAF, 0xAF, 0xAF, 0xAF, 0xAF, // A0..AF
    0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // B0..BF
    0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // C0..CF
    0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // D0..DF
    0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // E0..EF
    0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF  // F0..FF
};

// 8/4bpp -> 2bpp dark (0, -, -, 1-15)
//
static u8 contrast_table_darkest[256] =
{
    0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 00..0F
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // 10..1F
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // 20..2F
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // 30..3F
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // 40..4F
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // 50..5F
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // 60..6F
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // 70..7F
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // 80..8F
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // 90..9F
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // A0..AF
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // B0..BF
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // C0..CF
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // D0..DF
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF, // E0..EF
    0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF ,0xFF ,0xFF  // F0..FF
};

// 8/4bpp -> 8/4bpp inversion
//
static u8 contrast_table_invert[256] =
{
    0xFF, 0xFE, 0xFD, 0xFC, 0xFB, 0xFA, 0xF9, 0xF8, 0xF7, 0xF6, 0xF5, 0xF4, 0xF3, 0xF2, 0xF1, 0xF0, // 00..0F
    0xEF, 0xEE, 0xED, 0xEC, 0xEB, 0xEA, 0xE9, 0xE8, 0xE7, 0xE6, 0xE5, 0xE4, 0xE3, 0xE2, 0xE1, 0xE0, // 10..1F
    0xDF, 0xDE, 0xDD, 0xDC, 0xDB, 0xDA, 0xD9, 0xD8, 0xD7, 0xD6, 0xD5, 0xD4, 0xD3, 0xD2, 0xD1, 0xD0, // 20..2F
    0xCF, 0xCE, 0xCD, 0xCC, 0xCB, 0xCA, 0xC9, 0xC8, 0xC7, 0xC6, 0xC5, 0xC4, 0xC3, 0xC2, 0xC1, 0xC0, // 30..3F
    0xBF, 0xBE, 0xBD, 0xBC, 0xBB, 0xBA, 0xB9, 0xB8, 0xB7, 0xB6, 0xB5, 0xB4, 0xB3, 0xB2, 0xB1, 0xB0, // 40..4F
    0xAF, 0xAE, 0xAD, 0xAC, 0xAB, 0xAA, 0xA9, 0xA8, 0xA7, 0xA6, 0xA5, 0xA4, 0xA3, 0xA2, 0xA1, 0xA0, // 50..5F
    0x9F, 0x9E, 0x9D, 0x9C, 0x9B, 0x9A, 0x99, 0x98, 0x97, 0x96, 0x95, 0x94, 0x93, 0x92, 0x91, 0x90, // 60..6F
    0x8F, 0x8E, 0x8D, 0x8C, 0x8B, 0x8A, 0x89, 0x88, 0x87, 0x86, 0x85, 0x84, 0x83, 0x82, 0x81, 0x80, // 70..7F
    0x7F, 0x7E, 0x7D, 0x7C, 0x7B, 0x7A, 0x79, 0x78, 0x77, 0x76, 0x75, 0x74, 0x73, 0x72, 0x71, 0x70, // 80..8F
    0x6F, 0x6E, 0x6D, 0x6C, 0x6B, 0x6A, 0x69, 0x68, 0x67, 0x66, 0x65, 0x64, 0x63, 0x62, 0x61, 0x60, // 90..9F
    0x5F, 0x5E, 0x5D, 0x5C, 0x5B, 0x5A, 0x59, 0x58, 0x57, 0x56, 0x55, 0x54, 0x53, 0x52, 0x51, 0x50, // A0..AF
    0x4F, 0x4E, 0x4D, 0x4C, 0x4B, 0x4A, 0x49, 0x48, 0x47, 0x46, 0x45, 0x44, 0x43, 0x42, 0x41, 0x40, // B0..BF
    0x3F, 0x3E, 0x3D, 0x3C, 0x3B, 0x3A, 0x39, 0x38, 0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31, 0x30, // C0..CF
    0x2F, 0x2E, 0x2D, 0x2C, 0x2B, 0x2A, 0x29, 0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21, 0x20, // D0..DF
    0x1F, 0x1E, 0x1D, 0x1C, 0x1B, 0x1A, 0x19, 0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11, 0x10, // E0..EF
    0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00, // F0..FF
};

einkfb_power_level saved_power_level = einkfb_power_level_init;
static int einkfb_fast_page_turn_counter = 0;
static u8  *einkfb_posterize_table = NULL;
static u8  *einkfb_contrast_table = NULL;
static atomic_t einkfb_lock_count = ATOMIC_INIT(0);
static EINKFB_MUTEX(einkfb_lock);

#if PRAGMAS
    #pragma mark -
    #pragma mark Local Utilites
    #pragma mark -
#endif

static int skip_vfb = 0;

#ifdef MODULE
module_param_named(skip_vfb, skip_vfb, int, S_IRUGO);
MODULE_PARM_DESC(skip_vfb, "non-zero to skip VFB");
#endif // MODULE

#define EINKFB_UPDATE_VFB_AREA(u)   \
    (skip_vfb ? false : einkfb_update_vfb_area(u))

#define EINKFB_UPDATE_VFB(u)        \
    (skip_vfb ? false : einkfb_update_vfb(u))

struct vfb_blit_t
{
    bool buffers_equal;
    u8 *src, *dst;
};
typedef struct vfb_blit_t vfb_blit_t;

static void einkfb_vfb_blit(int x, int y, int rowbytes, int bytes, void *data)
{
    vfb_blit_t *vfb_blit = (vfb_blit_t *)data;
    int dst_bytes  = (rowbytes * y) + x;
    u8  old_pixels = vfb_blit->dst[dst_bytes];
    
    vfb_blit->dst[dst_bytes] = vfb_blit->src[bytes];
    vfb_blit->buffers_equal &= old_pixels == vfb_blit->dst[dst_bytes];
}

static bool einkfb_buffers_equal(bool buffers_equal, fx_type update_mode)
{
    struct einkfb_info info;
    bool result = false;

    einkfb_get_info(&info);

    // We don't want to skip a buffer update, even if the buffers equal,
    // when we're in an update mode where skipping a buffer update
    // wouldn't work.
    //
    result = buffers_equal && !SKIP_BUFFERS_EQUAL(update_mode, info.contrast);
    
    // And we should NEVER skip a buffer update when it's been requested
    // that we don't do such a thing.
    //
    if ( !EINKFB_SKIP_BUFS_EQ() )
        result = false;

    return ( result );
}

static bool einkfb_update_vfb_area(update_area_t *update_area)
{
    // If we get here, the update_area has already been validated.  So, all we
    // need to do is blit things into the virtual framebuffer at the right
    // spot.
    //
    u32 x_start, x_end, area_xres,
        y_start, y_end, area_yres;

    u8  *fb_start, *area_start;
    
    vfb_blit_t vfb_blit;
    
    struct einkfb_info info;
    einkfb_get_info(&info);
    
    area_xres  = update_area->x2 - update_area->x1;
    area_yres  = update_area->y2 - update_area->y1;
    area_start = update_area->buffer;
    
    fb_start   = info.vfb;

    // Get the (x1, y1)...(x2, y2) offset into the framebuffer.
    //
    x_start = update_area->x1;
    x_end   = x_start + area_xres;
    
    y_start = update_area->y1;
    y_end   = y_start + area_yres;
    
    // Blit the area data into the virtual framebuffer.
    //
    vfb_blit.buffers_equal = true;
    vfb_blit.src = area_start;
    vfb_blit.dst = fb_start;
    
    einkfb_blit(x_start, x_end, y_start, y_end, einkfb_vfb_blit, (void *)&vfb_blit);

    // Say that an update-display event has occurred if the buffers aren't equal.
    //
    if ( !vfb_blit.buffers_equal )
    {
        einkfb_event_t event; einkfb_init_event(&event);
        
        event.update_mode = UPDATE_AREA_MODE(update_area->which_fx);
        event.x1 = update_area->x1; event.x2 = update_area->x2;
        event.y1 = update_area->y1; event.y2 = update_area->y2;
        event.event = einkfb_event_update_display_area;
        
        einkfb_post_event(&event);
    }

    return ( einkfb_buffers_equal(vfb_blit.buffers_equal, update_area->which_fx) );
}

static bool einkfb_update_vfb(fx_type update_mode)
{
    bool buffers_equal = true;
    
    struct einkfb_info info;
    einkfb_get_info(&info);
    
    // Copy the real framebuffer into the virtual framebuffer if we're
    // not doing a restore.
    //
    if ( EINKFB_RESTORE(info) )
    {
        // On restores, say that the buffers aren't equal to force
        // an update.
        //
        buffers_equal = false;
    }
    else
    {    
        fb_apply_fx_t fb_apply_fx = get_fb_apply_fx();
        int i, len;

        if ( fb_apply_fx )
        {
            u8 old_pixels, *vfb = info.vfb, *fb = info.start;
            len = info.size;
            
            for ( i = 0; i < len; i++ )
            {
                old_pixels = vfb[i];
                vfb[i] = fb_apply_fx(fb[i], i);
                
                buffers_equal &= old_pixels == vfb[i];
                EINKFB_SCHEDULE_BLIT(i+1);
            }
        }
        else
        {
            u32 old_pixels, *vfb = (u32 *)info.vfb, *fb = (u32 *)info.start;
            len = info.size >> 2;
            
            for ( i = 0; i < len; i++ )
            {
                old_pixels = vfb[i];
                vfb[i] = fb[i];
                
                buffers_equal &= old_pixels == vfb[i];
            }
            
            if ( EINKFB_MEMCPY_MIN < i )
                EINKFB_SCHEDULE();
        }
    }

    // Say that an update-display event has occurred if the buffers aren't equal.
    //
    if ( !buffers_equal )
    {
        einkfb_event_t event; einkfb_init_event(&event);

        event.event = einkfb_event_update_display;
        event.update_mode = UPDATE_MODE(update_mode);
    
        einkfb_post_event(&event);
    }

    return ( einkfb_buffers_equal(buffers_equal, update_mode) );
}

#if PRAGMAS
    #pragma mark -
    #pragma mark Locking Utilities
    #pragma mark -
#endif

static int einkfb_begin_power_on_activity(void)
{
    int result = EINKFB_FAILURE;
    struct einkfb_info info;
    
    einkfb_get_info(&info);

    if ( info.dev )
    {
        // Save the current power level.
        //
        saved_power_level = EINKFB_GET_POWER_LEVEL();
           
        // Only power on if it's allowable to do so at the moment.
        //
        if ( POWER_OVERRIDE() )
        {
            result = EINKFB_SUCCESS;
        }
        else
        {
            switch ( saved_power_level )
            {
                case einkfb_power_level_init:
                    saved_power_level = einkfb_power_level_on;

                case einkfb_power_level_on:
                case einkfb_power_level_standby:
                    einkfb_prime_power_timer(EINKFB_DELAY_TIMER);
                case einkfb_power_level_blank:
                    result = EINKFB_SUCCESS;
                break;
                
                default:
                    einkfb_debug_power("power (%s) lockout, ignoring call\n",
                        einkfb_get_power_level_string(saved_power_level));
                    
                    result = EINKFB_FAILURE;
                break;
            }
        }

        // Power on if need be and allowable.
        //
        if ( EINKFB_SUCCESS == result )
        {
            einkfb_set_jif_on(jiffies);
            
            if ( einkfb_power_level_on != saved_power_level )
                EINKFB_SET_POWER_LEVEL(einkfb_power_level_on);
        }
    }
    
    return ( result );
}

static void einkfb_end_power_on_activity(void)
{
    struct einkfb_info info; einkfb_get_info(&info);

    if ( info.dev )
    {
        einkfb_power_level power_level = EINKFB_GET_POWER_LEVEL();
        
        // Only restore the saved power level if we haven't been purposely
        // taken out of the "on" level.
        //
        if ( einkfb_power_level_on == power_level )
            EINKFB_SET_POWER_LEVEL(saved_power_level);
        else
            saved_power_level = power_level;
    }
}

bool einkfb_lock_ready(bool release)
{
    bool ready = false;
    
    if ( release )
    {
        ready = EINKFB_SUCCESS == down_trylock(&einkfb_lock);
        
        if ( ready )
            einkfb_lock_release();
    }
    else
    {
        einkfb_inc_lock_count();
        ready = EINKFB_SUCCESS == einkfb_down(&einkfb_lock);
        
        if ( !ready )
            einkfb_dec_lock_count();
    }
    
    return ( ready );
}

void einkfb_lock_release(void)
{
    up(&einkfb_lock);
    einkfb_dec_lock_count();
}

int einkfb_lock_entry(char *function_name)
{
    int result = EINKFB_FAILURE;
    
    if ( EINK_DISPLAY_ASIS() )
    {
        einkfb_debug_power("%s: display asis, ignoring call\n", function_name);
    }
    else
    {
        einkfb_debug_lock("%s: getting power lock...\n", function_name);
    
        if ( EINKFB_LOCK_READY() )
        {
            einkfb_debug_lock("%s: got lock, getting power...\n", function_name);
            result = einkfb_begin_power_on_activity();
    
            if ( EINKFB_SUCCESS == result )
            {
                einkfb_debug_lock("%s: got power...\n", function_name);
            }
            else
            {
                einkfb_debug_lock("%s: could not get power, releasing lock\n", function_name);
                EINFFB_LOCK_RELEASE();
            }
        }
        else
        {
            einkfb_debug_lock("%s: could not get lock, bailing\n", function_name);
        }
    }

	return ( result );
}

void einkfb_lock_exit(char *function_name)
{
    if ( EINK_DISPLAY_ASIS() )
    {
        einkfb_debug_power("%s: display asis, ignoring call\n", function_name);
    }
    else
    {
        einkfb_end_power_on_activity();
        EINFFB_LOCK_RELEASE();
    
        einkfb_debug_lock("%s released power, released lock\n", function_name);
    }
}

bool einkfb_busy(void)
{
    int result = atomic_read(&einkfb_lock_count);
    einkfb_debug_lock("lock count = %d\n", result);
    
    return ( 0 != result );
}

void einkfb_inc_lock_count(void)
{
    int result = atomic_inc_return(&einkfb_lock_count);
    einkfb_debug_lock("lock count = %d\n", result);
}

void einkfb_dec_lock_count(void)
{
    int result = atomic_dec_return(&einkfb_lock_count);
    
    if ( 0 > result )
    {
        einkfb_debug_lock("lock count went negative, resetting to 0\n");
        atomic_set(&einkfb_lock_count, 0);
    }
    
    einkfb_debug_lock("lock count = %d\n", result);
}

#if PRAGMAS
    #pragma mark -
    #pragma mark Display Utilities
    #pragma mark -
#endif

void einkfb_blit(int xstart, int xend, int ystart, int yend, einkfb_blit_t blit, void *data)
{
    if ( blit )
    {
        int x, y, rowbytes, bytes;
        
        struct einkfb_info info;
        einkfb_get_info(&info);
    
        // Make bpp-related adjustments.
        //
        xstart   = BPP_SIZE(xstart,    info.bpp);
        xend     = BPP_SIZE(xend,	   info.bpp);
        rowbytes = BPP_SIZE(info.xres, info.bpp);
    
        // Blit EINKFB_MEMCPY_MIN bytes at a time before yielding.
        //
        for (bytes = 0, y = ystart; y < yend; y++ )
        {
            for ( x = xstart; x < xend; x++ )
            {
                (*blit)(x, y, rowbytes, bytes, data);
                EINKFB_SCHEDULE_BLIT(++bytes);
            }
        }
    }
}

static einkfb_bounds_failure bounds_failure = einkfb_bounds_failure_none;

einkfb_bounds_failure einkfb_get_last_bounds_failure(void)
{
    return ( bounds_failure );
}

bool einkfb_bounds_are_acceptable(int xstart, int xres, int ystart, int yres)
{
    bool acceptable = true;
    int alignment_x, alignment_y;
    
    struct einkfb_info info;
    einkfb_get_info(&info);
    
    bounds_failure = einkfb_bounds_failure_none;
    alignment_x = BPP_BYTE_ALIGN(info.align_x);
    alignment_y = BPP_BYTE_ALIGN(info.align_y);

    // The limiting boundary must be of non-zero size and be within the screen's boundaries.
    //
    acceptable &= ((0 < xres) && IN_RANGE(xres, 0, info.xres));
    
    if ( !acceptable )
        bounds_failure |= einkfb_bounds_failure_x1;

    acceptable &= ((0 < yres) && IN_RANGE(yres, 0, info.yres));
    
    if ( !acceptable )
        bounds_failure |= einkfb_bounds_failure_y1;
    
    // The bounds must be and fit within the framebuffer.
    //
    acceptable &= ((0 <= xstart) && ((xstart + xres) <= info.xres));

    if ( !acceptable )
        bounds_failure |= einkfb_bounds_failure_x2;

    acceptable &= ((0 <= ystart) && ((ystart + yres) <= info.yres));
    
    if ( !acceptable )
        bounds_failure |= einkfb_bounds_failure_y2;

    // Our horizontal starting and ending points must be byte aligned.
    //
    acceptable &= ((0 == (xstart % alignment_x)) && (0 == (xres % alignment_x)));
    
    if ( !acceptable )
        bounds_failure |= einkfb_bounds_failure_x;

    // Our vertical starting and ending points must be byte aligned.
    //
    acceptable &= ((0 == (ystart % alignment_y)) && (0 == (yres % alignment_y)));
    
    if ( !acceptable )
        bounds_failure |= einkfb_bounds_failure_y;
        
    // If debugging is enabled, print out the apparently errant
    // passed-in values.
    //
    if ( !acceptable )
    {
        einkfb_debug("Image couldn't be rendered:\n");
        einkfb_debug(" Screen bounds:         %4d x %4d\n", info.xres, info.yres);
        einkfb_debug(" Image resolution:      %4d x %4d\n", xres, yres);
        einkfb_debug(" Image offset:          %4d x %4d\n", xstart, ystart);
        einkfb_debug(" Image row start align: %4d\n",       (xstart % alignment_x));
        einkfb_debug(" Image width align:     %4d\n",       (xres % alignment_x));
        einkfb_debug(" Image col start align: %4d\n",       (ystart % alignment_y));
        einkfb_debug(" Image height align:    %4d\n",       (yres % alignment_y));
	}

	return ( acceptable );
}

bool einkfb_align_bounds(rect_t *rect)
{
    int xstart = rect->x1, xend = rect->x2,
        ystart = rect->y1, yend = rect->y2,
        
        xres = xend - xstart,
        yres = yend - ystart,
        
        alignment_x, alignment_y;
    
    struct einkfb_info info;
    bool aligned = false;
    
    einkfb_get_info(&info);
    alignment_x = BPP_BYTE_ALIGN(info.align_x);
    alignment_y = BPP_BYTE_ALIGN(info.align_y);
    
    // Only re-align the bounds that aren't aligned.
    //
    if ( 0 != (xstart % alignment_x) )
    {
        xstart = BPP_BYTE_ALIGN_DN(xstart, info.align_x);
        xres = xend - xstart;
    }

    if ( 0 != (xres % alignment_x) )
    {
        xend = BPP_BYTE_ALIGN_UP(xend, info.align_x);
        xres = xend - xstart;
    }

    // Only re-align the bounds that aren't aligned.
    //
    if ( 0 != (ystart % alignment_y) )
    {
        ystart = BPP_BYTE_ALIGN_DN(ystart, info.align_y);
        yres = yend - ystart;
    }

    if ( 0 != (yres % alignment_y) )
    {
        yend = BPP_BYTE_ALIGN_UP(yend, info.align_y);
        yres = yend - ystart;
    }

    // If the re-aligned bounds are acceptable, use them.
    //
    if ( einkfb_bounds_are_acceptable(xstart, xres, ystart, yres) )
    {
        einkfb_debug("x bounds re-aligned, x1: %d -> %d; x2: %d -> %d\n",
            rect->x1, xstart, rect->x2, xend);

        einkfb_debug("y bounds re-aligned, y1: %d -> %d; y2: %d -> %d\n",
            rect->y1, ystart, rect->y2, yend);
        
        rect->x1 = xstart;
        rect->x2 = xend;

        rect->y1 = ystart;
        rect->y2 = yend;
        
        aligned = true;
    }
    
    return ( aligned );
}

unsigned char einkfb_stretch_nybble(unsigned char nybble, unsigned long bpp)
{
    unsigned char *which_nybble_table = NULL, result = nybble;

    switch ( nybble )
    {
        // Special-case the table endpoints since they're always the same.
        //
        case 0x00:
            result = EINKFB_WHITE;
        break;
        
        case 0x0F:
            result = EINKFB_BLACK;
        break;
        
        // Handle everything else on a bit-per-pixel basis.
        //
        default:
            switch ( bpp )
            {
                case EINKFB_1BPP:
                    which_nybble_table = stretch_nybble_table_1_to_2bpp;
                break;
                
                case EINKFB_2BPP:
                    which_nybble_table = stretch_nybble_table_2_to_4bpp;
                break;
                
                case EINKFB_4BPP:
                    which_nybble_table = stretch_nybble_table_4_to_8bpp;
                break;
            }
            
            if ( which_nybble_table )
                result = which_nybble_table[nybble];
        break;
    }

    return ( result );
}

void einkfb_posterize_to_1bpp_begin(void)
{
    // Ensure that we're truly starting over.
    //
    einkfb_posterize_to_1bpp_end();
    
    // If the hardware needs to override the eInk HAL's 1bpp posterization tables,
    // then let it do so.
    //
    if ( hal_ops.hal_posterize_table )
        einkfb_posterize_table = hal_ops.hal_posterize_table();

    // Otherwise, use the eInk HAL's 1bpp posterization tables.
    //
    if ( !einkfb_posterize_table )
    {
        struct einkfb_info info;
        einkfb_get_info(&info);

        // Get the table that tranlates 1 byte's worth of pixels per depth
        // into their corresponding 1bpp posterization values.
        //
        switch ( info.bpp )
        {
            case EINKFB_2BPP:
                einkfb_posterize_table = posterize_table_2bpp;
            break;
            
            case EINKFB_4BPP:
                einkfb_posterize_table = posterize_table_4bpp;
            break;
            
            case EINKFB_8BPP:
                einkfb_posterize_table = posterize_table_8bpp;
            break;
            
            default:
                einkfb_posterize_table = NULL;
            break;
        }
    }
    
    // We exploit the FX mechanism in order to get posterization to work for full-screen
    // updates.
    //
    if ( einkfb_posterize_table )
        set_fb_apply_fx(einkfb_posterize_to_1bpp);
}

void einkfb_posterize_to_1bpp_end(void)
{
    if ( einkfb_posterize_table )
    {
        set_fb_apply_fx(NULL);
        einkfb_posterize_table = NULL;
    }
}

u8 einkfb_posterize_to_1bpp(u8 data, int i)
{
    u8 result = data;

    if ( einkfb_posterize_table )
        result = einkfb_posterize_table[data];
    
    return ( result );
}

void einkfb_contrast_begin(void)
{
    struct einkfb_info info;
    einkfb_get_info(&info);

    // Ensure that we're truly starting over.
    //
    einkfb_contrast_end();
    
    // We can only apply contrast when we're at either 4bpp or 8bpp.
    //
    if ( (EINKFB_4BPP == info.bpp) || (EINKFB_8BPP == info.bpp) )
    {
        // Get the table that tranlates 1 byte's worth of pixels into the
        // desired contrast (light, medium, or dark).
        //
        switch ( info.contrast )
        {
            case contrast_lightest:
                einkfb_contrast_table = contrast_table_lightest;
            break;
            
            case contrast_lighter:
                einkfb_contrast_table = contrast_table_lighter;
            break;
            
            case contrast_light:
                einkfb_contrast_table = contrast_table_light;
            break;
            
            case contrast_medium:
                einkfb_contrast_table = contrast_table_medium;
            break;
            
            case contrast_dark:
                einkfb_contrast_table = contrast_table_dark;
            break;
            
            case contrast_darker:
                einkfb_contrast_table = contrast_table_darker;
            break;
            
            case contrast_darkest:
                einkfb_contrast_table = contrast_table_darkest;
            break;
            
            case contrast_invert:
                einkfb_contrast_table = contrast_table_invert;
            break;
            
            case contrast_off:
            default:
                einkfb_contrast_table = NULL;
            break;
        }
        
        // We exploit the FX mechanism in order to get contrast to work for full-screen
        // updates.
        //
        if ( einkfb_contrast_table )
            set_fb_apply_fx(einkfb_apply_contrast);
    }
}

void einkfb_contrast_end(void)
{
    if ( einkfb_contrast_table )
    {
        set_fb_apply_fx(NULL);
        einkfb_contrast_table = NULL;
    }
}

u8 einkfb_apply_contrast(u8 data, int i)
{
    u8 result = data;
    
    if ( einkfb_contrast_table )
        result = einkfb_contrast_table[data];
        
    return ( result );
}

void einkfb_display_grayscale_ramp(void)
{
    int row, num_rows, row_bytes, row_size, row_height, height, adj_count, adj_start;
    u8 row_gray, *gray_table = NULL, *row_start = NULL;
    struct einkfb_info info;
    einkfb_get_info(&info);
    
    // Get the table that translates pixels into bytes per bit depth.
    //
    switch ( info.bpp )
    {
        case EINKFB_1BPP:
            gray_table = gray_table_1bpp;
        break;
        
        case EINKFB_2BPP:
            gray_table = gray_table_2bpp;
        break;
        
        case EINKFB_4BPP:
            gray_table = gray_table_4bpp;
        break;
    }
    
    // Divide the display into the appropriate number of rows.
    //
    row_bytes = BPP_SIZE(info.xres, info.bpp);
    num_rows  = 1 << info.bpp;
    row_start = info.start;
    
    // In order to run the rows to the end, we need to adjust
    // row_height adj_count times.  We do this by "centering"
    // adj_count to the num_rows.  We're zero-based (i), so we
    // don't add 1 to adj_start here
    //
    row_height = info.yres / num_rows;
    adj_count  = info.yres % num_rows;
    adj_start  = adj_count ? (num_rows - adj_count) >> 1 : 0;
    
    for ( row = 0; row < num_rows; row++ )
    {
        // Determine height.
        //
        height = row_height;
        
        // Quit adjusting height when/if adj_count is zero or
        // less (as noted above, adj_count is "centered" to
        // num_rows).
        //
        if ( 0 < adj_count )
        {
            if ( row >= adj_start )
            {
                adj_count--;
                height++;
            }
        }
        
        // Blit the appropriate gray into the framebuffer for
        // this row.
        //
        row_gray = gray_table ? gray_table[row] : einkfb_pixels(info.bpp, (u8)row);
        row_size = row_bytes * height;
        
        einkfb_memset(row_start, row_gray, row_size);
        row_start += row_size;
    }

    // Now, update the display with our grayscale ramp.
    //
    einkfb_update_display(fx_update_full);
}

#if PRAGMAS
    #pragma mark -
    #pragma mark Update Utilities
    #pragma mark -
#endif

#define UPDATE_TIMING_TOTL_TYPE 0
#define UPDATE_TIMING_VIRT_TYPE 1
#define UPDATE_TIMING_REAL_TYPE 2

#define UPDATE_TIMING_ID_LEN    32

#define UPDATE_TIMING           "update_timing_%s"
#define UPDATE_TIMING_TOTL_NAME "totl"
#define UPDATE_TIMING_VIRT_NAME "virt"
#define UPDATE_TIMING_REAL_NAME "real"

#define UPDATE_TIMING_AREA      "area"
#define UPDATE_TIMING_DISP      "disp"

static void einkfb_print_update_timing(unsigned long time, int which, char *kind)
{
    char *name = NULL, id[UPDATE_TIMING_ID_LEN];

    sprintf(id, UPDATE_TIMING, kind);
    
    switch ( which )
    {
        case UPDATE_TIMING_VIRT_TYPE:
            name = UPDATE_TIMING_VIRT_NAME;
        goto relative_common;
        
        case UPDATE_TIMING_REAL_TYPE:
            name = UPDATE_TIMING_REAL_NAME;
        /* goto relative_common; */
        
        relative_common:
            EINKFB_PRINT_PERF_REL(id, time, name);
        break;
        
        case UPDATE_TIMING_TOTL_TYPE:
            EINKFB_PRINT_PERF_ABS(id, time, UPDATE_TIMING_TOTL_NAME);
        break;
    }
}

#define EINKFB_BUFFERS_EQUAL_MESSAGE_AREA_UPDATE    "area_update(%s):x1=%d,x2=%d,y1=%d,y2=%d"
#define EINKFB_BUFFERS_EQUAL_MESSAGE_FULL_SCREEN    "full_screen(%s)"
#define EINKFB_BUFFERS_EQUAL_MESSAGE_SIZE           256
#define EINKFB_BUFFERS_EQUAL_MESSAGE                "%s\n"

#define EINKFB_BUFFERS_EQUAL_MESSAGE_PART           "non-flashing"
#define EINKFB_BUFFERS_EQUAL_MESSAGE_FULL           "flashing"

#define EINKFB_PRINT_BUFFERS_EQUAL_MESSAGE_AREA(u)  \
    einkfb_print_buffers_equal_message(false, fx_none, u)
#define EINKFB_PRINT_BUFFERS_EQUAL_MESSAGE_FULL(m)  \
    einkfb_print_buffers_equal_message(true, m, NULL)

static void einkfb_print_buffers_equal_message(bool full_screen, fx_type update_mode, update_area_t *update_area)
{
    if ( EINKFB_DEBUG() )
    {
        char einkfb_buffers_equal_message[EINKFB_BUFFERS_EQUAL_MESSAGE_SIZE] = { 0 };    
    
        if ( full_screen )
        {
            sprintf(einkfb_buffers_equal_message, EINKFB_BUFFERS_EQUAL_MESSAGE_FULL_SCREEN,
               (UPDATE_MODE(update_mode) ? EINKFB_BUFFERS_EQUAL_MESSAGE_FULL :
                                           EINKFB_BUFFERS_EQUAL_MESSAGE_PART));
        }
        else
        {
            sprintf(einkfb_buffers_equal_message, EINKFB_BUFFERS_EQUAL_MESSAGE_AREA_UPDATE,
                (UPDATE_AREA_MODE(update_area->which_fx) ? EINKFB_BUFFERS_EQUAL_MESSAGE_FULL :
                                                           EINKFB_BUFFERS_EQUAL_MESSAGE_PART),
                update_area->x1,
                update_area->x2,
                update_area->y1,
                update_area->y2);
        }
        
        einkfb_debug(EINKFB_BUFFERS_EQUAL_MESSAGE, einkfb_buffers_equal_message);
    }
}

void einkfb_update_display_area(update_area_t *update_area)
{
    if ( update_area )
    {
        unsigned long strt_time = jiffies, virt_strt = strt_time, virt_stop,
                      real_strt = 0, real_stop = 0, stop_time;
        
        // Update the virtual display.
        //
        bool buffers_equal = EINKFB_UPDATE_VFB_AREA(update_area);
        virt_stop = jiffies;
        
        // Update the real display if the buffer actually changed.
        //
        if ( !buffers_equal && hal_ops.hal_update_area && (EINKFB_SUCCESS == EINKFB_LOCK_ENTRY()) )
        {
            real_strt = jiffies;
            hal_ops.hal_update_area(update_area);
            real_stop = jiffies;
            
            EINKFB_LOCK_EXIT();
        }
        
        stop_time = jiffies;
        
        if ( buffers_equal )
            EINKFB_PRINT_BUFFERS_EQUAL_MESSAGE_AREA(update_area);
            
        if ( EINKFB_PERF() )
        {
            einkfb_print_update_timing(jiffies_to_msecs(virt_stop - virt_strt),
                UPDATE_TIMING_VIRT_TYPE, UPDATE_TIMING_AREA);
            
            if ( real_strt )
                einkfb_print_update_timing(jiffies_to_msecs(real_stop - real_strt),
                    UPDATE_TIMING_REAL_TYPE, UPDATE_TIMING_AREA);
        
            einkfb_print_update_timing(jiffies_to_msecs(stop_time - strt_time),
                UPDATE_TIMING_TOTL_TYPE, UPDATE_TIMING_AREA);
        }
        
        // For all area updates, keep kicking the event timer out.  For a series of non-flashing
        // area updates, this will keep the update_display_complete event from happening until
        // the display is no longer in a potentially "torn" state.
        // 
        einkfb_prime_event_timer(EINKFB_DELAY_TIMER);
    }
}

void einkfb_update_display_sync(void)
{
    // The sync doesn't go through the normal update-display path because we're
    // not actually touching the display in this case.  Rather, we're simply
    // asking the controller to block (on whatever thread is calling us) until
    // the last display update is complete (which might be immediately if
    // there are no display updates pending).
    //
    if ( hal_ops.hal_update_display && (EINKFB_SUCCESS == EINKFB_LOCK_ENTRY()) )
    {
        hal_ops.hal_update_display(fx_display_sync);
        EINKFB_LOCK_EXIT();
    }
}

void einkfb_update_display(fx_type update_mode)
{
    unsigned long strt_time = jiffies, virt_strt = strt_time, virt_stop,
                  real_strt = 0, real_stop = 0,  stop_time;
    bool buffers_equal, cancel_restore = false;
    struct einkfb_info info;
    einkfb_get_info(&info);

    // For all full-screen updates, first prevent any pending update_display_complete events
    // from occurring until after we complete this update.
    //
    einkfb_prime_event_timer(EINKFB_EXPIRE_TIMER);

    switch ( update_mode )
    {
        case fx_update_fast:
        case fx_update_fast_invert:
            // If we're starting up a new fast page turn cycle, we must get
            // the screen cleared first.  But, obviously, we don't want to
            // clear the public framebuffer.  So, we do this by using the
            // private (kernel) buffer.
            //
            if ( 0 == einkfb_fast_page_turn_counter++ )
                EINKFB_IOCTL(FBIO_EINK_CLEAR_SCREEN, EINK_CLEAR_SCREEN_NOT_BUFFER);
            
            // Next, while we're in fast page turn mode, we must posterize the
            // framebuffer data to 1bpp.
            //
            einkfb_posterize_to_1bpp_begin();
        
        break;
        
        case fx_update_slow:
            
            // Fast page turn mode is a protocol that requires the screen to again
            // be cleared before going back to the more normal update modes.
            //
            if ( einkfb_fast_page_turn_counter )
            {
                einkfb_fast_page_turn_counter = 0;
                EINKFB_IOCTL(FBIO_EINK_CLEAR_SCREEN, EINK_CLEAR_SCREEN_NOT_BUFFER);
            }
            else
                goto update_mode_common;
        break;
        
        update_mode_common:
        default:

            // Apply contrast if it's been set.
            //
            if ( info.contrast )
                einkfb_contrast_begin();
        
        break;
    }
    
    // Update the virtual display.
    //
    buffers_equal = EINKFB_UPDATE_VFB(update_mode);
    virt_stop = jiffies;

    // If the buffers aren't the same, check to see whether we're doing a restore.
    //
    if ( !buffers_equal )
    {
        // If we're not doing a restore but we applied an FX, force a restore so
        // that the module doesn't have to apply the FX.
        //
        if ( !EINKFB_RESTORE(info) )
        {
            fb_apply_fx_t fb_apply_fx = get_fb_apply_fx();
            
            if ( fb_apply_fx )
            {
                einkfb_set_fb_restore(true);
                cancel_restore = true;
            }
        }
    }
    
    // Update the real display if the buffer actually changed.
    //
    if ( !buffers_equal && hal_ops.hal_update_display && (EINKFB_SUCCESS == EINKFB_LOCK_ENTRY()) )
    {
        real_strt = jiffies;
        hal_ops.hal_update_display(update_mode);
        real_stop = jiffies;
        
        EINKFB_LOCK_EXIT();
    }

    stop_time = jiffies;

    // Take us out of restore mode and/or de-apply the fx if we put ourselves there.
    //
    if ( cancel_restore )
        einkfb_set_fb_restore(false);
        
    einkfb_posterize_to_1bpp_end();
    einkfb_contrast_end();

    if ( buffers_equal )
        EINKFB_PRINT_BUFFERS_EQUAL_MESSAGE_FULL(update_mode);
        
    if ( EINKFB_PERF() )
    {
        einkfb_print_update_timing(jiffies_to_msecs(virt_stop - virt_strt),
            UPDATE_TIMING_VIRT_TYPE, UPDATE_TIMING_DISP);
        
        if ( real_strt )
            einkfb_print_update_timing(jiffies_to_msecs(real_stop - real_strt),
                UPDATE_TIMING_REAL_TYPE, UPDATE_TIMING_DISP);
    
        einkfb_print_update_timing(jiffies_to_msecs(stop_time - strt_time),
            UPDATE_TIMING_TOTL_TYPE, UPDATE_TIMING_DISP);
    }
    
    // Say that we'd like an update_display_complete event when this update is done.
    //
    einkfb_prime_event_timer(EINKFB_DELAY_TIMER);
}

void einkfb_restore_display(fx_type update_mode)
{
    // Switch the real display to the virtual one.
    //
    einkfb_set_fb_restore(true);
    
    // Update the real display with the virtual one's data.
    //
    einkfb_update_display(update_mode); 
    
    // Switch back to the real display.
    //
    einkfb_set_fb_restore(false);
}

void einkfb_clear_display(fx_type update_mode)
{
    struct einkfb_info info; einkfb_get_info(&info);
    einkfb_memset(info.start, einkfb_white(info.bpp), info.size);
    
    einkfb_update_display(update_mode);
}

#if PRAGMAS
    #pragma mark -
    #pragma mark Orientation Utilities
    #pragma mark -
#endif

bool einkfb_set_display_orientation(orientation_t orientation)
{
    bool result = false;
    
    if ( hal_ops.hal_set_display_orientation && (EINKFB_SUCCESS == EINKFB_LOCK_ENTRY()) )
    {
        result = hal_ops.hal_set_display_orientation(orientation);
        EINKFB_LOCK_EXIT();
        
        // Post a rotate event if setting display orientation succeeded.
        //
        if ( result )
        {
            einkfb_event_t event;
            einkfb_init_event(&event);
            
            event.event = einkfb_event_rotate_display;
            event.orientation = orientation;
            
            einkfb_post_event(&event);
        }
    }

    return ( result );
}

orientation_t einkfb_get_display_orientation(void)
{
    orientation_t orientation = orientation_portrait;

    if ( hal_ops.hal_get_display_orientation && (EINKFB_SUCCESS == EINKFB_LOCK_ENTRY()) )
    {
        orientation = hal_ops.hal_get_display_orientation();
        EINKFB_LOCK_EXIT();
    }
    else
    {
        struct einkfb_info info; einkfb_get_info(&info);
        
        if ( EINK_ORIENT_LANDSCAPE == ORIENTATION(info.xres, info.yres) )
            orientation = orientation_landscape;
    }

    return ( orientation );
}

#if PRAGMAS
    #pragma mark -
    #pragma mark Schedule Utilities
    #pragma mark -
#endif

#define FORCE_INTERRUPTIBLE()       einkfb_get_force_interruptible()
#define UNINTERRUPTIBLE             false
#define INTERRUPTIBLE               true

#define INTERRUPTIBLE_TIMEOUT_COUNT 3

static int einkfb_schedule_timeout_guts(unsigned long hardware_timeout, einkfb_hardware_ready_t hardware_ready, void *data, bool interruptible)
{
    unsigned long start_time = jiffies, stop_time = start_time + hardware_timeout,
        timeout = EINKFB_TIMEOUT_MIN;
    int result = EINKFB_SUCCESS;

    // Ask the hardware whether it's ready or not.  And, if it's not ready, start yielding
    // the CPU for EINKFB_TIMEOUT_MIN jiffies, increasing the yield time up to
    // EINKFB_TIMEOUT_MAX jiffies.  Time out after the requested number of
    // of jiffies has occurred.
    //
    while ( !(*hardware_ready)(data) && time_before_eq(jiffies, stop_time) )
    {
        timeout = min(timeout++, EINKFB_TIMEOUT_MAX);
        
        if ( interruptible )
            schedule_timeout_interruptible(timeout);
        else
            schedule_timeout(timeout);
    }

    if ( time_after(jiffies, stop_time) )
    {
       einkfb_print_crit("Timed out waiting for the hardware to become ready!\n");
       result = EINKFB_FAILURE;
    }
    else
    {
        // For debugging purposes, dump the time it took for the hardware to
        // become ready if it was more than EINKFB_TIMEOUT_MAX.
        //
        stop_time = jiffies - start_time;
        
        if ( EINKFB_TIMEOUT_MAX < stop_time )
            einkfb_debug("Timeout time = %ld\n", stop_time);
    }

    return ( result );    
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26))
static int einkfb_schedule_timeout_uninterruptible(unsigned long hardware_timeout, einkfb_hardware_ready_t hardware_ready, void *data)
{
    return ( einkfb_schedule_timeout_guts(hardware_timeout, hardware_ready, data, UNINTERRUPTIBLE) );
}
#else
static int einkfb_schedule_timeout_uninterruptible(unsigned long hardware_timeout, einkfb_hardware_ready_t hardware_ready, void *data)
{
    int timeout_count = 0, result = EINKFB_FAILURE;
    
    // In the uninterruptible case, we try to be more CPU friendly by first doing things
    // in an interruptible way.
    //
    while ( (INTERRUPTIBLE_TIMEOUT_COUNT > timeout_count++) && (EINKFB_FAILURE == result) )
        result = einkfb_schedule_timeout_guts(hardware_timeout, hardware_ready, data, INTERRUPTIBLE);
    
    // It's possible that we haven't been able to access the hardware enough times by doing
    // things in an interruptible way.  So, try again once more in an uninterruptible way.
    //
    if ( EINKFB_FAILURE == result )
        result = einkfb_schedule_timeout_guts(hardware_timeout, hardware_ready, data, UNINTERRUPTIBLE);

    return ( result );
}
#endif

int einkfb_schedule_timeout(unsigned long hardware_timeout, einkfb_hardware_ready_t hardware_ready, void *data, bool interruptible)
{
    int result = EINKFB_SUCCESS;
    
    if ( hardware_timeout && hardware_ready )
    {
        if ( FORCE_INTERRUPTIBLE() || interruptible )
            result = einkfb_schedule_timeout_guts(hardware_timeout, hardware_ready, data, INTERRUPTIBLE);
        else
            result = einkfb_schedule_timeout_uninterruptible(hardware_timeout, hardware_ready, data);
    }
    else
    {
        // Yield the CPU with schedule.
        //
        einkfb_debug("Yielding CPU with schedule.\n");
        schedule();
    }
    
    return ( result );
}

#if PRAGMAS
	#pragma mark -
	#pragma mark Compression/Decompression Utilities
	#pragma mark -
#endif

#define HEAD_CRC	2
#define EXTRA_FIELD	4
#define ORIG_NAME	8
#define COMMENT		0x10
#define RESERVED	0xe0

#define DEFLATED	8

#define HEADER_0	0x1f
#define HEADER_1	0x8b
#define BEST_COMP	2
#define UNIX_OS		3

static void *z_inflate_workspace = NULL;
static void *z_deflate_workspace = NULL;

static int init_z_inflate_workspace(void)
{
	int result = 0;
	
	if (!z_inflate_workspace) {
		z_inflate_workspace = kzalloc(zlib_inflate_workspacesize(), GFP_ATOMIC);
	}
	
	if (z_inflate_workspace) {
		result = 1;
	}

	return (result);
}

static void done_z_inflate_workspace(void)
{
	if (z_inflate_workspace) {
		kfree(z_inflate_workspace);
		z_inflate_workspace = NULL;
	}
}

static int init_z_deflate_workspace(void)
{
	int result = 0;
	
	if (!z_deflate_workspace) {
		z_deflate_workspace = kzalloc(zlib_deflate_workspacesize(), GFP_ATOMIC);
	}
	
	if (z_deflate_workspace) {
		result = 1;
	}

	return (result);
}

static void done_z_deflate_workspace(void)
{
	if (z_deflate_workspace) {
		kfree(z_deflate_workspace);
		z_deflate_workspace = NULL;
	}
}

// CRC-32 algorithm from:
//  <http://glacier.lbl.gov/cgi-bin/viewcvs.cgi/dor-test/crc32.c?rev=HEAD>

/* Table of CRCs of all 8-bit messages. */
static unsigned long crc_table[256];

/* Flag: has the table been computed? Initially false. */
static int crc_table_computed = 0;

/* Make the table for a fast CRC. */
static void make_crc_table(void)
{
  unsigned long c;
  int n, k;

  for (n = 0; n < 256; n++) {
    c = (unsigned long) n;
    for (k = 0; k < 8; k++) {
      if (c & 1) {
        c = 0xedb88320L ^ (c >> 1);
      } else {
        c = c >> 1;
      }
    }
    crc_table[n] = c;
  }
  crc_table_computed = 1;
}

/*
 Update a running crc with the bytes buf[0..len-1] and return
 the updated crc. The crc should be initialized to zero. Pre- and
 post-conditioning (one's complement) is performed within this
 function so it shouldn't be done by the caller. Usage example:

   unsigned long crc = 0L;

   while (read_buffer(buffer, length) != EOF) {
     crc = update_crc(crc, buffer, length);
   }
   if (crc != original_crc) error();
*/
static unsigned long update_crc(unsigned long crc,
                unsigned char *buf, int len)
{
  unsigned long c = crc ^ 0xffffffffL;
  int n;

  if (!crc_table_computed)
    make_crc_table();
  for (n = 0; n < len; n++) {
    c = crc_table[(c ^ buf[n]) & 0xff] ^ (c >> 8);
  }
  return c ^ 0xffffffffL;
}

/* Return the CRC of the bytes buf[0..len-1]. */
static unsigned long crc32(unsigned char *buf, int len)
{
  return update_crc(0L, buf, len);
}

int einkfb_gunzip(unsigned char *dst, int dstlen, unsigned char *src, unsigned long *lenp)
{
	z_stream s;
	int r, i, flags;

	if (!init_z_inflate_workspace()) {
		einkfb_print_warn ("Error: gunzip failed to allocate workspace\n");
		return (-1);
	}
	
	/* skip header */
	i = 10;
	flags = src[3];
	if (src[2] != DEFLATED || (flags & RESERVED) != 0) {
		einkfb_print_warn ("Error: Bad gzipped data\n");
		return (-1);
	}
	if ((flags & EXTRA_FIELD) != 0)
		i = 12 + src[10] + (src[11] << 8);
	if ((flags & ORIG_NAME) != 0)
		while (src[i++] != 0)
			;
	if ((flags & COMMENT) != 0)
		while (src[i++] != 0)
			;
	if ((flags & HEAD_CRC) != 0)
		i += 2;
	if (i >= *lenp) {
		einkfb_print_warn ("Error: gunzip out of data in header\n");
		return (-1);
	}

	/* Initialize ourself. */
	s.workspace = z_inflate_workspace;
	r = zlib_inflateInit2(&s, -MAX_WBITS);
	if (r != Z_OK) {
		einkfb_print_warn ("Error: zlib_inflateInit2() returned %d\n", r);
		return (-1);
	}
	s.next_in = src + i;
	s.avail_in = *lenp - i;
	s.next_out = dst;
	s.avail_out = dstlen;
	r = zlib_inflate(&s, Z_FINISH);
	if (r != Z_OK && r != Z_STREAM_END) {
		einkfb_print_warn ("Error: zlib_inflate() returned %d\n", r);
		return (-1);
	}
	*lenp = s.next_out - (unsigned char *) dst;
	zlib_inflateEnd(&s);

	return (0);
}

int einkfb_gzip(unsigned char *dst, int dstlen, unsigned char *src, unsigned long *lenp)
{
	z_stream s;
	int r, i;
	
	if (!init_z_deflate_workspace()) {
		einkfb_print_warn ("Error: gzip failed to allocate workspace\n");
		return (-1);
	}

	/* write header 1 (leave hole for deflate's non-gzip header) */
	for (i = 0; i < 8; i++) {
		dst[i] = 0;
	}
	dst[0] = HEADER_0;
	dst[1] = HEADER_1;
	dst[2] = DEFLATED;
	
	/* Initialize ourself. */
	s.workspace = z_deflate_workspace;
	r = zlib_deflateInit(&s, Z_BEST_COMPRESSION);
	if (r != Z_OK) {
		einkfb_print_warn ("Error: zlib_deflateInit() returned %d\n", r);
		return (-1);
	}
	s.next_in = src;
	s.avail_in = *lenp;
	s.next_out = dst + i;
	s.avail_out = dstlen - i;
	r = zlib_deflate(&s, Z_FINISH);
	if (r != Z_OK && r != Z_STREAM_END) {
		einkfb_print_warn ("Error: zlib_deflate() returned %d\n", r);
		return (-1);
	}

	/* write header 2 (fill in deflate's header with gzip's)*/
	dst[8] = BEST_COMP;
	dst[9] = UNIX_OS;
	
	/* write trailer (replace adler32 with crc32 & length) */
	i = s.next_out - (unsigned char *) dst - 4;
	s.adler = crc32(src, *lenp);
	
	dst[i++] = (unsigned char)(s.adler & 0xff);
	dst[i++] = (unsigned char)((s.adler >> 8) & 0xff);
	dst[i++] = (unsigned char)((s.adler >> 16) & 0xff);
	dst[i++] = (unsigned char)((s.adler >> 24) & 0xff);
	dst[i++] = (unsigned char)(s.total_in & 0xff);
	dst[i++] = (unsigned char)((s.total_in >> 8) & 0xff);
	dst[i++] = (unsigned char)((s.total_in >> 16) & 0xff);
	dst[i++] = (unsigned char)((s.total_in >> 24) & 0xff);

	*lenp = i;
	zlib_deflateEnd(&s);
	
	return (0);
}

void einkfb_utils_done(void)
{
	// Say that we're done with the zlib workspaces.
	//
	done_z_inflate_workspace();
	done_z_deflate_workspace();
}

#if PRAGMAS
    #pragma mark -
    #pragma mark Exports
    #pragma mark -
#endif

EXPORT_SYMBOL(einkfb_lock_ready);
EXPORT_SYMBOL(einkfb_lock_release);
EXPORT_SYMBOL(einkfb_lock_entry);
EXPORT_SYMBOL(einkfb_lock_exit);
EXPORT_SYMBOL(einkfb_blit);
EXPORT_SYMBOL(einkfb_stretch_nybble);
EXPORT_SYMBOL(einkfb_get_last_bounds_failure);
EXPORT_SYMBOL(einkfb_bounds_are_acceptable);
EXPORT_SYMBOL(einkfb_align_bounds);
EXPORT_SYMBOL(einkfb_schedule_timeout);
EXPORT_SYMBOL(einkfb_gunzip);
EXPORT_SYMBOL(einkfb_gzip);

