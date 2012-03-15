/*
 * eink_commands.h
 *
 * Copyright (C) 2005-2010 Amazon Technologies
 *
 */

#ifndef _EINK_COMMANDS_H
#define _EINK_COMMANDS_H

#include <linux/einkwf.h>

#define EINK_COMMANDS_FILESIZE          0x0886  // We need to have EXACTLY...
#define EINK_COMMANDS_COUNT             1       // ...2182 bytes of data for Broadsheet.

#define EINK_ADDR_COMMANDS_VERS_MAJOR   0x0003  // 1 byte
#define EINK_ADDR_COMMANDS_VERS_MINOR   0x0002  // 1 byte
#define EINK_ADDR_COMMANDS_TYPE         0x0000  // 2 bytes (big-endian, 0x0000 or 'bs' -> 0x6273)

#define EINK_ADDR_COMMANDS_CHECKSUM     0x0882  // 4 bytes (checksum of bytes 0x000-0x881)

#define EINK_ADDR_COMMANDS_VERSION      0x0000  // 4 bytes (little-endian)
#define EINK_ADDR_COMMANDS_CODE_SIZE    0x0004  // 2 bytes (big-endian, zero-based)

#define EINK_COMMANDS_FIXED_CODE_SIZE           \
                                        (  4 +  /* EINK_ADDR_COMMANDS_VERSION */ \
                                           2 +  /* EINK_ADDR_COMMANDS_CODE_SIZE_1 + EINK_ADDR_COMMANDS_CODE_SIZE_2 */ \
                                         128 +  /* 64-command vector table (64 * 2-bytes) */ \
                                           4 )  /* checksum */

#define EINK_COMMANDS_BROADSHEET        0
#define EINK_COMMANDS_ISIS              1

#define IS_BROADSHEET()                 (EINK_COMMANDS_FILESIZE == einkwf_get_buffer_size())

extern void eink_get_commands_info(eink_commands_info_t *info);
extern char *eink_get_commands_version_string(void);
extern bool eink_commands_valid(void);

extern unsigned long eink_get_embedded_commands_checksum(unsigned char *buffer);
extern unsigned long eink_get_computed_commands_checksum(unsigned char *buffer);

#endif // _EINK_COMMANDS_H
