/*
 *  linux/drivers/video/eink/waveform/eink_panel.h --
 *  eInk panel access defs
 *
 *      Copyright (c) 2011 Amazon Technologies, Inc.
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#ifndef _EINK_PANEL_H
#define _EINK_PANEL_H

#define WFM_ADDR             0x00886     // See AM300_MMC_IMAGE_X03a/source/broadsheet_soft/bs60_init/run_bs60_init.sh.
#define CMD_ADDR             0x00000     // Base of flash holds the commands (0x00000...(WFM_ADDR - 1)).
#define PNL_ADDR             0x30000     // Start of panel data.
#define TST_ADDR_128K        0x1E000     // Test area (last 8K of 128K).
#define TST_ADDR_256K        0x3E000     // Test area (last 8K of 256K).

#define CMD_SIZE             (WFM_ADDR - CMD_ADDR)
#define WFM_SIZE             (PNL_ADDR - WFM_ADDR)
#define WFM_HDR_SIZE         (0x30)

#define PNL_BASE_PART_NUMBER 0x00
#define PNL_SIZE_PART_NUMBER 16

#define PNL_BASE_VCOM        0x10
#define PNL_SIZE_VCOM        5
#define PNL_SIZE_VCOM_STR    (PNL_SIZE_VCOM + 1)

#define PNL_BASE_WAVEFORM    0x20
#define PNL_SIZE_WAVEFORM    23

#define PNL_BASE_FPL         0x40
#define PNL_SIZE_FPL         3

#define PNL_BASE_BCD         0x50
#define PNL_SIZE_BCD         32
#define PNL_SIZE_BCD_STR     (PNL_SIZE_BCD + 1)

#define PNL_BASE             0x00
#define PNL_SIZE             256
#define PNL_LAST             (PNL_SIZE - 1)

#define PNL_FLASH_BASE       PNL_ADDR

#define PNL_CHAR_UNKNOWN     '!'

#define SFM_SECTOR_COUNT     4
#define SFM_SECTOR_SIZE_128K (32*1024)
#define SFM_SECTOR_SIZE_256K (64*1024)
#define SFM_SIZE_128K        (SFM_SECTOR_SIZE_128K * SFM_SECTOR_COUNT)
#define SFM_SIZE_256K        (SFM_SECTOR_SIZE_256K * SFM_SECTOR_COUNT)
#define SFM_PAGE_SIZE        256
#define SFM_PAGE_COUNT_128K  (SFM_SECTOR_SIZE_128K/SFM_PAGE_SIZE)
#define SFM_PAGE_COUNT_256K  (SFM_SECTOR_SIZE_256K/SFM_PAGE_SIZE)

enum panel_flash_select
{
    panel_flash_waveform,
    panel_flash_commands,
    panel_flash_test,
};
typedef enum panel_flash_select panel_flash_select;


#define WF_PATH_LEN	256

bool panel_flash_present(void);
bool panel_flash_init(void);
void panel_flash_exit(void);
char *panel_get_proxy_buffer(void);

#endif // _EINK_PANEL_H
 
