/*
 * boardid.c
 *
 * Copyright (C) 2008,2010 Amazon Technologies, Inc. All rights reserved.
 * Jon Mayo <jonmayo@amazon.com>
 * Arnaud Froment <afroment@amazon.com>
 *
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sysdev.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <linux/proc_fs.h>
#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <mach/boardid.h>

#define DRIVER_VER "1.0"
#define DRIVER_INFO "Board ID and Serial Number driver for Lab126 boards version " DRIVER_VER

#define BOARDID_USID_PROCNAME		"usid"
#define BOARDID_PROCNAME_BOARDID	"board_id"
#define BOARDID_PROCNAME_PANELID	"panel_id"
#define BOARDID_PROCNAME_PCBSN		"pcbsn"
#define BOARDID_PROCNAME_MACADDR	"mac_addr"
#define BOARDID_PROCNAME_MACSEC		"mac_sec"
#define BOARDID_PROCNAME_BOOTMODE	"bootmode"
#define BOARDID_PROCNAME_POSTMODE	"postmode"

#define SERIAL_NUM_SIZE         16
#define BOARD_ID_SIZE           16
#define PANEL_ID_SIZE           32

#ifndef MIN
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif

char mx50_serial_number[SERIAL_NUM_SIZE + 1];
EXPORT_SYMBOL(mx50_serial_number);

char mx50_board_id[BOARD_ID_SIZE + 1];
EXPORT_SYMBOL(mx50_board_id);

char mx50_panel_id[PANEL_ID_SIZE + 1];
EXPORT_SYMBOL(mx50_panel_id);

char mx50_mac_address[MAC_ADDR_SIZE + 1];
EXPORT_SYMBOL(mx50_mac_address);

char mx50_mac_secret[MAC_SEC_SIZE + 1];
EXPORT_SYMBOL(mx50_mac_secret);

char mx50_bootmode[BOOTMODE_SIZE + 1];
char mx50_postmode[BOOTMODE_SIZE + 1];

int mx50_board_is(char *id) {
    return (BOARD_IS_(mx50_board_id, id, strlen(id)));
}
EXPORT_SYMBOL(mx50_board_is);

int mx50_board_rev_greater(char *id)
{
  return (BOARD_REV_GREATER(mx50_board_id, id));
}
EXPORT_SYMBOL(mx50_board_rev_greater);

int mx50_board_rev_greater_eq(char *id)
{
  return (BOARD_REV_GREATER_EQ(mx50_board_id, id));
}
EXPORT_SYMBOL(mx50_board_rev_greater_eq);

static int proc_id_read(char *page, char **start, off_t off, int count,
				int *eof, void *data, char *id)
{
	strcpy(page, id);
	*eof = 1;

	return strlen(page);
}

#define PROC_ID_READ(id) proc_id_read(page, start, off, count, eof, data, id)

static int proc_usid_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
        return PROC_ID_READ(mx50_serial_number);
}

static int proc_board_id_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
        return PROC_ID_READ(mx50_board_id);
}

static int proc_panel_id_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
        return PROC_ID_READ(mx50_panel_id);
}

static int proc_mac_address_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
        return PROC_ID_READ(mx50_mac_address);
}

static int proc_mac_secret_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
        return PROC_ID_READ(mx50_mac_secret);
}

static int proc_bootmode_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
        return PROC_ID_READ(mx50_bootmode);
}

static int proc_postmode_read(char *page, char **start, off_t off, int count,
				int *eof, void *data)
{
        return PROC_ID_READ(mx50_postmode);
}

/* initialize the proc accessors */
static int __init mx50_serialnumber_init(void)
{
	struct proc_dir_entry *proc_usid = create_proc_entry(BOARDID_USID_PROCNAME, S_IRUGO, NULL);
	struct proc_dir_entry *proc_board_id = create_proc_entry(BOARDID_PROCNAME_BOARDID, S_IRUGO, NULL);
	struct proc_dir_entry *proc_panel_id = create_proc_entry(BOARDID_PROCNAME_PANELID, S_IRUGO, NULL);
	struct proc_dir_entry *proc_mac_address = create_proc_entry(BOARDID_PROCNAME_MACADDR, S_IRUGO, NULL);
	struct proc_dir_entry *proc_mac_secret = create_proc_entry(BOARDID_PROCNAME_MACSEC, S_IRUGO, NULL);
	struct proc_dir_entry *proc_bootmode = create_proc_entry(BOARDID_PROCNAME_BOOTMODE, S_IRUGO, NULL);
	struct proc_dir_entry *proc_postmode = create_proc_entry(BOARDID_PROCNAME_POSTMODE, S_IRUGO, NULL);

	if (proc_usid != NULL) {
		proc_usid->data = NULL;
		proc_usid->read_proc = proc_usid_read;
		proc_usid->write_proc = NULL;
	}

	if (proc_board_id != NULL) {
		proc_board_id->data = NULL;
		proc_board_id->read_proc = proc_board_id_read;
		proc_board_id->write_proc = NULL;
	}

	if (proc_panel_id != NULL) {
		proc_panel_id->data = NULL;
		proc_panel_id->read_proc = proc_panel_id_read;
		proc_panel_id->write_proc = NULL;
	}

	if (proc_mac_address != NULL) {
		proc_mac_address->data = NULL;
		proc_mac_address->read_proc = proc_mac_address_read;
		proc_mac_address->write_proc = NULL;
	}

	if (proc_mac_secret != NULL) {
		proc_mac_secret->data = NULL;
		proc_mac_secret->read_proc = proc_mac_secret_read;
		proc_mac_secret->write_proc = NULL;
	}

	if (proc_bootmode != NULL) {
		proc_bootmode->data = NULL;
		proc_bootmode->read_proc = proc_bootmode_read;
		proc_bootmode->write_proc = NULL;
	}

	if (proc_postmode != NULL) {
		proc_postmode->data = NULL;
		proc_postmode->read_proc = proc_postmode_read;
		proc_postmode->write_proc = NULL;
	}

	return 0;
}
module_init(mx50_serialnumber_init);

/* copy the serial numbers from the special area of memory into the kernel */
static void mx50_serial_board_numbers(void)
{
	memcpy(mx50_serial_number, system_serial16, MIN(SERIAL_NUM_SIZE, sizeof(system_serial16)));
	mx50_serial_number[SERIAL_NUM_SIZE] = '\0';

	memcpy(mx50_board_id, system_rev16, MIN(BOARD_ID_SIZE, sizeof(system_rev16)));
	mx50_board_id[BOARD_ID_SIZE] = '\0';

	strcpy(mx50_panel_id, ""); /* start these as empty and populate later. */

	memcpy(mx50_mac_address, system_mac_addr, MIN(sizeof(mx50_mac_address)-1, sizeof(system_mac_addr))); 
	mx50_mac_address[sizeof(mx50_mac_address)-1] = '\0';

	memcpy(mx50_mac_secret, system_mac_sec, MIN(sizeof(mx50_mac_secret)-1, sizeof(system_mac_sec))); 
	mx50_mac_secret[sizeof(mx50_mac_secret)-1] = '\0';

	memcpy(mx50_bootmode, system_bootmode, MIN(sizeof(mx50_bootmode)-1, sizeof(system_bootmode))); 
	mx50_bootmode[sizeof(mx50_bootmode)-1] = '\0';

	memcpy(mx50_postmode, system_postmode, MIN(sizeof(mx50_postmode)-1, sizeof(system_postmode))); 
	mx50_postmode[sizeof(mx50_postmode)-1] = '\0';
	
	printk ("MX50 Board id - %s\n", mx50_board_id);
}

void mx50_init_boardid(void)
{
	pr_info(DRIVER_INFO "\n");

	mx50_serial_board_numbers();
}
