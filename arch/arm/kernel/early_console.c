/*
 *  linux/arch/arm/kernel/early_printk.c
 *
 * Copyright 2010-2011 Amazon.com Technologies Inc., All Rights Reserved.
 * Manish Lachwani (lachwani@lab126.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/console.h>
#include <linux/init.h>

extern void printch(int);

static void early_console_write(struct console *con, const char *s, unsigned n)
{
	while (*s && n-- > 0) {
		if (*s == '\n')
			printch('\r');
		printch(*s);
		s++;
	}
}

static struct console early_serial_console = {
	.name =		"earlycons",
	.write =	early_console_write,
	.flags =	CON_PRINTBUFFER | CON_BOOT,
	.index =	-1,
};

static int __init setup_early_console(char *buf)
{
	register_console(&early_serial_console);
	return 0;
}

early_param("earlyconsole", setup_early_console);
