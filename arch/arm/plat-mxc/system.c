/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2000 Deep Blue Solutions Ltd
 * Copyright 2006-2012 Freescale Semiconductor, Inc.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 * Copyright 2009 Ilya Yanok, Emcraft Systems Ltd, yanok@emcraft.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <asm/proc-fns.h>
#include <asm/system.h>
#ifdef CONFIG_SMP
#include <linux/smp.h>
#endif
#include <asm/mach-types.h>

static void __iomem *wdog_base;

#ifdef CONFIG_MXC_REBOOT_MFGMODE
void do_switch_mfgmode(void);
void mxc_clear_mfgmode(void);
#else
void do_switch_mfgmode() {}
void mxc_clear_mfgmode() {}
#endif

#ifdef CONFIG_MXC_REBOOT_ANDROID_CMD
void do_switch_recovery(void);
void do_switch_fastboot(void);
#else
void do_switch_recovery() {}
void do_switch_fastboot() {}
#endif

static void arch_reset_special_mode(char mode, const char *cmd)
{
	if (strcmp(cmd, "download") == 0)
		do_switch_mfgmode();
	else if (strcmp(cmd, "recovery") == 0)
		do_switch_recovery();
	else if (strcmp(cmd, "fastboot") == 0)
		do_switch_fastboot();
}

/*
 * Reset the system. It is called by machine_restart().
 */
void arch_reset(char mode, const char *cmd)
{
	unsigned int wcr_enable;

	arch_reset_special_mode(mode, cmd);

#ifdef CONFIG_ARCH_MX6
	/* wait for reset to assert... */
	#ifdef CONFIG_MX6_INTER_LDO_BYPASS
	wcr_enable = 0x14; /*reset system by extern pmic*/
	#else
	wcr_enable = (1 << 2);
	#endif
	__raw_writew(wcr_enable, wdog_base);
	/* errata TKT039676, SRS bit may be missed when
	SRC sample it, need to write the wdog controller
	twice to avoid it */
	__raw_writew(wcr_enable, wdog_base);

	/* wait for reset to assert... */
	mdelay(500);

	printk(KERN_ERR "Watchdog reset failed to assert reset\n");

	return;
#endif

#ifdef CONFIG_MACH_MX51_EFIKAMX
	if (machine_is_mx51_efikamx()) {
		mx51_efikamx_reset();
		return;
	}
#endif

	if (cpu_is_mx1()) {
		wcr_enable = (1 << 0);
	} else {
		struct clk *clk;

		clk = clk_get_sys("imx2-wdt.0", NULL);
		if (!IS_ERR(clk))
			clk_enable(clk);
		wcr_enable = (1 << 2);
	}

	/* Assert SRS signal */
	__raw_writew(wcr_enable, wdog_base);

	/* wait for reset to assert... */
	mdelay(500);

	printk(KERN_ERR "Watchdog reset failed to assert reset\n");

	/* delay to allow the serial port to show the message */
	mdelay(50);

	/* we'll take a jump through zero as a poor second */
	cpu_reset(0);
}

void mxc_arch_reset_init(void __iomem *base)
{
	wdog_base = base;
}
