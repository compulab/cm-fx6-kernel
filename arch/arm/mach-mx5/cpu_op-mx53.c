/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <mach/hardware.h>
#include "cpu_op-mx53.h"

/*!
 * @file mach-mx5/cpu_op-mx53.c
 *
 * @brief This file contains the information about MX53 CPU working points.
 *
 * @ingroup MSL_MX53
 */
extern struct cpu_op *(*get_cpu_op)(int *op);
extern void (*set_num_cpu_op)(int num);
static int num_cpu_op;
static struct cpu_op *cpu_op_table;

/* working point for auto*/
static struct cpu_op cpu_op_aec[] = {
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1050000,},
};

/* working point for consumer 1G*/
static struct cpu_op cpu_op_ces[] = {
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
	 .cpu_voltage = 1200000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1050000,},
	 {
	  .pll_rate = 800000000,
	  .cpu_rate = 400000000,
	  .cpu_podf = 1,
	  .cpu_voltage = 950000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 160000000,
	 .cpu_podf = 4,
	 .cpu_voltage = 900000,},
};

/* working point for consumer 1.2G*/
static struct cpu_op cpu_op_ces_1_2g[] = {
	{
	 .pll_rate = 1200000000,
	 .cpu_rate = 1200000000,
	 .pdf = 0,
	 .mfi = 12,
	 .mfd = 1,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1300000,},
	{
	 .pll_rate = 1000000000,
	 .cpu_rate = 1000000000,
	 .pdf = 0,
	 .mfi = 10,
	 .mfd = 11,
	 .mfn = 5,
	 .cpu_podf = 0,
	 .cpu_voltage = 1200000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 800000000,
	 .pdf = 0,
	 .mfi = 8,
	 .mfd = 2,
	 .mfn = 1,
	 .cpu_podf = 0,
	 .cpu_voltage = 1050000,},
	 {
	  .pll_rate = 800000000,
	  .cpu_rate = 400000000,
	  .cpu_podf = 1,
	  .cpu_voltage = 950000,},
	{
	 .pll_rate = 800000000,
	 .cpu_rate = 160000000,
	 .cpu_podf = 4,
	 .cpu_voltage = 900000,},
};


struct cpu_op *mx53_get_cpu_op(int *op)
{
	*op = num_cpu_op;
	return cpu_op_table;
}

void mx53_set_num_cpu_op(int num)
{
	num_cpu_op = num;
	return;
}

void mx53_set_cpu_part_number(enum mx53_cpu_part_number part_num)
{
	get_cpu_op = mx53_get_cpu_op;
	set_num_cpu_op = mx53_set_num_cpu_op;

	switch (part_num) {
	case IMX53_CEC_1_2G:
		cpu_op_table = cpu_op_ces_1_2g;
		num_cpu_op = ARRAY_SIZE(cpu_op_ces_1_2g);
		break;
	case IMX53_CEC:
		cpu_op_table = cpu_op_ces;
		num_cpu_op = ARRAY_SIZE(cpu_op_ces);
		break;
	case IMX53_AEC:
	default:
		cpu_op_table = cpu_op_aec;
		num_cpu_op = ARRAY_SIZE(cpu_op_aec);
		break;
	}
}


