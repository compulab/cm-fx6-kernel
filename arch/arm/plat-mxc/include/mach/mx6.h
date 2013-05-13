/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 *
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

#ifndef __ASM_ARCH_MXC_MX6_H__
#define __ASM_ARCH_MXC_MX6_H__

#ifndef __ASM_ARCH_MXC_HARDWARE_H__
#error "Do not include directly."
#endif

/*!
 * @file arch-mxc/mx6.h
 * @brief This file contains register definitions.
 *
 * @ingroup MSL_MX6
 */

/*!
 * Register an interrupt handler for the SMN as well as the SCC.  In some
 * implementations, the SMN is not connected at all, and in others, it is
 * on the same interrupt line as the SCM. Comment this line out accordingly
 */
#define USE_SMN_INTERRUPT

/*!
 * This option is used to set or clear the RXDMUXSEL bit in control reg 3.
 * Certain platforms need this bit to be set in order to receive Irda data.
 */
#define MXC_UART_IR_RXDMUX      0x0004
/*!
 * This option is used to set or clear the RXDMUXSEL bit in control reg 3.
 * Certain platforms need this bit to be set in order to receive UART data.
 */
#define MXC_UART_RXDMUX         0x0004

/*!
 * The maximum frequency that the pixel clock can be at so as to
 * activate DVFS-PER.
 */
#define DVFS_MAX_PIX_CLK	54000000


/* IROM
 */
#define IROM_BASE_ADDR		0x0
#define IROM_SIZE			SZ_64K

/* CPU Memory Map */
#define MMDC0_ARB_BASE_ADDR             0x10000000
#define MMDC0_ARB_END_ADDR              0x7FFFFFFF
#define MMDC1_ARB_BASE_ADDR             0x80000000
#define MMDC1_ARB_END_ADDR              0xFFFFFFFF
#define OCRAM_ARB_BASE_ADDR             0x00900000
#define OCRAM_ARB_END_ADDR              0x009FFFFF
#define IRAM_BASE_ADDR                  OCRAM_ARB_BASE_ADDR
#define PCIE_ARB_BASE_ADDR              0x01000000
#define PCIE_ARB_END_ADDR               0x01FFFFFF

/* Blocks connected via pl301periph */
#define ROMCP_ARB_BASE_ADDR             0x00000000
#define ROMCP_ARB_END_ADDR              0x000FFFFF
#define BOOT_ROM_BASE_ADDR              ROMCP_ARB_BASE_ADDR
#define CAAM_ARB_BASE_ADDR              0x00100000
#define CAAM_ARB_END_ADDR               0x00103FFF
#define APBH_DMA_ARB_BASE_ADDR          0x00110000
#define APBH_DMA_ARB_END_ADDR           0x00117FFF
#define HDMI_ARB_BASE_ADDR              0x00120000
#define HDMI_ARB_END_ADDR               0x00128FFF
#define GPU_3D_ARB_BASE_ADDR            0x00130000
#define GPU_3D_ARB_END_ADDR             0x00133FFF
#define GPU_2D_ARB_BASE_ADDR            0x00134000
#define GPU_2D_ARB_END_ADDR             0x00137FFF
#define DTCP_ARB_BASE_ADDR              0x00138000
#define DTCP_ARB_END_ADDR               0x0013BFFF
#define GPU_MEM_BASE_ADDR               GPU_3D_ARB_BASE_ADDR

/* GPV - PL301 configuration ports */
#define GPV0_BASE_ADDR                  0x00B00000
#define GPV1_BASE_ADDR                  0x00C00000
#define GPV2_BASE_ADDR                  0x00200000
#define GPV3_BASE_ADDR                  0x00300000
#define GPV4_BASE_ADDR                  0x00800000

#define AIPS1_ARB_BASE_ADDR             0x02000000
#define AIPS1_ARB_END_ADDR              0x020FFFFF
#define AIPS2_ARB_BASE_ADDR             0x02100000
#define AIPS2_ARB_END_ADDR              0x021FFFFF
#define SATA_ARB_BASE_ADDR              0x02200000
#define SATA_ARB_END_ADDR               0x02203FFF
#define OPENVG_ARB_BASE_ADDR            0x02204000
#define OPENVG_ARB_END_ADDR             0x02207FFF
#define HSI_ARB_BASE_ADDR               0x02208000
#define HSI_ARB_END_ADDR                0x0220BFFF
#define IPU1_ARB_BASE_ADDR              0x02400000
#define IPU1_ARB_END_ADDR               0x027FFFFF
#define IPU2_ARB_BASE_ADDR              0x02800000
#define IPU2_ARB_END_ADDR               0x02BFFFFF
#define WEIM_ARB_BASE_ADDR              0x08000000
#define WEIM_ARB_END_ADDR               0x0FFFFFFF

/* Legacy Defines */
#define CSD0_DDR_BASE_ADDR              MMDC0_ARB_BASE_ADDR
#define CSD1_DDR_BASE_ADDR              MMDC1_ARB_BASE_ADDR
#define CS0_BASE_ADDR                   WEIM_ARB_BASE_ADDR
#define NAND_FLASH_BASE_ADDR            APBH_DMA_ARB_BASE_ADDR

/* Defines for Blocks connected via AIPS (SkyBlue) */
#define ATZ1_BASE_ADDR              AIPS1_ARB_BASE_ADDR
#define ATZ2_BASE_ADDR              AIPS2_ARB_BASE_ADDR

/* slots 0,7 of SDMA reserved, therefore left unused in IPMUX3 */
#define SPDIF_BASE_ADDR             (ATZ1_BASE_ADDR + 0x04000) /* slot 1 */
#define MX6Q_ECSPI1_BASE_ADDR            (ATZ1_BASE_ADDR + 0x08000) /* slot 2 */
#define MX6Q_ECSPI2_BASE_ADDR            (ATZ1_BASE_ADDR + 0x0C000) /* slot 3 */
#define MX6Q_ECSPI3_BASE_ADDR            (ATZ1_BASE_ADDR + 0x10000) /* slot 4 */
#define MX6Q_ECSPI4_BASE_ADDR            (ATZ1_BASE_ADDR + 0x14000) /* slot 5 */
#define MX6Q_ECSPI5_BASE_ADDR            (ATZ1_BASE_ADDR + 0x18000) /* slot 6 */
#define UART1_BASE_ADDR             (ATZ1_BASE_ADDR + 0x20000) /* slot 8 */
#define ESAI1_BASE_ADDR             (ATZ1_BASE_ADDR + 0x24000) /* slot 9 */
#define SSI1_BASE_ADDR              (ATZ1_BASE_ADDR + 0x28000) /* slot 10 */
#define SSI2_BASE_ADDR              (ATZ1_BASE_ADDR + 0x2C000) /* slot 11 */
#define SSI3_BASE_ADDR              (ATZ1_BASE_ADDR + 0x30000) /* slot 12 */
#define ASRC_BASE_ADDR              (ATZ1_BASE_ADDR + 0x34000) /* slot 13 */
#define SPBA_BASE_ADDR              (ATZ1_BASE_ADDR + 0x3C000) /* slot 15 */
#define VPU_BASE_ADDR               (ATZ1_BASE_ADDR + 0x40000) /* slot 33,
					global en[1], til 0x7BFFF */

/* ATZ#1- On Platform */
#define AIPS1_ON_BASE_ADDR              (ATZ1_BASE_ADDR + 0x7C000)

/* ATZ#1- Off Platform */
#define AIPS1_OFF_BASE_ADDR             (ATZ1_BASE_ADDR + 0x80000)

#define PWM1_BASE_ADDR              (AIPS1_OFF_BASE_ADDR + 0x0000)
#define PWM2_BASE_ADDR              (AIPS1_OFF_BASE_ADDR + 0x4000)
#define PWM3_BASE_ADDR              (AIPS1_OFF_BASE_ADDR + 0x8000)
#define PWM4_BASE_ADDR              (AIPS1_OFF_BASE_ADDR + 0xC000)
#define CAN1_BASE_ADDR              (AIPS1_OFF_BASE_ADDR + 0x10000)
#define CAN2_BASE_ADDR              (AIPS1_OFF_BASE_ADDR + 0x14000)
#define GPT_BASE_ADDR               (AIPS1_OFF_BASE_ADDR + 0x18000)
#define GPIO1_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x1C000)
#define GPIO2_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x20000)
#define GPIO3_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x24000)
#define GPIO4_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x28000)
#define GPIO5_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x2C000)
#define GPIO6_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x30000)
#define GPIO7_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x34000)
#define KPP_BASE_ADDR               (AIPS1_OFF_BASE_ADDR + 0x38000)
#define WDOG1_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x3C000)
#define WDOG2_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x40000)
#define CCM_BASE_ADDR               (AIPS1_OFF_BASE_ADDR + 0x44000)
#define ANATOP_BASE_ADDR            (AIPS1_OFF_BASE_ADDR + 0x48000)
#define SNVS_BASE_ADDR              (AIPS1_OFF_BASE_ADDR + 0x4C000)
#define EPIT1_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x50000)
#define EPIT2_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x54000)
#define SRC_BASE_ADDR               (AIPS1_OFF_BASE_ADDR + 0x58000)
#define GPC_BASE_ADDR               (AIPS1_OFF_BASE_ADDR + 0x5C000)
#define IOMUXC_BASE_ADDR            (AIPS1_OFF_BASE_ADDR + 0x60000)
#define DCIC1_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x64000)
#define DCIC2_BASE_ADDR             (AIPS1_OFF_BASE_ADDR + 0x68000)
#define MX6Q_SDMA_BASE_ADDR			(AIPS1_OFF_BASE_ADDR + 0x6C000)

/* ATZ#2- On Platform */
#define AIPS2_ON_BASE_ADDR              (ATZ2_BASE_ADDR + 0x7C000)

/* ATZ#2- Off Platform */
#define AIPS2_OFF_BASE_ADDR             (ATZ2_BASE_ADDR + 0x80000)

/* ATZ#2  - Global enable (0) */
#define CAAM_BASE_ADDR              ATZ2_BASE_ADDR
#define ARM_BASE_ADDR		        (ATZ2_BASE_ADDR + 0x40000)

#define USBOH3_PL301_BASE_ADDR          (AIPS2_OFF_BASE_ADDR + 0x0000)
#define USBOH3_USB_BASE_ADDR            (AIPS2_OFF_BASE_ADDR + 0x4000)
/* Frank Li Need IC confirm OTG base address*/
/* Zeng Zhaoming: FIXME, conflict with plat-mxc/include/mach/arc_otg.h:21 */
/* #define OTG_BASE_ADDR		USBOH3_USB_BASE_ADDR */
#define ENET_BASE_ADDR                  (AIPS2_OFF_BASE_ADDR+0x8000)
#define MLB_BASE_ADDR                   (AIPS2_OFF_BASE_ADDR+0xC000)

#define MX6Q_USDHC1_BASE_ADDR                (AIPS2_OFF_BASE_ADDR + 0x10000)
#define MX6Q_USDHC2_BASE_ADDR                (AIPS2_OFF_BASE_ADDR + 0x14000)
#define MX6Q_USDHC3_BASE_ADDR                (AIPS2_OFF_BASE_ADDR + 0x18000)
#define MX6Q_USDHC4_BASE_ADDR                (AIPS2_OFF_BASE_ADDR + 0x1C000)
#define I2C1_BASE_ADDR                  (AIPS2_OFF_BASE_ADDR + 0x20000)
#define I2C2_BASE_ADDR                  (AIPS2_OFF_BASE_ADDR + 0x24000)
#define I2C3_BASE_ADDR                  (AIPS2_OFF_BASE_ADDR + 0x28000)
#define ROMCP_BASE_ADDR                 (AIPS2_OFF_BASE_ADDR + 0x2C000)
#define MMDC_P0_BASE_ADDR               (AIPS2_OFF_BASE_ADDR + 0x30000)
#define MMDC_P1_BASE_ADDR               (AIPS2_OFF_BASE_ADDR + 0x34000)
#define WEIM_BASE_ADDR                  (AIPS2_OFF_BASE_ADDR + 0x38000)
#define OCOTP_BASE_ADDR                 (AIPS2_OFF_BASE_ADDR + 0x3C000)
#define CSU_BASE_ADDR                   (AIPS2_OFF_BASE_ADDR + 0x40000)
#define IP2APB_PERFMON1_BASE_ADDR       (AIPS2_OFF_BASE_ADDR + 0x44000)
#define IP2APB_PERFMON2_BASE_ADDR       (AIPS2_OFF_BASE_ADDR + 0x48000)
#define IP2APB_PERFMON3_BASE_ADDR       (AIPS2_OFF_BASE_ADDR + 0x4C000)
#define IP2APB_TZASC1_BASE_ADDR         (AIPS2_OFF_BASE_ADDR + 0x50000)
#define IP2APB_TZASC2_BASE_ADDR         (AIPS2_OFF_BASE_ADDR + 0x54000)
#define AUDMUX_BASE_ADDR                (AIPS2_OFF_BASE_ADDR + 0x58000)
#define MIPI_CSI2_BASE_ADDR             (AIPS2_OFF_BASE_ADDR + 0x5C000)
#define MIPI_DSI_BASE_ADDR              (AIPS2_OFF_BASE_ADDR + 0x60000)
#define VDOA_BASE_ADDR                  (AIPS2_OFF_BASE_ADDR + 0x64000)
#define UART2_BASE_ADDR                 (AIPS2_OFF_BASE_ADDR + 0x68000)
#define UART3_BASE_ADDR                 (AIPS2_OFF_BASE_ADDR + 0x6C000)
#define UART4_BASE_ADDR                 (AIPS2_OFF_BASE_ADDR + 0x70000)
#define UART5_BASE_ADDR                 (AIPS2_OFF_BASE_ADDR + 0x74000)
#define IP2APB_USBPHY1_BASE_ADDR        (AIPS2_OFF_BASE_ADDR + 0x78000)
#define IP2APB_USBPHY2_BASE_ADDR        (AIPS2_OFF_BASE_ADDR + 0x7C000)

/* Cortex-A9 MPCore private memory region */
#define ARM_PERIPHBASE                  0x00A00000
#define SCU_BASE_ADDR                   ARM_PERIPHBASE
#define IC_INTERFACES_BASE_ADDR         (ARM_PERIPHBASE + 0x0100)
#define GLOBAL_TIMER_BASE_ADDR          (ARM_PERIPHBASE + 0x0200)
#define PRIVATE_TIMERS_WD_BASE_ADDR     (ARM_PERIPHBASE + 0x0600)
#define IC_DISTRIBUTOR_BASE_ADDR        (ARM_PERIPHBASE + 0x1000)
#define L2_BASE_ADDR			(ARM_PERIPHBASE + 0x2000)


#define MX6Q_UART1_BASE_ADDR UART1_BASE_ADDR
#define MX6Q_UART2_BASE_ADDR UART2_BASE_ADDR
#define MX6Q_UART3_BASE_ADDR UART3_BASE_ADDR
#define MX6Q_UART4_BASE_ADDR UART4_BASE_ADDR
#define MX6Q_FEC_BASE_ADDR	 ENET_BASE_ADDR

/* define virtual address */
#define PERIPBASE_VIRT 0xF4000000
#define AIPS1_BASE_ADDR_VIRT (PERIPBASE_VIRT + AIPS1_ARB_BASE_ADDR)
#define AIPS2_BASE_ADDR_VIRT (PERIPBASE_VIRT + AIPS2_ARB_BASE_ADDR)
#define ARM_PERIPHBASE_VIRT	 (PERIPBASE_VIRT + ARM_PERIPHBASE)
#define AIPS1_SIZE SZ_1M
#define AIPS2_SIZE SZ_1M
#define ARM_PERIPHBASE_SIZE (SZ_8K + SZ_4K)

#define MX6_IO_ADDRESS(x) (void __force __iomem *)((x)  + PERIPBASE_VIRT)

/*!
 * This macro defines the physical to virtual address mapping for all the
 * peripheral modules. It is used by passing in the physical address as x
 * and returning the virtual address. If the physical address is not mapped,
 * it returns 0xDEADBEEF
 */
#define IO_ADDRESS(x)   \
	(void __force __iomem *) \
	(((((x) >= (unsigned long)AIPS1_ARB_BASE_ADDR) && \
	  ((x) <= (unsigned long)AIPS2_ARB_END_ADDR)) || \
	  ((x) >= (unsigned long)ARM_PERIPHBASE && \
	  ((x) <= (unsigned long)(ARM_PERIPHBASE + ARM_PERIPHBASE)))) ? \
	   MX6_IO_ADDRESS(x) : 0xDEADBEEF)

/*
 * Interrupt numbers
 */
#define MXC_INT_GPR                                32
#define MXC_INT_CHEETAH_CSYSPWRUPREQ               33
#define MX6Q_INT_SDMA                              34
#define MXC_INT_VPU_JPG                            35
#define MXC_INT_INTERRUPT_36_NUM                   36
#define MXC_INT_IPU1_ERR                           37
#define MXC_INT_IPU1_FUNC                          38
#define MXC_INT_IPU2_ERR                           39
#define MXC_INT_IPU2_FUNC                          40
#define MXC_INT_GPU3D_IRQ                          41
#define MXC_INT_GPU2D_IRQ                          42
#define MXC_INT_OPENVG_XAQ2                        43
#define MXC_INT_VPU_IPI                            44
#define MXC_INT_APBHDMA_DMA                        45
#define MXC_INT_WEIM                               46
#define MXC_INT_RAWNAND_BCH                        47
#define MXC_INT_RAWNAND_GPMI                       48
#define MXC_INT_DTCP                               49
#define MXC_INT_VDOA                               50
#define MXC_INT_SNVS                               51
#define MXC_INT_SNVS_SEC                           52
#define MXC_INT_CSU                                53
#define MX6Q_INT_USDHC1                             54
#define MX6Q_INT_USDHC2                             55
#define MX6Q_INT_USDHC3                             56
#define MX6Q_INT_USDHC4                             57
#define MXC_INT_UART1_ANDED                        58
#define MXC_INT_UART2_ANDED                        59
#define MXC_INT_UART3_ANDED                        60
#define MXC_INT_UART4_ANDED                        61
#define MXC_INT_UART5_ANDED                        62
#define MX6Q_INT_ECSPI1                             63
#define MX6Q_INT_ECSPI2                             64
#define MX6Q_INT_ECSPI3                             65
#define MX6Q_INT_ECSPI4                             66
#define MX6Q_INT_ECSPI5                             67
#define MXC_INT_I2C1                               68
#define MXC_INT_I2C2                               69
#define MXC_INT_I2C3                               70
#define MXC_INT_SATA                               71
#define MXC_INT_USBOH3_UH1                         72
#define MXC_INT_USBOH3_UH2                         73
#define MXC_INT_USBOH3_UH3                         74
#define MXC_INT_USBOH3_UOTG                        75
#define MXC_INT_ANATOP_UTMI0                       76
#define MXC_INT_ANATOP_UTMI1                       77
#define MXC_INT_SSI1                               78
#define MXC_INT_SSI2                               79
#define MXC_INT_SSI3                               80
#define MXC_INT_ANATOP_TEMPSNSR                    81
#define MXC_INT_ASRC                               82
#define MXC_INT_ESAI                              83
#define MXC_INT_SPDIF                              84
#define MXC_INT_MLB                                85
#define MXC_INT_ANATOP_ANA1                        86
#define MXC_INT_GPT                                87
#define MXC_INT_EPIT1                              88
#define MXC_INT_EPIT2                              89
#define MXC_INT_GPIO1_INT7_NUM                             90
#define MXC_INT_GPIO1_INT6_NUM                             91
#define MXC_INT_GPIO1_INT5_NUM                             92
#define MXC_INT_GPIO1_INT4_NUM                             93
#define MXC_INT_GPIO1_INT3_NUM                             94
#define MXC_INT_GPIO1_INT2_NUM                             95
#define MXC_INT_GPIO1_INT1_NUM                             96
#define MXC_INT_GPIO1_INT0_NUM                             97
#define MXC_INT_GPIO1_INT15_0_NUM                          98
#define MXC_INT_GPIO1_INT31_16_NUM                         99
#define MXC_INT_GPIO2_INT15_0_NUM                          100
#define MXC_INT_GPIO2_INT31_16_NUM                         101
#define MXC_INT_GPIO3_INT15_0_NUM                          102
#define MXC_INT_GPIO3_INT31_16_NUM                         103
#define MXC_INT_GPIO4_INT15_0_NUM                          104
#define MXC_INT_GPIO4_INT31_16_NUM                         105
#define MXC_INT_GPIO5_INT15_0_NUM                          106
#define MXC_INT_GPIO5_INT31_16_NUM                         107
#define MXC_INT_GPIO6_INT15_0_NUM                          108
#define MXC_INT_GPIO6_INT31_16_NUM                         109
#define MXC_INT_GPIO7_INT15_0_NUM                          110
#define MXC_INT_GPIO7_INT31_16_NUM                         111
#define MXC_INT_WDOG1                              112
#define MXC_INT_WDOG2                              113
#define MXC_INT_KPP                                114
#define MXC_INT_PWM1                               115
#define MXC_INT_PWM2                               116
#define MXC_INT_PWM3                               117
#define MXC_INT_PWM4                               118
#define MXC_INT_CCM_INT1_NUM                               119
#define MXC_INT_CCM_INT2_NUM                               120
#define MXC_INT_GPC_INT1_NUM                               121
#define MXC_INT_GPC_INT2_NUM                               122
#define MXC_INT_SRC                                123
#define MXC_INT_CHEETAH_L2                         124
#define MXC_INT_CHEETAH_PARITY                     125
#define MXC_INT_CHEETAH_PERFORM                    126
#define MXC_INT_CHEETAH_TRIGGER                    127
#define MXC_INT_SRC_CPU_WDOG                       128
#define MXC_INT_INTERRUPT_129_NUM                          129
#define MXC_INT_INTERRUPT_130_NUM                          130
#define MXC_INT_INTERRUPT_131_NUM                          131
#define MXC_INT_CSI_INTR1                          132
#define MXC_INT_CSI_INTR2                          133
#define MXC_INT_DSI                                134
#define MXC_INT_HSI                                135
#define MXC_INT_SJC                                136
#define MXC_INT_CAAM_INT0_NUM                              137
#define MXC_INT_CAAM_INT1_NUM                              138
#define MXC_INT_INTERRUPT_139_NUM                          139
#define MXC_INT_TZASC1                             140
#define MXC_INT_TZASC2                             141
#define MXC_INT_CAN1                               142
#define MXC_INT_CAN2                               143
#define MXC_INT_PERFMON1                           144
#define MXC_INT_PERFMON2                           145
#define MXC_INT_PERFMON3                           146
#define MXC_INT_HDMI_TX                            147
#define MXC_INT_HDMI_TX_WAKEUP                     148
#define MXC_INT_MLB_AHB0                           149
#define MXC_INT_ENET1                              150
#define MXC_INT_ENET2                              151
#define MXC_INT_PCIE_0                             152
#define MXC_INT_PCIE_1                             153
#define MXC_INT_PCIE_2                             154
#define MXC_INT_PCIE_3                             155
#define MXC_INT_DCIC1                              156
#define MXC_INT_DCIC2                              157
#define MXC_INT_MLB_AHB1                           158
#define MXC_INT_ANATOP_ANA2                        159

#define MX6Q_INT_UART1	MXC_INT_UART1_ANDED
#define MX6Q_INT_UART2	MXC_INT_UART2_ANDED
#define MX6Q_INT_UART3	MXC_INT_UART3_ANDED
#define MX6Q_INT_UART4	MXC_INT_UART4_ANDED
#define MX6Q_INT_FEC	MXC_INT_ENET1

#define IRQ_LOCALTIMER				29

#endif				/*  __ASM_ARCH_MXC_MX6_H__ */
