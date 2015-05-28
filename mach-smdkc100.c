/* linux/arch/arm/mach-s5pc100/mach-smdkc100.c
 *
 * Copyright 2009 Samsung Electronics Co.
 * Author: Byungho Min <bhmin@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/fb.h>
#include <linux/delay.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/map.h>
#include <mach/regs-fb.h>
#include <video/platform_lcd.h>

#include <asm/irq.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <plat/gpio-cfg.h>

#include <plat/clock.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/s5pc100.h>
#include <plat/fb.h>
#include <plat/iic.h>

#ifdef CONFIG_DM9000
#include <linux/dm9000.h>
#include <linux/irq.h>
#endif 

#ifdef CONFIG_MTD_NAND_S3C
#include <linux/mtd/partitions.h>
#include <linux/mtd/mtd.h>
#include <plat/nand.h>
#endif

#ifdef CONFIG_USB_SUPPORT
#include <plat/pll.h>
#include <linux/usb/ch9.h>
#include <mach/regs-clock.h>
#endif

#ifdef CONFIG_TOUCHSCREEN_S3C2410
#include <plat/ts.h>
#endif
/* Following are default values for UCON, ULCON and UFCON UART registers */
#define S5PC100_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define S5PC100_ULCON_DEFAULT	S3C2410_LCON_CS8

#define S5PC100_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S3C2440_UFCON_RXTRIG8 |	\
				 S3C2440_UFCON_TXTRIG16)

static struct s3c2410_uartcfg smdkc100_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = S5PC100_UCON_DEFAULT,
		.ulcon	     = S5PC100_ULCON_DEFAULT,
		.ufcon	     = S5PC100_UFCON_DEFAULT,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = S5PC100_UCON_DEFAULT,
		.ulcon	     = S5PC100_ULCON_DEFAULT,
		.ufcon	     = S5PC100_UFCON_DEFAULT,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = S5PC100_UCON_DEFAULT,
		.ulcon	     = S5PC100_ULCON_DEFAULT,
		.ufcon	     = S5PC100_UFCON_DEFAULT,
	},
	[3] = {
		.hwport	     = 3,
		.flags	     = 0,
		.ucon	     = S5PC100_UCON_DEFAULT,
		.ulcon	     = S5PC100_ULCON_DEFAULT,
		.ufcon	     = S5PC100_UFCON_DEFAULT,
	},
};

/* DM9000 Support*/
#ifdef CONFIG_DM9000
static struct resource dm9000_resources[] = {
	[0] = {
		.start	=	0x88000000,
		.end	=	0x88000000+0x3,
		.flags	=	IORESOURCE_MEM,
	},
	[1] = {
		.start	=	0x88000000+0x4,
		.end	=	0x88000000+0x4+0x3,
		.flags	=	IORESOURCE_MEM,
	},
	[2] = {
		.start	=	IRQ_EINT(10),
		.end	=	IRQ_EINT(10),
		.flags	=	IORESOURCE_IRQ|IRQ_TYPE_LEVEL_HIGH,
	},
};

static struct dm9000_plat_data s5pc100_dm9000_platdata = {
	.flags = DM9000_PLATF_16BITONLY,
	.dev_addr[0] = 0x00,
	.dev_addr[1] = 0x00,
	.dev_addr[2] = 0x3e,
	.dev_addr[3] = 0x26,
	.dev_addr[4] = 0x0a,
	.dev_addr[5] = 0x00,
};

static struct platform_device s5pc100_device_dm9000 = {
	.name = "dm9000",
	.id = -1,
	.num_resources = ARRAY_SIZE(dm9000_resources),
	.resource = dm9000_resources,
	.dev = {
		.platform_data = &s5pc100_dm9000_platdata,
	}
};
#endif

static struct resource led_resourcces0[] = {
    [0] = {
    .start = 0xE03001C0,
    .end = 0xE03001C0 +8 - 1,
    .flags = IORESOURCE_MEM,
    },
};
static struct resource led_resourcces1[] = {
    [0] = {
    .start = 0xE03001C0,
    .end = 0xE03001C0 +8 - 1,
    .flags = IORESOURCE_MEM,
    },
};
static struct platform_device led_device0 = {
    .name = "led",
    .id = 0,
    .resource = led_resourcces0,
    .num_resources = ARRAY_SIZE(led_resourcces0),
};

static struct platform_device led_device1 = {
    .name = "led",
    .resource = led_resourcces1,
    .num_resources = ARRAY_SIZE(led_resourcces1),
    .id = 1,
};

#ifdef CONFIG_MTD_NAND_S3C
/* Nand Flash Support */
static struct mtd_partition s5pc100_nand_part[] = {
	[0] = {
			.name = "bootloader",
			.size = SZ_1M,
			.offset = 0,
	},
	[1] = {
			.name = "kernel",
			.size = SZ_1M*3,
			.offset = MTDPART_OFS_APPEND,
	},
	[2] = {
			.name = "rootfs",
			.size = SZ_4M,
			.offset = MTDPART_OFS_APPEND,
	},
	[3] = {
			.name = "usrfs",
			.size = MTDPART_SIZ_FULL,
			.offset = MTDPART_OFS_APPEND,
	},
};

struct s3c_nand_mtd_info s5pc100_nand_mtd_part_info = {
	.chip_nr = 1,
	.mtd_part_nr = ARRAY_SIZE(s5pc100_nand_part),
	.partition = s5pc100_nand_part,
};

static struct resource s5pc100_nand_resource[] = {
	[0] = {
			.start = 0xE7200000,
			.end = 0xE7200000 + SZ_1M,
			.flags = IORESOURCE_MEM,
	}
};

struct platform_device s5pc100_device_nand = {
	.name = "s5pc100-nand",
	.id = -1,
	.num_resources = ARRAY_SIZE(s5pc100_nand_resource),
	.resource = s5pc100_nand_resource,
	.dev = {
			.platform_data = &s5pc100_nand_mtd_part_info,
	}
};	
#endif

#ifdef CONFIG_TOUCHSCREEN_S3C2410
static struct s3c2410_ts_mach_info s5pc100_ts_cfg __initdata = {
	.delay = 20000,
	.presc = 49,
	.oversampling_shift = 2,
};
#endif

/* I2C0 */
static struct i2c_board_info i2c_devs0[] __initdata = {
};

/* I2C1 */
static struct i2c_board_info i2c_devs1[] __initdata = {
};

/* LCD power controller */
static void smdkc100_lcd_power_set(struct plat_lcd_data *pd,
				   unsigned int power)
{
	/* backlight */
	gpio_direction_output(S5PC100_GPD(0), power);

	if (power) {
		/* module reset */
		gpio_direction_output(S5PC100_GPH0(6), 1);
		mdelay(100);
		gpio_direction_output(S5PC100_GPH0(6), 0);
		mdelay(10);
		gpio_direction_output(S5PC100_GPH0(6), 1);
		mdelay(10);
	}
}

static struct plat_lcd_data smdkc100_lcd_power_data = {
	.set_power	= smdkc100_lcd_power_set,
};

static struct platform_device smdkc100_lcd_powerdev = {
	.name			= "platform-lcd",
	.dev.parent		= &s3c_device_fb.dev,
	.dev.platform_data	= &smdkc100_lcd_power_data,
};

/* Frame Buffer */
static struct s3c_fb_pd_win smdkc100_fb_win0 = {
	/* this is to ensure we use win0 */
	.win_mode	= {
		.pixclock = 1000000000000ULL / ((8+43+1+480)*(4+10+12+272)*80),
		.left_margin	= 8,
		.right_margin	= 43,
		.upper_margin	= 4,
		.lower_margin	= 12,
		.hsync_len	= 1,
		.vsync_len	= 10,
		.xres		= 480,
		.yres		= 272,
	},
	.max_bpp	= 32,
	.default_bpp	= 16,
};

static struct s3c_fb_platdata smdkc100_lcd_pdata __initdata = {
	.win[0]		= &smdkc100_fb_win0,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB,
	.vidcon1	= VIDCON1_INV_HSYNC | VIDCON1_INV_VSYNC,
	.setup_gpio	= s5pc100_fb_gpio_setup_24bpp,
};

static struct platform_device *smdkc100_devices[] __initdata = {
	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_fb,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_hsmmc2,
	&smdkc100_lcd_powerdev,
	&s5pc100_device_iis0,
	&s5pc100_device_ac97,
#ifdef CONFIG_DM9000
	&s5pc100_device_dm9000,
#endif
#ifdef CONFIG_MTD_NAND_S3C
	&s5pc100_device_nand,
#endif
#ifdef CONFIG_USB_SUPPORT
	&s3c_device_ohci,
#endif
#ifdef CONFIG_TOUCHSCREEN_S3C2410
	&s3c_device_adc,
	&s3c_device_ts,
#endif
    &led_device0,
    &led_device1,
};

void usb_host_clk_en(void) 
{
	writel((readl(S5P_EPLL_CON)&~(S5P_EPLL_MASK)) | (S5P_EPLL_EN|S5P_EPLLVAL(96,3,3)), S5P_EPLL_CON);
	writel((readl(S5P_CLK_SRC0)|S5P_CLKSRC0_EPLL_MASK), S5P_CLK_SRC0);
	writel((readl(S5P_CLK_SRC1)&~S5P_CLKSRC1_UHOST_MASK), S5P_CLK_SRC1);

	writel((readl(S5P_CLK_DIV2)&~S5P_CLKDIV2_UHOST_MASK), S5P_CLK_DIV2);
	writel(readl(S5P_CLKGATE_D10)|S5P_CLKGATE_D10_USBHOST, S5P_CLKGATE_D10);
	writel(readl(S5P_SCLKGATE0)|S5P_CLKGATE_SCLK0_USBHOST, S5P_SCLKGATE0);
}
EXPORT_SYMBOL(usb_host_clk_en);


static void __init smdkc100_map_io(void)
{
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(12000000);
	s3c24xx_init_uarts(smdkc100_uartcfgs, ARRAY_SIZE(smdkc100_uartcfgs));
}

static void __init smdkc100_machine_init(void)
{
	/* I2C */
	s3c_i2c0_set_platdata(NULL);
	s3c_i2c1_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));

	s3c_fb_set_platdata(&smdkc100_lcd_pdata);
#ifdef CONFIG_TOUCHSCREEN_S3C2410
	s3c24xx_ts_set_platdata(&s5pc100_ts_cfg);
#endif

	/* LCD init */
	gpio_request(S5PC100_GPD(0), "GPD");
	gpio_request(S5PC100_GPH0(6), "GPH0");
	smdkc100_lcd_power_set(&smdkc100_lcd_power_data, 0);
	platform_add_devices(smdkc100_devices, ARRAY_SIZE(smdkc100_devices));
}

MACHINE_START(SMDKC100, "SMDKC100")
	/* Maintainer: Byungho Min <bhmin@samsung.com> */
	.phys_io	= S3C_PA_UART & 0xfff00000,
	.io_pg_offst	= (((u32)S3C_VA_UART) >> 18) & 0xfffc,
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= s5pc100_init_irq,
	.map_io		= smdkc100_map_io,
	.init_machine	= smdkc100_machine_init,
	.timer		= &s3c24xx_timer,
MACHINE_END
