/*
 * Copyright (C) 2004-2007, 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2008 Juergen Beisert
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation
 * 51 Franklin Street, Fifth Floor
 * Boston, MA  02110-1301, USA.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/types.h>

#include <linux/proc_fs.h>
#include <asm/ioctl.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/sched.h> 
#include <linux/wait.h>
#include <linux/ioport.h>
#include <linux/poll.h>     
#include <asm/system.h>     
#include <asm/uaccess.h>
#include <asm/unistd.h>
#include <linux/string.h>

#include <mach/spi.h>

#define DRIVER_NAME "spi_imx"

#define MXC_CSPIRXDATA		0x00
#define MXC_CSPITXDATA		0x04
#define MXC_CSPICTRL		0x08
#define MXC_CSPIINT		0x0c
#define MXC_RESET		0x1c

#define MX3_CSPISTAT		0x14
#define MX3_CSPISTAT_RR		(1 << 3)

/* generic defines to abstract from the different register layouts */
#define MXC_INT_RR	(1 << 0) /* Receive data ready interrupt */
#define MXC_INT_TE	(1 << 1) /* Transmit FIFO empty interrupt */

/* ---------- falinux ------------------------------------------------------- */
static int btds_transfer_flag = 0;
static int test_call_count = 4;

static char btds_rx_buf[4096];

static struct spi_imx_data *btds_spi_imx = NULL;

static void test_transfer(void);

#define	DEV_TEST_NAME	"btds-spi-imx"

#define DEBUG_SPI 0

#if DEBUG_SPI
	#define debug_spi(fmt, ...) {printk(KERN_ALERT"[%s:%d] "fmt, __func__,\
	__LINE__, ##__VA_ARGS__);}
#else
	#define debug_spi(fmt, ...)
#endif 

#define debug_err(fmt, ...) {printk(KERN_ALERT"[%s:%d] "fmt, __func__,\
	__LINE__, ##__VA_ARGS__);}
/* -------------------------------------------------------------------------- */

struct spi_imx_config {
	unsigned int speed_hz;
	unsigned int bpw;
	unsigned int mode;
	u8 cs;
};

enum spi_imx_devtype {
	SPI_IMX_VER_IMX1,
	SPI_IMX_VER_0_0,
	SPI_IMX_VER_0_4,
	SPI_IMX_VER_0_5,
	SPI_IMX_VER_0_7,
	SPI_IMX_VER_2_3,
};

struct spi_imx_data;

struct spi_imx_devtype_data {
	void (*intctrl)(struct spi_imx_data *, int);
	int (*config)(struct spi_imx_data *, struct spi_imx_config *);
	void (*trigger)(struct spi_imx_data *);
	int (*rx_available)(struct spi_imx_data *);
	void (*reset)(struct spi_imx_data *);
	unsigned int fifosize;
};

struct spi_imx_data {
	struct spi_bitbang bitbang;

	struct completion xfer_done;
//	int xfer_done_flag;
	void *base;
	int irq;
	struct clk *clk;
	unsigned long spi_clk;
	int *chipselect;

	unsigned int count;
	void (*tx)(struct spi_imx_data *);
	void (*rx)(struct spi_imx_data *);
	void *rx_buf;
	const void *tx_buf;
	unsigned int txfifo; /* number of words pushed in tx FIFO */

	struct spi_imx_devtype_data devtype_data;
};

#define MXC_SPI_BUF_RX(type)						\
static void spi_imx_buf_rx_##type(struct spi_imx_data *spi_imx)		\
{									\
	unsigned int val = readl(spi_imx->base + MXC_CSPIRXDATA);	\
									\
	if (spi_imx->rx_buf) {						\
		*(type *)spi_imx->rx_buf = val;				\
		spi_imx->rx_buf += sizeof(type);			\
	}								\
}

#define MXC_SPI_BUF_TX(type)						\
static void spi_imx_buf_tx_##type(struct spi_imx_data *spi_imx)		\
{									\
	type val = 0;							\
									\
	if (spi_imx->tx_buf) {						\
		val = *(type *)spi_imx->tx_buf;				\
		spi_imx->tx_buf += sizeof(type);			\
	}								\
									\
	spi_imx->count -= sizeof(type);					\
									\
	writel(val, spi_imx->base + MXC_CSPITXDATA);			\
}

MXC_SPI_BUF_RX(u8)
MXC_SPI_BUF_TX(u8)
MXC_SPI_BUF_RX(u16)
MXC_SPI_BUF_TX(u16)
MXC_SPI_BUF_RX(u32)
MXC_SPI_BUF_TX(u32)

/* First entry is reserved, second entry is valid only if SDHC_SPIEN is set
 * (which is currently not the case in this driver)
 */
static int mxc_clkdivs[] = {0, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96, 128, 192,
	256, 384, 512, 768, 1024};

/* MX21, MX27 */
static unsigned int spi_imx_clkdiv_1(unsigned int fin,
		unsigned int fspi)
{
	int i, max;

	if (cpu_is_mx21())
		max = 18;
	else
		max = 16;

	for (i = 2; i < max; i++)
		if (fspi * mxc_clkdivs[i] >= fin)
			return i;

	return max;
}

/* MX1, MX31, MX35, MX51 CSPI */
static unsigned int spi_imx_clkdiv_2(unsigned int fin,
		unsigned int fspi)
{
	int i, div = 4;

	for (i = 0; i < 7; i++) {
		if (fspi * div >= fin)
			return i;
		div <<= 1;
	}

	return 7;
}

#define SPI_IMX2_3_CTRL		0x08
#define SPI_IMX2_3_CTRL_ENABLE		(1 <<  0)
#define SPI_IMX2_3_CTRL_XCH		(1 <<  2)
#define SPI_IMX2_3_CTRL_MODE_MASK	(0xf << 4)
#define SPI_IMX2_3_CTRL_POSTDIV_OFFSET	8
#define SPI_IMX2_3_CTRL_PREDIV_OFFSET	12
#define SPI_IMX2_3_CTRL_CS(cs)		((cs) << 18)
#define SPI_IMX2_3_CTRL_BL_OFFSET	20

#define SPI_IMX2_3_CONFIG	0x0c
#define SPI_IMX2_3_CONFIG_SCLKPHA(cs)	(1 << ((cs) +  0))
#define SPI_IMX2_3_CONFIG_SCLKPOL(cs)	(1 << ((cs) +  4))
#define SPI_IMX2_3_CONFIG_SBBCTRL(cs)	(1 << ((cs) +  8))
#define SPI_IMX2_3_CONFIG_SSBPOL(cs)	(1 << ((cs) + 12))

#define SPI_IMX2_3_INT		0x10
#define SPI_IMX2_3_INT_TEEN		(1 <<  0)
#define SPI_IMX2_3_INT_RREN		(1 <<  3)

#define SPI_IMX2_3_STAT		0x18
#define SPI_IMX2_3_STAT_RR		(1 <<  3)


#if 0
static int my_pre = 0;
static int my_post = 0;

static int __init spi_clk_post(char *s)
{
	my_post = simple_strtoul(s, NULL, 0);
	return 1;
}

static int __init spi_clk_pre(char *s)
{
	my_pre = simple_strtoul(s, NULL, 0);
	return 1;
}

__setup("pre=", spi_clk_pre);
__setup("post=", spi_clk_post);

#endif

/* MX51 eCSPI */
static unsigned int spi_imx2_3_clkdiv(unsigned int fin, unsigned int fspi)
{
	/*
	 * there are two 4-bit dividers, the pre-divider divides by
	 * $pre, the post-divider by 2^$post
	 */
	unsigned int pre, post;

	debug_spi("------------------>spi_imx->spi_clk=%d, config->speed_hz=%d\n", fin, fspi);

	if (unlikely(fspi > fin))
		return 0;

	post = fls(fin) - fls(fspi);
	if (fin > fspi << post)
		post++;

	/* now we have: (fin <= fspi << post) with post being minimal */

	post = max(4U, post) - 4;
	if (unlikely(post > 0xf)) {
		pr_err("%s: cannot set clock freq: %u (base freq: %u)\n",
				__func__, fspi, fin);
		return 0xff;
	}

	pre = DIV_ROUND_UP(fin, fspi << post) - 1;
	
//	pre = my_pre;
//	post = my_post;

	pr_debug("%s: fin: %u, fspi: %u, post: %u, pre: %u\n",
			__func__, fin, fspi, post, pre);

	debug_spi("%s: fin: %u, fspi: %u, post: %u, pre: %u\n",
			__func__, fin, fspi, post, pre);
	return (pre << SPI_IMX2_3_CTRL_PREDIV_OFFSET) |
		(post << SPI_IMX2_3_CTRL_POSTDIV_OFFSET);
}

static void __maybe_unused spi_imx2_3_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned val = 0;

	if (enable & MXC_INT_TE)
		val |= SPI_IMX2_3_INT_TEEN;

	if (enable & MXC_INT_RR)
		val |= SPI_IMX2_3_INT_RREN;

	writel(val, spi_imx->base + SPI_IMX2_3_INT);
}

static void __maybe_unused spi_imx2_3_trigger(struct spi_imx_data *spi_imx)
{
	u32 reg;

	reg = readl(spi_imx->base + SPI_IMX2_3_CTRL);
	reg |= SPI_IMX2_3_CTRL_XCH;
	writel(reg, spi_imx->base + SPI_IMX2_3_CTRL);
}

static int __maybe_unused spi_imx2_3_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	u32 ctrl = SPI_IMX2_3_CTRL_ENABLE, cfg = 0;

	/*
	 * The hardware seems to have a race condition when changing modes. The
	 * current assumption is that the selection of the channel arrives
	 * earlier in the hardware than the mode bits when they are written at
	 * the same time.
	 * So set master mode for all channels as we do not support slave mode.
	 */
	ctrl |= SPI_IMX2_3_CTRL_MODE_MASK;

	/* set clock speed */
	ctrl |= spi_imx2_3_clkdiv(spi_imx->spi_clk, config->speed_hz);

	/* set chip select to use */
	ctrl |= SPI_IMX2_3_CTRL_CS(config->cs);

	ctrl |= (config->bpw - 1) << SPI_IMX2_3_CTRL_BL_OFFSET;

	cfg |= SPI_IMX2_3_CONFIG_SBBCTRL(config->cs);

	if (config->mode & SPI_CPHA)
		cfg |= SPI_IMX2_3_CONFIG_SCLKPHA(config->cs);

	if (config->mode & SPI_CPOL)
		cfg |= SPI_IMX2_3_CONFIG_SCLKPOL(config->cs);

	if (config->mode & SPI_CS_HIGH)
		cfg |= SPI_IMX2_3_CONFIG_SSBPOL(config->cs);

	writel(ctrl, spi_imx->base + SPI_IMX2_3_CTRL);
	writel(cfg, spi_imx->base + SPI_IMX2_3_CONFIG);

	return 0;
}

static int __maybe_unused spi_imx2_3_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + SPI_IMX2_3_STAT) & SPI_IMX2_3_STAT_RR;
}

static void __maybe_unused spi_imx2_3_reset(struct spi_imx_data *spi_imx)
{
	/* drain receive buffer */
	while (spi_imx2_3_rx_available(spi_imx))
		readl(spi_imx->base + MXC_CSPIRXDATA);
}

#define MX31_INTREG_TEEN	(1 << 0)
#define MX31_INTREG_RREN	(1 << 3)

#define MX31_CSPICTRL_ENABLE	(1 << 0)
#define MX31_CSPICTRL_MASTER	(1 << 1)
#define MX31_CSPICTRL_XCH	(1 << 2)
#define MX31_CSPICTRL_POL	(1 << 4)
#define MX31_CSPICTRL_PHA	(1 << 5)
#define MX31_CSPICTRL_SSCTL	(1 << 6)
#define MX31_CSPICTRL_SSPOL	(1 << 7)
#define MX31_CSPICTRL_BC_SHIFT	8
#define MX35_CSPICTRL_BL_SHIFT	20
#define MX31_CSPICTRL_CS_SHIFT	24
#define MX35_CSPICTRL_CS_SHIFT	12
#define MX31_CSPICTRL_DR_SHIFT	16

#define MX31_CSPISTATUS		0x14
#define MX31_STATUS_RR		(1 << 3)

/* These functions also work for the i.MX35, but be aware that
 * the i.MX35 has a slightly different register layout for bits
 * we do not use here.
 */
static void __maybe_unused mx31_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX31_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX31_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void __maybe_unused mx31_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX31_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int __maybe_unused spi_imx0_4_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX31_CSPICTRL_ENABLE | MX31_CSPICTRL_MASTER;
	int cs = spi_imx->chipselect[config->cs];

	reg |= spi_imx_clkdiv_2(spi_imx->spi_clk, config->speed_hz) <<
		MX31_CSPICTRL_DR_SHIFT;

	reg |= (config->bpw - 1) << MX31_CSPICTRL_BC_SHIFT;

	if (config->mode & SPI_CPHA)
		reg |= MX31_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX31_CSPICTRL_POL;
	if (config->mode & SPI_CS_HIGH)
		reg |= MX31_CSPICTRL_SSPOL;
	if (cs < 0)
		reg |= (cs + 32) << MX31_CSPICTRL_CS_SHIFT;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused spi_imx0_7_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX31_CSPICTRL_ENABLE | MX31_CSPICTRL_MASTER;
	int cs = spi_imx->chipselect[config->cs];

	reg |= spi_imx_clkdiv_2(spi_imx->spi_clk, config->speed_hz) <<
		MX31_CSPICTRL_DR_SHIFT;

	reg |= (config->bpw - 1) << MX35_CSPICTRL_BL_SHIFT;
	reg |= MX31_CSPICTRL_SSCTL;

	if (config->mode & SPI_CPHA)
		reg |= MX31_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX31_CSPICTRL_POL;
	if (config->mode & SPI_CS_HIGH)
		reg |= MX31_CSPICTRL_SSPOL;
	if (cs < 0)
		reg |= (cs + 32) << MX35_CSPICTRL_CS_SHIFT;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused mx31_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MX31_CSPISTATUS) & MX31_STATUS_RR;
}

static void __maybe_unused spi_imx0_4_reset(struct spi_imx_data *spi_imx)
{
	/* drain receive buffer */
	while (readl(spi_imx->base + MX3_CSPISTAT) & MX3_CSPISTAT_RR)
		readl(spi_imx->base + MXC_CSPIRXDATA);
}

#define MX27_INTREG_RR		(1 << 4)
#define MX27_INTREG_TEEN	(1 << 9)
#define MX27_INTREG_RREN	(1 << 13)

#define MX27_CSPICTRL_POL	(1 << 5)
#define MX27_CSPICTRL_PHA	(1 << 6)
#define MX27_CSPICTRL_SSPOL	(1 << 8)
#define MX27_CSPICTRL_XCH	(1 << 9)
#define MX27_CSPICTRL_ENABLE	(1 << 10)
#define MX27_CSPICTRL_MASTER	(1 << 11)
#define MX27_CSPICTRL_DR_SHIFT	14
#define MX27_CSPICTRL_CS_SHIFT	19

static void __maybe_unused mx27_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX27_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX27_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void __maybe_unused mx27_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX27_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int __maybe_unused mx27_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX27_CSPICTRL_ENABLE | MX27_CSPICTRL_MASTER;
	int cs = spi_imx->chipselect[config->cs];

	reg |= spi_imx_clkdiv_1(spi_imx->spi_clk, config->speed_hz) <<
		MX27_CSPICTRL_DR_SHIFT;
	reg |= config->bpw - 1;

	if (config->mode & SPI_CPHA)
		reg |= MX27_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX27_CSPICTRL_POL;
	if (config->mode & SPI_CS_HIGH)
		reg |= MX27_CSPICTRL_SSPOL;
	if (cs < 0)
		reg |= (cs + 32) << MX27_CSPICTRL_CS_SHIFT;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused mx27_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MXC_CSPIINT) & MX27_INTREG_RR;
}

static void __maybe_unused spi_imx0_0_reset(struct spi_imx_data *spi_imx)
{
	writel(1, spi_imx->base + MXC_RESET);
}

#define MX1_INTREG_RR		(1 << 3)
#define MX1_INTREG_TEEN		(1 << 8)
#define MX1_INTREG_RREN		(1 << 11)

#define MX1_CSPICTRL_POL	(1 << 4)
#define MX1_CSPICTRL_PHA	(1 << 5)
#define MX1_CSPICTRL_XCH	(1 << 8)
#define MX1_CSPICTRL_ENABLE	(1 << 9)
#define MX1_CSPICTRL_MASTER	(1 << 10)
#define MX1_CSPICTRL_DR_SHIFT	13

static void __maybe_unused mx1_intctrl(struct spi_imx_data *spi_imx, int enable)
{
	unsigned int val = 0;

	if (enable & MXC_INT_TE)
		val |= MX1_INTREG_TEEN;
	if (enable & MXC_INT_RR)
		val |= MX1_INTREG_RREN;

	writel(val, spi_imx->base + MXC_CSPIINT);
}

static void __maybe_unused mx1_trigger(struct spi_imx_data *spi_imx)
{
	unsigned int reg;

	reg = readl(spi_imx->base + MXC_CSPICTRL);
	reg |= MX1_CSPICTRL_XCH;
	writel(reg, spi_imx->base + MXC_CSPICTRL);
}

static int __maybe_unused mx1_config(struct spi_imx_data *spi_imx,
		struct spi_imx_config *config)
{
	unsigned int reg = MX1_CSPICTRL_ENABLE | MX1_CSPICTRL_MASTER;

	reg |= spi_imx_clkdiv_2(spi_imx->spi_clk, config->speed_hz) <<
		MX1_CSPICTRL_DR_SHIFT;
	reg |= config->bpw - 1;

	if (config->mode & SPI_CPHA)
		reg |= MX1_CSPICTRL_PHA;
	if (config->mode & SPI_CPOL)
		reg |= MX1_CSPICTRL_POL;

	writel(reg, spi_imx->base + MXC_CSPICTRL);

	return 0;
}

static int __maybe_unused mx1_rx_available(struct spi_imx_data *spi_imx)
{
	return readl(spi_imx->base + MXC_CSPIINT) & MX1_INTREG_RR;
}

static void __maybe_unused mx1_reset(struct spi_imx_data *spi_imx)
{
	writel(1, spi_imx->base + MXC_RESET);
}

/*
 * These version numbers are taken from the Freescale driver.  Unfortunately it
 * doesn't support i.MX1, so this entry doesn't match the scheme. :-(
 */
static struct spi_imx_devtype_data spi_imx_devtype_data[] __devinitdata = {
#ifdef CONFIG_SPI_IMX_VER_IMX1
	[SPI_IMX_VER_IMX1] = {
		.intctrl = mx1_intctrl,
		.config = mx1_config,
		.trigger = mx1_trigger,
		.rx_available = mx1_rx_available,
		.reset = mx1_reset,
		.fifosize = 8,
	},
#endif
#ifdef CONFIG_SPI_IMX_VER_0_0
	[SPI_IMX_VER_0_0] = {
		.intctrl = mx27_intctrl,
		.config = mx27_config,
		.trigger = mx27_trigger,
		.rx_available = mx27_rx_available,
		.reset = spi_imx0_0_reset,
		.fifosize = 8,
	},
#endif
#ifdef CONFIG_SPI_IMX_VER_0_4
	[SPI_IMX_VER_0_4] = {
		.intctrl = mx31_intctrl,
		.config = spi_imx0_4_config,
		.trigger = mx31_trigger,
		.rx_available = mx31_rx_available,
		.reset = spi_imx0_4_reset,
		.fifosize = 8,
	},
#endif
#ifdef CONFIG_SPI_IMX_VER_0_7
	[SPI_IMX_VER_0_7] = {
		.intctrl = mx31_intctrl,
		.config = spi_imx0_7_config,
		.trigger = mx31_trigger,
		.rx_available = mx31_rx_available,
		.reset = spi_imx0_4_reset,
		.fifosize = 8,
	},
#endif
#ifdef CONFIG_SPI_IMX_VER_2_3
	[SPI_IMX_VER_2_3] = {
		.intctrl = spi_imx2_3_intctrl,
		.config = spi_imx2_3_config,
		.trigger = spi_imx2_3_trigger,
		.rx_available = spi_imx2_3_rx_available,
		.reset = spi_imx2_3_reset,
		.fifosize = 64,
	},
#endif
};

static void spi_imx_chipselect(struct spi_device *spi, int is_active)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	int gpio = spi_imx->chipselect[spi->chip_select];
	int active = is_active != BITBANG_CS_INACTIVE;
	int dev_is_lowactive = !(spi->mode & SPI_CS_HIGH);

	if (gpio < 0)
		return;

	gpio_set_value(gpio, dev_is_lowactive ^ active);
}

static int irq_flag = 0;


static void spi_imx_push_we(struct spi_imx_data *spi_imx, int trigger)
{
    int tr_count;
    unsigned long flags;
    unsigned int val;

	tr_count = spi_imx->count - trigger;

if (irq_flag)
    printk(KERN_ALERT"----------->tr_count=%d, spi_imx->count=%d\n", tr_count, spi_imx->count);

	local_irq_save(flags);

	while (spi_imx->txfifo < spi_imx->devtype_data.fifosize) {
		if (spi_imx->count == tr_count)
		    spi_imx->devtype_data.trigger(spi_imx);

		if (!spi_imx->count)
			break;
		
		// spi_imx->tx(spi_imx);
		{
		    val = *(u32 *)spi_imx->tx_buf;				
		    spi_imx->tx_buf += 4;

		    spi_imx->count -= 4;

		    writel(val, spi_imx->base + MXC_CSPITXDATA);			
		}
	
		spi_imx->txfifo++;
	}
	if (1 <= spi_imx->devtype_data.fifosize)
	    spi_imx->devtype_data.trigger(spi_imx);

	local_irq_restore(flags);
}

static void spi_imx_push_org(struct spi_imx_data *spi_imx)
{
	while (spi_imx->txfifo < spi_imx->devtype_data.fifosize) {
		if (!spi_imx->count)
			break;
		spi_imx->tx(spi_imx);
		spi_imx->txfifo++;
	}
	spi_imx->devtype_data.trigger(spi_imx);
}

int spi_imx_push_tr_count_enable = 0;
EXPORT_SYMBOL(spi_imx_push_tr_count_enable);

#define  TRIGGER_COUNT 128

static void spi_imx_push(struct spi_imx_data *spi_imx)
{

#if 1
	if (!spi_imx_push_tr_count_enable) {

//			printk(KERN_ALERT"---> Kernel org\n");
			spi_imx_push_org(spi_imx);
			return ;
	}

//	printk(KERN_ALERT"---> tr count start\n");

	if (spi_imx->count >= TRIGGER_COUNT) {
		spi_imx_push_we(spi_imx, TRIGGER_COUNT);
	} else {
		spi_imx_push_org(spi_imx);
	}
#else
//	spi_imx_push_org(spi_imx);
//	return ;
    // fifo count >= 8 or 16  
	if (spi_imx->count >= TRIGGER_COUNT) {
		spi_imx_push_we(spi_imx, TRIGGER_COUNT);
	} else {
		spi_imx_push_org(spi_imx);
	}
#endif
}

/* 
 * 인터럽트용 spi_imx_push를 구현한다.
 */
static void spi_imx_push_irq(struct spi_imx_data *spi_imx)
{
	debug_spi("spi_imx->devtype_data.fifosize=%d\n", spi_imx->devtype_data.fifosize);
}

/* ---------- falinux : callback function ----------------------------------- */
static void (*btds_spi_irq_callback)(char *buf, int blen) = NULL;
void btds_register_irq_callback(void (*func)(char *buf, int blen))
{
	btds_spi_irq_callback = func;
}
EXPORT_SYMBOL(btds_register_irq_callback);
/* -------------------------------------------------------------------------- */


static irqreturn_t spi_imx_isr(int irq, void *dev_id)
{
	struct spi_imx_data *spi_imx = dev_id;

	void *start_rx_buf;             /* falinux */
	start_rx_buf = spi_imx->rx_buf; /* falinux */

	irq_flag = 1;

	while (spi_imx->devtype_data.rx_available(spi_imx)) { // 가용한 게 있는지 확인
		// 정확한 rx카운터를 알수 있다.
		spi_imx->rx(spi_imx);
		spi_imx->txfifo--;
	}


#if 1
	if (spi_imx->count) {
		spi_imx_push_irq(spi_imx);
		return IRQ_HANDLED;
	}

#else	//origin
	if (spi_imx->count) {
		spi_imx_push(spi_imx);
		return IRQ_HANDLED;
	}
#endif

	if (spi_imx->txfifo) {
		/* No data left to push, but still waiting for rx data,
		 * enable receive data available interrupt.
		 */
		spi_imx->devtype_data.intctrl(
				spi_imx, MXC_INT_RR); /* Receive data ready interrupt */

		return IRQ_HANDLED;
	}

	spi_imx->devtype_data.intctrl(spi_imx, 0);
	complete(&spi_imx->xfer_done);
//	spi_imx->xfer_done_flag = 1;

	if (btds_transfer_flag && btds_spi_irq_callback) { /* falinux */
		btds_spi_irq_callback(start_rx_buf, spi_imx->rx_buf - start_rx_buf);
	}
	irq_flag = 0;

	return IRQ_HANDLED;
}

static int spi_imx_setupxfer(struct spi_device *spi,
				 struct spi_transfer *t)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	struct spi_imx_config config;

	clk_enable(spi_imx->clk);
	config.bpw = t ? t->bits_per_word : spi->bits_per_word;
	config.speed_hz  = t ? t->speed_hz : spi->max_speed_hz;
	config.mode = spi->mode;
	config.cs = spi->chip_select;


	debug_spi("spi --> config.speed_hz=%d, spi->max_speed_hz=%d\n", config.speed_hz, spi->max_speed_hz); 

	if (!config.speed_hz)
		config.speed_hz = spi->max_speed_hz;
	if (!config.bpw)
		config.bpw = spi->bits_per_word;
	if (!config.speed_hz)
		config.speed_hz = spi->max_speed_hz;

	/* Initialize the functions for transfer */
	if (config.bpw <= 8) {
		spi_imx->rx = spi_imx_buf_rx_u8;
		spi_imx->tx = spi_imx_buf_tx_u8;
	} else if (config.bpw <= 16) {
		spi_imx->rx = spi_imx_buf_rx_u16;
		spi_imx->tx = spi_imx_buf_tx_u16;
	} else if (config.bpw <= 32) {
		spi_imx->rx = spi_imx_buf_rx_u32;
		spi_imx->tx = spi_imx_buf_tx_u32;
	} else
		BUG();

	spi_imx->devtype_data.config(spi_imx, &config);
	clk_disable(spi_imx->clk);
	return 0;
}

static int spi_imx_transfer(struct spi_device *spi,
				struct spi_transfer *transfer)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);

	clk_enable(spi_imx->clk);
	spi_imx->tx_buf = transfer->tx_buf;
	spi_imx->rx_buf = transfer->rx_buf;
	spi_imx->count = transfer->len;
	spi_imx->txfifo = 0;

//	debug_spi("transfer->len=%d\n", transfer->len);

//	init_completion(&spi_imx->xfer_done);
//	
//	spi_imx->xfer_done_flag = 0;
	
	spi_imx_push(spi_imx);
//	debug_spi("spi_imx->txfifo = %d\n", spi_imx->txfifo);

	spi_imx->devtype_data.intctrl(spi_imx, MXC_INT_TE);

//	while (!spi_imx->xfer_done_flag)

	wait_for_completion(&spi_imx->xfer_done);
	clk_disable(spi_imx->clk);

	return transfer->len;
}

#if 1
/* ---------- falinux ------------------------------------------------------- */
/*
 spi_imx_transfer()에서는 wait_for_completion()를 이용하여 
 SPI 전송이 완료 되는것을 기다렸기렸다. 
 이러한 구성 때문에, 어플리케이션 입장에서 SPI로 전송 하는 시간만큼
 지연이 발생한다. 이러한 문제를 해결하기 위해 
 btds_spi_imx_transfer()를 구혀하여 위와 같은 문제를 해결 하였다. 
 */

/*
 @return byte lenght 
 */
int btds_spi_imx_transfer(char *tx_buf, char *rx_buf, int blen)
{
	struct spi_imx_data *spi_imx = btds_spi_imx;

	if (!btds_transfer_flag) clk_enable(spi_imx->clk);

	debug_spi("\n");
//	debug_spi("tx_buf = %08X\n", *(unsigned long *)tx_buf);
//	debug_spi("rx_buf = %08X\n", *(unsigned long *)rx_buf);

	spi_imx->tx_buf = tx_buf;
	spi_imx->rx_buf = rx_buf;
	spi_imx->count = blen;
	spi_imx->txfifo = 0;
//	debug_spi("-----------------------\n");
//	debug_spi("transfer->len=%d\n", blen);
//
	spi_imx_push(spi_imx);                                   /* 데이터를 전송 */
//	debug_spi("spi_imx->txfifo = %d\n", spi_imx->txfifo);

	btds_transfer_flag = 1;                             /* 전송 플래그를 세팅 */

	spi_imx->devtype_data.intctrl(spi_imx, MXC_INT_TE); /* 전송 완료 INT 발생 */

	return blen;
}
EXPORT_SYMBOL(btds_spi_imx_transfer);

void btds_spi_imx_transfer_end(void)
{
	struct spi_imx_data *spi_imx = btds_spi_imx;
	btds_transfer_flag = 0;                           /* 전송 플래그를 클리어 */
	spi_imx->devtype_data.intctrl(spi_imx, MXC_INT_TE); /* 전송 완료 INT 발생 */
//	clk_disable(spi_imx->clk);
}
EXPORT_SYMBOL(btds_spi_imx_transfer_end);

static void test_callback(char *buf, int blen)
{
	int *ptr = (int *)buf;
	int rx_len = blen / sizeof(int);

	debug_spi("test callback rx_len = %d, blen = %d, buf = %08X %08X %08X %08X\n",
		rx_len, blen, ptr[0], ptr[1], ptr[2], ptr[3]);

	test_call_count--;
	if (test_call_count > 0) {
		test_transfer();

	} else {
		btds_spi_imx_transfer_end();
	}
}

/*
 테스트 함수.
 proc write에서 호출되는 함수이다.
 콜백 함수로 test_callback()을 등록한다.
 어플리케이션의 지연 시간을 해결하기 위해 구현한,
 btds_spi_imx_trasfer()를 호출한다.
 */
static void test_transfer(void)
{
	int blen = 32 * sizeof(int);
	char tx_buf[1024];
	char *rx_buf;

	rx_buf = btds_rx_buf;

	memset(tx_buf, 0xaa, sizeof(tx_buf));
	memset(rx_buf, 0x00, sizeof(rx_buf));

	btds_register_irq_callback(test_callback);   /* callback function register */
	btds_spi_imx_transfer(tx_buf, rx_buf, blen);              /* data transfer */
}

static int std_proc_read(char *buf, char **start, off_t fpos, int lenght, int *eof, void *data)
{
	char *p;
	p = buf;
	p += sprintf(p, "\n%s \n", DEV_TEST_NAME);
	p += sprintf(p, "====================\n");
	p += sprintf(p, "\n");
	*eof = 1;

	return p - buf;
}

static int std_proc_write( struct file *file, const char __user *buffer, unsigned long count, void *data)
{
	char cmd[256];
	int	len = count;
	int ret;

	if (len > 256) len = 256;
	ret = copy_from_user(cmd, buffer, len);
	cmd[len-1] = 0;	// CR 제거


	if (strncmp("1", cmd, 1) == 0) { 
		test_transfer();
	}

	if (strncmp("2", cmd, 1) == 0) { 
		test_call_count = 100;
	}

	return count;
}

static void std_register_proc( void )
{
	struct proc_dir_entry *procdir;

	procdir = create_proc_entry(DEV_TEST_NAME, S_IFREG | S_IRUGO, 0);
	procdir->read_proc  = std_proc_read;
	procdir->write_proc = std_proc_write;
}

static void std_unregister_proc(void)
{
	remove_proc_entry(DEV_TEST_NAME, 0);
}
/* -------------------------------------------------------------------------- */
#endif

static int spi_imx_setup(struct spi_device *spi)
{
	struct spi_imx_data *spi_imx = spi_master_get_devdata(spi->master);
	int gpio = spi_imx->chipselect[spi->chip_select];

	dev_dbg(&spi->dev, "%s: mode %d, %u bpw, %d hz\n", __func__,
		 spi->mode, spi->bits_per_word, spi->max_speed_hz);

	if (gpio >= 0)
		gpio_direction_output(gpio, spi->mode & SPI_CS_HIGH ? 0 : 1);

	spi_imx_chipselect(spi, BITBANG_CS_INACTIVE);

	return 0;
}

static void spi_imx_cleanup(struct spi_device *spi)
{
}

static struct platform_device_id spi_imx_devtype[] = {
	{
		.name = "imx1-cspi",
		.driver_data = SPI_IMX_VER_IMX1,
	}, {
		.name = "imx21-cspi",
		.driver_data = SPI_IMX_VER_0_0,
	}, {
		.name = "imx25-cspi",
		.driver_data = SPI_IMX_VER_0_7,
	}, {
		.name = "imx27-cspi",
		.driver_data = SPI_IMX_VER_0_0,
	}, {
		.name = "imx31-cspi",
		.driver_data = SPI_IMX_VER_0_4,
	}, {
		.name = "imx35-cspi",
		.driver_data = SPI_IMX_VER_0_7,
	}, {
		.name = "imx50-cspi",
		.driver_data = SPI_IMX_VER_0_7,
	}, {
		.name = "imx51-cspi",
		.driver_data = SPI_IMX_VER_0_7,
	}, {
		.name = "imx51-ecspi",
		.driver_data = SPI_IMX_VER_2_3,
	}, {
		.name = "imx53-cspi",
		.driver_data = SPI_IMX_VER_0_7,
	}, {
		.name = "imx53-ecspi",
		.driver_data = SPI_IMX_VER_2_3,
	}, {
		.name = "imx6q-ecspi",
		.driver_data = SPI_IMX_VER_2_3,
	}, {
		/* sentinel */
	}
};

static int __devinit spi_imx_probe(struct platform_device *pdev)
{
	struct spi_imx_master *mxc_platform_info;
	struct spi_master *master;
	struct spi_imx_data *spi_imx;
	struct resource *res;
	int i, ret;

	mxc_platform_info = dev_get_platdata(&pdev->dev);
	if (!mxc_platform_info) {
		dev_err(&pdev->dev, "can't get the platform data\n");
		return -EINVAL;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(struct spi_imx_data));
	if (!master)
		return -ENOMEM;

	platform_set_drvdata(pdev, master);

	master->bus_num = pdev->id;
	master->num_chipselect = mxc_platform_info->num_chipselect;

	spi_imx = spi_master_get_devdata(master);
	spi_imx->bitbang.master = spi_master_get(master);
	spi_imx->chipselect = mxc_platform_info->chipselect;

	for (i = 0; i < master->num_chipselect; i++) {
		if (spi_imx->chipselect[i] < 0)
			continue;
		ret = gpio_request(spi_imx->chipselect[i], DRIVER_NAME);
		if (ret) {
			while (i > 0) {
				i--;
				if (spi_imx->chipselect[i] >= 0)
					gpio_free(spi_imx->chipselect[i]);
			}
			dev_err(&pdev->dev, "can't get cs gpios\n");
			goto out_master_put;
		}
	}

	spi_imx->bitbang.chipselect = spi_imx_chipselect;
	spi_imx->bitbang.setup_transfer = spi_imx_setupxfer;
	spi_imx->bitbang.txrx_bufs = spi_imx_transfer;
	spi_imx->bitbang.master->setup = spi_imx_setup;
	spi_imx->bitbang.master->cleanup = spi_imx_cleanup;
	spi_imx->bitbang.master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	init_completion(&spi_imx->xfer_done);

	spi_imx->devtype_data =
		spi_imx_devtype_data[pdev->id_entry->driver_data];

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "can't get platform resource\n");
		ret = -ENOMEM;
		goto out_gpio_free;
	}

	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		ret = -EBUSY;
		goto out_gpio_free;
	}

	spi_imx->base = ioremap(res->start, resource_size(res));
	if (!spi_imx->base) {
		ret = -EINVAL;
		goto out_release_mem;
	}

	spi_imx->irq = platform_get_irq(pdev, 0);
	if (spi_imx->irq < 0) {
		ret = -EINVAL;
		goto out_iounmap;
	}

	ret = request_irq(spi_imx->irq, spi_imx_isr, 0, DRIVER_NAME, spi_imx);
	if (ret) {
		dev_err(&pdev->dev, "can't get irq%d: %d\n", spi_imx->irq, ret);
		goto out_iounmap;
	}

	spi_imx->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(spi_imx->clk)) {
		dev_err(&pdev->dev, "unable to get clock\n");
		ret = PTR_ERR(spi_imx->clk);
		goto out_free_irq;
	}

	clk_enable(spi_imx->clk);
	spi_imx->spi_clk = clk_get_rate(spi_imx->clk);

	spi_imx->devtype_data.reset(spi_imx);

	spi_imx->devtype_data.intctrl(spi_imx, 0);
	ret = spi_bitbang_start(&spi_imx->bitbang);
	if (ret) {
		dev_err(&pdev->dev, "bitbang start failed with %d\n", ret);
		goto out_clk_put;
	}
	clk_disable(spi_imx->clk);

	dev_info(&pdev->dev, "probed\n");

	btds_spi_imx = spi_imx; /* falinux */
	std_register_proc();    /* falinux */

	return ret;

out_clk_put:
	clk_disable(spi_imx->clk);
	clk_put(spi_imx->clk);
out_free_irq:
	free_irq(spi_imx->irq, spi_imx);
out_iounmap:
	iounmap(spi_imx->base);
out_release_mem:
	release_mem_region(res->start, resource_size(res));
out_gpio_free:
	for (i = 0; i < master->num_chipselect; i++)
		if (spi_imx->chipselect[i] >= 0)
			gpio_free(spi_imx->chipselect[i]);
out_master_put:
	spi_master_put(master);
	kfree(master);
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int __devexit spi_imx_remove(struct platform_device *pdev)
{
	struct spi_master *master = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct spi_imx_data *spi_imx = spi_master_get_devdata(master);
	int i;

	spi_bitbang_stop(&spi_imx->bitbang);
	clk_enable(spi_imx->clk);
	writel(0, spi_imx->base + MXC_CSPICTRL);
	clk_disable(spi_imx->clk);
	clk_put(spi_imx->clk);
	free_irq(spi_imx->irq, spi_imx);
	iounmap(spi_imx->base);

	for (i = 0; i < master->num_chipselect; i++)
		if (spi_imx->chipselect[i] >= 0)
			gpio_free(spi_imx->chipselect[i]);

	spi_master_put(master);

	release_mem_region(res->start, resource_size(res));

	platform_set_drvdata(pdev, NULL);

	std_unregister_proc(); /* falinux */

	return 0;
}

static struct platform_driver spi_imx_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   },
	.id_table = spi_imx_devtype,
	.probe = spi_imx_probe,
	.remove = __devexit_p(spi_imx_remove),
};

static int __init spi_imx_init(void)
{
	return platform_driver_register(&spi_imx_driver);
}

static void __exit spi_imx_exit(void)
{
	platform_driver_unregister(&spi_imx_driver);
}

subsys_initcall(spi_imx_init);
module_exit(spi_imx_exit);

MODULE_DESCRIPTION("SPI Master Controller driver");
MODULE_LICENSE("Dual BSD/GPL");
