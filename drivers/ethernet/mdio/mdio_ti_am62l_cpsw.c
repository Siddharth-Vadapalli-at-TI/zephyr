/* TI AM62L CPSW MDIO controller driver
 *
 * Copyright (c) 2026 Texas Instruments Incorporated
 * Author: Siddharth Vadapalli <s-vadapalli@ti.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_am62l_cpsw_mdio

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(mdio_ti_am62l_cpsw, CONFIG_MDIO_LOG_LEVEL);

/* MDIO register offsets relative to module base */
#define MDIO_VERSION		0x00
#define MDIO_CONTROL		0x04
#define MDIO_ALIVE		0x08
#define MDIO_LINK		0x0c
#define MDIO_USERACCESS0	0x80

/* CONTROL register fields */
#define MDIO_CTL_ENABLE		BIT(30)
#define MDIO_CTL_IDLE		BIT(31)
#define MDIO_CTL_DIV_MASK	GENMASK(15, 0)
#define MDIO_CTL_DIV_DEF	0xffU

/* USERACCESS register fields */
#define MDIO_UA_GO		BIT(31)
#define MDIO_UA_WRITE		BIT(30)
#define MDIO_UA_ACK		BIT(29)
#define MDIO_UA_REGADR_SHIFT	21U
#define MDIO_UA_PHYADR_SHIFT	16U
#define MDIO_UA_DATA_MASK	GENMASK(15, 0)

/* Clause-22 indirect access registers for extended PHY regs */
#define MDIO_REGCR_REG		0x000dU
#define MDIO_ADDAR_REG		0x000eU
#define MDIO_REGCR_ADDR		0x001fU
#define MDIO_REGCR_DATA		0x401fU

/*
 * Wait at most 100 milliseconds for GO bit to clear. Poll every
 * 100 microseconds and retry 1000 times which is effectively a
 * timeout duration of 100 milliseconds.
 */
#define CPSW_MDIO_POLL_INTERVAL_US 100U
#define CPSW_MDIO_MAX_RETRIES      1000U

struct cpsw_mdio_config {
	DEVICE_MMIO_ROM;		/* Must be first: stores physical base + size */
	uint32_t  bus_freq;		/* Target MDC output frequency in Hz */
	const struct device *clk_dev;	/* Functional clock controller device */
	clock_control_subsys_t clk_subsys; /* Clock ID for functional clock */
};

struct cpsw_mdio_data {
	DEVICE_MMIO_RAM;		/* Must be first: stores virtual base address */
	struct k_mutex lock;		/* MDIO bus access lock */
};

static inline void mdio_writereg(uintptr_t base, uint32_t off, uint32_t val)
{
	sys_write32(val, base + off);
}

static inline uint32_t mdio_readreg(uintptr_t base, uint32_t off)
{
	return sys_read32(base + off);
}

/* Poll USERACCESS0 until GO clears. Caller must hold the bus mutex. */
static int cpsw_mdio_wait_go(uintptr_t base)
{
	uint16_t retries = CPSW_MDIO_MAX_RETRIES;

	while (retries--) {
		if (!(mdio_readreg(base, MDIO_USERACCESS0) & MDIO_UA_GO)) {
			return 0;
		}
		k_busy_wait(CPSW_MDIO_POLL_INTERVAL_US);
	}

	LOG_ERR("MDIO timeout: GO never cleared (UA=0x%08x)",
		mdio_readreg(base, MDIO_USERACCESS0));
	return -ETIMEDOUT;
}

static int cpsw_mdio_hw_read(uintptr_t base, uint8_t prtad, uint8_t regad,
			     uint16_t *data)
{
	uint32_t ua;
	int ret;

	ret = cpsw_mdio_wait_go(base);
	if (ret) {
		return ret;
	}

	mdio_writereg(base, MDIO_USERACCESS0,
		      MDIO_UA_GO |
		      ((uint32_t)regad << MDIO_UA_REGADR_SHIFT) |
		      ((uint32_t)prtad << MDIO_UA_PHYADR_SHIFT));

	ret = cpsw_mdio_wait_go(base);
	if (ret) {
		return ret;
	}

	ua = mdio_readreg(base, MDIO_USERACCESS0);
	LOG_DBG("MDIO hw_read prtad=%u regad=0x%02x UA=0x%08x", prtad, regad, ua);

	if (ua & MDIO_UA_ACK) {
		*data = (uint16_t)(ua & MDIO_UA_DATA_MASK);
		return 0;
	}

	LOG_ERR("MDIO read: no ACK (prtad=%u regad=0x%02x UA=0x%08x)",
		prtad, regad, ua);
	return -EIO;
}

static int cpsw_mdio_hw_write(uintptr_t base, uint8_t prtad, uint8_t regad,
			       uint16_t data)
{
	int ret;

	ret = cpsw_mdio_wait_go(base);
	if (ret) {
		return ret;
	}

	mdio_writereg(base, MDIO_USERACCESS0,
		      MDIO_UA_GO | MDIO_UA_WRITE |
		      ((uint32_t)regad << MDIO_UA_REGADR_SHIFT) |
		      ((uint32_t)prtad << MDIO_UA_PHYADR_SHIFT) |
		      ((uint32_t)data & MDIO_UA_DATA_MASK));

	return cpsw_mdio_wait_go(base);
}

/**
 * @brief Clause-22 read with transparent extended-register support
 *
 * Addresses 0x00-0x1f use direct USERACCESS access
 * Addresses > 0x1f use the 4-step REGCR/ADDAR indirect sequence
 */
static int cpsw_mdio_read(const struct device *dev, uint8_t prtad,
			   uint8_t regad, uint16_t *data)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	struct cpsw_mdio_data *d = dev->data;
	int ret;

	if (prtad > 0x1fU) {
		return -EINVAL;
	}

	k_mutex_lock(&d->lock, K_FOREVER);

	if (regad <= 0x1fU) {
		ret = cpsw_mdio_hw_read(base, prtad, regad, data);
	} else {
		LOG_DBG("MDIO ext read: prtad=%u regad=0x%02x", prtad, regad);
		ret = cpsw_mdio_hw_write(base, prtad,
					 MDIO_REGCR_REG, MDIO_REGCR_ADDR);
		if (ret) {
			goto out;
		}
		ret = cpsw_mdio_hw_write(base, prtad,
					 MDIO_ADDAR_REG, (uint16_t)regad);
		if (ret) {
			goto out;
		}
		ret = cpsw_mdio_hw_write(base, prtad,
					 MDIO_REGCR_REG, MDIO_REGCR_DATA);
		if (ret) {
			goto out;
		}
		ret = cpsw_mdio_hw_read(base, prtad, MDIO_ADDAR_REG, data);
	}

out:
	k_mutex_unlock(&d->lock);
	return ret;
}

/**
 * @brief Clause-22 write with transparent extended-register support
 *
 * Addresses 0x00-0x1f use direct USERACCESS access
 * Addresses > 0x1f use the 4-step REGCR/ADDAR indirect sequence
 */
static int cpsw_mdio_write(const struct device *dev, uint8_t prtad,
			   uint8_t regad, uint16_t data)
{
	mm_reg_t base = DEVICE_MMIO_GET(dev);
	struct cpsw_mdio_data *d = dev->data;
	int ret;

	if (prtad > 0x1fU) {
		return -EINVAL;
	}

	k_mutex_lock(&d->lock, K_FOREVER);

	if (regad <= 0x1fU) {
		ret = cpsw_mdio_hw_write(base, prtad, regad, data);
	} else {
		LOG_DBG("MDIO ext write: prtad=%u regad=0x%02x data=0x%04x",
			prtad, regad, data);
		ret = cpsw_mdio_hw_write(base, prtad,
					 MDIO_REGCR_REG, MDIO_REGCR_ADDR);
		if (ret) {
			goto out;
		}
		ret = cpsw_mdio_hw_write(base, prtad,
					 MDIO_ADDAR_REG, (uint16_t)regad);
		if (ret) {
			goto out;
		}
		ret = cpsw_mdio_hw_write(base, prtad,
					 MDIO_REGCR_REG, MDIO_REGCR_DATA);
		if (ret) {
			goto out;
		}
		ret = cpsw_mdio_hw_write(base, prtad, MDIO_ADDAR_REG, data);
	}

out:
	k_mutex_unlock(&d->lock);
	return ret;
}

static DEVICE_API(mdio, cpsw_mdio_api) = {
	.read  = cpsw_mdio_read,
	.write = cpsw_mdio_write,
};

static int cpsw_mdio_init(const struct device *dev)
{
	const struct cpsw_mdio_config *const cfg = dev->config;
	struct cpsw_mdio_data *const d = dev->data;
	uint32_t ver, div, ctrl, fck_freq;
	mm_reg_t base;
	uint16_t retries;

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	base = DEVICE_MMIO_GET(dev);

	k_mutex_init(&d->lock);

	ver = mdio_readreg(base, MDIO_VERSION);
	LOG_INF("MDIO base=0x%lx VERSION=0x%08x", (unsigned long)base, ver);

	clock_control_get_rate(cfg->clk_dev, cfg->clk_subsys, &fck_freq);
	div = (fck_freq > 0U && cfg->bus_freq > 0U)
		? (fck_freq / cfg->bus_freq) - 1U
		: MDIO_CTL_DIV_DEF;
	div &= MDIO_CTL_DIV_MASK;

	mdio_writereg(base, MDIO_CONTROL, div | MDIO_CTL_ENABLE);

	ctrl = mdio_readreg(base, MDIO_CONTROL);
	if (!(ctrl & MDIO_CTL_ENABLE)) {
		LOG_ERR("MDIO CONTROL ENABLE bit did not stick (readback=0x%08x)",
			ctrl);
	}

	/* Poll until IDLE clears: state machine is running. */
	retries = CPSW_MDIO_MAX_RETRIES;
	while (retries-- > 0) {
		if (!(mdio_readreg(base, MDIO_CONTROL) & MDIO_CTL_IDLE)) {
			break;
		}
		k_busy_wait(CPSW_MDIO_POLL_INTERVAL_US);
	}

	if (mdio_readreg(base, MDIO_CONTROL) & MDIO_CTL_IDLE) {
		LOG_ERR("MDIO IDLE never cleared (CONTROL=0x%08x)",
			mdio_readreg(base, MDIO_CONTROL));
		return -ETIMEDOUT;
	}

	/* Wait for 1 millisecond to allow the MDIO scan logic to settle */
	k_msleep(1);

	LOG_DBG("MDIO ALIVE=0x%08x LINK=0x%08x",
		mdio_readreg(base, MDIO_ALIVE),
		mdio_readreg(base, MDIO_LINK));

	return 0;
}

#define TI_AM62L_CPSW_MDIO_INIT(n)							\
	static const struct cpsw_mdio_config cpsw_mdio_cfg_##n = {			\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),					\
		.bus_freq   = DT_INST_PROP(n, clock_frequency),				\
		.clk_dev    = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR_BY_NAME(n, fck)),	\
		.clk_subsys = (clock_control_subsys_t)					\
			      UINT_TO_POINTER(						\
				DT_INST_CLOCKS_CELL_BY_NAME(n, fck, name)),		\
	};										\
											\
	static struct cpsw_mdio_data cpsw_mdio_data_##n;				\
											\
	DEVICE_DT_INST_DEFINE(n, cpsw_mdio_init, NULL,					\
			      &cpsw_mdio_data_##n, &cpsw_mdio_cfg_##n,			\
			      POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,			\
			      &cpsw_mdio_api);

DT_INST_FOREACH_STATUS_OKAY(TI_AM62L_CPSW_MDIO_INIT)
