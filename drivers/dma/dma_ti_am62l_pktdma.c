/* TI AM62L PKTDMA (Packet DMA) driver
 *
 * Copyright (c) 2026 Texas Instruments Incorporated
 * Author: Siddharth Vadapalli <s-vadapalli@ti.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_am62l_pktdma

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_ti_am62l_pktdma.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(dma_ti_am62l_pktdma, CONFIG_DMA_LOG_LEVEL);

/* Channel realtime (chanrt) register region */
#define PKTDMA_CHANRT_CTL           0x0U
#define PKTDMA_CHANRT_CTL_EN            BIT(31)
#define PKTDMA_CHANRT_CTL_TDOWN         BIT(30)
#define PKTDMA_CHANRT_CTL_AUTOPAIR      BIT(23)
#define PKTDMA_CHANRT_CTL_PAIR_TIMEOUT  BIT(17)
#define PKTDMA_CHANRT_CTL_PAIR_COMPLETE BIT(16)
#define PKTDMA_CHANRT_TCFG          0x04U
#define PKTDMA_CHANRT_TCQ           0x18U

/* Channel Type set to 2 for Packet Oriented data transfers */
#define PKTDMA_CHAN_TYPE_PACKET     (2U << 16)
/* Bits 16 to 19 of CHANRT TCFG register correspond to Channel Type */
#define PKTDMA_CHAN_TYPE_MASK       GENMASK(19, 16)

/* Step size for successive channel register sets */
#define PKTDMA_CHANRT_STEP          0x1000U

/* Ring realtime (ringrt) register region */
#define RINGACC_RING_RT_DB      0x10U
#define RINGACC_RING_RT_OCC     0x18U

/* Step size for successive ring-pair realtime register sets (forward + reverse) */
#define RINGACC_RING_RT_STEP    0x2000U

/* Offset of reverse ring RT base from forward ring RT base */
#define RINGACC_REVERSE_OFS     0x1000U

/* Offset of ring interrupt register space offset from ring RT base */
#define RINGACC_RT_INT_CLR_OFS          0x008U
#define RINGACC_RT_INT_STATUS_SET_OFS   0x010U
#define RINGACC_RT_INT_STATUS_OFS       0x018U
#define RINGACC_RT_INT_OFS              0x140U
#define RINGACC_RT_INT_ENABLE_COMPLETE      BIT(0)
#define RINGACC_RT_INT_ENABLE_TR            BIT(2)
#define RINGACC_RT_INT_ENABLE_COMPLETE_TR \
	(RINGACC_RT_INT_ENABLE_COMPLETE | RINGACC_RT_INT_ENABLE_TR)

/* Ring cfg register region */
#define RINGACC_CFG_BA_LO       0x40U
#define RINGACC_CFG_BA_HI       0x44U
#define RINGACC_CFG_SIZE        0x48U

/*
 * BA_LO register layout:
 *   bits 31:0 - addr[31:0] of ring memory physical address
 */
#define RINGACC_CFG_BA_LO_ADDR_MASK GENMASK(31, 0)

/*
 * BA_HI register layout:
 *   bits  3:0  - addr[35:32] (upper 4 bits of ring memory physical address)
 *   bits 19:16 - ASEL (address-space select for RINGACC ring-memory access)
 */
#define RINGACC_CFG_BA_HI_ADDR_MASK   GENMASK(3, 0)
#define RINGACC_CFG_BA_HI_ASEL_SHIFT  16U

/*
 * Layout of the CFG_SIZE register:
 * bits 26:24 - Ring Element Size
 * bits 15:0 - Number of elements
 */
#define RINGACC_CFG_SIZE_VAL(esize, cnt)			\
	((((uint32_t)(esize) << 24) & GENMASK(26, 24)) |	\
	 ((uint32_t)(cnt) & GENMASK(15, 0)))

/* Ring element size of 8 Bytes to be programmed into the CFG_SIZE register */
#define RINGACC_SIZE_ELM_8BYTE  1U

/* PKTDMA flow RT register region */
#define PKTDMA_RX_FLOWRT_RFA      0x8U
#define PKTDMA_RFLOW_RFA_EINFO    BIT(30)
#define PKTDMA_RFLOW_RFA_PSINFO   BIT(29)

/* Maximum hardware channel index supported by this driver */
#define PKTDMA_MAX_CHANS        128U

/* Maximum number of retries for waiting until autopair completes */
#define PKTDMA_CHANRT_PAIR_RETRIES 5U
/* Busy wait duration before rechecking if autopair has completed */
#define PKTDMA_AUTOPAIR_BUSY_WAIT_US 100U

/* Maximum number of retries for waiting until channel teardown completes */
#define PKTDMA_CHANRT_TDOWN_RETRIES 5U
/* Busy wait duration before rechecking if channel teardown has completed */
#define PKTDMA_TDOWN_BUSY_WAIT_US 100U

/* MMIO Helpers */
#define DEV_CFG(dev)  ((const struct pktdma_config *)(dev)->config)
#define DEV_DATA(dev) ((struct pktdma_data *)(dev)->data)

/* PKTDMA register access helpers */
static inline uint32_t pktdma_read32(uintptr_t addr)
{
	__sync_synchronize();
	return sys_read32(addr);
}

static inline void pktdma_write32(uintptr_t addr, uint32_t val)
{
	sys_write32(val, addr);
	__sync_synchronize();
}

/**
 * @brief Per-channel realtime state maintained by the PKTDMA driver.
 */
struct pktdma_chan {
	uintptr_t  fwd_rt;	/* Virtual Address of forward ring RT registers */
	uintptr_t  rev_rt;	/* Virtual Address of reverse ring RT registers */
	uintptr_t  chanrt;	/* Virtual Address of channel realtime control registers */
	uint64_t  *fwd_mem;	/* caller-owned forward ring buffer */
	uint64_t  *rev_mem;	/* caller-owned reverse ring buffer */
	uint32_t   ring_cnt;	/* number of entries in each ring */
	uint32_t   fwd_widx;	/* next write index into fwd_mem */
	uint32_t   rev_ridx;	/* next read index into rev_mem */
	bool       is_rx;	/* true = RX channel, false = TX channel */
	bool       configured;	/* true when pktdma_start() succeeds */
	uint8_t    asel;	/* address-space select (0=non-coherent, 14/15=coherent) */
};

/*
 * Use the interrupt-map property in the device-tree to generate a map of
 * channel numbers to their interrupt numbers
 *
 *   DT_N_{path}_P_interrupt_map_MAP_ENTRY_{N}_CHILD_SPECIFIER_IDX_0  - channel id
 *   DT_N_{path}_P_interrupt_map_MAP_ENTRY_{N}_PARENT_SPECIFIER_IDX_0 - GIC type (0=SPI)
 *   DT_N_{path}_P_interrupt_map_MAP_ENTRY_{N}_PARENT_SPECIFIER_IDX_1 - GIC SPI/PPI number
 *   DT_N_{path}_P_interrupt_map_LEN                                   - number of entries
 *
 * ARM GIC INTID = gic-irq + 32 for SPI, + 16 for PPI.
 *
 * Although all PKTDMA Channel interrupts in the AM62L SoC are GIC_SPI,
 * for the sake of completeness, the handling for GIC_PPI is also included.
 */
#define GIC_SPI_INTID_BASE  32U
#define GIC_PPI_INTID_BASE  16U

#define _PKTDMA_IMAP_CHAN_INNER(node_id, N)				\
	node_id##_P_interrupt_map_MAP_ENTRY_##N##_CHILD_SPECIFIER_IDX_0
#define _PKTDMA_IMAP_TYPE_INNER(node_id, N)				\
	node_id##_P_interrupt_map_MAP_ENTRY_##N##_PARENT_SPECIFIER_IDX_0
#define _PKTDMA_IMAP_IRQ_INNER(node_id, N)				\
	node_id##_P_interrupt_map_MAP_ENTRY_##N##_PARENT_SPECIFIER_IDX_1

#define _PKTDMA_IMAP_CHAN(node_id, N) _PKTDMA_IMAP_CHAN_INNER(node_id, N)
#define _PKTDMA_IMAP_TYPE(node_id, N) _PKTDMA_IMAP_TYPE_INNER(node_id, N)
#define _PKTDMA_IMAP_IRQ(node_id, N)  _PKTDMA_IMAP_IRQ_INNER(node_id, N)

#define _PKTDMA_IRQMAP_ENTRY(i, node_id)					\
	{									\
		.chan_id = _PKTDMA_IMAP_CHAN(node_id, i),			\
		.irq = (unsigned int)(						\
			(_PKTDMA_IMAP_TYPE(node_id, i) == 0U)			\
			? _PKTDMA_IMAP_IRQ(node_id, i) + GIC_SPI_INTID_BASE	\
			: _PKTDMA_IMAP_IRQ(node_id, i) + GIC_PPI_INTID_BASE)	\
	}

struct pktdma_irq_entry {
	uint32_t chan_id;
	unsigned int irq;
};

/**
 * @brief PKTDMA configuration fetched from device-tree
 */
struct pktdma_config {
	DEVICE_MMIO_NAMED_ROM(gcfg);
	DEVICE_MMIO_NAMED_ROM(chanrt);
	DEVICE_MMIO_NAMED_ROM(ringrt);
	const struct pktdma_irq_entry *irq_table;
	size_t irq_table_len;
};

/**
 * @brief Driver data containing the realtime state of all PKTDMA channels
 */
struct pktdma_data {
	DEVICE_MMIO_NAMED_RAM(gcfg);
	DEVICE_MMIO_NAMED_RAM(chanrt);
	DEVICE_MMIO_NAMED_RAM(ringrt);
	struct pktdma_chan chans[PKTDMA_MAX_CHANS];
};

/**
 * @brief Configure a PKTDMA channel
 *
 * @param dev     PKTDMA device
 * @param chan_id PKTDMA channel number
 * @param cfg     Configuration data
 * @retval 0 on success, negative errno on error
 */
static int pktdma_cfg(const struct device *dev, uint32_t chan_id,
		      struct dma_config *cfg)
{
	struct pktdma_data *data = dev->data;
	struct ti_pktdma_chan_cfg *pcfg;
	struct pktdma_chan *ch;
	uintptr_t fwd_rt, chanrt;
	uint32_t tcfg;

	if (chan_id >= PKTDMA_MAX_CHANS) {
		LOG_ERR("chan_id %u out of range (max %u)", chan_id,
			PKTDMA_MAX_CHANS - 1U);
		return -EINVAL;
	}

	if (!cfg->user_data) {
		LOG_ERR("chan %u: configuration data (ti_pktdma_chan_cfg *) is NULL",
			chan_id);
		return -EINVAL;
	}

	pcfg = (struct ti_pktdma_chan_cfg *)cfg->user_data;
	ch   = &data->chans[chan_id];

	fwd_rt = DEVICE_MMIO_NAMED_GET(dev, ringrt) + (uintptr_t)chan_id * RINGACC_RING_RT_STEP;
	chanrt = DEVICE_MMIO_NAMED_GET(dev, chanrt) + (uintptr_t)chan_id * PKTDMA_CHANRT_STEP;

	/* Reset channel */
	pktdma_write32(chanrt + PKTDMA_CHANRT_CTL, 0U);

	/* Configure forward ring */
	pktdma_write32(fwd_rt + RINGACC_CFG_BA_LO,
		       (uint32_t)(pcfg->fwd_ring_mem & RINGACC_CFG_BA_LO_ADDR_MASK));
	pktdma_write32(fwd_rt + RINGACC_CFG_BA_HI,
		       ((uint32_t)((uint64_t)pcfg->fwd_ring_mem >> 32) &
			RINGACC_CFG_BA_HI_ADDR_MASK) |
		       ((uint32_t)pcfg->asel << RINGACC_CFG_BA_HI_ASEL_SHIFT));
	pktdma_write32(fwd_rt + RINGACC_CFG_SIZE,
		       RINGACC_CFG_SIZE_VAL(RINGACC_SIZE_ELM_8BYTE,
					    pcfg->ring_cnt));

	/* Configure reverse ring */
	pktdma_write32(fwd_rt + RINGACC_REVERSE_OFS + RINGACC_CFG_BA_LO,
		       (uint32_t)(pcfg->rev_ring_mem & RINGACC_CFG_BA_LO_ADDR_MASK));
	pktdma_write32(fwd_rt + RINGACC_REVERSE_OFS + RINGACC_CFG_BA_HI,
		       ((uint32_t)((uint64_t)pcfg->rev_ring_mem >> 32) &
			RINGACC_CFG_BA_HI_ADDR_MASK) |
		       ((uint32_t)pcfg->asel << RINGACC_CFG_BA_HI_ASEL_SHIFT));
	pktdma_write32(fwd_rt + RINGACC_REVERSE_OFS + RINGACC_CFG_SIZE,
		       RINGACC_CFG_SIZE_VAL(RINGACC_SIZE_ELM_8BYTE,
					    pcfg->ring_cnt));

	/* Configure channel type */
	tcfg = pktdma_read32(chanrt + PKTDMA_CHANRT_TCFG);
	tcfg = (tcfg & ~PKTDMA_CHAN_TYPE_MASK) | PKTDMA_CHAN_TYPE_PACKET;
	pktdma_write32(chanrt + PKTDMA_CHANRT_TCFG, tcfg);

	if (!pcfg->is_rx) {
		/* TX: set completion queue (TCQ = tflow_id = chan_id). */
		pktdma_write32(chanrt + PKTDMA_CHANRT_TCQ, chan_id);
	} else {
		/* RX: configure flow RT (EPIB + PSD present) */
		pktdma_write32(fwd_rt + PKTDMA_RX_FLOWRT_RFA,
			       PKTDMA_RFLOW_RFA_EINFO | PKTDMA_RFLOW_RFA_PSINFO);
	}

	/* Save ring pointers for push and pop operations */
	ch->fwd_rt   = fwd_rt;
	ch->rev_rt   = fwd_rt + RINGACC_REVERSE_OFS;
	ch->chanrt   = chanrt;
	ch->fwd_mem  = (uint64_t *)pcfg->fwd_ring_mem;
	ch->rev_mem  = (uint64_t *)pcfg->rev_ring_mem;
	ch->ring_cnt = pcfg->ring_cnt;
	ch->fwd_widx = 0U;
	ch->rev_ridx = 0U;
	ch->is_rx    = pcfg->is_rx;
	ch->asel     = pcfg->asel;
	ch->configured = false;

	LOG_DBG("PKTDMA: chan %u configured (fwd_rt=0x%08x cnt=%u %s ASEL=%u%s)",
		chan_id, (uint32_t)fwd_rt, pcfg->ring_cnt,
		pcfg->is_rx ? "RX" : "TX", ch->asel,
		AM62L_PKTDMA_IS_COHERENT(ch->asel) ? " [ACP coherent]" : "");
	LOG_DBG("  fwd BA=0x%08x SIZE=0x%08x",
		pktdma_read32(fwd_rt + RINGACC_CFG_BA_LO),
		pktdma_read32(fwd_rt + RINGACC_CFG_SIZE));
	LOG_DBG("  rev BA=0x%08x SIZE=0x%08x",
		pktdma_read32(fwd_rt + RINGACC_REVERSE_OFS + RINGACC_CFG_BA_LO),
		pktdma_read32(fwd_rt + RINGACC_REVERSE_OFS + RINGACC_CFG_SIZE));

	return 0;
}

/**
 * @brief Enable a PKTDMA channel
 *
 * @param dev     PKTDMA device
 * @param chan_id PKTDMA channel number
 * @retval 0 on success, -ETIMEDOUT if PAIR_COMPLETE is never set
 */
static int pktdma_start(const struct device *dev, uint32_t chan_id)
{
	struct pktdma_data *data = dev->data;
	struct pktdma_chan *ch;
	uint32_t ctl;
	uint8_t i;

	if (chan_id >= PKTDMA_MAX_CHANS) {
		return -EINVAL;
	}

	ch = &data->chans[chan_id];

	pktdma_write32(ch->chanrt + PKTDMA_CHANRT_CTL, 0U);
	pktdma_write32(ch->chanrt + PKTDMA_CHANRT_CTL,
		       PKTDMA_CHANRT_CTL_EN | PKTDMA_CHANRT_CTL_AUTOPAIR);

	for (i = 0; i < PKTDMA_CHANRT_PAIR_RETRIES; i++) {
		ctl = pktdma_read32(ch->chanrt + PKTDMA_CHANRT_CTL);
		if (ctl & PKTDMA_CHANRT_CTL_PAIR_TIMEOUT) {
			LOG_ERR("PKTDMA: chan %u AUTOPAIR timed out (ctl=0x%08x)",
				chan_id, ctl);
			return -ETIMEDOUT;
		}
		if (ctl & PKTDMA_CHANRT_CTL_PAIR_COMPLETE) {
			break;
		}
		k_busy_wait(PKTDMA_AUTOPAIR_BUSY_WAIT_US);
	}

	if (i > PKTDMA_CHANRT_PAIR_RETRIES) {
		LOG_ERR("PKTDMA: chan %u PAIR_COMPLETE never set", chan_id);
		return -ETIMEDOUT;
	}

	ch->configured = true;

	LOG_DBG("PKTDMA: chan %u enabled after %d iterations (ctl=0x%08x)",
		chan_id, i, pktdma_read32(ch->chanrt + PKTDMA_CHANRT_CTL));

	pktdma_write32(ch->fwd_rt + RINGACC_RT_INT_OFS,
		       RINGACC_RT_INT_ENABLE_COMPLETE_TR);

	return 0;
}

/**
 * @brief Teardown and disable a PKTDMA channel.
 *
 * @param dev     PKTDMA device
 * @param chan_id PKTDMA channel number
 * @retval 0 on success, errno on failure
 */
static int pktdma_stop(const struct device *dev, uint32_t chan_id)
{
	struct pktdma_data *data = dev->data;
	struct pktdma_chan *ch;
	uint32_t ctl;
	int i;

	if (chan_id >= PKTDMA_MAX_CHANS) {
		return -EINVAL;
	}

	ch = &data->chans[chan_id];

	if (!ch->configured) {
		return 0;
	}

	ctl = pktdma_read32(ch->chanrt + PKTDMA_CHANRT_CTL);
	pktdma_write32(ch->chanrt + PKTDMA_CHANRT_CTL,
		       ctl | PKTDMA_CHANRT_CTL_TDOWN);

	for (i = 0; i < PKTDMA_CHANRT_TDOWN_RETRIES; i++) {
		ctl = pktdma_read32(ch->chanrt + PKTDMA_CHANRT_CTL);
		if (!(ctl & PKTDMA_CHANRT_CTL_TDOWN)) {
			break;
		}
		k_busy_wait(PKTDMA_TDOWN_BUSY_WAIT_US);
	}

	if (i > PKTDMA_CHANRT_TDOWN_RETRIES) {
		LOG_ERR("PKTDMA: chan %u teardown completion timed out", chan_id);
	}

	pktdma_write32(ch->chanrt + PKTDMA_CHANRT_CTL, 0U);
	ch->configured = false;

	LOG_DBG("PKTDMA: chan %u stopped", chan_id);

	return 0;
}

/**
 * @brief Report channel busy state and direction
 *
 * @param dev     PKTDMA device
 * @param chan_id PKTDMA channel number
 * @param status  Channel status structure to be filled in
 * @retval 0 on success, negative errno on error
 */
static int pktdma_get_status(const struct device *dev, uint32_t chan_id,
			     struct dma_status *status)
{
	struct pktdma_data *data = dev->data;
	struct pktdma_chan *ch;

	if (chan_id >= PKTDMA_MAX_CHANS || !status) {
		return -EINVAL;
	}

	ch = &data->chans[chan_id];
	status->busy = ch->configured &&
		       (pktdma_read32(ch->fwd_rt + RINGACC_RING_RT_OCC) > 0U);
	status->dir  = ch->is_rx ? PERIPHERAL_TO_MEMORY : MEMORY_TO_PERIPHERAL;

	return 0;
}

/**
 * @brief Submit a descriptor to the channel's forward ring
 *
 * @param dev       PKTDMA device
 * @param chan_id   PKTDMA channel number
 * @param desc_phys Physical address of the descriptor to push
 * @retval 0 on success, errno on failure
 */
static int pktdma_ring_push(const struct device *dev, uint32_t chan_id,
			    uint64_t desc_phys)
{
	struct pktdma_data *data = dev->data;
	struct pktdma_chan *ch;

	if (chan_id >= PKTDMA_MAX_CHANS) {
		return -EINVAL;
	}

	ch = &data->chans[chan_id];

	ch->fwd_mem[ch->fwd_widx] = desc_phys |
				    ((uint64_t)ch->asel << AM62L_ADDRESS_ASEL_SHIFT);
	if (!AM62L_PKTDMA_IS_COHERENT(ch->asel)) {
		sys_cache_data_flush_range(&ch->fwd_mem[ch->fwd_widx],
					   sizeof(uint64_t));
	}
	ch->fwd_widx = (ch->fwd_widx + 1U) % ch->ring_cnt;

	pktdma_write32(ch->fwd_rt + RINGACC_RING_RT_DB, 1U);

	return 0;
}

/**
 * @brief Pop a completed descriptor from the channel's reverse ring
 *
 * @param dev       PKTDMA device
 * @param chan_id   PKTDMA channel number
 * @param desc_phys Pointer to the physical address of the popped descriptor
 * @retval 0 on success, -ENODATA if ring is empty, errno on failure
 */
static int pktdma_ring_pop(const struct device *dev, uint32_t chan_id,
			   uint64_t *desc_phys)
{
	struct pktdma_data *data = dev->data;
	struct pktdma_chan *ch;
	uint32_t occ;

	if (chan_id >= PKTDMA_MAX_CHANS || !desc_phys) {
		return -EINVAL;
	}

	ch = &data->chans[chan_id];

	occ = pktdma_read32(ch->rev_rt + RINGACC_RING_RT_OCC);
	if (occ == 0U) {
		return -ENODATA;
	}

	if (!AM62L_PKTDMA_IS_COHERENT(ch->asel)) {
		sys_cache_data_invd_range(&ch->rev_mem[ch->rev_ridx],
					  sizeof(uint64_t));
	}
	*desc_phys = ch->rev_mem[ch->rev_ridx] & ~AM62L_ADDRESS_ASEL_MASK;
	ch->rev_ridx = (ch->rev_ridx + 1U) % ch->ring_cnt;

	pktdma_write32(ch->rev_rt + RINGACC_RING_RT_DB, (uint32_t)(-1));

	return 0;
}

/**
 * @brief (Re)arm the ring completion interrupt
 *
 * @param dev     PKTDMA device
 * @param chan_id PKTDMA channel ID
 */
static void pktdma_ring_irq_arm(const struct device *dev, uint32_t chan_id)
{
	struct pktdma_data *data = dev->data;

	if (chan_id >= PKTDMA_MAX_CHANS) {
		return;
	}
	pktdma_write32(data->chans[chan_id].fwd_rt + RINGACC_RT_INT_OFS,
		       RINGACC_RT_INT_ENABLE_COMPLETE_TR);
}

/**
 * @brief Clear the ring completion interrupt
 *
 * @param dev     PKTDMA device
 * @param chan_id PKTDMA channel ID
 */
static void pktdma_ring_irq_clear(const struct device *dev, uint32_t chan_id)
{
	struct pktdma_data *data = dev->data;

	if (chan_id >= PKTDMA_MAX_CHANS) {
		return;
	}
	pktdma_write32(data->chans[chan_id].fwd_rt +
		       RINGACC_RT_INT_OFS + RINGACC_RT_INT_STATUS_OFS, 0xFFU);
}

/**
 * @brief Initialize PKTDMA regions
 *
 * @param dev PKTDMA device
 * @retval 0 always
 */
static int pktdma_init(const struct device *dev)
{
	DEVICE_MMIO_NAMED_MAP(dev, gcfg,   K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, chanrt, K_MEM_CACHE_NONE);
	DEVICE_MMIO_NAMED_MAP(dev, ringrt, K_MEM_CACHE_NONE);

	LOG_DBG("PKTDMA: gcfg=0x%08x chanrt=0x%08x ringrt=0x%08x",
		(uint32_t)DEVICE_MMIO_NAMED_GET(dev, gcfg),
		(uint32_t)DEVICE_MMIO_NAMED_GET(dev, chanrt),
		(uint32_t)DEVICE_MMIO_NAMED_GET(dev, ringrt));

	return 0;
}

/**
 * @brief Return the interrupt number for a PKTDMA channel's completion ring
 *
 * @param dev     PKTDMA device
 * @param chan_id PKTDMA channel ID
 * @retval Interrupt number (GIC INTID) on success, -EINVAL on failure
 */
static int pktdma_get_chan_irq(const struct device *dev, uint32_t chan_id)
{
	const struct pktdma_config *cfg = dev->config;

	for (size_t i = 0; i < cfg->irq_table_len; i++) {
		if (cfg->irq_table[i].chan_id == chan_id) {
			return (int)cfg->irq_table[i].irq;
		}
	}

	LOG_ERR("PKTDMA: no interrupt-map entry for chan %u", chan_id);
	return -EINVAL;
}

static DEVICE_API(ti_pktdma, pktdma_driver_api) = {
	.dma_api = {
		.config     = pktdma_cfg,
		.start      = pktdma_start,
		.stop       = pktdma_stop,
		.get_status = pktdma_get_status,
	},
	.ring_push      = pktdma_ring_push,
	.ring_pop       = pktdma_ring_pop,
	.get_chan_irq   = pktdma_get_chan_irq,
	.ring_irq_arm   = pktdma_ring_irq_arm,
	.ring_irq_clear = pktdma_ring_irq_clear,
};

#define PKTDMA_DEVICE_INIT(n)								\
	static struct pktdma_data pktdma_data_##n;					\
											\
	static const struct pktdma_irq_entry pktdma_irq_table_##n[] = {			\
		LISTIFY(DT_INST_PROP_LEN(n, interrupt_map),				\
			_PKTDMA_IRQMAP_ENTRY, (,), DT_DRV_INST(n))			\
	};										\
											\
	static const struct pktdma_config pktdma_cfg_##n = {				\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(gcfg,   DT_DRV_INST(n)),		\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(chanrt, DT_DRV_INST(n)),		\
		DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(ringrt, DT_DRV_INST(n)),		\
		.irq_table     = pktdma_irq_table_##n,					\
		.irq_table_len = ARRAY_SIZE(pktdma_irq_table_##n),			\
	};										\
	DEVICE_DT_INST_DEFINE(n, pktdma_init, NULL,					\
			      &pktdma_data_##n, &pktdma_cfg_##n,			\
			      POST_KERNEL, CONFIG_DMA_INIT_PRIORITY,			\
			      &pktdma_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PKTDMA_DEVICE_INIT)
