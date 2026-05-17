/* TI AM62L CPSW3G Ethernet driver
 *
 * Copyright (c) 2026 Texas Instruments Incorporated
 * Author: Siddharth Vadapalli <s-vadapalli@ti.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_am62l_cpsw

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/devicetree/nvmem.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/phy.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(eth_ti_am62l_cpsw, CONFIG_ETHERNET_LOG_LEVEL);

#include "eth_ti_am62l_cpsw.h"

/* Get the CPSW PKTDMA Channel IDs from device-tree */
#define CPSW_TX_CHAN_ID DT_INST_DMAS_CELL_BY_NAME(0, tx0, channel_id)
#define CPSW_RX_CHAN_ID DT_INST_DMAS_CELL_BY_NAME(0, rx, channel_id)

/* Get the ASEL values for the TX and RX PKTDMA channels from DT */
#define CPSW_TX_ASEL  DT_INST_DMAS_CELL_BY_NAME(0, tx0, asel)
#define CPSW_RX_ASEL  DT_INST_DMAS_CELL_BY_NAME(0, rx, asel)

static const uint32_t cpsw_port_bases[CPSW_NUM_MAC_PORTS] = {
	CPSW_PORT1_BASE,
	CPSW_PORT2_BASE,
};

static const uint32_t cpsw_ale_portctls[CPSW_NUM_MAC_PORTS] = {
	ALE_PORTCTL1,
	ALE_PORTCTL2,
};

/**
 * @brief Per-MAC-port configuration resolved from Device Tree at build time
 *
 * @param phy_dev       PHY device for this MAC port
 * @param phy_conn_type PHY connection type resolved at compile time from DT
 * @param mac_cfg       MAC address configuration for the port
 * @param gmii_sel_addr Physical address of this port's GMII_SEL MMR register
 * @param gmii_sel_size Size of the GMII_SEL MMR region in bytes (from DT reg)
 * @param port_num	MAC Port number (1-based)
 * @param mac_only      MAC Only mode, port does not participate in L2 switching
 */
struct cpsw_port_dt_config {
	const struct device *phy_dev;
	enum cpsw_phy_conn_type phy_conn_type;
	struct net_eth_mac_config mac_cfg;
	uintptr_t gmii_sel_addr;
	size_t gmii_sel_size;
	uint8_t port_num;
	bool mac_only;
};

/**
 * @brief CPSW NUSS Device configuration from Device Tree
 *
 * @param DEVICE_MMIO_ROM  physical address and size of the CPSW NU Subsystem
 *                         register region.  Must be the first member.
 * @param port   Per-MAC-port configuration, indexed by (port_num - 1)
 */
struct cpsw_dt_config {
	DEVICE_MMIO_ROM;
	struct cpsw_port_dt_config port[CPSW_NUM_MAC_PORTS];
};

/**
 * @brief Get 0-based port index for the given network interface
 *
 * @param priv  Driver private data
 * @param iface Network interface to look up
 * @retval port index on success, negative errno if not found
 */
static inline int cpsw_port_idx(const struct cpsw_priv *priv,
				const struct net_if *iface)
{
	for (int i = 0; i < CPSW_NUM_MAC_PORTS; i++) {
		if (priv->iface[i] == iface) {
			return i;
		}
	}
	return -EINVAL;
}


/**
 * @brief Prepare a TX CPPI5 host descriptor
 *
 * @param tx_desc  Pointer to the descriptor to fill
 * @param buf_phys Physical address of the packet data buffer
 * @param pkt_len  Length of the packet in bytes
 * @param port_id  CPSW MAC port index to transmit the packet on
 * @param asel     Address Select value for IO Coherency
 */
static void cppi5_fill_tx_desc(struct cppi5_host_desc *tx_desc,
			       uintptr_t buf_phys, uint32_t pkt_len,
			       uint8_t port_id, uint8_t asel)
{
	memset(tx_desc, 0, CPPI5_DESC_SIZE);

	/* Word 0: descriptor type, EPIB present, PSD=16 bytes, packet length */
	tx_desc->pkt_info0 = CPPI5_INFO0_HDESC_TYPE_HOST |
			     CPPI5_INFO0_HDESC_EPIB_PRESENT |
			     ((CPPI5_PSD_SIZE_CPSW >> 2) <<
			      CPPI5_INFO0_HDESC_PSFLGS_SHIFT) |
			     (pkt_len & CPPI5_INFO0_HDESC_PKTLEN_MASK);

	/* Word 1: transmit flow index */
	tx_desc->pkt_info1 = CPPI5_INFO1_HDESC_FLOW_ID;

	/* Word 2: packet type, return queue */
	tx_desc->pkt_info2 = (CPPI5_INFO2_HDESC_PKTTYPE_CPSW <<
			      CPPI5_INFO2_HDESC_PKTTYPE_SHIFT) |
			     (CPSW_TX_CHAN_ID & CPPI5_INFO2_HDESC_RETQ_MASK);

	/* Word 3: destination tag = MAC port index. */
	tx_desc->src_dst_tag = (port_id & CPPI5_TAG_DSTTAG_MASK);

	/* Words 6-7: buffer pointer with ASEL added to bits 48-51 */
	tx_desc->buf_ptr = (uint64_t)buf_phys |
			   ((uint64_t)asel << AM62L_ADDRESS_ASEL_SHIFT);

	/* Word 8: buffer length = packet length. */
	tx_desc->buf_len = pkt_len;

	/* sw_data[0-7]: original buffer pointer (set lower 32 bits, higher bits = 0) */
	tx_desc->sw_data[0] = (uint8_t)(buf_phys & 0xffU);
	tx_desc->sw_data[1] = (uint8_t)((buf_phys >>  8) & 0xffU);
	tx_desc->sw_data[2] = (uint8_t)((buf_phys >> 16) & 0xffU);
	tx_desc->sw_data[3] = (uint8_t)((buf_phys >> 24) & 0xffU);
}

/**
 * @brief Prepare a CPPI5 buffer descriptor for TX Scatter Gather
 *
 * @param desc          Pointer to the descriptor to fill
 * @param buf_phys      Physical address of the fragment's data buffer
 * @param buf_len       Length of the fragment in bytes
 * @param next_desc_phys Physical address of the next descriptor in the chain
 *                      (0 for the final fragment)
 * @param asel          Address Select value for IO Coherency
 */
static void cppi5_fill_buf_desc(struct cppi5_host_desc *desc,
				uintptr_t buf_phys, uint32_t buf_len,
				uint64_t next_desc_phys, uint8_t asel)
{
	memset(desc, 0, CPPI5_DESC_SIZE);

	/* Word 0: descriptor type + buffer length */
	desc->pkt_info0  = CPPI5_INFO0_HDESC_TYPE_HOST |
			   (buf_len & CPPI5_INFO0_HDESC_PKTLEN_MASK);

	/* Words 4-5: link to the next descriptor in the chain (or null) */
	desc->next_desc  = next_desc_phys;

	/* Words 6-7: buffer pointer with ASEL added to bits 48-51 */
	desc->buf_ptr    = (uint64_t)buf_phys |
			   ((uint64_t)asel << AM62L_ADDRESS_ASEL_SHIFT);

	/* Word 8: buffer length */
	desc->buf_len    = buf_len;
}

/**
 * @brief Prepare an RX CPPI5 host descriptor for the free-descriptor ring
 *
 * @param rx_desc  Pointer to the descriptor to fill
 * @param buf_phys Physical address of the receive buffer
 * @param buf_size Size of the receive buffer in bytes
 * @param asel     Address Select value for IO Coherency
 */
static void cppi5_fill_rx_desc(struct cppi5_host_desc *rx_desc,
				uintptr_t buf_phys, uint32_t buf_size,
				uint8_t asel)
{
	memset(rx_desc, 0, CPPI5_DESC_SIZE);

	/* Word 0: descriptor type, buffer size (overwritten by DMA on completion) */
	rx_desc->pkt_info0 = CPPI5_INFO0_HDESC_TYPE_HOST |
			     (buf_size & CPPI5_INFO0_HDESC_PKTLEN_MASK);

	/* Words 6-7: receive buffer pointer with ASEL in bits 48-51 */
	rx_desc->buf_ptr = (uint64_t)buf_phys |
			   ((uint64_t)asel << AM62L_ADDRESS_ASEL_SHIFT);

	/* Word 8: buffer length */
	rx_desc->buf_len = buf_size;
}

/**
 * @brief Reset a CPSW MAC port
 *
 * @param priv         Driver private data
 * @param port_base_ofs Offset of the MAC Port base register region
 * @retval 0 on success, -ETIMEDOUT on reset timeout
 */
static int cpsw_macsl_reset(struct cpsw_priv *priv, uint32_t port_base_ofs)
{
	uint32_t macsl_base = port_base_ofs + CPSW_MACSL_OFS;

	cpsw_write(priv, macsl_base + CPSW_MACSL_RESET_REG, MACSL_RESET_BIT);

	for (int i = 0; i < CPSW_MAC_RESET_MAX_RETRIES; i++) {
		if (!(cpsw_read(priv, macsl_base + CPSW_MACSL_RESET_REG) &
		      MACSL_RESET_BIT)) {
			return 0;
		}
		k_busy_wait(CPSW_POLL_INTERVAL_US);
	}

	LOG_ERR("MAC Port reset timeout");
	return -ETIMEDOUT;
}

/**
 * @brief Drain stale completions left in the ring by another entity that may have
 *        used the ring earlier.
 *
 * @param priv Driver private data
 * @param chan_id Channel number whose completion ring needs to be drained
 */
static void cpsw_drain_stale_completions(struct cpsw_priv *priv,
					 uint32_t chan_id)
{
	uint64_t desc;
	int n = 0;

	while (ti_pktdma_ring_pop(priv->pktdma_dev, chan_id, &desc) == 0) {
		n++;
	}
	if (n > 0) {
		LOG_DBG("chan %u: drained %d stale completion(s)", chan_id, n);
	}
}

/**
 * @brief Configure CPSW's PKTDMA channels for TX and RX
 *
 * @param priv Driver private data
 * @retval 0 on success, negative errno on failure
 */
static int cpsw_pktdma_setup(struct cpsw_priv *priv)
{
	struct cpsw_pktdma *dma = &priv->dma;
	struct net_buf *frag;
	uintptr_t buf_phys;
	int ret;

	/*
	 * Initialize interrupt-driven completion semaphores:
	 * For TX, all TX Descriptors are initially unused and therefore free.
	 * For RX, all RX Descriptors have been submitted to DMA and we do not
	 * have any of them available (0).
	 */
	k_sem_init(&dma->tx_free_sem, CPSW_TX_DESC_NUM, CPSW_TX_DESC_NUM);
	k_sem_init(&dma->tx_compl_sem, 0, CPSW_TX_DESC_NUM);
	k_sem_init(&dma->rx_avail_sem, 0, CPSW_RX_DESC_NUM);
	struct ti_pktdma_chan_cfg tx_chan_cfg = {
		.fwd_ring_mem = (uintptr_t)dma->tx_ring_mem,
		/* Completions are written back to the same ring */
		.rev_ring_mem = (uintptr_t)dma->tx_ring_mem,
		.ring_cnt = CPSW_TX_DESC_NUM,
		.is_rx = false,
		.asel  = CPSW_TX_ASEL,
	};
	struct dma_config tx_dma_cfg = {
		.channel_direction = MEMORY_TO_PERIPHERAL,
		.user_data = &tx_chan_cfg,
	};
	struct ti_pktdma_chan_cfg rx_chan_cfg = {
		.fwd_ring_mem = (uintptr_t)dma->rx_ring_mem,
		.rev_ring_mem = (uintptr_t)dma->rx_ring_mem,
		.ring_cnt = CPSW_RX_DESC_NUM,
		.is_rx = true,
		.asel  = CPSW_RX_ASEL,
	};
	struct dma_config rx_dma_cfg = {
		.channel_direction = PERIPHERAL_TO_MEMORY,
		.user_data = &rx_chan_cfg,
	};

	ret = dma_config(priv->pktdma_dev, CPSW_TX_CHAN_ID, &tx_dma_cfg);
	if (ret < 0) {
		LOG_ERR("TX dma_config failed: %d", ret);
		return ret;
	}

	ret = dma_start(priv->pktdma_dev, CPSW_TX_CHAN_ID);
	if (ret < 0) {
		LOG_ERR("TX dma_start failed: %d", ret);
		return ret;
	}

	cpsw_drain_stale_completions(priv, CPSW_TX_CHAN_ID);

	ret = dma_config(priv->pktdma_dev, CPSW_RX_CHAN_ID, &rx_dma_cfg);
	if (ret < 0) {
		LOG_ERR("RX dma_config failed: %d", ret);
		return ret;
	}

	ret = dma_start(priv->pktdma_dev, CPSW_RX_CHAN_ID);
	if (ret < 0) {
		LOG_ERR("RX dma_start failed: %d", ret);
		return ret;
	}

	cpsw_drain_stale_completions(priv, CPSW_RX_CHAN_ID);

	/*
	 * Push all RX descriptors to the RX Free Descriptor ring to allow
	 * PKTDMA to send the packets received from CPSW to Software
	 */
	for (int i = 0; i < CPSW_RX_DESC_NUM; i++) {
		frag = net_pkt_get_reserve_rx_data(CPSW_RX_BUF_SIZE, K_NO_WAIT);

		if (frag == NULL) {
			LOG_ERR("RX: failed to alloc frag[%d]", i);
			for (int j = 0; j < i; j++) {
				net_pkt_frag_unref(dma->rx_frags[j]);
				dma->rx_frags[j] = NULL;
			}
			return -ENOMEM;
		}

		dma->rx_frags[i] = frag;
		buf_phys = (uintptr_t)frag->data;

		if (!AM62L_PKTDMA_IS_COHERENT(CPSW_RX_ASEL)) {
			sys_cache_data_invd_range(frag->data, CPSW_RX_BUF_SIZE);
		}
		cppi5_fill_rx_desc(&dma->rx_descs[i], buf_phys,
				   CPSW_RX_BUF_SIZE, CPSW_RX_ASEL);
		if (!AM62L_PKTDMA_IS_COHERENT(CPSW_RX_ASEL)) {
			sys_cache_data_flush_range(&dma->rx_descs[i], CPPI5_DESC_SIZE);
		}

		ret = ti_pktdma_ring_push(priv->pktdma_dev, CPSW_RX_CHAN_ID,
					  (uint64_t)(uintptr_t)&dma->rx_descs[i]);
		if (ret < 0) {
			LOG_ERR("RX ring_push[%d] failed: %d", i, ret);
			for (int j = 0; j <= i; j++) {
				net_pkt_frag_unref(dma->rx_frags[j]);
				dma->rx_frags[j] = NULL;
			}
			return ret;
		}
	}

	dma->initialized = true;
	LOG_DBG("PKTDMA: TX chan %u and RX chan %u configured",
		CPSW_TX_CHAN_ID, CPSW_RX_CHAN_ID);

	return 0;
}

/**
 * @brief PHY link state-change callback, shared by all MAC ports
 *
 * @param phy_dev   PHY device that generated the callback (unused)
 * @param state     Current link state (speed, duplex, is_up)
 * @param user_data Pointer to the cpsw_port_ctx for the corresponding MAC port
 */
static void cpsw_phy_link_cb(const struct device *phy_dev,
			      struct phy_link_state *state,
			      void *user_data)
{
	struct cpsw_port_ctx *ctx = user_data;
	const struct device *dev = ctx->dev;
	struct cpsw_priv *priv = dev->data;
	uint32_t port_num = ctx->port_num;
	uint32_t port_base = cpsw_port_bases[port_num - 1];
	uint32_t macsl_ctl;
	int ret;

	ARG_UNUSED(phy_dev);

	if (!state->is_up) {
		LOG_INF("MAC port %u: link down", port_num);
		net_eth_carrier_off(priv->iface[port_num - 1]);
		return;
	}

	ret = cpsw_macsl_reset(priv, CPSW_NU_BASE + port_base);
	if (ret < 0) {
		LOG_ERR("MAC port %u reset failed: %d", port_num, ret);
	}

	macsl_ctl = MACSL_CTL_GMII_EN;

	if (PHY_LINK_IS_FULL_DUPLEX(state->speed)) {
		macsl_ctl |= MACSL_CTL_FULL_DUPLEX;
	}

	if (PHY_LINK_IS_SPEED_1000M(state->speed)) {
		macsl_ctl |= MACSL_CTL_GIG;
	}

	cpsw_write(priv,
		   CPSW_NU_BASE + port_base + CPSW_MACSL_OFS +
		   CPSW_MACSL_CTL_REG,
		   macsl_ctl);

	LOG_INF("MAC port %u: link up %s Mbps %s-duplex", port_num,
		PHY_LINK_IS_SPEED_1000M(state->speed) ? "1000" :
		PHY_LINK_IS_SPEED_100M(state->speed) ? "100" : "10",
		PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");

	net_eth_carrier_on(priv->iface[port_num - 1]);
}

/**
 * @brief Initialize CPSW
 *
 * @param priv Driver private data
 * @retval 0 on success, negative errno on failure
 */
static int cpsw_hw_init(struct cpsw_priv *priv)
{
	uint32_t stat_en = CPSW_NUSS_STAT_P0_EN;
	int ret, i;

	/* Initialize PKTDMA channels */
	ret = cpsw_pktdma_setup(priv);
	if (ret < 0) {
		LOG_ERR("PKTDMA setup failed: %d", ret);
		return ret;
	}

	/* Enable host port */
	cpsw_write(priv, CPSW_NU_BASE + CPSW_NUSS_CONTROL,
		   CPSW_NUSS_CTL_P0_ENABLE |
		   CPSW_NUSS_CTL_P0_TX_CRC_REMOVE |
		   CPSW_NUSS_CTL_P0_RX_PAD);

	/* Disable priority elevation. */
	cpsw_write(priv, CPSW_NU_BASE + CPSW_NUSS_PTYPE, 0U);

	/* Enable statistics */
	for (i = 0; i < CPSW_NUM_MAC_PORTS; i++) {
		stat_en |= BIT(i+1);
	}
	cpsw_write(priv, CPSW_NU_BASE + CPSW_NUSS_STAT_PORT_EN, stat_en);

	/* Set maximum receive frame length for the host port */
	cpsw_write(priv, CPSW_NU_BASE + CPSW_PORT0_BASE + CPSW_PN_RX_MAXLEN,
		   CPSW_MAX_PACKET_SIZE);

	/* Set host port RX flow ID base */
	cpsw_write(priv, CPSW_NU_BASE + CPSW_PORT0_BASE + CPSW_P0_FLOW_ID_REG,
		   CPSW_RX_CHAN_ID);

	/*
	 * Set max receive frame length, source MAC address, ALE port state
	 * for the MAC Ports.
	 */
	for (i = 0; i < CPSW_NUM_MAC_PORTS; i++) {
		uint32_t port_base = cpsw_port_bases[i];
		uint32_t sa_l, sa_h;
		uint8_t *mac_addr = priv->mac_addr[i];

		sa_l = ((uint32_t)mac_addr[5] <<  0) | ((uint32_t)mac_addr[4] <<  8) |
			((uint32_t)mac_addr[3] << 16) | ((uint32_t)mac_addr[2] << 24);
		sa_h = ((uint32_t)mac_addr[1] <<  0) | ((uint32_t)mac_addr[0] <<  8);

		/* RMW isn't required since other fields aren't present in registers */
		cpsw_write(priv,
			CPSW_NU_BASE + port_base + CPSW_PN_RX_MAXLEN,
			CPSW_MAX_PACKET_SIZE);
		cpsw_write(priv, CPSW_NU_BASE + port_base + CPSW_PN_SA_L,
			sa_l);
		cpsw_write(priv, CPSW_NU_BASE + port_base + CPSW_PN_SA_H,
			sa_h);
	}

	/* Clear ALE table and enable ALE in bypass mode */
	cpsw_write(priv, CPSW_NU_BASE + CPSW_ALE_BASE + ALE_CONTROL,
		   ALE_CONTROL_ENABLE | ALE_CONTROL_CLEAR_TBL |
		   ALE_CONTROL_BYPASS);

	/* Set Host Port ALE state to Forwarding */
	cpsw_write(priv, CPSW_NU_BASE + CPSW_ALE_BASE + ALE_PORTCTL0,
		   ALE_PORTCTL_FORWARD);

	/* Enable the default ALE thread mapping */
	cpsw_write(priv, CPSW_NU_BASE + CPSW_ALE_BASE + ALE_THREADMAPDEF,
		   ALE_DEFTHREAD_EN);

	return 0;
}

/**
 * @brief TX completion interrupt handler
 *
 * @param arg CPSW device pointer
 */
static void cpsw_tx_irq_handler(const void *arg)
{
	const struct device *dev = arg;
	struct cpsw_priv *priv = dev->data;

	ti_pktdma_ring_irq_clear(priv->pktdma_dev, CPSW_TX_CHAN_ID);

	irq_disable(priv->tx_irq);
	k_sem_give(&priv->dma.tx_compl_sem);
}

/**
 * @brief Process a single TX completion
 *
 * @param dma Pointer to the PKTDMA state
 * @param compl_phys Pointer to the physical address of the completion descriptor
 */
static inline void cpsw_tx_process_one_compl(struct cpsw_pktdma *dma,
					     const uint64_t *compl_phys)
{
	uint32_t idx = (uint32_t)(((uintptr_t)*compl_phys -
				   (uintptr_t)dma->tx_descs) /
				  sizeof(dma->tx_descs[0]));
	uint8_t num_descs;

	if (idx >= CPSW_TX_DESC_NUM) {
		return;
	}
	if (dma->tx_pkts[idx] != NULL) {
		net_pkt_unref(dma->tx_pkts[idx]);
		dma->tx_pkts[idx] = NULL;
	}

	num_descs = dma->tx_n_descs[idx];

	while (num_descs--) {
		k_sem_give(&dma->tx_free_sem);
	}
}

/**
 * @brief Process a batch of TX completions
 *
 * @param priv Driver private data
 */
static inline void cpsw_tx_handle_cmpl_batch(struct cpsw_priv *priv)
{
	struct cpsw_pktdma *dma = &priv->dma;
	uint64_t compl_phys;
	int count;

	do {
		count = 0;

		while (count < CPSW_TX_BUDGET &&
		       ti_pktdma_ring_pop(priv->pktdma_dev, CPSW_TX_CHAN_ID,
					  &compl_phys) == 0) {
			cpsw_tx_process_one_compl(dma, &compl_phys);
			count++;
		}

		if (count == CPSW_TX_BUDGET) {
			k_yield();
		}

	} while (count == CPSW_TX_BUDGET && priv->ports_started != 0U);
}

/**
 * @brief TX completion thread
 *
 * @param arg1 Driver private data
 * @param arg2 Unused
 * @param arg3 Unused
 */
static void cpsw_tx_cmpl_thread(void *arg1, void *arg2, void *arg3)
{
	struct cpsw_priv *priv = (struct cpsw_priv *)arg1;
	struct cpsw_pktdma *dma = &priv->dma;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_DBG("TX completion thread started");

	while (priv->ports_started != 0U) {
		k_sem_take(&dma->tx_compl_sem, K_FOREVER);

		/* Handle completions in a batch until budget limit */
		cpsw_tx_handle_cmpl_batch(priv);

		if (priv->ports_started != 0U) {
			ti_pktdma_ring_irq_arm(priv->pktdma_dev, CPSW_TX_CHAN_ID);
			irq_enable(priv->tx_irq);
		}
	}

	LOG_DBG("TX completion thread stopped");
}

/**
 * @brief RX completion interrupt handler
 *
 * @param arg CPSW device pointer
 */
static void cpsw_rx_irq_handler(const void *arg)
{
	const struct device *dev = arg;
	struct cpsw_priv *priv = dev->data;

	ti_pktdma_ring_irq_clear(priv->pktdma_dev, CPSW_RX_CHAN_ID);

	irq_disable(priv->rx_irq);
	k_sem_give(&priv->dma.rx_avail_sem);
}

/**
 * @brief Build a net_pkt from a completed RX fragment
 *
 * @param rx_iface Network interface the packet is meant for
 * @param frag Packet fragment
 * @param pkt_len  Received packet length reported by the descriptor
 * @retval Non-NULL net_pkt with frag attached on success and NULL on error
 */
static inline struct net_pkt *cpsw_rx_build_pkt(struct net_if *rx_iface,
						 struct net_buf *frag,
						 uint32_t pkt_len)
{
	struct net_pkt *pkt;

	if (pkt_len == 0U || pkt_len > CPSW_MAX_PACKET_SIZE) {
		LOG_WRN("RX: invalid pkt_len=%u, dropping", pkt_len);
		return NULL;
	}
	if (frag == NULL) {
		return NULL;
	}
	if (!AM62L_PKTDMA_IS_COHERENT(CPSW_RX_ASEL)) {
		sys_cache_data_invd_range(frag->data, pkt_len);
	}
	pkt = net_pkt_rx_alloc_on_iface(rx_iface, K_NO_WAIT);
	if (pkt == NULL) {
		LOG_ERR("RX: failed to alloc net_pkt");
		return NULL;
	}
	frag->len = pkt_len;
	net_pkt_frag_add(pkt, frag);
	return pkt;
}

/**
 * @brief Refill an RX descriptor slot with a new fragment
 *
 * @param priv      Driver private data
 * @param rx_desc   Descriptor to refill
 * @param desc_phys Physical address of the descriptor
 * @param idx       Index of the descriptor slot in rx_frags[]
 * @param old_frag  Unconsumed fragment to reuse if reserving new frag fails, or NULL
 */
static inline void cpsw_rx_refill_desc(struct cpsw_priv *priv,
					struct cppi5_host_desc *rx_desc,
					const uint64_t *desc_phys,
					uint32_t idx,
					struct net_buf *old_frag)
{
	struct cpsw_pktdma *dma = &priv->dma;
	struct net_buf *new_frag;

	new_frag = net_pkt_get_reserve_rx_data(CPSW_RX_BUF_SIZE, K_NO_WAIT);
	if (new_frag == NULL) {
		new_frag = old_frag;
		old_frag = NULL;
	}

	if (new_frag != NULL) {
		if (!AM62L_PKTDMA_IS_COHERENT(CPSW_RX_ASEL)) {
			sys_cache_data_invd_range(new_frag->data, CPSW_RX_BUF_SIZE);
		}
		dma->rx_frags[idx] = new_frag;
		cppi5_fill_rx_desc(rx_desc, (uintptr_t)new_frag->data,
				   CPSW_RX_BUF_SIZE, CPSW_RX_ASEL);
		if (!AM62L_PKTDMA_IS_COHERENT(CPSW_RX_ASEL)) {
			sys_cache_data_flush_range(rx_desc, CPPI5_DESC_SIZE);
		}
		ti_pktdma_ring_push(priv->pktdma_dev, CPSW_RX_CHAN_ID,
				    *desc_phys);
	} else {
		LOG_ERR("RX: out of buffers, descriptor slot lost");
	}

	if (old_frag != NULL) {
		net_pkt_frag_unref(old_frag);
	}
}

/**
 * @brief Process a single RX packet completion
 *
 * @param priv      Driver private data
 * @param desc_phys Pointer to the physical address of the completed descriptor
 */
static inline void cpsw_rx_process_one_pkt(struct cpsw_priv *priv,
					   const uint64_t *desc_phys)
{
	struct cpsw_pktdma *dma = &priv->dma;
	struct cppi5_host_desc *rx_desc =
		(struct cppi5_host_desc *)(uintptr_t)*desc_phys;
	struct net_if *rx_iface;
	struct net_pkt *pkt;
	struct net_buf *frag;
	uint32_t src_port, pkt_len, idx;

	if (!AM62L_PKTDMA_IS_COHERENT(CPSW_RX_ASEL)) {
		sys_cache_data_invd_range(rx_desc, CPPI5_DESC_SIZE);
	}

	pkt_len  = rx_desc->pkt_info0 & CPPI5_INFO0_HDESC_PKTLEN_MASK;
	src_port = (rx_desc->src_dst_tag >> CPPI5_TAG_SRCTAG_SHIFT) &
		   CPPI5_SRCTAG_PORT_MASK;

	idx = (uint32_t)(rx_desc - dma->rx_descs);
	if (idx >= CPSW_RX_DESC_NUM) {
		LOG_ERR("RX: invalid descriptor idx=%u", idx);
		return;
	}

	if (src_port >= 1U && src_port <= CPSW_NUM_MAC_PORTS &&
	    priv->iface[src_port - 1] != NULL) {
		rx_iface = priv->iface[src_port - 1];
	} else {
		rx_iface = priv->iface[0];
		LOG_WRN("RX: unexpected src_port=%u", src_port);
	}

	frag = dma->rx_frags[idx];
	dma->rx_frags[idx] = NULL;

	pkt = cpsw_rx_build_pkt(rx_iface, frag, pkt_len);
	cpsw_rx_refill_desc(priv, rx_desc, desc_phys, idx,
			    (pkt != NULL) ? NULL : frag);

	if (pkt != NULL) {
		LOG_DBG("RX: %u bytes (port %u)", pkt_len, src_port);
		if (net_recv_data(rx_iface, pkt) < 0) {
			net_pkt_unref(pkt);
		}
	}
}

/**
 * @brief Process a batch of RX completions
 *
 * @param priv Driver private data
 */
static inline void cpsw_rx_handle_batch(struct cpsw_priv *priv)
{
	uint64_t desc_phys;
	int count;

	do {
		count = 0;

		while (count < CPSW_RX_BUDGET &&
		       ti_pktdma_ring_pop(priv->pktdma_dev, CPSW_RX_CHAN_ID,
					  &desc_phys) == 0) {
			cpsw_rx_process_one_pkt(priv, &desc_phys);
			count++;
		}

		if (count == CPSW_RX_BUDGET) {
			k_yield();
		}

	} while (count == CPSW_RX_BUDGET && priv->ports_started != 0U);
}

/**
 * @brief RX completion thread
 *
 * @param arg1 Driver private data
 * @param arg2 Unused
 * @param arg3 Unused
 */
static void cpsw_rx_thread(void *arg1, void *arg2, void *arg3)
{
	struct cpsw_priv *priv = (struct cpsw_priv *)arg1;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_DBG("RX thread started");

	while (priv->ports_started != 0U) {
		k_sem_take(&priv->dma.rx_avail_sem, K_FOREVER);

		cpsw_rx_handle_batch(priv);

		if (priv->ports_started != 0U) {
			ti_pktdma_ring_irq_arm(priv->pktdma_dev, CPSW_RX_CHAN_ID);
			irq_enable(priv->rx_irq);
		}
	}

	LOG_DBG("RX thread stopped");
}

/**
 * @brief Collect TX fragments for scatter-gather
 *
 * @param pkt     Packet to be transmitted
 * @param frags   Output array of fragment pointers
 * @param pkt_len Total packet length
 * @retval Number of fragments collected, or 0 to use bounce-buffer path
 */
static inline uint16_t cpsw_tx_collect_frags(struct net_pkt *pkt,
					      struct net_buf *frags[],
					      uint16_t pkt_len)
{
	struct net_buf *frag;
	uint16_t n_frags = 0;

	if (pkt_len < CPSW_MIN_PACKET_SIZE) {
		return 0;
	}

	for (frag = pkt->frags;
	     frag && n_frags < CPSW_TX_MAX_SG_FRAGS;
	     frag = frag->frags) {
		if (frag->len > 0) {
			frags[n_frags++] = frag;
		}
	}

	/* If non-empty fragments remain past the limit, use bounce buffer */
	for (; frag; frag = frag->frags) {
		if (frag->len > 0) {
			return 0;
		}
	}

	return n_frags;
}

/**
 * @brief Transmit a packet via scatter-gather
 *
 * @param priv     Driver private data
 * @param port_num MAC port number (1-based) for the CPPI5 destination tag
 * @param pkt      Packet to transmit
 * @param frags    Array of packet fragments
 * @param n_frags  Number of entries in frags[]
 * @param pkt_len  Total packet length in bytes
 * @retval 0 on success, negative errno on failure
 */
static int cpsw_tx_sg(struct cpsw_priv *priv, uint8_t port_num,
		      struct net_pkt *pkt, struct net_buf *frags[],
		      uint16_t n_frags, uint16_t pkt_len)
{
	struct cpsw_pktdma *dma = &priv->dma;
	struct cppi5_host_desc *head_desc, *desc;
	uint64_t next_phys;
	uint32_t head_idx, desc_idx, nxt;
	uint16_t i;
	int ret;

	for (i = 0; i < n_frags; i++) {
		if (k_sem_take(&dma->tx_free_sem,
			       K_MSEC(CPSW_TX_DESC_ACQUIRE_TIMEOUT_MS)) != 0) {
			for (uint16_t j = 0; j < i; j++) {
				k_sem_give(&dma->tx_free_sem);
			}
			LOG_ERR("TX SG: no free slots (need %u)", n_frags);
			return -ENOBUFS;
		}
	}

	if (k_mutex_lock(&priv->tx_lock,
			 K_MSEC(CPSW_TX_DESC_ACQUIRE_TIMEOUT_MS)) != 0) {
		for (i = 0; i < n_frags; i++) {
			k_sem_give(&dma->tx_free_sem);
		}
		LOG_ERR("TX SG: tx_lock timeout");
		return -ENOBUFS;
	}

	head_idx  = dma->tx_head % CPSW_TX_DESC_NUM;
	head_desc = &dma->tx_descs[head_idx];

	cppi5_fill_tx_desc(head_desc, (uintptr_t)frags[0]->data, frags[0]->len,
			   port_num, CPSW_TX_ASEL);
	head_desc->pkt_info0 =
		(head_desc->pkt_info0 & ~CPPI5_INFO0_HDESC_PKTLEN_MASK)
		| (pkt_len & CPPI5_INFO0_HDESC_PKTLEN_MASK);
	if (!AM62L_PKTDMA_IS_COHERENT(CPSW_TX_ASEL)) {
		sys_cache_data_flush_range(frags[0]->data, frags[0]->len);
	}

	for (i = 1; i < n_frags; i++) {
		desc_idx  = (head_idx + i) % CPSW_TX_DESC_NUM;
		desc      = &dma->tx_descs[desc_idx];
		next_phys = 0;
		if (i + 1 < n_frags) {
			nxt       = (head_idx + i + 1) % CPSW_TX_DESC_NUM;
			next_phys = (uint64_t)(uintptr_t)&dma->tx_descs[nxt] |
				    ((uint64_t)CPSW_TX_ASEL
				     << AM62L_ADDRESS_ASEL_SHIFT);
		}
		cppi5_fill_buf_desc(desc, (uintptr_t)frags[i]->data,
				    frags[i]->len, next_phys, CPSW_TX_ASEL);
		if (!AM62L_PKTDMA_IS_COHERENT(CPSW_TX_ASEL)) {
			sys_cache_data_flush_range(frags[i]->data,
						   frags[i]->len);
		}
	}

	if (n_frags > 1) {
		nxt = (head_idx + 1) % CPSW_TX_DESC_NUM;
		head_desc->next_desc =
			(uint64_t)(uintptr_t)&dma->tx_descs[nxt] |
			((uint64_t)CPSW_TX_ASEL << AM62L_ADDRESS_ASEL_SHIFT);
	}

	if (!AM62L_PKTDMA_IS_COHERENT(CPSW_TX_ASEL)) {
		for (i = 0; i < n_frags; i++) {
			sys_cache_data_flush_range(
				&dma->tx_descs[(head_idx + i) % CPSW_TX_DESC_NUM],
				CPPI5_DESC_SIZE);
		}
	}

	/* Hold a packet reference until all descriptors complete */
	net_pkt_ref(pkt);
	dma->tx_pkts[head_idx]    = pkt;
	dma->tx_n_descs[head_idx] = (uint8_t)n_frags;

	ret = ti_pktdma_ring_push(priv->pktdma_dev, CPSW_TX_CHAN_ID,
				  (uint64_t)(uintptr_t)head_desc);
	if (ret < 0) {
		LOG_ERR("TX SG: ring_push failed: %d", ret);
		net_pkt_unref(dma->tx_pkts[head_idx]);
		dma->tx_pkts[head_idx] = NULL;
		for (i = 0; i < n_frags; i++) {
			k_sem_give(&dma->tx_free_sem);
		}
	} else {
		LOG_DBG("TX SG: %u bytes in %u frag(s) queued", pkt_len, n_frags);
		dma->tx_head += n_frags;
	}

	k_mutex_unlock(&priv->tx_lock);
	return ret;
}

/**
 * @brief Transmit a packet via the bounce-buffer path
 *
 * @param priv     Driver private data
 * @param port_num MAC port number (1-based) for the CPPI5 destination tag
 * @param pkt      Packet to transmit
 * @param pkt_len  Total packet length in bytes
 * @retval 0 on success, negative errno on failure
 */
static int cpsw_tx_bounce(struct cpsw_priv *priv, uint8_t port_num,
			   struct net_pkt *pkt, uint16_t pkt_len)
{
	struct cpsw_pktdma *dma = &priv->dma;
	struct cppi5_host_desc *head_desc;
	uint32_t head_idx;
	uint8_t *buf;
	int ret;

	if (k_sem_take(&dma->tx_free_sem,
		       K_MSEC(CPSW_TX_DESC_ACQUIRE_TIMEOUT_MS)) != 0) {
		LOG_ERR("TX: no free descriptor slot");
		return -ENOBUFS;
	}

	if (k_mutex_lock(&priv->tx_lock,
			 K_MSEC(CPSW_TX_DESC_ACQUIRE_TIMEOUT_MS)) != 0) {
		k_sem_give(&dma->tx_free_sem);
		LOG_ERR("TX: tx_lock timeout");
		return -ENOBUFS;
	}

	head_idx = dma->tx_head % CPSW_TX_DESC_NUM;
	head_desc = &dma->tx_descs[head_idx];
	buf = dma->tx_bufs[head_idx];

	if (net_pkt_read(pkt, buf, pkt_len) < 0) {
		LOG_ERR("TX: net_pkt_read failed");
		k_sem_give(&dma->tx_free_sem);
		k_mutex_unlock(&priv->tx_lock);
		return -EIO;
	}

	if (pkt_len < CPSW_MIN_PACKET_SIZE) {
		memset(buf + pkt_len, 0, CPSW_MIN_PACKET_SIZE - pkt_len);
		pkt_len = CPSW_MIN_PACKET_SIZE;
	}

	if (!AM62L_PKTDMA_IS_COHERENT(CPSW_TX_ASEL)) {
		sys_cache_data_flush_range(buf, pkt_len);
	}

	cppi5_fill_tx_desc(head_desc, (uintptr_t)buf, pkt_len,
			   port_num, CPSW_TX_ASEL);
	if (!AM62L_PKTDMA_IS_COHERENT(CPSW_TX_ASEL)) {
		sys_cache_data_flush_range(head_desc, CPPI5_DESC_SIZE);
	}

	dma->tx_pkts[head_idx]    = NULL;
	dma->tx_n_descs[head_idx] = 1U;

	ret = ti_pktdma_ring_push(priv->pktdma_dev, CPSW_TX_CHAN_ID,
				  (uint64_t)(uintptr_t)head_desc);
	if (ret < 0) {
		LOG_ERR("TX: ring_push failed: %d", ret);
		k_sem_give(&dma->tx_free_sem);
	} else {
		LOG_DBG("TX: %u bytes queued (bounce)", pkt_len);
		dma->tx_head++;
	}

	k_mutex_unlock(&priv->tx_lock);
	return ret;
}

/**
 * @brief Transmit a packet via CPSW
 *
 * @param dev CPSW device pointer
 * @param pkt Packet to transmit
 * @retval 0 on success, negative errno on failure
 */
static int cpsw_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct cpsw_priv *priv = dev->data;
	struct net_buf *frags[CPSW_TX_MAX_SG_FRAGS];
	struct cpsw_pktdma *dma = &priv->dma;
	int i = cpsw_port_idx(priv, net_pkt_iface(pkt));
	uint16_t pkt_len, n_frags;
	uint8_t port_num;

	if (i < 0 || !(priv->ports_started & BIT(i)) || !dma->initialized) {
		return -ENETDOWN;
	}

	port_num = (uint8_t)(i + 1);
	pkt_len = net_pkt_get_len(pkt);
	if (pkt_len > CPSW_MAX_PACKET_SIZE) {
		LOG_ERR("TX: packet too large (%u)", pkt_len);
		return -EMSGSIZE;
	}

	n_frags = cpsw_tx_collect_frags(pkt, frags, pkt_len);
	if (n_frags > 0) {
		return cpsw_tx_sg(priv, port_num, pkt, frags, n_frags, pkt_len);
	}

	return cpsw_tx_bounce(priv, port_num, pkt, pkt_len);
}

/**
 * @brief Program the GMII_SEL MMR register for a MAC Port based on requested
 *        PHY Connection Type
 *
 * @param port_cfg      Per-port configuration containing phy_conn_type
 * @param gmii_sel_virt Virtual address of this port's GMII_SEL register
 */
static void cpsw_gmii_sel_set(const struct cpsw_port_dt_config *port_cfg,
			      mm_reg_t gmii_sel_virt)
{
	uint32_t reg, mode_val, rgmii_id = 0U;

	switch (port_cfg->phy_conn_type) {
	case CPSW_PHY_CONN_RMII:
		mode_val = GMII_SEL_MODE_RMII;
		break;
	case CPSW_PHY_CONN_RGMII_ID:
	case CPSW_PHY_CONN_RGMII_TXID:
		/*
		 * MAC Port has to add a delay for RGMII_ID and RGMII_TX_ID
		 * modes. A delay is added when 'RGMII_ID' field of the
		 * register is 'cleared', we have to set it to '0'.
		 */
		mode_val = GMII_SEL_MODE_RGMII;
		break;
	case CPSW_PHY_CONN_RGMII:
	case CPSW_PHY_CONN_RGMII_RXID:
		mode_val = GMII_SEL_MODE_RGMII;
		/* Do not add a delay */
		rgmii_id = GMII_SEL_RGMII_ID_BIT;
		break;
	default:
		LOG_WRN("Unsupported phy_conn_type: %d", port_cfg->phy_conn_type);
		return;
	}

	reg = sys_read32(gmii_sel_virt);
	reg &= ~(GMII_SEL_MODE_MASK | GMII_SEL_RGMII_ID_BIT);
	reg |= (mode_val & GMII_SEL_MODE_MASK) | rgmii_id;
	sys_write32(reg, gmii_sel_virt);

	LOG_DBG("MAC port %u: GMII_SEL @0x%08lx = 0x%08x",
		port_cfg->port_num, (unsigned long)gmii_sel_virt, reg);
}

/**
 * @brief Start the CPSW network interface for the MAC Port
 *
 * @param dev CPSW device pointer
 * @param iface Network interface pointer
 * @retval 0 on success, negative errno on failure
 */
static int cpsw_start(const struct device *dev, struct net_if *iface)
{
	const struct cpsw_dt_config *config = dev->config;
	struct cpsw_priv *priv = dev->data;
	int ret, i = cpsw_port_idx(priv, iface);
	uint8_t port_num;

	if (i < 0) {
		return -EINVAL;
	}

	port_num = (uint8_t)(i + 1);

	k_mutex_lock(&priv->start_lock, K_FOREVER);

	/* Exit if interface has already been started */
	if (priv->ports_started & BIT(i)) {
		k_mutex_unlock(&priv->start_lock);
		return 0;
	}

	/*
	 * First port to start initialises the shared hardware i.e.
	 * setup DMA Channels, configure ALE and Host Port, request interrupts
	 * for DMA Completion and create completion threads.
	 * Subsequent ports will skip this setup.
	 */
	if (priv->ports_started == 0U) {
		LOG_DBG("Starting CPSW shared hardware");

		priv->pktdma_dev = DEVICE_DT_GET(DT_NODELABEL(pktdma));
		if (!device_is_ready(priv->pktdma_dev)) {
			k_mutex_unlock(&priv->start_lock);
			LOG_ERR("PKTDMA device not ready");
			return -ENODEV;
		}

		ret = cpsw_hw_init(priv);
		if (ret < 0) {
			k_mutex_unlock(&priv->start_lock);
			LOG_ERR("HW init failed: %d", ret);
			return ret;
		}

		priv->tx_irq = ti_pktdma_get_chan_irq(priv->pktdma_dev,
						      CPSW_TX_CHAN_ID);
		priv->rx_irq = ti_pktdma_get_chan_irq(priv->pktdma_dev,
						      CPSW_RX_CHAN_ID);
		LOG_DBG("CPSW: tx_irq=%d rx_irq=%d", priv->tx_irq, priv->rx_irq);
		if (priv->tx_irq < 0 || priv->rx_irq < 0) {
			k_mutex_unlock(&priv->start_lock);
			LOG_ERR("PKTDMA IRQ not found for TX/RX channels");
			return -ENODEV;
		}

		irq_connect_dynamic(priv->tx_irq, IRQ_DEFAULT_PRIORITY,
				    cpsw_tx_irq_handler, priv->dev,
				    IRQ_TYPE_LEVEL);
		irq_connect_dynamic(priv->rx_irq, IRQ_DEFAULT_PRIORITY,
				    cpsw_rx_irq_handler, priv->dev,
				    IRQ_TYPE_LEVEL);

		LOG_DBG("CPSW: TX IRQ %d RX IRQ %d enabled",
			priv->tx_irq, priv->rx_irq);

		priv->ports_started |= BIT(i);

		priv->tx_tid = k_thread_create(&priv->tx_thread,
					       priv->tx_stack,
					       CPSW_TX_THREAD_STACK_SIZE,
					       cpsw_tx_cmpl_thread,
					       priv, NULL, NULL,
					       CPSW_TX_THREAD_PRIORITY,
					       0, K_NO_WAIT);
		k_thread_name_set(priv->tx_tid, "cpsw_tx");

		priv->rx_tid = k_thread_create(&priv->rx_thread,
					       priv->rx_stack,
					       CPSW_RX_THREAD_STACK_SIZE,
					       cpsw_rx_thread,
					       priv, NULL, NULL,
					       CPSW_RX_THREAD_PRIORITY,
					       0, K_NO_WAIT);
		k_thread_name_set(priv->rx_tid, "cpsw_rx");

		ti_pktdma_ring_irq_arm(priv->pktdma_dev, CPSW_TX_CHAN_ID);
		ti_pktdma_ring_irq_arm(priv->pktdma_dev, CPSW_RX_CHAN_ID);

		irq_enable(priv->tx_irq);
		irq_enable(priv->rx_irq);
	} else {
		priv->ports_started |= BIT(i);
	}

	k_mutex_unlock(&priv->start_lock);

	/* Program the GMII_SEL register with the phy connection type from DT */
	cpsw_gmii_sel_set(&config->port[i],
			  priv->gmii_sel_base +
			  (config->port[i].gmii_sel_addr & 0xfffU));

	/* Set MAC Port's ALE state to forwarding and set MAC Only mode based on DT */
	cpsw_write(priv, CPSW_NU_BASE + CPSW_ALE_BASE + cpsw_ale_portctls[i],
		   ALE_PORTCTL_FORWARD |
		   (config->port[i].mac_only ? ALE_PORTCTL_MAC_ONLY : 0U));

	LOG_DBG("Started MAC port %u", port_num);

	return 0;
}

/**
 * @brief Stop the CPSW network interface
 *
 * @param dev CPSW device pointer
 * @param iface Network interface pointer
 * @retval 0 always
 */
static int cpsw_stop(const struct device *dev, struct net_if *iface)
{
	struct cpsw_priv *priv = dev->data;
	int i = cpsw_port_idx(priv, iface);
	uint8_t port_num;

	/* Exit if port does not exist */
	if (i < 0) {
		return 0;
	}

	port_num = (uint8_t)(i + 1);

	/* Exit if interface has already been stopped */
	if (!(priv->ports_started & BIT(i))) {
		return 0;
	}

	priv->ports_started &= ~BIT(i);

	LOG_DBG("Stopped MAC port %u", port_num);

	/* If other ports are active, shared hardware should remain functional */
	if (priv->ports_started != 0U) {
		return 0;
	}

	/* Disable TX and RX completion interrupts */
	irq_disable(priv->tx_irq);
	irq_disable(priv->rx_irq);

	/* Stop the TX and RX completion threads */
	k_sem_give(&priv->dma.tx_compl_sem);
	k_sem_give(&priv->dma.rx_avail_sem);

	/* Stop the TX and RX DMA Channels */
	if (priv->pktdma_dev) {
		dma_stop(priv->pktdma_dev, CPSW_TX_CHAN_ID);
		dma_stop(priv->pktdma_dev, CPSW_RX_CHAN_ID);
	}

	cpsw_write(priv, CPSW_NU_BASE + CPSW_ALE_BASE + ALE_CONTROL, 0U);
	cpsw_write(priv, CPSW_NUSS_CONTROL, 0U);

	/* Allow the RX thread to finish draining existing packets and exit */
	k_sleep(K_MSEC(CPSW_RX_DRAIN_PERIOD_MS));

	/* Release all RX fragments remaining that aren't drained by the RX thread */
	for (int j = 0; j < CPSW_RX_DESC_NUM; j++) {
		if (priv->dma.rx_frags[j] != NULL) {
			net_pkt_frag_unref(priv->dma.rx_frags[j]);
			priv->dma.rx_frags[j] = NULL;
		}
	}

	priv->dma.initialized = false;

	return 0;
}

/**
 * @brief Return the link speeds supported by this CPSW MAC port
 *
 * @param dev CPSW device pointer
 * @param iface Network interface pointer
 * @return Bitmask of supported ETHERNET_LINK capabilities
 */
static enum ethernet_hw_caps cpsw_get_capabilities(const struct device *dev,
						   struct net_if *iface)
{
	const struct cpsw_dt_config *config = dev->config;
	struct cpsw_priv *priv = dev->data;
	int i = cpsw_port_idx(priv, iface);

	if (i >= 0 && config->port[i].phy_conn_type == CPSW_PHY_CONN_RMII) {
		return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE;
	}

	/* Default for RGMII */
	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE |
	       ETHERNET_LINK_1000BASE;
}

/**
 * @brief Handle CPSW network interface configuration commands
 *
 * @param dev    CPSW device pointer
 * @param iface Network interface pointer
 * @param type   Configuration type
 * @param config Configuration value
 * @retval 0 on success, -ENOTSUP for unsupported types
 */
static int cpsw_set_config(const struct device *dev,
			   struct net_if *iface,
			   enum ethernet_config_type type,
			   const struct ethernet_config *config)
{
	struct cpsw_priv *priv = dev->data;
	int i = cpsw_port_idx(priv, iface);
	uint32_t sa_l, sa_h;
	uint8_t *mac;

	if (i < 0) {
		return -EINVAL;
	}

	mac = priv->mac_addr[i];

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(mac, config->mac_address.addr, NET_ETH_ADDR_LEN);

		sa_l = ((uint32_t)mac[5] <<  0) | ((uint32_t)mac[4] <<  8) |
		       ((uint32_t)mac[3] << 16) | ((uint32_t)mac[2] << 24);
		sa_h = ((uint32_t)mac[1] <<  0) | ((uint32_t)mac[0] <<  8);

		cpsw_write(priv,
			   CPSW_NU_BASE + cpsw_port_bases[i] + CPSW_PN_SA_L,
			   sa_l);
		cpsw_write(priv,
			   CPSW_NU_BASE + cpsw_port_bases[i] + CPSW_PN_SA_H,
			   sa_h);

		LOG_DBG("MAC port %u address updated", i + 1);
		return 0;
	default:
		return -ENOTSUP;
	}
}

/**
 * @brief Initialize the network interface for CPSW
 *
 * @param iface The network interface to initialize
 */
static void cpsw_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	const struct cpsw_dt_config *config = dev->config;
	const struct cpsw_port_dt_config *port_cfg;
	struct cpsw_priv *priv = dev->data;
	int i;

	/* Determine which port slot this interface belongs to */
	for (i = 0; i < CPSW_NUM_MAC_PORTS; i++) {
		if (priv->iface[i] == NULL) {
			break;
		}
	}

	if (i >= CPSW_NUM_MAC_PORTS) {
		LOG_ERR("CPSW: interface does not have an associated MAC port");
		return;
	}

	port_cfg = &config->port[i];
	priv->iface[i] = iface;
	net_if_set_link_addr(iface, priv->mac_addr[i],
			     sizeof(priv->mac_addr[i]), NET_LINK_ETHERNET);
	ethernet_init(iface);

	/* Register PHY callback for the MAC Port */
	if (port_cfg->phy_dev != NULL) {
		/* Carrier should be enabled by PHY */
		net_if_carrier_off(iface);

		if (device_is_ready(port_cfg->phy_dev)) {
			priv->port_ctx[i].dev      = priv->dev;
			priv->port_ctx[i].port_num = (uint8_t)(i + 1);
			phy_link_callback_set(port_cfg->phy_dev,
					      cpsw_phy_link_cb,
					      &priv->port_ctx[i]);
		} else {
			LOG_ERR("PHY Device for MAC port %d is not ready", i + 1);
		}
	}

	LOG_INF("MAC port %u: network interface initialized (index=%d)",
		i + 1, net_if_get_by_iface(iface));
}

/**
 * @brief Get the Ethernet PHY device for CPSW's MAC Port
 *
 * @param dev CPSW device pointer
 * @param iface Network interface pointer
 * returns PHY Device pointer (NULL if PHY doesn't exist)
 */
static const struct device *cpsw_get_phy(const struct device *dev, struct net_if *iface)
{
	const struct cpsw_dt_config *config = dev->config;
	struct cpsw_priv *priv = dev->data;
	int i = cpsw_port_idx(priv, iface);

	if (i < 0) {
		return NULL;
	}

	return config->port[i].phy_dev;
}

static const struct ethernet_api cpsw_api = {
	.iface_api.init   = cpsw_iface_init,
	.get_phy          = cpsw_get_phy,
	.start            = cpsw_start,
	.stop             = cpsw_stop,
	.get_capabilities = cpsw_get_capabilities,
	.set_config       = cpsw_set_config,
	.send             = cpsw_tx,
};

/**
 * @brief CPSW driver initialization callback
 *
 * @param dev CPSW device pointer
 * @retval 0 always
 */
static int cpsw_init(const struct device *dev)
{
	const struct cpsw_dt_config *config = dev->config;
	struct cpsw_priv *priv = dev->data;
	uintptr_t gmii_addr, page_off;
	mm_reg_t page_virt;
	size_t map_size;

	/* Map the CPSW SS register region */
	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
	priv->base = (uint32_t *)DEVICE_MMIO_GET(dev);

	/*
	 * Map the GMII_SEL region using port 0's physical address to determine
	 * the page base.  Port 1 uses base + 0x0, port 2 uses base + 0x4.
	 */
	gmii_addr = config->port[0].gmii_sel_addr;
	page_off  = gmii_addr & (CONFIG_MMU_PAGE_SIZE - 1U);
	if (page_off == 0U) {
		/* Physical address is page-aligned */
		device_map(&priv->gmii_sel_base, gmii_addr,
			   ROUND_UP(config->port[0].gmii_sel_size,
				    CONFIG_MMU_PAGE_SIZE),
			   K_MEM_CACHE_NONE);
	} else {
		/* Physical address is not page-aligned */
		map_size = ROUND_UP(page_off + config->port[0].gmii_sel_size,
				    CONFIG_MMU_PAGE_SIZE);
		device_map(&page_virt, gmii_addr - page_off, map_size,
			   K_MEM_CACHE_NONE);
		priv->gmii_sel_base = page_virt;
	}

	for (int i = 0; i < CPSW_NUM_MAC_PORTS; i++) {
		net_eth_mac_load(&config->port[i].mac_cfg, priv->mac_addr[i]);
	}

	priv->dev = dev;
	k_mutex_init(&priv->tx_lock);
	k_mutex_init(&priv->start_lock);

	LOG_INF("CPSW version: 0x%08x", cpsw_read(priv, CPSW_NUSS_VER));

	return 0;
}

/* MAC ports, MDIO and PHY nodes fetched from device-tree */
#define CPSW_ETH_PORTS_NODE(n)		DT_INST_CHILD(n, ethernet_ports)
#define CPSW_PORT_NODE(n, idx)		DT_CHILD(CPSW_ETH_PORTS_NODE(n), port_##idx)
#define CPSW_MDIO_NODE(n)		DT_INST_CHILD(n, mdio_f00)
#define CPSW_PHY_NODE(n, mdio_addr)	DT_CHILD(CPSW_MDIO_NODE(n), ethernet_phy_##mdio_addr)

/* MAC Port device-tree configuration */
#define CPSW_PORT_DT_CONFIG_INIT(n, port_idx, mdio_addr)			\
{										\
	.phy_dev = COND_CODE_1(							\
		DT_NODE_HAS_STATUS(CPSW_PHY_NODE(n, mdio_addr), okay),		\
		(DEVICE_DT_GET(CPSW_PHY_NODE(n, mdio_addr))),			\
		(NULL)),							\
	.phy_conn_type = CPSW_PHY_CONN_TYPE(CPSW_PORT_NODE(n, port_idx)),	\
	.mac_cfg       = NET_ETH_MAC_DT_CONFIG_INIT(				\
		CPSW_PORT_NODE(n, port_idx)),					\
	.gmii_sel_addr = DT_REG_ADDR(						\
		DT_PHANDLE(CPSW_PORT_NODE(n, port_idx), phys)) +		\
		((port_idx) == 2 ? 0x4 : 0x0),					\
	.gmii_sel_size = DT_REG_SIZE(						\
		DT_PHANDLE(CPSW_PORT_NODE(n, port_idx), phys)),			\
	.mac_only      = DT_PROP(CPSW_PORT_NODE(n, port_idx), ti_mac_only),	\
	.port_num      = (port_idx),						\
}

#define CPSW_DEVICE_INIT(n)						\
	static struct cpsw_priv cpsw_priv_##n;				\
									\
	static const struct cpsw_dt_config cpsw_cfg_##n = {		\
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),			\
		.port = {						\
			[0] = CPSW_PORT_DT_CONFIG_INIT(n, 1, 0),	\
			[1] = CPSW_PORT_DT_CONFIG_INIT(n, 2, 1),	\
		},							\
	};								\
									\
	ETH_NET_DEVICE_DT_INST_DEFINE(n, cpsw_init, NULL,		\
				      &cpsw_priv_##n, &cpsw_cfg_##n,	\
				      CONFIG_ETH_INIT_PRIORITY,		\
				      &cpsw_api, NET_ETH_MTU);		\
									\
	NET_DEVICE_DT_INST_ADD_IFACE(n, ETHERNET_L2,			\
				     NET_L2_GET_CTX_TYPE(ETHERNET_L2),	\
				     NET_ETH_MTU, 1);

DT_INST_FOREACH_STATUS_OKAY(CPSW_DEVICE_INIT)
