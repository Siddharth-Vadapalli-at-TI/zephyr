/*
 * TI AM62L PKTDMA driver API
 *
 * Copyright (c) 2026 Texas Instruments Incorporated
 * Author: Siddharth Vadapalli <s-vadapalli@ti.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_TI_AM62L_PKTDMA_H_
#define ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_TI_AM62L_PKTDMA_H_

#include <zephyr/drivers/dma.h>
#include <zephyr/sys/util.h>

/**
 * @brief TI AM62L PKTDMA driver API version
 *
 * Increment rules:
 *  - MAJOR: changes affecting backward-compatibility
 *  - MINOR: new features or bug fixes retaining backward-compatibility
 *  - PATCH: documentation fixes and refactoring which don't change API
 */
#define TI_PKTDMA_API_VERSION_MAJOR  1
#define TI_PKTDMA_API_VERSION_MINOR  0
#define TI_PKTDMA_API_VERSION_PATCH  0

/**
 * @brief Encode a major.minor.patch triplet into a single integer
 *        (bits [23:16] = major, [15:8] = minor, [7:0] = patch)
 */
#define TI_PKTDMA_MAKE_VERSION(major, minor, patch) \
	(((major) << 16) | ((minor) << 8) | (patch))

/**
 * @brief Encoded current API version allows users to verify compatibility
 */
#define TI_PKTDMA_API_VERSION \
	TI_PKTDMA_MAKE_VERSION(TI_PKTDMA_API_VERSION_MAJOR, \
			       TI_PKTDMA_API_VERSION_MINOR, \
			       TI_PKTDMA_API_VERSION_PATCH)

/**
 * @brief Bit position of the ASEL field within a 64-bit PKTDMA/CPPI5 address
 */
#define AM62L_ADDRESS_ASEL_SHIFT   48U

/**
 * @brief The 4 bits corresponding to ASEL in a 64-bit address
 */
#define AM62L_ADDRESS_ASEL_MASK    GENMASK64(51, 48)

/**
 * @brief ASEL values 14 and 15 route DMA transactions through the ARM Accelerator
 * Coherency Port (ACP).  ACP transactions are fully coherent with the CPU cache,
 * so no explicit cache flush or invalidate is required for DMA buffers and
 * descriptors on coherent channels.
 */
#define AM62L_PKTDMA_IS_COHERENT(asel)  ((asel) == 14U || (asel) == 15U)

/**
 * @brief PKTDMA channel configuration passed via dma_config.user_data
 */
struct ti_pktdma_chan_cfg {
	/** Virtual address of the forward ring buffer */
	uintptr_t fwd_ring_mem;
	/** Virtual address of the reverse/completion ring buffer */
	uintptr_t rev_ring_mem;
	/** Number of 8-byte entries in each ring */
	uint32_t ring_cnt;
	/** RX channel: PERIPHERAL_TO_MEMORY, TX channel: MEMORY_TO_PERIPHERAL */
	bool is_rx;
	/**
	 * Address Space Select (ASEL) value (defaults to zero)
	 * 0  = non-coherent (cache flush/invalidate required)
	 * 14 or 15 = coherent (cache operations not required)
	 */
	uint8_t asel;
};

/**
 * @brief PKTDMA driver API
 *
 * struct dma_driver_api MUST be the first member.
 */
__subsystem struct ti_pktdma_driver_api {
	/** Standard Zephyr DMA API (config / start / stop / get_status) */
	struct dma_driver_api dma_api;

	/**
	 * @brief Submit a descriptor to the channel's forward ring
	 *
	 * @param dev    PKTDMA device
	 * @param chan_id PKTDMA channel number
	 * @param desc_phys Physical address of the descriptor to push
	 * @retval 0 on success, negative errno on error
	 */
	int (*ring_push)(const struct device *dev, uint32_t chan_id,
			 uint64_t desc_phys);

	/**
	 * @brief Retrieve a descriptor from the channel's reverse ring
	 *
	 * @param dev       PKTDMA device
	 * @param chan_id   PKTDMA channel number
	 * @param desc_phys Physical address of the popped descriptor
	 * @retval 0 on success, -ENODATA if the ring is empty, negative errno
	 *			on other errors.
	 */
	int (*ring_pop)(const struct device *dev, uint32_t chan_id,
			uint64_t *desc_phys);

	/**
	 * @brief Return the interrupt number for a PKTDMA channel's completion ring
	 *
	 * @param dev     PKTDMA device
	 * @param chan_id PKTDMA channel number
	 * @return Interrupt number on success, negative errno if not found
	 */
	int (*get_chan_irq)(const struct device *dev, uint32_t chan_id);

	/**
	 * @brief Arm the ring completion interrupt for the PKTDMA channel
	 *
	 * @param dev     PKTDMA device
	 * @param chan_id PKTDMA channel number
	 */
	void (*ring_irq_arm)(const struct device *dev, uint32_t chan_id);

	/**
	 * @brief Clear the ring completion interrupt for the PKTDMA channel
	 *
	 * @param dev     PKTDMA device
	 * @param chan_id PKTDMA channel number
	 */
	void (*ring_irq_clear)(const struct device *dev, uint32_t chan_id);
};

DEVICE_API_EXTENDS(ti_pktdma, dma, dma_api);

/**
 * @brief Push a descriptor to the PKTDMA channel's forward ring
 */
static inline int ti_pktdma_ring_push(const struct device *dev,
				      uint32_t chan_id, uint64_t desc_phys)
{
	const struct ti_pktdma_driver_api *api =
		(const struct ti_pktdma_driver_api *)dev->api;

	return api->ring_push(dev, chan_id, desc_phys);
}

/**
 * @brief Pop a descriptor from the PKTDMA channel's reverse ring
 */
static inline int ti_pktdma_ring_pop(const struct device *dev,
				     uint32_t chan_id, uint64_t *desc_phys)
{
	const struct ti_pktdma_driver_api *api =
		(const struct ti_pktdma_driver_api *)dev->api;

	return api->ring_pop(dev, chan_id, desc_phys);
}

/**
 * @brief Arm the ring completion interrupt
 */
static inline void ti_pktdma_ring_irq_arm(const struct device *dev,
					   uint32_t chan_id)
{
	const struct ti_pktdma_driver_api *api =
		(const struct ti_pktdma_driver_api *)dev->api;

	api->ring_irq_arm(dev, chan_id);
}

/**
 * @brief Clear the ring completion interrupt
 */
static inline void ti_pktdma_ring_irq_clear(const struct device *dev,
					    uint32_t chan_id)
{
	const struct ti_pktdma_driver_api *api =
		(const struct ti_pktdma_driver_api *)dev->api;

	api->ring_irq_clear(dev, chan_id);
}

/**
 * @brief Get the interrupt number for a PKTDMA channel's completion ring
 *
 * @param dev     PKTDMA device
 * @param chan_id PKTDMA channel number
 * @return Interrupt number on success, negative errno if not found
 */
static inline int ti_pktdma_get_chan_irq(const struct device *dev,
					 uint32_t chan_id)
{
	const struct ti_pktdma_driver_api *api =
		(const struct ti_pktdma_driver_api *)dev->api;

	return api->get_chan_irq(dev, chan_id);
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_TI_AM62L_PKTDMA_H_ */
