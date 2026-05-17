/**
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
 * MAJOR.MINOR.PATCH
 */

/**
 * Increment rule for MAJOR:
 * Changes affecting backward-compatibility
 */
#define TI_PKTDMA_API_VERSION_MAJOR  1

/**
 * Increment rule for MINOR:
 * New features or bug fixes retaining backward-compatibility
 */
#define TI_PKTDMA_API_VERSION_MINOR  0

/**
 * Increment rule for PATCH:
 * Documentation fixes and refactoring which don't change API
 */
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
 * @def_driverbackendgroup{TI AM62L PKTDMA,dma_interface}
 * @ingroup dma_interface
 * @{
 */

/**
 * @brief Type definition for the PKTDMA channel configure operation.
 * See ti_pktdma_chan_configure() for argument descriptions.
 */
typedef int (*ti_pktdma_api_chan_configure)(const struct device *dev, uint32_t chan_id,
					   const struct ti_pktdma_chan_cfg *chan_cfg);

/**
 * @brief Type definition for the PKTDMA ring push operation.
 * See ti_pktdma_ring_push() for argument descriptions.
 */
typedef int (*ti_pktdma_api_ring_push)(const struct device *dev, uint32_t chan_id,
				       uint64_t desc_phys);

/**
 * @brief Type definition for the PKTDMA ring pop operation.
 * See ti_pktdma_ring_pop() for argument descriptions.
 */
typedef int (*ti_pktdma_api_ring_pop)(const struct device *dev, uint32_t chan_id,
				      uint64_t *desc_phys);

/**
 * @brief Type definition for the PKTDMA get channel IRQ operation.
 * See ti_pktdma_get_chan_irq() for argument descriptions.
 */
typedef int (*ti_pktdma_api_get_chan_irq)(const struct device *dev, uint32_t chan_id);

/**
 * @brief Type definition for the PKTDMA ring IRQ arm operation.
 * See ti_pktdma_ring_irq_arm() for argument descriptions.
 */
typedef void (*ti_pktdma_api_ring_irq_arm)(const struct device *dev, uint32_t chan_id);

/**
 * @brief Type definition for the PKTDMA ring IRQ clear operation.
 * See ti_pktdma_ring_irq_clear() for argument descriptions.
 */
typedef void (*ti_pktdma_api_ring_irq_clear)(const struct device *dev, uint32_t chan_id);

/**
 * @driver_ops{TI AM62L PKTDMA}
 */
__subsystem struct ti_pktdma_driver_api {
	/** @cond INTERNAL_HIDDEN */
	struct dma_driver_api dma_api;
	/** @endcond */

	/** @driver_ops_mandatory @copybrief ti_pktdma_chan_configure */
	ti_pktdma_api_chan_configure chan_configure;

	/** @driver_ops_mandatory @copybrief ti_pktdma_ring_push */
	ti_pktdma_api_ring_push ring_push;

	/** @driver_ops_mandatory @copybrief ti_pktdma_ring_pop */
	ti_pktdma_api_ring_pop ring_pop;

	/** @driver_ops_mandatory @copybrief ti_pktdma_get_chan_irq */
	ti_pktdma_api_get_chan_irq get_chan_irq;

	/** @driver_ops_mandatory @copybrief ti_pktdma_ring_irq_arm */
	ti_pktdma_api_ring_irq_arm ring_irq_arm;

	/** @driver_ops_mandatory @copybrief ti_pktdma_ring_irq_clear */
	ti_pktdma_api_ring_irq_clear ring_irq_clear;
};

/** @} */

/** @cond INTERNAL_HIDDEN */
DEVICE_API_EXTENDS(ti_pktdma, dma, dma_api);
/** @endcond */

/**
 * @brief Configure a PKTDMA channel
 *
 * @param dev      PKTDMA device
 * @param chan_id  PKTDMA channel number
 * @param chan_cfg Channel configuration
 * @retval 0 on success, negative errno on error
 */
static inline int ti_pktdma_chan_configure(const struct device *dev, uint32_t chan_id,
					   const struct ti_pktdma_chan_cfg *chan_cfg)
{
	return DEVICE_API_GET(ti_pktdma, dev)->chan_configure(dev, chan_id, chan_cfg);
}

/**
 * @brief Push a descriptor to the PKTDMA channel's forward ring
 */
static inline int ti_pktdma_ring_push(const struct device *dev,
				      uint32_t chan_id, uint64_t desc_phys)
{
	return DEVICE_API_GET(ti_pktdma, dev)->ring_push(dev, chan_id, desc_phys);
}

/**
 * @brief Pop a descriptor from the PKTDMA channel's reverse ring
 */
static inline int ti_pktdma_ring_pop(const struct device *dev,
				     uint32_t chan_id, uint64_t *desc_phys)
{
	return DEVICE_API_GET(ti_pktdma, dev)->ring_pop(dev, chan_id, desc_phys);
}

/**
 * @brief Arm the ring completion interrupt
 */
static inline void ti_pktdma_ring_irq_arm(const struct device *dev,
					   uint32_t chan_id)
{
	DEVICE_API_GET(ti_pktdma, dev)->ring_irq_arm(dev, chan_id);
}

/**
 * @brief Clear the ring completion interrupt
 */
static inline void ti_pktdma_ring_irq_clear(const struct device *dev,
					    uint32_t chan_id)
{
	DEVICE_API_GET(ti_pktdma, dev)->ring_irq_clear(dev, chan_id);
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
	return DEVICE_API_GET(ti_pktdma, dev)->get_chan_irq(dev, chan_id);
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_DMA_DMA_TI_AM62L_PKTDMA_H_ */
