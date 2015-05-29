/* Stupid, but it works. This is to keep the crc for dma_get_required_mask. */
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/bootmem.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/idr.h>
#include <linux/acpi.h>
#include "../../../drivers/base/base.h"
#include "../../../drivers/base/power/power.h"

#include <xen/interface/memory.h>

u64 dma_get_required_mask(struct device *dev)
{
	unsigned long max_mfn = HYPERVISOR_memory_op(XENMEM_maximum_ram_page,
						     NULL);

	return DMA_BIT_MASK(__fls(max_mfn - 1) + 1 + PAGE_SHIFT);
}
EXPORT_SYMBOL_GPL(dma_get_required_mask);
