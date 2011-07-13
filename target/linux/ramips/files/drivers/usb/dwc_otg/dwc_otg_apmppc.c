/*
 * DesignWare HS OTG controller driver
 * Copyright (C) 2006 Synopsys, Inc.
 * Portions Copyright (C) 2010 Applied Micro Circuits Corporation.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License version 2 for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see http://www.gnu.org/licenses
 * or write to the Free Software Foundation, Inc., 51 Franklin Street,
 * Suite 500, Boston, MA 02110-1335 USA.
 *
 * Based on Synopsys driver version 2.60a
 * Modified by Mark Miesfeld <mmiesfeld at apm.com>
 * Modified by Stefan Roese <sr at denx.de>, DENX Software Engineering
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SYNOPSYS, INC. BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES
 * (INCLUDING BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * The dwc_otg module provides the initialization and cleanup entry
 * points for the dwcotg driver. This module will be dynamically installed
 * after Linux is booted using the insmod command. When the module is
 * installed, the dwc_otg_driver_init function is called. When the module is
 * removed (using rmmod), the dwc_otg_driver_cleanup function is called.
 *
 * This module also defines a data structure for the dwc_otg driver, which is
 * used in conjunction with the standard device structure. These
 * structures allow the OTG driver to comply with the standard Linux driver
 * model in which devices and drivers are registered with a bus driver. This
 * has the benefit that Linux can expose attributes of the driver and device
 * in its special sysfs file system. Users can then read or write files in
 * this file system to perform diagnostics on the driver components or the
 * device.
 */

#include <linux/of_platform.h>

#include "dwc_otg_driver.h"

#define DWC_DRIVER_VERSION		"1.05"
#define DWC_DRIVER_DESC			"HS OTG USB Controller driver"
static const char dwc_driver_name[] = "dwc_otg";

/**
 * This function is the top level interrupt handler for the Common
 * (Device and host modes) interrupts.
 */
static irqreturn_t dwc_otg_common_irq(int _irq, void *dev)
{
	struct dwc_otg_device *dwc_dev = dev;
	int retval = IRQ_NONE;

	retval = dwc_otg_handle_common_intr(dwc_dev->core_if);
	return IRQ_RETVAL(retval);
}

/**
 * This function is the interrupt handler for the OverCurrent condition
 * from the external charge pump (if enabled)
 */
static irqreturn_t dwc_otg_externalchgpump_irq(int _irq, void *dev)
{
	struct dwc_otg_device *dwc_dev = dev;

	if (dwc_otg_is_host_mode(dwc_dev->core_if)) {
		struct dwc_hcd *dwc_hcd;
		union hprt0_data hprt0 = {.d32 = 0};

		dwc_hcd = dwc_dev->hcd;
		spin_lock(&dwc_hcd->lock);
		dwc_hcd->flags.b.port_over_current_change = 1;

		hprt0.b.prtpwr = 0;
		dwc_write_reg32(dwc_dev->core_if->host_if->hprt0,
				hprt0.d32);
		spin_unlock(&dwc_hcd->lock);
	} else {
		/* Device mode - This int is n/a for device mode */
		printk(KERN_ERR "DeviceMode: OTG OverCurrent Detected\n");
	}

	return IRQ_HANDLED;
}

/**
 * This function is called when a device is unregistered with the
 * dwc_otg_driver. This happens, for example, when the rmmod command is
 * executed. The device may or may not be electrically present. If it is
 * present, the driver stops device processing. Any resources used on behalf
 * of this device are freed.
 */
static int __devexit dwc_otg_driver_remove(struct platform_device *ofdev)
{
	struct device *dev = &ofdev->dev;
	struct dwc_otg_device *dwc_dev = dev_get_drvdata(dev);

	/* Memory allocation for dwc_otg_device may have failed. */
	if (!dwc_dev)
		return 0;

	usb_nop_xceiv_unregister();

	/* Free the IRQ	*/
	if (dwc_dev->common_irq_installed)
		free_irq(dwc_dev->irq, dwc_dev);

	if (!dwc_has_feature(dwc_dev->core_if, DWC_DEVICE_ONLY))
		if (dwc_dev->hcd)
			dwc_otg_hcd_remove(dev);

	if (!dwc_has_feature(dwc_dev->core_if, DWC_HOST_ONLY))
		if (dwc_dev->pcd)
			dwc_otg_pcd_remove(dev);

	if (dwc_dev->core_if)
		dwc_otg_cil_remove(dwc_dev->core_if);

	/* Return the memory. */
	if (dwc_dev->base)
		iounmap(dwc_dev->base);
	if (dwc_dev->phys_addr)
		release_mem_region(dwc_dev->phys_addr, dwc_dev->base_len);
	kfree(dwc_dev);

	/* Clear the drvdata pointer. */
	dev_set_drvdata(dev, 0);
	return 0;
}

/**
 * This function is called when an device is bound to a
 * dwc_otg_driver. It creates the driver components required to
 * control the device (CIL, HCD, and PCD) and it initializes the
 * device. The driver components are stored in a dwc_otg_device
 * structure. A reference to the dwc_otg_device is saved in the
 * device. This allows the driver to access the dwc_otg_device
 * structure on subsequent calls to driver methods for this device.
 */
static int __devinit dwc_otg_driver_probe(struct platform_device *ofdev,
		const struct of_device_id *match)
{
	int retval = 0;
	struct dwc_otg_device *dwc_dev;
	struct device *dev = &ofdev->dev;
	struct resource res;
	u32 *gusbcfg_addr;
	union gusbcfg_data usbcfg = {.d32 = 0};
	u32 cp_irq;

	dev_dbg(dev, "dwc_otg_driver_probe(%p)\n", dev);

	dwc_dev = kzalloc(sizeof(*dwc_dev), GFP_KERNEL);
	if (!dwc_dev) {
		dev_err(dev, "kmalloc of dwc_otg_device failed\n");
		retval = -ENOMEM;
		goto fail;
	}

	/* Retrieve the memory and IRQ resources. */
	dwc_dev->irq = irq_of_parse_and_map(ofdev->dev.of_node, 0);
	if (dwc_dev->irq == NO_IRQ) {
		dev_err(dev, "no device irq\n");
		retval = -ENODEV;
		goto fail;
	}
	dev_dbg(dev, "OTG - device irq: %d\n", dwc_dev->irq);

	if (of_address_to_resource(ofdev->dev.of_node, 0, &res)) {
		printk(KERN_ERR "%s: Can't get USB-OTG register address\n",
			__func__);
		retval = -ENOMEM;
		goto fail;
	}
	dev_dbg(dev, "OTG - ioresource_mem start0x%08x: end:0x%08x\n",
			(u32)res.start, (u32)res.end);

	dwc_dev->phys_addr = res.start;
	dwc_dev->base_len = res.end - res.start + 1;
	if (!request_mem_region(dwc_dev->phys_addr,
					dwc_dev->base_len,
					dwc_driver_name)) {
		dev_err(dev, "request_mem_region failed\n");
		retval = -EBUSY;
		goto fail;
	}

	/* Map the DWC_otg Core memory into virtual address space. */
	dwc_dev->base = ioremap(dwc_dev->phys_addr,
					dwc_dev->base_len);
	if (!dwc_dev->base) {
		dev_err(dev, "ioremap() failed\n");
		retval = -ENOMEM;
		goto fail;
	}
	dev_dbg(dev, "mapped base=0x%08x\n", (unsigned)dwc_dev->base);

	/*
	 * Initialize driver data to point to the global DWC_otg
	 * Device structure.
	 */
	dev_set_drvdata(dev, dwc_dev);

	dev_dbg(dev, "dwc_dev=0x%p\n", dwc_dev);
	dwc_dev->core_if =
		dwc_otg_cil_init(dwc_dev->base, &dwc_otg_module_params);
	if (!dwc_dev->core_if) {
		dev_err(dev, "CIL initialization failed!\n");
		retval = -ENOMEM;
		goto fail;
	}
	usb_nop_xceiv_register();
	dwc_dev->core_if->xceiv = otg_get_transceiver();
	if (!dwc_dev->core_if->xceiv) {
		retval = -ENODEV;
		goto fail;
	}
	dwc_set_feature(dwc_dev->core_if);

	gusbcfg_addr = &dwc_dev->core_if->core_global_regs->gusbcfg;

	/*
	 * Validate parameter values.
	 */
	if (check_parameters(dwc_dev->core_if)) {
		retval = -EINVAL;
		goto fail;
	}

	/* Added for PLB DMA phys virt mapping */
	dwc_dev->core_if->phys_addr = dwc_dev->phys_addr;

	/*
	 * Disable the global interrupt until all the interrupt
	 * handlers are installed.
	 */
	dwc_otg_disable_global_interrupts(dwc_dev->core_if);

	/*
	 * Install the interrupt handler for the common interrupts before
	 * enabling common interrupts in core_init below.
	 */
	retval = request_irq(dwc_dev->irq, dwc_otg_common_irq,
			IRQF_SHARED, "dwc_otg", dwc_dev);
	if (retval) {
		printk(KERN_ERR "request of irq%d failed retval: %d\n",
				dwc_dev->irq, retval);
		retval = -EBUSY;
		goto fail;
	} else {
		dwc_dev->common_irq_installed = 1;
	}

	/* Initialize the DWC_otg core.	*/
	dwc_otg_core_init(dwc_dev->core_if);

	/* configure chargepump interrupt */
	cp_irq = irq_of_parse_and_map(ofdev->dev.of_node, 3);
	if (cp_irq) {
		retval = request_irq(cp_irq, dwc_otg_externalchgpump_irq,
				IRQF_SHARED, "dwc_otg_ext_chg_pump", dwc_dev);
		if (retval) {
			printk(KERN_ERR "request of irq failed retval: %d\n",
				retval);
			retval = -EBUSY;
			goto fail;
		} else {
			printk(KERN_INFO "%s: ExtChgPump Detection "
					"IRQ registered\n", dwc_driver_name);
		}
	}

	if (!dwc_has_feature(dwc_dev->core_if, DWC_HOST_ONLY)) {
		/* Initialize the PCD */
		retval = dwc_otg_pcd_init(dev);
		if (retval) {
			printk(KERN_ERR "dwc_otg_pcd_init failed\n");
			dwc_dev->pcd = NULL;
			goto fail;
		}
	}

	if (!dwc_has_feature(dwc_dev->core_if, DWC_DEVICE_ONLY)) {
		/* Initialize the HCD and force_host_mode */
		usbcfg.d32 = dwc_read_reg32(gusbcfg_addr);
		usbcfg.b.force_host_mode = 1;
		dwc_write_reg32(gusbcfg_addr, usbcfg.d32);

		retval = dwc_otg_hcd_init(dev, dwc_dev);
		if (retval) {
			printk(KERN_ERR "dwc_otg_hcd_init failed\n");
			dwc_dev->hcd = NULL;
			goto fail;
		}
	}
	/*
	 * Enable the global interrupt after all the interrupt
	 * handlers are installed.
	 */
	dwc_otg_enable_global_interrupts(dwc_dev->core_if);

	usbcfg.d32 = dwc_read_reg32(gusbcfg_addr);
	usbcfg.b.force_host_mode = 0;
	dwc_write_reg32(gusbcfg_addr, usbcfg.d32);

	return 0;

fail:
	dwc_otg_driver_remove(ofdev);
	return retval;
}

/*
 * This structure defines the methods to be called by a bus driver
 * during the lifecycle of a device on that bus. Both drivers and
 * devices are registered with a bus driver. The bus driver matches
 * devices to drivers based on information in the device and driver
 * structures.
 *
 * The probe function is called when the bus driver matches a device
 * to this driver. The remove function is called when a device is
 * unregistered with the bus driver.
 */
static const struct of_device_id dwc_otg_match[] = {
	{ .compatible = "amcc,dwc-otg", },
	{}
};
MODULE_DEVICE_TABLE(of, dwc_otg_match);

static struct of_platform_driver dwc_otg_driver = {
	.probe = dwc_otg_driver_probe,
	.remove = __devexit_p(dwc_otg_driver_remove),
	.driver = {
		.name = "dwc_otg",
		.owner = THIS_MODULE,
		.of_match_table = dwc_otg_match,
	},
};

/**
 * This function is called when the dwc_otg_driver is installed with the
 * insmod command. It registers the dwc_otg_driver structure with the
 * appropriate bus driver. This will cause the dwc_otg_driver_probe function
 * to be called. In addition, the bus driver will automatically expose
 * attributes defined for the device and driver in the special sysfs file
 * system.
 */
static int  __init dwc_otg_driver_init(void)
{
	int retval = 0;

	printk(KERN_INFO "%s: version %s\n", dwc_driver_name,
			DWC_DRIVER_VERSION);
	retval = of_register_platform_driver(&dwc_otg_driver);
	if (retval < 0)
		printk(KERN_ERR "%s registration failed. retval=%d\n",
				dwc_driver_name, retval);
	return retval;
}
module_init(dwc_otg_driver_init);

/**
 * This function is called when the driver is removed from the kernel
 * with the rmmod command. The driver unregisters itself with its bus
 * driver.
 *
 */
static void __exit dwc_otg_driver_cleanup(void)
{
	of_unregister_platform_driver(&dwc_otg_driver);
	printk(KERN_INFO "%s module removed\n", dwc_driver_name);
}
module_exit(dwc_otg_driver_cleanup);

MODULE_DESCRIPTION(DWC_DRIVER_DESC);
MODULE_AUTHOR("Mark Miesfeld <mmiesfeld at apm.com");
MODULE_LICENSE("GPL");
