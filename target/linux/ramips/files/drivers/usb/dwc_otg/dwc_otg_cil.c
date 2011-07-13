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
 * The Core Interface Layer provides basic services for accessing and
 * managing the DWC_otg hardware. These services are used by both the
 * Host Controller Driver and the Peripheral Controller Driver.
 *
 * The CIL manages the memory map for the core so that the HCD and PCD
 * don't have to do this separately. It also handles basic tasks like
 * reading/writing the registers and data FIFOs in the controller.
 * Some of the data access functions provide encapsulation of several
 * operations required to perform a task, such as writing multiple
 * registers to start a transfer. Finally, the CIL performs basic
 * services that are not specific to either the host or device modes
 * of operation. These services include management of the OTG Host
 * Negotiation Protocol (HNP) and Session Request Protocol (SRP). A
 * Diagnostic API is also provided to allow testing of the controller
 * hardware.
 *
 * The Core Interface Layer has the following requirements:
 * - Provides basic controller operations.
 * - Minimal use of OS services.
 * - The OS services used will be abstracted by using inline functions
 *	 or macros.
 */
#include <linux/delay.h>

#include "dwc_otg_cil.h"

const char *op_state_str(enum usb_otg_state state)
{
	switch (state) {
	case OTG_STATE_A_IDLE:		return "a_idle";
	case OTG_STATE_A_WAIT_VRISE:	return "a_wait_vrise";
	case OTG_STATE_A_WAIT_BCON:	return "a_wait_bcon";
	case OTG_STATE_A_HOST:		return "a_host";
	case OTG_STATE_A_SUSPEND:	return "a_suspend";
	case OTG_STATE_A_PERIPHERAL:	return "a_peripheral";
	case OTG_STATE_A_WAIT_VFALL:	return "a_wait_vfall";
	case OTG_STATE_A_VBUS_ERR:	return "a_vbus_err";
	case OTG_STATE_B_IDLE:		return "b_idle";
	case OTG_STATE_B_SRP_INIT:	return "b_srp_init";
	case OTG_STATE_B_PERIPHERAL:	return "b_peripheral";
	case OTG_STATE_B_WAIT_ACON:	return "b_wait_acon";
	case OTG_STATE_B_HOST:		return "b_host";
	default:			return "UNDEFINED";
	}
}

/**
 * This function enables the controller's Global Interrupt in the AHB Config
 * register.
 */
void dwc_otg_enable_global_interrupts(struct core_if *core_if)
{
	union gahbcfg_data ahbcfg = {.d32 = 0};

	ahbcfg.b.glblintrmsk = 1;
	dwc_modify_reg32(&core_if->core_global_regs->gahbcfg, 0, ahbcfg.d32);
}

/**
 * This function disables the controller's Global Interrupt in the AHB Config
 * register.
 */
void dwc_otg_disable_global_interrupts(struct core_if *core_if)
{
	union gahbcfg_data ahbcfg = {.d32 = 0};

	ahbcfg.b.glblintrmsk = 1;
	dwc_modify_reg32(&core_if->core_global_regs->gahbcfg, ahbcfg.d32, 0);
}

/**
 * Tests if the current hardware is using a full speed phy.
 */
static inline int full_speed_phy(struct core_if *core_if)
{
	if ((core_if->hwcfg2.b.hs_phy_type == 2 &&
			core_if->hwcfg2.b.fs_phy_type == 1 &&
			core_if->core_params->ulpi_fs_ls) ||
			core_if->core_params->phy_type ==
			DWC_PHY_TYPE_PARAM_FS)
		return 1;
	return 0;
}

/**
 * Initializes the FSLSPClkSel field of the HCFG register depending on the PHY
 * type.
 */
void init_fslspclksel(struct core_if *core_if)
{
	u32 val;
	union hcfg_data hcfg;

	if (full_speed_phy(core_if))
		val = DWC_HCFG_48_MHZ;
	else
		/* High speed PHY running at full speed or high speed */
		val = DWC_HCFG_30_60_MHZ;

	hcfg.d32 = dwc_read_reg32(&core_if->host_if->host_global_regs->hcfg);
	hcfg.b.fslspclksel = val;
	dwc_write_reg32(&core_if->host_if->host_global_regs->hcfg, hcfg.d32);
}

/**
 * Initializes the DevSpd field of the DCFG register depending on the PHY type
 * and the enumeration speed of the device.
 */
static void init_devspd(struct core_if *core_if)
{
	u32 val;
	union  dcfg_data dcfg;

	if (full_speed_phy(core_if))
		val = 0x3;
	else if (core_if->core_params->speed == DWC_SPEED_PARAM_FULL)
		/* High speed PHY running at full speed */
		val = 0x1;
	else
		/* High speed PHY running at high speed */
		val = 0x0;

	dcfg.d32 = dwc_read_reg32(&core_if->dev_if->dev_global_regs->dcfg);
	dcfg.b.devspd = val;
	dwc_write_reg32(&core_if->dev_if->dev_global_regs->dcfg, dcfg.d32);
}

/**
 * This function calculates the number of IN EPS using GHWCFG1 and GHWCFG2
 * registers values
 */
static u32 calc_num_in_eps(struct core_if *core_if)
{
	u32 num_in_eps = 0;
	u32 num_eps = core_if->hwcfg2.b.num_dev_ep;
	u32 hwcfg1 = core_if->hwcfg1.d32 >> 2;
	u32 num_tx_fifos = core_if->hwcfg4.b.num_in_eps;
	u32 i;

	for (i = 0; i < num_eps; ++i) {
		if (!(hwcfg1 & 0x1))
			num_in_eps++;
		hwcfg1 >>= 2;
	}

	if (core_if->hwcfg4.b.ded_fifo_en)
		num_in_eps = num_in_eps > num_tx_fifos ?
				num_tx_fifos : num_in_eps;

	return num_in_eps;
}

/**
 * This function calculates the number of OUT EPS using GHWCFG1 and GHWCFG2
 * registers values
 */
static u32 calc_num_out_eps(struct core_if *core_if)
{
	u32 num_out_eps = 0;
	u32 num_eps = core_if->hwcfg2.b.num_dev_ep;
	u32 hwcfg1 = core_if->hwcfg1.d32 >> 2;
	u32 i;

	for (i = 0; i < num_eps; ++i) {
		if (!(hwcfg1 & 0x2))
			num_out_eps++;
		hwcfg1 >>= 2;
	}
	return num_out_eps;
}

/**
 * Do core a soft reset of the core.  Be careful with this because it
 * resets all the internal state machines of the core.
 */
static void dwc_otg_core_reset(struct core_if *core_if)
{
	struct core_global_regs *global_regs = core_if->core_global_regs;
	union grstctl_data greset = {.d32 = 0};
	int count = 0;

	/* Wait for AHB master IDLE state. */
	do {
		udelay(10);
		greset.d32 = dwc_read_reg32(&global_regs->grstctl);
		if (++count > 100000) {
			printk(KERN_WARNING
				"%s() HANG! AHB Idle GRSTCTL=%0x\n",
				__func__, greset.d32);
			return;
		}
	} while (!greset.b.ahbidle);

	/* Core Soft Reset */
	count = 0;
	greset.b.csftrst = 1;
	dwc_write_reg32(&global_regs->grstctl, greset.d32);

	do {
		greset.d32 = dwc_read_reg32(&global_regs->grstctl);
		if (++count > 10000) {
			printk(KERN_WARNING "%s() HANG! Soft Reset "
				"GRSTCTL=%0x\n", __func__, greset.d32);
			break;
		}
		udelay(1);
	} while (greset.b.csftrst);

	/* Wait for 3 PHY Clocks */
	msleep(100);
}

/**
 * This function initializes the commmon interrupts, used in both
 * device and host modes.
 */
void dwc_otg_enable_common_interrupts(struct core_if *core_if)
{
	struct core_global_regs *global_regs = core_if->core_global_regs;
	union gintmsk_data intr_mask = {.d32 = 0};

	/* Clear any pending OTG Interrupts */
	dwc_write_reg32(&global_regs->gotgint, 0xFFFFFFFF);

	/* Clear any pending interrupts */
	dwc_write_reg32(&global_regs->gintsts, 0xFFFFFFFF);

	/* Enable the interrupts in the GINTMSK. */
	intr_mask.b.modemismatch = 1;
	intr_mask.b.otgintr = 1;
	intr_mask.b.conidstschng = 1;
	intr_mask.b.wkupintr = 1;
	intr_mask.b.disconnect = 1;
	intr_mask.b.usbsuspend = 1;
	intr_mask.b.sessreqintr = 1;
	if (!core_if->dma_enable)
		intr_mask.b.rxstsqlvl = 1;
	dwc_write_reg32(&global_regs->gintmsk, intr_mask.d32);
}

/**
 * This function initializes the DWC_otg controller registers and prepares the
 * core for device mode or host mode operation.
 */
void dwc_otg_core_init(struct core_if *core_if)
{
	u32 i;
	struct core_global_regs *global_regs = core_if->core_global_regs;
	struct device_if *dev_if = core_if->dev_if;
	union gahbcfg_data ahbcfg = {.d32 = 0};
	union gusbcfg_data usbcfg = {.d32 = 0};
	union gi2cctl_data i2cctl = {.d32 = 0};

	/* Common Initialization */
	usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);

	/* Program the ULPI External VBUS bit if needed */
	usbcfg.b.ulpi_ext_vbus_drv = 1;

	/* Set external TS Dline pulsing */
	usbcfg.b.term_sel_dl_pulse = core_if->core_params->ts_dline == 1 ?
						1 : 0;
	dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);

	/* Reset the Controller */
	dwc_otg_core_reset(core_if);

	/* Initialize parameters from Hardware configuration registers. */
	dev_if->num_in_eps = calc_num_in_eps(core_if);
	dev_if->num_out_eps = calc_num_out_eps(core_if);

	for (i = 0; i < core_if->hwcfg4.b.num_dev_perio_in_ep; i++) {
		dev_if->perio_tx_fifo_size[i] =
			dwc_read_reg32(&global_regs->dptxfsiz_dieptxf[i]) >> 16;
	}
	for (i = 0; i < core_if->hwcfg4.b.num_in_eps; i++) {
		dev_if->tx_fifo_size[i] =
			dwc_read_reg32(&global_regs->dptxfsiz_dieptxf[i]) >> 16;
	}

	core_if->total_fifo_size = core_if->hwcfg3.b.dfifo_depth;
	core_if->rx_fifo_size = dwc_read_reg32(&global_regs->grxfsiz);
	core_if->nperio_tx_fifo_size =
			dwc_read_reg32(&global_regs->gnptxfsiz) >> 16;
	/*
	 * This programming sequence needs to happen in FS mode before any
	 * other programming occurs
	 */
	if (core_if->core_params->speed == DWC_SPEED_PARAM_FULL &&
			core_if->core_params->phy_type ==
			DWC_PHY_TYPE_PARAM_FS) {
		/*
		 * core_init() is now called on every switch so only call the
		 * following for the first time through.
		 */
		if (!core_if->phy_init_done) {
			core_if->phy_init_done = 1;
			usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);
			usbcfg.b.physel = 1;
			dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);

			/* Reset after a PHY select */
			dwc_otg_core_reset(core_if);
		}

		/*
		 * Program DCFG.DevSpd or HCFG.FSLSPclkSel to 48Mhz in FS.
		 * Also do this on HNP Dev/Host mode switches (done in dev_init
		 * and host_init).
		 */
		if (dwc_otg_is_host_mode(core_if))
			init_fslspclksel(core_if);
		else
			init_devspd(core_if);

		if (core_if->core_params->i2c_enable) {
			/* Program GUSBCFG.OtgUtmifsSel to I2C */
			usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);
			usbcfg.b.otgutmifssel = 1;
			dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);

			/* Program GI2CCTL.I2CEn */
			i2cctl.d32 = dwc_read_reg32(&global_regs->gi2cctl);
			i2cctl.b.i2cdevaddr = 1;
			i2cctl.b.i2cen = 0;
			dwc_write_reg32(&global_regs->gi2cctl, i2cctl.d32);
			i2cctl.b.i2cen = 1;
			dwc_write_reg32(&global_regs->gi2cctl, i2cctl.d32);
		}
	} else if (!core_if->phy_init_done) {
		/*
		 * High speed PHY. These parameters are preserved during soft
		 * reset so only program them the first time. Do a soft reset
		 * immediately after setting phyif.
		 */
		core_if->phy_init_done = 1;
		usbcfg.b.ulpi_utmi_sel = core_if->core_params->phy_type;
		if (usbcfg.b.ulpi_utmi_sel == 1) {
			/* ULPI interface */
			usbcfg.b.phyif = 0;
			usbcfg.b.ddrsel = core_if->core_params->phy_ulpi_ddr;
		} else {
			/* UTMI+ interface */
			if (core_if->core_params->phy_utmi_width == 16)
				usbcfg.b.phyif = 1;
			else
				usbcfg.b.phyif = 0;
		}
		dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);

		/* Reset after setting the PHY parameters */
		dwc_otg_core_reset(core_if);
	}

	if (core_if->hwcfg2.b.hs_phy_type == 2 &&
			core_if->hwcfg2.b.fs_phy_type == 1 &&
			core_if->core_params->ulpi_fs_ls) {
		usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);
		usbcfg.b.ulpi_fsls = 1;
		usbcfg.b.ulpi_clk_sus_m = 1;
		dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);
	} else {
		usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);
		usbcfg.b.ulpi_fsls = 0;
		usbcfg.b.ulpi_clk_sus_m = 0;
		dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);
	}

	/* Program the GAHBCFG Register. */
	switch (core_if->hwcfg2.b.architecture) {
	case DWC_SLAVE_ONLY_ARCH:
		ahbcfg.b.nptxfemplvl_txfemplvl =
			DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY;
		ahbcfg.b.ptxfemplvl = DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY;
		core_if->dma_enable = 0;
		break;
	case DWC_EXT_DMA_ARCH:
		ahbcfg.b.hburstlen = core_if->core_params->dma_burst_size;
		core_if->dma_enable = (core_if->core_params->dma_enable != 0);
		break;
	case DWC_INT_DMA_ARCH:
		ahbcfg.b.hburstlen = DWC_GAHBCFG_INT_DMA_BURST_INCR;
		core_if->dma_enable = (core_if->core_params->dma_enable != 0);
		break;
	}

	ahbcfg.b.dmaenable = core_if->dma_enable;
	dwc_write_reg32(&global_regs->gahbcfg, ahbcfg.d32);
	core_if->en_multiple_tx_fifo = core_if->hwcfg4.b.ded_fifo_en;

	/* Program the GUSBCFG register. */
	usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);
	switch (core_if->hwcfg2.b.op_mode) {
	case DWC_MODE_HNP_SRP_CAPABLE:
		usbcfg.b.hnpcap = (core_if->core_params->otg_cap ==
			DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE);
		usbcfg.b.srpcap = (core_if->core_params->otg_cap !=
			DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;
	case DWC_MODE_SRP_ONLY_CAPABLE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = (core_if->core_params->otg_cap !=
			DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;
	case DWC_MODE_NO_HNP_SRP_CAPABLE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = 0;
		break;
	case DWC_MODE_SRP_CAPABLE_DEVICE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = (core_if->core_params->otg_cap !=
			DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;
	case DWC_MODE_NO_SRP_CAPABLE_DEVICE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = 0;
		break;
	case DWC_MODE_SRP_CAPABLE_HOST:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = (core_if->core_params->otg_cap !=
			DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;
	case DWC_MODE_NO_SRP_CAPABLE_HOST:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = 0;
		break;
	}
	dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);

	/* Enable common interrupts */
	dwc_otg_enable_common_interrupts(core_if);

	/*
	 * Do device or host intialization based on mode during PCD
	 * and HCD initialization
	 */
	if (dwc_otg_is_host_mode(core_if)) {
		core_if->xceiv->state = OTG_STATE_A_HOST;
	} else {
		core_if->xceiv->state = OTG_STATE_B_PERIPHERAL;
		if (dwc_has_feature(core_if, DWC_DEVICE_ONLY))
			dwc_otg_core_dev_init(core_if);
	}
}

/**
 * This function enables the Device mode interrupts.
 *
 * Note that the bits in the Device IN endpoint mask register are laid out
 * exactly the same as the Device IN endpoint interrupt register.
 */
static void dwc_otg_enable_device_interrupts(struct core_if *core_if)
{
	union gintmsk_data intr_mask = {.d32 = 0};
	union diepint_data msk = {.d32 = 0};
	struct core_global_regs *global_regs = core_if->core_global_regs;

	/* Disable all interrupts. */
	dwc_write_reg32(&global_regs->gintmsk, 0);

	/* Clear any pending interrupts */
	dwc_write_reg32(&global_regs->gintsts, 0xFFFFFFFF);

	/* Enable the common interrupts */
	dwc_otg_enable_common_interrupts(core_if);

	/* Enable interrupts */
	intr_mask.b.usbreset = 1;
	intr_mask.b.enumdone = 1;
	intr_mask.b.inepintr = 1;
	intr_mask.b.outepintr = 1;
	intr_mask.b.erlysuspend = 1;
	if (!core_if->en_multiple_tx_fifo)
		intr_mask.b.epmismatch = 1;

	/* Periodic EP */
	intr_mask.b.isooutdrop = 1;
	intr_mask.b.eopframe = 1;
	intr_mask.b.incomplisoin = 1;
	intr_mask.b.incomplisoout = 1;

	dwc_modify_reg32(&global_regs->gintmsk, intr_mask.d32, intr_mask.d32);

	msk.b.txfifoundrn = 1;
	dwc_modify_reg32(&core_if->dev_if->dev_global_regs->diepmsk,
				msk.d32, msk.d32);
}

/**
 *  Configures the device data fifo sizes when dynamic sizing is enabled.
 */
static void config_dev_dynamic_fifos(struct core_if *core_if)
{
	u32 i;
	struct core_global_regs *regs = core_if->core_global_regs;
	struct core_params *params = core_if->core_params;
	union fifosize_data txsize;
	union fifosize_data nptxsize;
	union fifosize_data ptxsize;

	 /* Rx FIFO */
	dwc_write_reg32(&regs->grxfsiz, params->dev_rx_fifo_size);

	/* Set Periodic and Non-periodic Tx FIFO Mask bits to all 0 */
	core_if->p_tx_msk = 0;
	core_if->tx_msk = 0;

	if (core_if->en_multiple_tx_fifo == 0) {
		/* Non-periodic Tx FIFO */
		nptxsize.b.depth = params->dev_nperio_tx_fifo_size;
		nptxsize.b.startaddr = params->dev_rx_fifo_size;
		dwc_write_reg32(&regs->gnptxfsiz, nptxsize.d32);

		/*
		 * Periodic Tx FIFOs These FIFOs are numbered from 1 to
		 * 15. Indexes of the FIFO size module parameters in the
		 * dev_perio_tx_fifo_size array and the FIFO size
		 * registers in the dptxfsiz array run from 0 to 14.
		 */
		ptxsize.b.startaddr = nptxsize.b.startaddr + nptxsize.b.depth;
		for (i = 0; i < core_if->hwcfg4.b.num_dev_perio_in_ep; i++) {
			ptxsize.b.depth = params->dev_perio_tx_fifo_size[i];
			dwc_write_reg32(&regs->dptxfsiz_dieptxf[i],
						ptxsize.d32);
			ptxsize.b.startaddr += ptxsize.b.depth;
		}
	} else {
		/*
		 * Non-periodic Tx FIFOs These FIFOs are numbered from
		 * 1 to 15. Indexes of the FIFO size module parameters
		 * in the dev_tx_fifo_size array and the FIFO size
		 * registers in the dptxfsiz_dieptxf array run from 0 to
		 * 14.
		 */
		nptxsize.b.depth = params->dev_nperio_tx_fifo_size;
		nptxsize.b.startaddr = params->dev_rx_fifo_size;
		dwc_write_reg32(&regs->gnptxfsiz, nptxsize.d32);

		txsize.b.startaddr = nptxsize.b.startaddr + nptxsize.b.depth;
		for (i = 1; i < core_if->hwcfg4.b.num_dev_perio_in_ep; i++) {
			txsize.b.depth = params->dev_tx_fifo_size[i];
			dwc_write_reg32(&regs->dptxfsiz_dieptxf[i - 1],
						txsize.d32);
			txsize.b.startaddr += txsize.b.depth;
		}
	}
}

/**
 * This function initializes the DWC_otg controller registers for
 * device mode.
 */
void dwc_otg_core_dev_init(struct core_if *c_if)
{
	u32 i;
	struct device_if *d_if = c_if->dev_if;
	struct core_params *params = c_if->core_params;
	union dcfg_data dcfg = {.d32 = 0};
	union grstctl_data resetctl = {.d32 = 0};
	union dthrctl_data dthrctl;

	/* Restart the Phy Clock */
	dwc_write_reg32(c_if->pcgcctl, 0);

	/* Device configuration register */
	init_devspd(c_if);
	dcfg.d32 = dwc_read_reg32(&d_if->dev_global_regs->dcfg);
	dcfg.b.perfrint = DWC_DCFG_FRAME_INTERVAL_80;
	dwc_write_reg32(&d_if->dev_global_regs->dcfg, dcfg.d32);

	/* If needed configure data FIFO sizes */
	if (c_if->hwcfg2.b.dynamic_fifo && params->enable_dynamic_fifo)
		config_dev_dynamic_fifos(c_if);

	/* Flush the FIFOs */
	dwc_otg_flush_tx_fifo(c_if, DWC_GRSTCTL_TXFNUM_ALL);
	dwc_otg_flush_rx_fifo(c_if);

	/* Flush the Learning Queue. */
	resetctl.b.intknqflsh = 1;
	dwc_write_reg32(&c_if->core_global_regs->grstctl, resetctl.d32);

	/* Clear all pending Device Interrupts */
	dwc_write_reg32(&d_if->dev_global_regs->diepmsk, 0);
	dwc_write_reg32(&d_if->dev_global_regs->doepmsk, 0);
	dwc_write_reg32(&d_if->dev_global_regs->daint, 0xFFFFFFFF);
	dwc_write_reg32(&d_if->dev_global_regs->daintmsk, 0);

	for (i = 0; i <= d_if->num_in_eps; i++) {
		union depctl_data depctl;

		depctl.d32 = dwc_read_reg32(&d_if->in_ep_regs[i]->diepctl);
		if (depctl.b.epena) {
			depctl.d32 = 0;
			depctl.b.epdis = 1;
			depctl.b.snak = 1;
		} else {
			depctl.d32 = 0;
		}

		dwc_write_reg32(&d_if->in_ep_regs[i]->diepctl, depctl.d32);
		dwc_write_reg32(&d_if->in_ep_regs[i]->dieptsiz, 0);
		dwc_write_reg32(&d_if->in_ep_regs[i]->diepdma, 0);
		dwc_write_reg32(&d_if->in_ep_regs[i]->diepint, 0xFF);
	}

	for (i = 0; i <= d_if->num_out_eps; i++) {
		union depctl_data depctl;
		depctl.d32 = dwc_read_reg32(&d_if->out_ep_regs[i]->doepctl);
		if (depctl.b.epena) {
			depctl.d32 = 0;
			depctl.b.epdis = 1;
			depctl.b.snak = 1;
		} else {
			depctl.d32 = 0;
		}
		dwc_write_reg32(&d_if->out_ep_regs[i]->doepctl, depctl.d32);
		dwc_write_reg32(&d_if->out_ep_regs[i]->doeptsiz, 0);
		dwc_write_reg32(&d_if->out_ep_regs[i]->doepdma, 0);
		dwc_write_reg32(&d_if->out_ep_regs[i]->doepint, 0xFF);
	}

	if (c_if->en_multiple_tx_fifo && c_if->dma_enable) {
		d_if->non_iso_tx_thr_en = c_if->core_params->thr_ctl & 0x1;
		d_if->iso_tx_thr_en = (c_if->core_params->thr_ctl >> 1) & 0x1;
		d_if->rx_thr_en = (c_if->core_params->thr_ctl >> 2) & 0x1;
		d_if->rx_thr_length = c_if->core_params->rx_thr_length;
		d_if->tx_thr_length = c_if->core_params->tx_thr_length;

		dthrctl.d32 = 0;
		dthrctl.b.non_iso_thr_en = d_if->non_iso_tx_thr_en;
		dthrctl.b.iso_thr_en = d_if->iso_tx_thr_en;
		dthrctl.b.tx_thr_len = d_if->tx_thr_length;
		dthrctl.b.rx_thr_en = d_if->rx_thr_en;
		dthrctl.b.rx_thr_len = d_if->rx_thr_length;
		dwc_write_reg32(&d_if->dev_global_regs->dtknqr3_dthrctl,
					dthrctl.d32);

	}

	dwc_otg_enable_device_interrupts(c_if);
}

/**
 * This function reads a packet from the Rx FIFO into the destination buffer.
 * To read SETUP data use dwc_otg_read_setup_packet.
 */
void dwc_otg_read_packet(struct core_if *core_if, u8 *dest,
				u16 _bytes)
{
	u32 i;
	int word_count = (_bytes + 3) / 4;
	u32 *fifo = core_if->data_fifo[0];
	u32 *data_buff = (u32 *) dest;

	/*
	 * This requires reading data from the FIFO into a u32 temp buffer,
	 * then moving it into the data buffer.
	 */
	for (i = 0; i < word_count; i++, data_buff++)
		*data_buff = dwc_read_datafifo32(fifo);
}

/**
 * Flush a Tx FIFO.
 */
void dwc_otg_flush_tx_fifo(struct core_if *core_if, const int num)
{
	struct core_global_regs *global_regs = core_if->core_global_regs;
	union grstctl_data greset = {.d32 = 0 };
	int count = 0;

	greset.b.txfflsh = 1;
	greset.b.txfnum = num;
	dwc_write_reg32(&global_regs->grstctl, greset.d32);

	do {
		greset.d32 = dwc_read_reg32(&global_regs->grstctl);
		if (++count > 10000) {
			printk(KERN_WARNING "%s() HANG! GRSTCTL=%0x "
				"GNPTXSTS=0x%08x\n", __func__, greset.d32,
				dwc_read_reg32(&global_regs->gnptxsts));
			break;
		}
		udelay(1);
	} while (greset.b.txfflsh == 1);

	/* Wait for 3 PHY Clocks */
	udelay(1);
}

/**
 * Flush Rx FIFO.
 */
void dwc_otg_flush_rx_fifo(struct core_if *core_if)
{
	struct core_global_regs *global_regs = core_if->core_global_regs;
	union grstctl_data greset = {.d32 = 0 };
	int count = 0;

	greset.b.rxfflsh = 1;
	dwc_write_reg32(&global_regs->grstctl, greset.d32);

	do {
		greset.d32 = dwc_read_reg32(&global_regs->grstctl);
		if (++count > 10000) {
			printk(KERN_WARNING "%s() HANG! GRSTCTL=%0x\n",
				__func__, greset.d32);
			break;
		}
		udelay(1);
	} while (greset.b.rxfflsh);

	/* Wait for 3 PHY Clocks */
	udelay(1);
}

/**
 * Register HCD callbacks.
 * The callbacks are used to start and stop the HCD for interrupt processing.
 */
void __devinit dwc_otg_cil_register_hcd_callbacks(struct core_if *c_if,
				struct cil_callbacks *cb, void *p)
{
	c_if->hcd_cb = cb;
	cb->p = p;
}

/**
 * Register PCD callbacks.
 * The callbacks are used to start and stop the PCD for interrupt processing.
 */
void __devinit dwc_otg_cil_register_pcd_callbacks(struct core_if *c_if,
				struct cil_callbacks *cb, void *p)
{
	c_if->pcd_cb = cb;
	cb->p = p;
}

/**
 * This function is called to initialize the DWC_otg CSR data structures.
 *
 * The register addresses in the device and host structures are initialized from
 * the base address supplied by the caller. The calling function must make the
 * OS calls to get the base address of the DWC_otg controller registers.
 *
 * The params argument holds the parameters that specify how the core should be
 * configured.
 */
struct core_if __devinit *dwc_otg_cil_init(const u32 *base,
			struct core_params *params)
{
	struct core_if *core_if = NULL;
	struct device_if *dev_if = NULL;
	struct dwc_host_if *host_if = NULL;
	u8 *reg_base = (u8 *) base;
	u32 offset;
	u32 i;

	core_if = kzalloc(sizeof(*core_if), GFP_KERNEL);
	if (!core_if)
		return NULL;

	core_if->core_params = params;
	core_if->core_global_regs = (struct core_global_regs *) reg_base;

	/* Allocate the Device Mode structures. */
	dev_if = kmalloc(sizeof(*dev_if), GFP_KERNEL);
	if (!dev_if) {
		kfree(core_if);
		return NULL;
	}

	dev_if->dev_global_regs = (struct device_global_regs *) (reg_base +
					DWC_DEV_GLOBAL_REG_OFFSET);

	for (i = 0; i < MAX_EPS_CHANNELS; i++) {
		offset = i * DWC_EP_REG_OFFSET;

		dev_if->in_ep_regs[i] = (struct device_in_ep_regs *)
			(reg_base + DWC_DEV_IN_EP_REG_OFFSET + offset);

		dev_if->out_ep_regs[i] = (struct device_out_ep_regs *)
			(reg_base + DWC_DEV_OUT_EP_REG_OFFSET + offset);
	}

	dev_if->speed = 0;	/* unknown */
	core_if->dev_if = dev_if;

	/* Allocate the Host Mode structures. */
	host_if = kmalloc(sizeof(*host_if), GFP_KERNEL);
	if (!host_if) {
		kfree(dev_if);
		kfree(core_if);
		return NULL;
	}

	host_if->host_global_regs = (struct host_global_regs *)
	    (reg_base + DWC_OTG_HOST_GLOBAL_REG_OFFSET);

	host_if->hprt0 = (u32 *) (reg_base + DWC_OTG_HOST_PORT_REGS_OFFSET);

	for (i = 0; i < MAX_EPS_CHANNELS; i++) {
		offset = i * DWC_OTG_CHAN_REGS_OFFSET;

		host_if->hc_regs[i] = (struct dwc_hc_regs *)
			(reg_base + DWC_OTG_HOST_CHAN_REGS_OFFSET + offset);
	}

	host_if->num_host_channels = MAX_EPS_CHANNELS;
	core_if->host_if = host_if;
	for (i = 0; i < MAX_EPS_CHANNELS; i++) {
		core_if->data_fifo[i] =
			(u32 *) (reg_base + DWC_OTG_DATA_FIFO_OFFSET +
				(i * DWC_OTG_DATA_FIFO_SIZE));
	}
	core_if->pcgcctl = (u32 *) (reg_base + DWC_OTG_PCGCCTL_OFFSET);

	/*
	 * Store the contents of the hardware configuration registers here for
	 * easy access later.
	 */
	core_if->hwcfg1.d32 =
		dwc_read_reg32(&core_if->core_global_regs->ghwcfg1);
	core_if->hwcfg2.d32 =
		dwc_read_reg32(&core_if->core_global_regs->ghwcfg2);

	core_if->hwcfg2.b.architecture = DWC_ARCH;

	core_if->hwcfg3.d32 =
		dwc_read_reg32(&core_if->core_global_regs->ghwcfg3);
	core_if->hwcfg4.d32 =
		dwc_read_reg32(&core_if->core_global_regs->ghwcfg4);

	/* Set the SRP sucess bit for FS-I2c */
	core_if->srp_success = 0;
	core_if->srp_timer_started = 0;
	return core_if;
}

/**
 * This function frees the structures allocated by dwc_otg_cil_init().
 */
void dwc_otg_cil_remove(struct core_if *core_if)
{
	/* Disable all interrupts */
	dwc_modify_reg32(&core_if->core_global_regs->gahbcfg, 1, 0);
	dwc_write_reg32(&core_if->core_global_regs->gintmsk, 0);

	if (core_if) {
		kfree(core_if->dev_if);
		kfree(core_if->host_if);
	}
	kfree(core_if);
}
