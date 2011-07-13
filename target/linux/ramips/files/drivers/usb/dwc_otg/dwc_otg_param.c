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
 * This file provides dwc_otg driver parameter and parameter checking.
 */

#include "dwc_otg_cil.h"

/* Global Debug Level Mask. */
static u32 g_dbg_lvl = 0x0;	/* OFF */

/*
 * Encapsulate the module parameter settings
 */
struct core_params dwc_otg_module_params = {
	.opt = -1,
	.otg_cap = -1,
	.dma_enable = -1,
	.dma_burst_size = -1,
	.speed = -1,
	.host_support_fs_ls_low_power = -1,
	.host_ls_low_power_phy_clk = -1,
	.enable_dynamic_fifo = -1,
	.data_fifo_size = -1,
	.dev_rx_fifo_size = -1,
	.dev_nperio_tx_fifo_size = -1,
	.dev_perio_tx_fifo_size = {
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
	}, /* 15 */
	.host_rx_fifo_size = -1,
	.host_nperio_tx_fifo_size = -1,
	.host_perio_tx_fifo_size = -1,
	.max_transfer_size = -1,
	.max_packet_count = -1,
	.host_channels = -1,
	.dev_endpoints = -1,
	.phy_type = -1,
	.phy_utmi_width = -1,
	.phy_ulpi_ddr = -1,
	.phy_ulpi_ext_vbus = -1,
	.i2c_enable = -1,
	.ulpi_fs_ls = -1,
	.ts_dline = -1,
	.en_multiple_tx_fifo = -1,
	.dev_tx_fifo_size = {
		-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
	}, /* 15 */
	.thr_ctl = -1,
	.tx_thr_length = -1,
	.rx_thr_length = -1,
};

/**
 * Determines if the OTG capability parameter is valid under the current
 * hardware configuration.
 */
static int is_valid_otg_cap(struct core_if *core_if)
{
	int valid = 1;

	switch (dwc_otg_module_params.otg_cap) {
	case DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE:
		if (core_if->hwcfg2.b.op_mode !=
				DWC_HWCFG2_OP_MODE_HNP_SRP_CAPABLE_OTG)
			valid = 0;
		break;
	case DWC_OTG_CAP_PARAM_SRP_ONLY_CAPABLE:
		if (core_if->hwcfg2.b.op_mode !=
				DWC_HWCFG2_OP_MODE_HNP_SRP_CAPABLE_OTG &&
				core_if->hwcfg2.b.op_mode !=
				DWC_HWCFG2_OP_MODE_SRP_ONLY_CAPABLE_OTG &&
				core_if->hwcfg2.b.op_mode !=
				DWC_HWCFG2_OP_MODE_SRP_CAPABLE_DEVICE &&
				core_if->hwcfg2.b.op_mode !=
				DWC_HWCFG2_OP_MODE_SRP_CAPABLE_HOST)
			valid = 0;
		break;
	case DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE:
		/* always valid */
		break;
	}
	return valid;
}

/**
 * Returns a valid OTG capability setting for the current hardware
 * configuration.
 */
static int get_valid_otg_cap(struct core_if *core_if)
{
	if (core_if->hwcfg2.b.op_mode ==
			DWC_HWCFG2_OP_MODE_HNP_SRP_CAPABLE_OTG ||
			core_if->hwcfg2.b.op_mode ==
			DWC_HWCFG2_OP_MODE_SRP_ONLY_CAPABLE_OTG ||
			core_if->hwcfg2.b.op_mode ==
			DWC_HWCFG2_OP_MODE_SRP_CAPABLE_DEVICE ||
			core_if->hwcfg2.b.op_mode ==
			DWC_HWCFG2_OP_MODE_SRP_CAPABLE_HOST)
		return DWC_OTG_CAP_PARAM_SRP_ONLY_CAPABLE;
	else
		return DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE;
}

/**
 * Checks that, if the user set a periodic Tx FIFO size parameter, the size is
 * within the valid range.  If not, the parameter is set to the default.
 */
static int chk_param_perio_tx_fifo_sizes(int retval)
{
	u32 *s = &dwc_otg_module_params.dev_perio_tx_fifo_size[0];
	u32 i;

	for (i = 0; i < MAX_PERIO_FIFOS; i++, s++) {
		if (*s != -1 && (*s < 4 || *s > 768)) {
			printk(KERN_ERR "`%d' invalid for parameter "
				"`dev_perio_tx_fifo_size_%d'\n", *s, i);

			*s = dwc_param_dev_perio_tx_fifo_size_default;
			retval++;
		}
	}
	return retval;
}

/**
 * Checks that, if the user set a Tx FIFO size parameter, the size is within the
 * valid range.  If not, the parameter is set to the default.
 */
static int chk_param_tx_fifo_sizes(int retval)
{
	u32 *s = &dwc_otg_module_params.dev_tx_fifo_size[0];
	u32 i;

	for (i = 0; i < MAX_TX_FIFOS; i++, s++) {
		if (*s != -1 && (*s < 4 || *s > 768)) {
			printk(KERN_ERR "`%d' invalid for parameter "
				"`dev_perio_tx_fifo_size_%d'\n", *s, i);

			*s = dwc_param_dev_tx_fifo_size_default;
			retval++;
		}
	}
	return retval;
}

/**
 * Checks that parameter settings for the periodic Tx FIFO sizes are correct
 * according to the hardware configuration. Sets the size to the hardware
 * configuration if an incorrect size is detected.
 */
static int chk_valid_perio_tx_fifo_sizes(struct core_if *core_if, int retval)
{
	struct core_global_regs *regs = core_if->core_global_regs;
	u32 *param_size = &dwc_otg_module_params.dev_perio_tx_fifo_size[0];
	u32 i;

	for (i = 0; i < MAX_PERIO_FIFOS; i++, param_size++) {
		int changed = 1;
		int error = 0;
		u32 size;

		if (*param_size == -1) {
			changed = 0;
			*param_size = dwc_param_dev_perio_tx_fifo_size_default;
		}

		size = dwc_read_reg32(&regs->dptxfsiz_dieptxf[i]);
		if (*param_size > size) {
			if (changed) {
				printk(KERN_ERR "%d' invalid for parameter "
					"`dev_perio_tx_fifo_size_%d'. Check HW "
					"configuration.\n", *param_size, i);
				error = 1;
			}
			*param_size = size;
		}
		retval += error;
	}
	return retval;
}

/**
 * Checks that parameter settings for the Tx FIFO sizes are correct according to
 * the hardware configuration.  Sets the size to the hardware configuration if
 * an incorrect size is detected.
 */
static int chk_valid_tx_fifo_sizes(struct core_if *core_if, int retval)
{
	struct core_global_regs *regs = core_if->core_global_regs;
	u32 *param_size = &dwc_otg_module_params.dev_tx_fifo_size[0];
	u32 i;

	for (i = 0; i < MAX_TX_FIFOS; i++, param_size) {
		int changed = 1;
		int error = 0;
		u32 size;

		if (*param_size == -1) {
			changed = 0;
			*param_size = dwc_param_dev_tx_fifo_size_default;
		}

		size = dwc_read_reg32(&regs->dptxfsiz_dieptxf[i]);
		if (*param_size > size) {
			if (changed) {
				printk(KERN_ERR "%d' invalid for parameter "
					"`dev_tx_fifo_size_%d'. Check HW "
					"configuration.\n", *param_size, i);
				error = 1;
			}
			*param_size = size;
		}
		retval += error;
	}
	return retval;
}

/**
 * This function is called during module intialization to verify that
 * the module parameters are in a valid state.
 */
int __devinit check_parameters(struct core_if *core_if)
{
	int retval = 0;

	/* Checks if the parameter is outside of its valid range of values */
#define DWC_OTG_PARAM_TEST(_param_, _low_, _high_) \
	((dwc_otg_module_params._param_ < (_low_)) || \
	 (dwc_otg_module_params._param_ > (_high_)))

	/*
	 * If the parameter has been set by the user, check that the parameter
	 * value is within the value range of values.  If not, report a module
	 * error.
	 */
#define DWC_OTG_PARAM_ERR(_param_, _low_, _high_, _string_) \
	do { \
		if (dwc_otg_module_params._param_ != -1) { \
			if (DWC_OTG_PARAM_TEST(_param_, (_low_), (_high_))) { \
				printk(KERN_ERR "`%d' invalid for parameter " \
						"`%s'\n", \
						dwc_otg_module_params._param_, \
						_string_); \
				dwc_otg_module_params._param_ = \
					dwc_param_##_param_##_default; \
				retval++; \
			} \
		} \
	} while (0)

	DWC_OTG_PARAM_ERR(opt, 0, 1, "opt");
	DWC_OTG_PARAM_ERR(otg_cap, 0, 2, "otg_cap");
	DWC_OTG_PARAM_ERR(dma_enable, 0, 1, "dma_enable");
	DWC_OTG_PARAM_ERR(speed, 0, 1, "speed");

	DWC_OTG_PARAM_ERR(host_support_fs_ls_low_power, 0, 1,
				"host_support_fs_ls_low_power");
	DWC_OTG_PARAM_ERR(host_ls_low_power_phy_clk, 0, 1,
				"host_ls_low_power_phy_clk");

	DWC_OTG_PARAM_ERR(enable_dynamic_fifo, 0, 1, "enable_dynamic_fifo");
	DWC_OTG_PARAM_ERR(data_fifo_size, 32, 32768, "data_fifo_size");
	DWC_OTG_PARAM_ERR(dev_rx_fifo_size, 16, 32768, "dev_rx_fifo_size");
	DWC_OTG_PARAM_ERR(dev_nperio_tx_fifo_size, 16, 32768,
				"dev_nperio_tx_fifo_size");
	DWC_OTG_PARAM_ERR(host_rx_fifo_size, 16, 32768, "host_rx_fifo_size");
	DWC_OTG_PARAM_ERR(host_nperio_tx_fifo_size, 16, 32768,
				"host_nperio_tx_fifo_size");
	DWC_OTG_PARAM_ERR(host_perio_tx_fifo_size, 16, 32768,
				"host_perio_tx_fifo_size");

	DWC_OTG_PARAM_ERR(max_transfer_size, 2047, 524288,
				"max_transfer_size");
	DWC_OTG_PARAM_ERR(max_packet_count, 15, 511, "max_packet_count");

	DWC_OTG_PARAM_ERR(host_channels, 1, 16, "host_channels");
	DWC_OTG_PARAM_ERR(dev_endpoints, 1, 15, "dev_endpoints");

	DWC_OTG_PARAM_ERR(phy_type, 0, 2, "phy_type");
	DWC_OTG_PARAM_ERR(phy_ulpi_ddr, 0, 1, "phy_ulpi_ddr");
	DWC_OTG_PARAM_ERR(phy_ulpi_ext_vbus, 0, 1, "phy_ulpi_ext_vbus");
	DWC_OTG_PARAM_ERR(i2c_enable, 0, 1, "i2c_enable");
	DWC_OTG_PARAM_ERR(ulpi_fs_ls, 0, 1, "ulpi_fs_ls");
	DWC_OTG_PARAM_ERR(ts_dline, 0, 1, "ts_dline");

	if (dwc_otg_module_params.dma_burst_size != -1) {
		if (DWC_OTG_PARAM_TEST(dma_burst_size, 1, 1) &&
				DWC_OTG_PARAM_TEST(dma_burst_size, 4, 4) &&
				DWC_OTG_PARAM_TEST(dma_burst_size, 8, 8) &&
				DWC_OTG_PARAM_TEST(dma_burst_size, 16, 16) &&
				DWC_OTG_PARAM_TEST(dma_burst_size, 32, 32) &&
				DWC_OTG_PARAM_TEST(dma_burst_size, 64, 64) &&
				DWC_OTG_PARAM_TEST(dma_burst_size, 128, 128) &&
				DWC_OTG_PARAM_TEST(dma_burst_size, 256, 256)) {
			printk(KERN_ERR "`%d' invalid for parameter "
					"`dma_burst_size'\n",
					dwc_otg_module_params.dma_burst_size);
			dwc_otg_module_params.dma_burst_size = 32;
			retval++;
		}
	}

	if (dwc_otg_module_params.phy_utmi_width != -1) {
		if (DWC_OTG_PARAM_TEST(phy_utmi_width, 8, 8) &&
				DWC_OTG_PARAM_TEST(phy_utmi_width, 16, 16)) {
			printk(KERN_ERR "`%d'invalid for parameter "
					"`phy_utmi_width'\n",
					dwc_otg_module_params.phy_utmi_width);
			dwc_otg_module_params.phy_utmi_width = 8;
			retval++;
		}
	}

	DWC_OTG_PARAM_ERR(en_multiple_tx_fifo, 0, 1, "en_multiple_tx_fifo");
	retval += chk_param_perio_tx_fifo_sizes(retval);
	retval += chk_param_tx_fifo_sizes(retval);

	DWC_OTG_PARAM_ERR(thr_ctl, 0, 7, "thr_ctl");
	DWC_OTG_PARAM_ERR(tx_thr_length, 8, 128, "tx_thr_length");
	DWC_OTG_PARAM_ERR(rx_thr_length, 8, 128, "rx_thr_length");

	/*
	 * At this point, all module parameters that have been set by the user
	 * are valid, and those that have not are left unset.  Now set their
	 * default values and/or check the parameters against the hardware
	 * configurations of the OTG core.
	 */

	/*
	 * This sets the parameter to the default value if it has not been set
	 * by the user
	 */
#define DWC_OTG_PARAM_SET_DEFAULT(_param_) ({ \
		int changed = 1; \
		if (dwc_otg_module_params._param_ == -1) { \
			changed = 0; \
			dwc_otg_module_params._param_ = \
				dwc_param_##_param_##_default; \
		} \
		changed; \
	 })

	/*
	 * This checks the macro against the hardware configuration to see if it
	 * is valid.  It is possible that the default value could be invalid.
	 * In this case, it will report a module error if the user touched the
	 * parameter. Otherwise it will adjust the value without any error.
	 */
#define DWC_OTG_PARAM_CHECK_VALID(_param_, _str_, _is_valid_ , _set_valid_) ({ \
		int changed = DWC_OTG_PARAM_SET_DEFAULT(_param_); \
		int error = 0; \
		if (!(_is_valid_)) { \
			if (changed) { \
				printk(KERN_ERR "`%d' invalid for parameter " \
					"`%s' Check HW configuration.\n", \
					dwc_otg_module_params._param_, \
					_str_); \
				error = 1; \
			} \
			dwc_otg_module_params._param_ = (_set_valid_); \
		} \
		error; \
	})

	/* OTG Cap */
	retval += DWC_OTG_PARAM_CHECK_VALID(otg_cap, "otg_cap",
			is_valid_otg_cap(core_if), get_valid_otg_cap(core_if));

	retval += DWC_OTG_PARAM_CHECK_VALID(dma_enable, "dma_enable",
			((dwc_otg_module_params.dma_enable == 1) &&
			 (core_if->hwcfg2.b.architecture == 0)) ? 0 : 1, 0);
	retval += DWC_OTG_PARAM_CHECK_VALID(opt, "opt", 1, 0);
	DWC_OTG_PARAM_SET_DEFAULT(dma_burst_size);
	retval += DWC_OTG_PARAM_CHECK_VALID(host_support_fs_ls_low_power,
			"host_support_fs_ls_low_power", 1, 0);
	retval += DWC_OTG_PARAM_CHECK_VALID(enable_dynamic_fifo,
			"enable_dynamic_fifo",
			((dwc_otg_module_params.enable_dynamic_fifo == 0) ||
			 (core_if->hwcfg2.b.dynamic_fifo == 1)), 0);
	retval += DWC_OTG_PARAM_CHECK_VALID(data_fifo_size, "data_fifo_size",
			(dwc_otg_module_params.data_fifo_size <=
			 core_if->hwcfg3.b.dfifo_depth),
			core_if->hwcfg3.b.dfifo_depth);
	retval += DWC_OTG_PARAM_CHECK_VALID(dev_rx_fifo_size,
			"dev_rx_fifo_size",
			(dwc_otg_module_params.dev_rx_fifo_size <=
			 dwc_read_reg32(&core_if->core_global_regs->grxfsiz)),
			dwc_read_reg32(&core_if->core_global_regs->grxfsiz));
	retval += DWC_OTG_PARAM_CHECK_VALID(dev_nperio_tx_fifo_size,
			"dev_nperio_tx_fifo_size",
			(dwc_otg_module_params.dev_nperio_tx_fifo_size <=
			(dwc_read_reg32(&core_if->core_global_regs->gnptxfsiz)
				>> 16)),
			(dwc_read_reg32(&core_if->core_global_regs->gnptxfsiz)
				>> 16));
	retval += DWC_OTG_PARAM_CHECK_VALID(host_rx_fifo_size,
			"host_rx_fifo_size",
			(dwc_otg_module_params.host_rx_fifo_size <=
			dwc_read_reg32(&core_if->core_global_regs->grxfsiz)),
			dwc_read_reg32(&core_if->core_global_regs->grxfsiz));
	retval += DWC_OTG_PARAM_CHECK_VALID(host_nperio_tx_fifo_size,
			"host_nperio_tx_fifo_size",
			(dwc_otg_module_params.host_nperio_tx_fifo_size <=
			(dwc_read_reg32(&core_if->core_global_regs->gnptxfsiz)
					>> 16)),
			(dwc_read_reg32(&core_if->core_global_regs->gnptxfsiz)
					>> 16));
	retval += DWC_OTG_PARAM_CHECK_VALID(host_perio_tx_fifo_size,
			"host_perio_tx_fifo_size",
			(dwc_otg_module_params.host_perio_tx_fifo_size <=
			((dwc_read_reg32(&core_if->core_global_regs->hptxfsiz)
					>> 16))),
			((dwc_read_reg32(&core_if->core_global_regs->hptxfsiz)
					>> 16)));
	retval += DWC_OTG_PARAM_CHECK_VALID(max_transfer_size,
			"max_transfer_size",
			(dwc_otg_module_params.max_transfer_size <
			(1 << (core_if->hwcfg3.b.xfer_size_cntr_width + 11))),
			((1 << (core_if->hwcfg3.b.xfer_size_cntr_width + 11))
					- 1));
	retval += DWC_OTG_PARAM_CHECK_VALID(max_packet_count,
			"max_packet_count",
			(dwc_otg_module_params.max_packet_count <
			(1 << (core_if->hwcfg3.b.packet_size_cntr_width + 4))),
			((1 << (core_if->hwcfg3.b.packet_size_cntr_width + 4))
					- 1));
	retval += DWC_OTG_PARAM_CHECK_VALID(host_channels, "host_channels",
			(dwc_otg_module_params.host_channels <=
			(core_if->hwcfg2.b.num_host_chan + 1)),
			(core_if->hwcfg2.b.num_host_chan + 1));
	retval += DWC_OTG_PARAM_CHECK_VALID(dev_endpoints, "dev_endpoints",
			(dwc_otg_module_params.dev_endpoints <=
			(core_if->hwcfg2.b.num_dev_ep)),
			core_if->hwcfg2.b.num_dev_ep);

	retval += DWC_OTG_PARAM_CHECK_VALID(phy_type, "phy_type", 1, 0);
	retval += DWC_OTG_PARAM_CHECK_VALID(speed, "speed",
			(dwc_otg_module_params.speed == 0) &&
			(dwc_otg_module_params.phy_type ==
			 DWC_PHY_TYPE_PARAM_FS) ? 0 : 1,
			dwc_otg_module_params.phy_type ==
			DWC_PHY_TYPE_PARAM_FS ? 1 : 0);
	retval += DWC_OTG_PARAM_CHECK_VALID(host_ls_low_power_phy_clk,
			"host_ls_low_power_phy_clk",
			((dwc_otg_module_params.host_ls_low_power_phy_clk ==
			DWC_HOST_LS_LOW_POWER_PHY_CLK_PARAM_48MHZ) &&
			(dwc_otg_module_params.phy_type ==
				DWC_PHY_TYPE_PARAM_FS) ? 0 : 1),
			((dwc_otg_module_params.phy_type ==
				DWC_PHY_TYPE_PARAM_FS) ?
				DWC_HOST_LS_LOW_POWER_PHY_CLK_PARAM_6MHZ :
				DWC_HOST_LS_LOW_POWER_PHY_CLK_PARAM_48MHZ));

	DWC_OTG_PARAM_SET_DEFAULT(phy_ulpi_ddr);
	DWC_OTG_PARAM_SET_DEFAULT(phy_ulpi_ext_vbus);
	DWC_OTG_PARAM_SET_DEFAULT(phy_utmi_width);
	DWC_OTG_PARAM_SET_DEFAULT(ulpi_fs_ls);
	DWC_OTG_PARAM_SET_DEFAULT(ts_dline);

	retval += DWC_OTG_PARAM_CHECK_VALID(i2c_enable, "i2c_enable", 1, 0);

	retval += DWC_OTG_PARAM_CHECK_VALID(en_multiple_tx_fifo,
			"en_multiple_tx_fifo",
			((dwc_otg_module_params.en_multiple_tx_fifo == 1) &&
			(core_if->hwcfg4.b.ded_fifo_en == 0)) ? 0 : 1, 0);

	retval += chk_valid_perio_tx_fifo_sizes(core_if, retval);
	retval += chk_valid_tx_fifo_sizes(core_if, retval);

	DWC_OTG_PARAM_SET_DEFAULT(thr_ctl);
	DWC_OTG_PARAM_SET_DEFAULT(tx_thr_length);
	DWC_OTG_PARAM_SET_DEFAULT(rx_thr_length);

	return retval;
}

module_param_named(otg_cap, dwc_otg_module_params.otg_cap, int, 0444);
MODULE_PARM_DESC(otg_cap, "OTG Capabilities 0=HNP&SRP 1=SRP Only 2=None");
module_param_named(opt, dwc_otg_module_params.opt, int, 0444);
MODULE_PARM_DESC(opt, "OPT Mode");
module_param_named(dma_enable, dwc_otg_module_params.dma_enable, int, 0444);
MODULE_PARM_DESC(dma_enable, "DMA Mode 0=Slave 1=DMA enabled");
module_param_named(dma_burst_size, dwc_otg_module_params.dma_burst_size,
			int, 0444);
MODULE_PARM_DESC(dma_burst_size, "DMA Burst Size 1, 4, 8, 16, 32, 64, "
				"128, 256");
module_param_named(speed, dwc_otg_module_params.speed, int, 0444);
MODULE_PARM_DESC(speed, "Speed 0=High Speed 1=Full Speed");
module_param_named(host_support_fs_ls_low_power,
			dwc_otg_module_params.host_support_fs_ls_low_power,
			int, 0444);
MODULE_PARM_DESC(host_support_fs_ls_low_power, "Support Low Power w/FS or LS "
				"0=Support 1=Don't Support");
module_param_named(host_ls_low_power_phy_clk,
			dwc_otg_module_params.host_ls_low_power_phy_clk,
			int, 0444);
MODULE_PARM_DESC(host_ls_low_power_phy_clk, "Low Speed Low Power Clock "
				"0=48Mhz 1=6Mhz");
module_param_named(enable_dynamic_fifo,
			dwc_otg_module_params.enable_dynamic_fifo, int, 0444);
MODULE_PARM_DESC(enable_dynamic_fifo, "0=cC Setting 1=Allow Dynamic Sizing");
module_param_named(data_fifo_size,
			dwc_otg_module_params.data_fifo_size, int, 0444);
MODULE_PARM_DESC(data_fifo_size, "Total number of words in the data FIFO "
				"memory 32-32768");
module_param_named(dev_rx_fifo_size, dwc_otg_module_params.dev_rx_fifo_size,
			int, 0444);
MODULE_PARM_DESC(dev_rx_fifo_size, "Number of words in the Rx FIFO 16-32768");
module_param_named(dev_nperio_tx_fifo_size,
			dwc_otg_module_params.dev_nperio_tx_fifo_size,
			int, 0444);
MODULE_PARM_DESC(dev_nperio_tx_fifo_size, "Number of words in the non-periodic "
				"Tx FIFO 16-32768");
module_param_named(dev_perio_tx_fifo_size_1,
			dwc_otg_module_params.dev_perio_tx_fifo_size[0],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_1, "Number of words in the periodic "
				"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_2,
			dwc_otg_module_params.dev_perio_tx_fifo_size[1],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_2, "Number of words in the periodic "
				"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_3,
			dwc_otg_module_params.dev_perio_tx_fifo_size[2],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_3, "Number of words in the periodic "
				"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_4,
			dwc_otg_module_params.dev_perio_tx_fifo_size[3],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_4, "Number of words in the periodic "
				"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_5,
			dwc_otg_module_params.dev_perio_tx_fifo_size[4],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_5, "Number of words in the periodic "
				"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_6,
			dwc_otg_module_params.dev_perio_tx_fifo_size[5],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_6, "Number of words in the periodic "
				"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_7,
			dwc_otg_module_params.dev_perio_tx_fifo_size[6],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_7, "Number of words in the periodic "
				"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_8,
			dwc_otg_module_params.dev_perio_tx_fifo_size[7],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_8, "Number of words in the periodic "
				"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_9,
			dwc_otg_module_params.dev_perio_tx_fifo_size[8],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_9, "Number of words in the periodic "
				"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_10,
			dwc_otg_module_params.dev_perio_tx_fifo_size[9],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_10, "Number of words in the periodic "
				"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_11,
			dwc_otg_module_params.dev_perio_tx_fifo_size[10],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_11, "Number of words in the periodic "
			"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_12,
			dwc_otg_module_params.dev_perio_tx_fifo_size[11],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_12, "Number of words in the periodic "
			"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_13,
			dwc_otg_module_params.dev_perio_tx_fifo_size[12],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_13, "Number of words in the periodic "
			"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_14,
			dwc_otg_module_params.dev_perio_tx_fifo_size[13],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_14, "Number of words in the periodic "
			"Tx FIFO 4-768");
module_param_named(dev_perio_tx_fifo_size_15,
			dwc_otg_module_params.dev_perio_tx_fifo_size[14],
			int, 0444);
MODULE_PARM_DESC(dev_perio_tx_fifo_size_15, "Number of words in the periodic "
			"Tx FIFO 4-768");
module_param_named(host_rx_fifo_size, dwc_otg_module_params.host_rx_fifo_size,
			int, 0444);
MODULE_PARM_DESC(host_rx_fifo_size, "Number of words in the Rx FIFO 16-32768");
module_param_named(host_nperio_tx_fifo_size,
			dwc_otg_module_params.host_nperio_tx_fifo_size,
			int, 0444);
MODULE_PARM_DESC(host_nperio_tx_fifo_size, "Number of words in the "
			"non-periodic Tx FIFO 16-32768");
module_param_named(host_perio_tx_fifo_size,
			dwc_otg_module_params.host_perio_tx_fifo_size,
			int, 0444);
MODULE_PARM_DESC(host_perio_tx_fifo_size, "Number of words in the host "
			"periodic Tx FIFO 16-32768");
module_param_named(max_transfer_size, dwc_otg_module_params.max_transfer_size,
			int, 0444);

MODULE_PARM_DESC(max_transfer_size, "The maximum transfer size supported in "
			"bytes 2047-65535");
module_param_named(max_packet_count, dwc_otg_module_params.max_packet_count,
			int, 0444);
MODULE_PARM_DESC(max_packet_count, "The maximum number of packets in a "
			"transfer 15-511");
module_param_named(host_channels, dwc_otg_module_params.host_channels,
			int, 0444);
MODULE_PARM_DESC(host_channels,	"The number of host channel registers to "
			"use 1-16");
module_param_named(dev_endpoints, dwc_otg_module_params.dev_endpoints,
			int, 0444);
MODULE_PARM_DESC(dev_endpoints,	"The number of endpoints in addition to EP0 "
			"available for device mode 1-15");
module_param_named(phy_type, dwc_otg_module_params.phy_type, int, 0444);
MODULE_PARM_DESC(phy_type, "0=Reserved 1=UTMI+ 2=ULPI");
module_param_named(phy_utmi_width, dwc_otg_module_params.phy_utmi_width,
			int, 0444);
MODULE_PARM_DESC(phy_utmi_width, "Specifies the UTMI+ Data Width 8 or 16 bits");
module_param_named(phy_ulpi_ddr, dwc_otg_module_params.phy_ulpi_ddr,
			int, 0444);
MODULE_PARM_DESC(phy_ulpi_ddr, "0");
module_param_named(phy_ulpi_ext_vbus, dwc_otg_module_params.phy_ulpi_ext_vbus,
			int, 0444);
MODULE_PARM_DESC(phy_ulpi_ext_vbus,
		  "ULPI PHY using internal or external vbus 0=Internal");
module_param_named(i2c_enable, dwc_otg_module_params.i2c_enable, int, 0444);
MODULE_PARM_DESC(i2c_enable, "FS PHY Interface");
module_param_named(ulpi_fs_ls, dwc_otg_module_params.ulpi_fs_ls, int, 0444);
MODULE_PARM_DESC(ulpi_fs_ls, "ULPI PHY FS/LS mode only");
module_param_named(ts_dline, dwc_otg_module_params.ts_dline, int, 0444);
MODULE_PARM_DESC(ts_dline, "Term select Dline pulsing for all PHYs");
module_param_named(debug, g_dbg_lvl, int, 0444);
MODULE_PARM_DESC(debug, "0");
module_param_named(en_multiple_tx_fifo,
			dwc_otg_module_params.en_multiple_tx_fifo, int, 0444);
MODULE_PARM_DESC(en_multiple_tx_fifo, "Dedicated Non Periodic Tx FIFOs "
			"0=disabled 1=enabled");
module_param_named(dev_tx_fifo_size_1,
		    dwc_otg_module_params.dev_tx_fifo_size[0], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_1, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_2,
			dwc_otg_module_params.dev_tx_fifo_size[1], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_2, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_3,
			dwc_otg_module_params.dev_tx_fifo_size[2], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_3, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_4,
			dwc_otg_module_params.dev_tx_fifo_size[3], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_4, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_5,
			dwc_otg_module_params.dev_tx_fifo_size[4], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_5, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_6,
			dwc_otg_module_params.dev_tx_fifo_size[5], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_6, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_7,
			dwc_otg_module_params.dev_tx_fifo_size[6], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_7, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_8,
			dwc_otg_module_params.dev_tx_fifo_size[7], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_8, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_9,
			dwc_otg_module_params.dev_tx_fifo_size[8], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_9, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_10,
			dwc_otg_module_params.dev_tx_fifo_size[9], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_10, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_11,
			dwc_otg_module_params.dev_tx_fifo_size[10], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_11, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_12,
			dwc_otg_module_params.dev_tx_fifo_size[11], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_12, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_13,
			dwc_otg_module_params.dev_tx_fifo_size[12], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_13, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_14,
			dwc_otg_module_params.dev_tx_fifo_size[13], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_14, "Number of words in the Tx FIFO 4-768");
module_param_named(dev_tx_fifo_size_15,
			dwc_otg_module_params.dev_tx_fifo_size[14], int, 0444);
MODULE_PARM_DESC(dev_tx_fifo_size_15, "Number of words in the Tx FIFO 4-768");
module_param_named(thr_ctl, dwc_otg_module_params.thr_ctl, int, 0444);
MODULE_PARM_DESC(thr_ctl, "Thresholding enable flag bit 0 - non ISO Tx thr., "
			"1 - ISO Tx thr., 2 - Rx thr.- bit "
			"0=disabled 1=enabled");
module_param_named(tx_thr_length, dwc_otg_module_params.tx_thr_length,
			int, 0444);
MODULE_PARM_DESC(tx_thr_length, "Tx Threshold length in 32 bit DWORDs");
module_param_named(rx_thr_length, dwc_otg_module_params.rx_thr_length,
			int, 0444);
MODULE_PARM_DESC(rx_thr_length, "Rx Threshold length in 32 bit DWORDs");
