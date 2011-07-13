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

#ifndef __DWC_OTG_REGS_H__
#define __DWC_OTG_REGS_H__

#include <linux/types.h>

/*
 * This file contains the data structures for accessing the DWC_otg core
 * registers.
 *
 * The application interfaces with the HS OTG core by reading from and
 * writing to the Control and Status Register (CSR) space through the
 * AHB Slave interface. These registers are 32 bits wide, and the
 * addresses are 32-bit-block aligned.
 * CSRs are classified as follows:
 * - Core Global Registers
 * - Device Mode Registers
 * - Device Global Registers
 * - Device Endpoint Specific Registers
 * - Host Mode Registers
 * - Host Global Registers
 * - Host Port CSRs
 * - Host Channel Specific Registers
 *
 * Only the Core Global registers can be accessed in both Device and
 * Host modes. When the HS OTG core is operating in one mode, either
 * Device or Host, the application must not access registers from the
 * other mode. When the core switches from one mode to another, the
 * registers in the new mode of operation must be reprogrammed as they
 * would be after a power-on reset.
 */

/*
 * DWC_otg Core registers.  The core_global_regs structure defines the
 * size and relative field offsets for the Core Global registers.
 */
struct core_global_regs {
	/* OTG Control and Status Register.		Offset: 000h */
	u32 gotgctl;
	/* OTG Interrupt Register.			Offset: 004h */
	u32 gotgint;
	/* Core AHB Configuration Register.		Offset: 008h */
	u32 gahbcfg;

#define DWC_GLBINTRMASK				0x0001
#define DWC_DMAENABLE				0x0020
#define DWC_NPTXEMPTYLVL_EMPTY			0x0080
#define DWC_NPTXEMPTYLVL_HALFEMPTY		0x0000
#define DWC_PTXEMPTYLVL_EMPTY			0x0100
#define DWC_PTXEMPTYLVL_HALFEMPTY		0x0000

	/* Core USB Configuration Register.		Offset: 00Ch */
	u32 gusbcfg;
	/* Core Reset Register.				Offset: 010h */
	u32 grstctl;
	/* Core Interrupt Register.			Offset: 014h */
	u32 gintsts;
	/* Core Interrupt Mask Register.		Offset: 018h */
	u32 gintmsk;
	/*
	 * Receive Status Queue Read Register
	 * (Read Only)					Offset: 01Ch
	 */
	u32 grxstsr;
	/*
	 * Receive Status Queue Read & POP Register
	 * (Read Only)					Offset: 020h
	 */
	u32 grxstsp;
	/* Receive FIFO Size Register.			Offset: 024h */
	u32 grxfsiz;
	/* Non Periodic Transmit FIFO Size Register.	Offset: 028h */
	u32 gnptxfsiz;
	/*
	 * Non Periodic Transmit FIFO/Queue Status Register
	 * (Read Only).					Offset: 02Ch
	 */
	u32 gnptxsts;
	/* I2C Access Register.				Offset: 030h */
	u32 gi2cctl;
	/* PHY Vendor Control Register.			Offset: 034h */
	u32 gpvndctl;
	/* General Purpose Input/Output Register.	Offset: 038h */
	u32 ggpio;
	/* User ID Register.				Offset: 03Ch */
	u32 guid;
	/* Synopsys ID Register (Read Only).		Offset: 040h */
	u32 gsnpsid;
	/* User HW Config1 Register (Read Only).	Offset: 044h */
	u32 ghwcfg1;
	/* User HW Config2 Register (Read Only).	Offset: 048h */
	u32 ghwcfg2;
#define DWC_SLAVE_ONLY_ARCH			0
#define DWC_EXT_DMA_ARCH			1
#define DWC_INT_DMA_ARCH			2

#define DWC_MODE_HNP_SRP_CAPABLE		0
#define DWC_MODE_SRP_ONLY_CAPABLE		1
#define DWC_MODE_NO_HNP_SRP_CAPABLE		2
#define DWC_MODE_SRP_CAPABLE_DEVICE		3
#define DWC_MODE_NO_SRP_CAPABLE_DEVICE		4
#define DWC_MODE_SRP_CAPABLE_HOST		5
#define DWC_MODE_NO_SRP_CAPABLE_HOST		6

	/* User HW Config3 Register (Read Only).	Offset: 04Ch */
	u32 ghwcfg3;
	/* User HW Config4 Register (Read Only).	Offset: 050h */
	u32 ghwcfg4;
	/*  Reserved					Offset: 054h-0FFh */
	u32 reserved[43];
	/*  Host Periodic Transmit FIFO Size Register.	Offset: 100h */
	u32 hptxfsiz;

	/*
	 * Device Periodic Transmit FIFO#n Register, if dedicated fifos are
	 * disabled.  Otherwise Device Transmit FIFO#n Register.
	 *
	 * Offset: 104h + (FIFO_Number-1)*04h, 1 <= FIFO Number <= 15 (1<=n<=15)
	 */
	u32 dptxfsiz_dieptxf[15];
};


#ifndef CONFIG_DWC_OTG_REG_LE
/*
 * This union represents the bit fields of the Core OTG Controland Status
 * Register (GOTGCTL).  Set the bits using the bit fields then write the d32
 * value to the register.
 */
union gotgctl_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved31_21:11;
		unsigned currmod:1;
		unsigned bsesvld:1;
		unsigned asesvld:1;
		unsigned reserved17:1;
		unsigned conidsts:1;
		unsigned reserved1_12:4;
		unsigned devhnpen:1;
		unsigned hstsethnpen:1;
		unsigned hnpreq:1;
		unsigned hstnegscs:1;
		unsigned reserved07_02:6;
		unsigned sesreq:1;
		unsigned sesreqscs:1;
	} b;
};

/*
 * This union represents the bit fields of the Core OTG Interrupt Register
 * (GOTGINT).  Set/clear the bits using the bit fields then write the d32
 * value to the register.
 */
union gotgint_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		/* Current Mode */
		unsigned reserved31_20:12;
		/* Debounce Done */
		unsigned debdone:1;
		/* A-Device Timeout Change */
		unsigned adevtoutchng:1;
		/* Host Negotiation Detected */
		unsigned hstnegdet:1;
		unsigned reserver16_10:7;
		/* Host Negotiation Success Status Change */
		unsigned hstnegsucstschng:1;
		/* Session Request Success Status Change */
		unsigned sesreqsucstschng:1;
		unsigned reserved3_7:5;
		/* Session End Detected */
		unsigned sesenddet:1;
		unsigned reserved01_00:2;
	} b;
};

/*
 * This union represents the bit fields of the Core AHB Configuration Register
 * (GAHBCFG).  Set/clear the bits using the bit fields then write the d32 value
 * to the register.
 */
union gahbcfg_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved9_31:23;
		unsigned ptxfemplvl:1;
#define DWC_GAHBCFG_TXFEMPTYLVL_EMPTY		1
#define DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY	0

		unsigned nptxfemplvl_txfemplvl:1;
		unsigned reserved:1;
		unsigned dmaenable:1;
#define DWC_GAHBCFG_DMAENABLE			1

		unsigned hburstlen:4;
#define DWC_GAHBCFG_INT_DMA_BURST_SINGLE	0
#define DWC_GAHBCFG_INT_DMA_BURST_INCR		1
#define DWC_GAHBCFG_INT_DMA_BURST_INCR4		3
#define DWC_GAHBCFG_INT_DMA_BURST_INCR8		5
#define DWC_GAHBCFG_INT_DMA_BURST_INCR16	7

		unsigned glblintrmsk:1;
#define DWC_GAHBCFG_GLBINT_ENABLE		1
	} b;
};

/*
 * This union represents the bit fields of the Core USB Configuration Register
 * (GUSBCFG).  Set the bits using the bit fields then write the d32 value to the
 * register.
 */
union gusbcfg_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned corrupt_tx_packet:1;
		unsigned force_device_mode:1;
		unsigned force_host_mode:1;
		unsigned reserved23_28:6;
		unsigned term_sel_dl_pulse:1;
		unsigned ulpi_int_vbus_indicator:1;
		unsigned ulpi_ext_vbus_drv:1;
		unsigned ulpi_clk_sus_m:1;
		unsigned ulpi_auto_res:1;
		unsigned ulpi_fsls:1;

		unsigned otgutmifssel:1;
		unsigned phylpwrclksel:1;
		unsigned nptxfrwnden:1;
		unsigned usbtrdtim:4;
		unsigned hnpcap:1;
		unsigned srpcap:1;
		unsigned ddrsel:1;
		unsigned physel:1;
		unsigned fsintf:1;
		unsigned ulpi_utmi_sel:1;
		unsigned phyif:1;
		unsigned toutcal:3;
	} b;
};

/*
 * This union represents the bit fields of the Core Reset Register (GRSTCTL).
 * Set/clear the bits using the bit fields then write the d32 value to the
 * register.
 */
union grstctl_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		/*
		 *  AHB Master Idle.  Indicates the AHB Master State Machine is
		 *  in IDLE condition.
		 */
		unsigned ahbidle:1;

		/*
		 * DMA Request Signal.  Indicated DMA request is in probress.
		   Used for debug purpose.
		 */
		unsigned dmareq:1;

		/* Reserved */
		unsigned reserved29_11:19;

		/*
		 * TxFIFO Number (TxFNum) (Device and Host).
		 *
		 * This is the FIFO number which needs to be flushed,
		 * using the TxFIFO Flush bit. This field should not
		 * be changed until the TxFIFO Flush bit is cleared by
		 * the core.
		 *   - 0x0:Non Periodic TxFIFO Flush
		 *   - 0x1 : Periodic TxFIFO #1 Flush in device mode
		 *     or Periodic TxFIFO in host mode
		 *   - 0x2 : Periodic TxFIFO #2 Flush in device mode.
		 *   - ...
		 *   - 0xF : Periodic TxFIFO #15 Flush in device mode
		 *   - 0x10: Flush all the Transmit NonPeriodic and
		 *     Transmit Periodic FIFOs in the core
		 */
		unsigned txfnum:5;
#define DWC_GRSTCTL_TXFNUM_ALL			0x10

		/*
		 * TxFIFO Flush (TxFFlsh) (Device and Host).
		 *
		 * This bit is used to selectively flush a single or all
		 * transmit FIFOs.  The application must first ensure that the
		 * core is not in the middle of a transaction.
		 *
		 * The application should write into this bit, only after
		 * making sure that neither the DMA engine is writing into the
		 * TxFIFO nor the MAC is reading the data out of the FIFO.
		 *
		 * The application should wait until the core clears this bit,
		 * before performing any operations. This bit will takes 8
		 * clocks (slowest of PHY or AHB clock) to clear.
		 */
		unsigned txfflsh:1;

		/*
		 * RxFIFO Flush (RxFFlsh) (Device and Host)
		 *
		 * The application can flush the entire Receive FIFO using this
		 * bit.
		 *
		 * The application must first ensure that the core is not in the
		 * middle of a transaction.
		 *
		 * The application should write into this bit, only after making
		 * sure that neither the DMA engine is reading from the RxFIFO
		 * nor the MAC is writing the data in to the FIFO.
		 *
		 * The application should wait until the bit is cleared before
		 * performing any other operations. This bit will takes 8 clocks
		 * (slowest of PHY or AHB clock) to clear.
		 */
		unsigned rxfflsh:1;

		/*
		 * In Token Sequence Learning Queue Flush
		 * (INTknQFlsh) (Device Only)
		 */
		unsigned intknqflsh:1;

		/*
		 * Host Frame Counter Reset (Host Only)<br>
		 *
		 * The application can reset the (micro)frame number
		 * counter inside the core, using this bit. When the
		 * (micro)frame counter is reset, the subsequent SOF
		 * sent out by the core, will have a (micro)frame
		 * number of 0.
		 */
		unsigned hstfrm:1;

		/*
		 * Hclk Soft Reset
		 *
		 * The application uses this bit to reset the control logic in
		 * the AHB clock domain. Only AHB clock domain pipelines are
		 * reset.
		 */
		unsigned hsftrst:1;

		/*
		 * Core Soft Reset (CSftRst) (Device and Host)
		 *
		 * The application can flush the control logic in the
		 * entire core using this bit. This bit resets the
		 * pipelines in the AHB Clock domain as well as the
		 * PHY Clock domain.
		 *
		 * The state machines are reset to an IDLE state, the
		 * control bits in the CSRs are cleared, all the
		 * transmit FIFOs and the receive FIFO are flushed.
		 *
		 * The status mask bits that control the generation of
		 * the interrupt, are cleared, to clear the
		 * interrupt. The interrupt status bits are not
		 * cleared, so the application can get the status of
		 * any events that occurred in the core after it has
		 * set this bit.
		 *
		 * Any transactions on the AHB are terminated as soon
		 * as possible following the protocol. Any
		 * transactions on the USB are terminated immediately.
		 *
		 * The configuration settings in the CSRs are
		 * unchanged, so the software doesn't have to
		 * reprogram these registers (Device
		 * Configuration/Host Configuration/Core System
		 * Configuration/Core PHY Configuration).
		 *
		 * The application can write to this bit, any time it
		 * wants to reset the core. This is a self clearing
		 * bit and the core clears this bit after all the
		 * necessary logic is reset in the core, which may
		 * take several clocks, depending on the current state
		 * of the core.
		 */
		unsigned csftrst:1;
	} b;
};

/*
 * This union represents the bit fields of the Core Interrupt Mask Register
 * (GINTMSK).  Set/clear the bits using the bit fields then write the d32 value
 * to the register.
 */
union gintmsk_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned wkupintr:1;
		unsigned sessreqintr:1;
		unsigned disconnect:1;
		unsigned conidstschng:1;
		unsigned reserved27:1;
		unsigned ptxfempty:1;
		unsigned hcintr:1;
		unsigned portintr:1;
		unsigned reserved23_22:2;
		unsigned incomplisoout:1;
		unsigned incomplisoin:1;
		unsigned outepintr:1;
		unsigned inepintr:1;
		unsigned epmismatch:1;
		unsigned reserved16:1;
		unsigned eopframe:1;
		unsigned isooutdrop:1;
		unsigned enumdone:1;
		unsigned usbreset:1;
		unsigned usbsuspend:1;
		unsigned erlysuspend:1;
		unsigned i2cintr:1;
		unsigned reserved08:1;
		unsigned goutnakeff:1;
		unsigned ginnakeff:1;
		unsigned nptxfempty:1;
		unsigned rxstsqlvl:1;
		unsigned sofintr:1;
		unsigned otgintr:1;
		unsigned modemismatch:1;
		unsigned reserved00:1;
	} b;
};

/*
 * This union represents the bit fields of the Core Interrupt Register
 * (GINTSTS).  Set/clear the bits using the bit fields then write the d32 value
 * to the register.
 */
union gintsts_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
#define DWC_SOF_INTR_MASK			0x0008
	struct {
#define DWC_HOST_MODE				1
		unsigned wkupintr:1;
		unsigned sessreqintr:1;
		unsigned disconnect:1;
		unsigned conidstschng:1;
		unsigned reserved27:1;
		unsigned ptxfempty:1;
		unsigned hcintr:1;
		unsigned portintr:1;
		unsigned reserved22_23:2;
		unsigned incomplisoout:1;
		unsigned incomplisoin:1;
		unsigned outepintr:1;
		unsigned inepint:1;
		unsigned epmismatch:1;
		unsigned intokenrx:1;
		unsigned eopframe:1;
		unsigned isooutdrop:1;
		unsigned enumdone:1;
		unsigned usbreset:1;
		unsigned usbsuspend:1;
		unsigned erlysuspend:1;
		unsigned i2cintr:1;
		unsigned reserved8:1;
		unsigned goutnakeff:1;
		unsigned ginnakeff:1;
		unsigned nptxfempty:1;
		unsigned rxstsqlvl:1;
		unsigned sofintr:1;
		unsigned otgintr:1;
		unsigned modemismatch:1;
		unsigned curmode:1;
	} b;
};

/*
 * This union represents the bit fields in the Device Receive Status Read and
 * Pop Registers (GRXSTSR, GRXSTSP) Read the register into the d32
 * element then read out the bits using the bit elements.
 */
union device_grxsts_data {			/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved:7;
		unsigned fn:4;
		unsigned pktsts:4;
#define DWC_STS_DATA_UPDT		0x2  /* OUT Data Packet */
#define DWC_STS_XFER_COMP		0x3  /* OUT Data Transfer Complete */
#define DWC_DSTS_GOUT_NAK		0x1  /* Global OUT NAK */
#define DWC_DSTS_SETUP_COMP		0x4  /* Setup Phase Complete */
#define DWC_DSTS_SETUP_UPDT		0x6  /* SETUP Packet */

		unsigned dpid:2;
		unsigned bcnt:11;
		unsigned epnum:4;
	} b;
};

/*
 * This union represents the bit fields in the Host Receive Status Read and
 * Pop Registers (GRXSTSR, GRXSTSP) Read the register into the d32
 * element then read out the bits using the bit elements.
 */
union host_grxsts_data {			/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved31_21:11;
		unsigned pktsts:4;
#define DWC_GRXSTS_PKTSTS_IN			0x2
#define DWC_GRXSTS_PKTSTS_IN_XFER_COMP		0x3
#define DWC_GRXSTS_PKTSTS_DATA_TOGGLE_ERR	0x5
#define DWC_GRXSTS_PKTSTS_CH_HALTED		0x7

		unsigned dpid:2;
		unsigned bcnt:11;
		unsigned chnum:4;
	} b;
};

/*
 * This union represents the bit fields in the FIFO Size Registers (HPTXFSIZ,
 * GNPTXFSIZ, DPTXFSIZn). Read the register into the d32 element then
 * read out the bits using the bit elements.
 */
union fifosize_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned depth:16;
		unsigned startaddr:16;
	} b;
};

/*
 * This union represents the bit fields in the Non-Periodic Transmit FIFO/Queue
 * Status Register (GNPTXSTS). Read the register into the d32 element then read
 * out the bits using the bit elements.
 */
union gnptxsts_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved:1;
		/* Top of the Non-Periodic Transmit Request Queue
		 *  - bits 30:27 - Channel/EP Number
		 *  - bits 26:25 - Token Type
		 *    - 2'b00 - IN/OUT
		 *    - 2'b01 - Zero Length OUT
		 *    - 2'b10 - PING/Complete Split
		 *    - 2'b11 - Channel Halt
		 *  - bit 24 - Terminate (Last entry for the selected
		 *    channel/EP)
		 */
		unsigned nptxqtop_chnep:4;
		unsigned nptxqtop_token:2;
		unsigned nptxqtop_terminate:1;
		unsigned nptxqspcavail:8;
		unsigned nptxfspcavail:16;
	} b;
};

/*
 * This union represents the bit fields in the Transmit	FIFO Status Register
 * (DTXFSTS). Read the register into the d32 element then read out the bits
 * using the bit elements.
 */
union dtxfsts_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved:16;
		unsigned txfspcavail:16;
	} b;
};

/*
 * This union represents the bit fields in the I2C Control Register (I2CCTL).
 * Read the register into the d32 element then read out the bits using the bit
 * elements.
 */
union gi2cctl_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned bsydne:1;
		unsigned rw:1;
		unsigned reserved:2;
		unsigned i2cdevaddr:2;
		unsigned i2csuspctl:1;
		unsigned ack:1;
		unsigned i2cen:1;
		unsigned addr:7;
		unsigned regaddr:8;
		unsigned rwdata:8;
	} b;
};

/*
 * This union represents the bit fields in the User HW Config1 Register.  Read
 * the register into the d32 element then read out the bits using the bit
 * elements.
 */
union hwcfg1_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned ep_dir15:2;
		unsigned ep_dir14:2;
		unsigned ep_dir13:2;
		unsigned ep_dir12:2;
		unsigned ep_dir11:2;
		unsigned ep_dir10:2;
		unsigned ep_dir9:2;
		unsigned ep_dir8:2;
		unsigned ep_dir7:2;
		unsigned ep_dir6:2;
		unsigned ep_dir5:2;
		unsigned ep_dir4:2;
		unsigned ep_dir3:2;
		unsigned ep_dir2:2;
		unsigned ep_dir1:2;
		unsigned ep_dir0:2;
	} b;
};

/*
 * This union represents the bit fields in the User HW Config2 Register.  Read
 * the register into the d32 element then read out the bits using the bit
 * elements.
 */
union hwcfg2_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		/* GHWCFG2 */
		unsigned reserved31:1;
		unsigned dev_token_q_depth:5;
		unsigned host_perio_tx_q_depth:2;
		unsigned nonperio_tx_q_depth:2;
		unsigned rx_status_q_depth:2;
		unsigned dynamic_fifo:1;
		unsigned perio_ep_supported:1;
		unsigned num_host_chan:4;
		unsigned num_dev_ep:4;
		unsigned fs_phy_type:2;
		unsigned hs_phy_type:2;
#define DWC_HWCFG2_HS_PHY_TYPE_NOT_SUPPORTED		0
#define DWC_HWCFG2_HS_PHY_TYPE_UTMI			1
#define DWC_HWCFG2_HS_PHY_TYPE_ULPI			2
#define DWC_HWCFG2_HS_PHY_TYPE_UTMI_ULPI		3

		unsigned point2point:1;
		unsigned architecture:2;
		unsigned op_mode:3;
#define DWC_HWCFG2_OP_MODE_HNP_SRP_CAPABLE_OTG		0
#define DWC_HWCFG2_OP_MODE_SRP_ONLY_CAPABLE_OTG		1
#define DWC_HWCFG2_OP_MODE_NO_HNP_SRP_CAPABLE_OTG	2
#define DWC_HWCFG2_OP_MODE_SRP_CAPABLE_DEVICE		3
#define DWC_HWCFG2_OP_MODE_NO_SRP_CAPABLE_DEVICE	4
#define DWC_HWCFG2_OP_MODE_SRP_CAPABLE_HOST		5
#define DWC_HWCFG2_OP_MODE_NO_SRP_CAPABLE_HOST		6
	} b;
};

/*
 * This union represents the bit fields in the User HW Config3 Register.  Read
 * the register into the d32 element then read out the bits using the bit
 * elements.
 */
union hwcfg3_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		/* GHWCFG3 */
		unsigned dfifo_depth:16;
		unsigned reserved15_13:3;
		unsigned ahb_phy_clock_synch:1;
		unsigned synch_reset_type:1;
		unsigned optional_features:1;
		unsigned vendor_ctrl_if:1;
		unsigned i2c:1;
		unsigned otg_func:1;
		unsigned packet_size_cntr_width:3;
		unsigned xfer_size_cntr_width:4;
	} b;
};

/*
 * This union represents the bit fields in the User HW Config4 Register.  Read
 * the register into the d32 element then read out the bits using the bit
 * elements.
 */
union hwcfg4_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved31_30:2;
		unsigned num_in_eps:4;
		unsigned ded_fifo_en:1;

		unsigned session_end_filt_en:1;
		unsigned b_valid_filt_en:1;
		unsigned a_valid_filt_en:1;
		unsigned vbus_valid_filt_en:1;
		unsigned iddig_filt_en:1;
		unsigned num_dev_mode_ctrl_ep:4;
		unsigned utmi_phy_data_width:2;
		unsigned min_ahb_freq:9;
		unsigned power_optimiz:1;
		unsigned num_dev_perio_in_ep:4;
	} b;
};

/*
 * Device Global Registers. Offsets 800h-BFFh
 *
 * The following structures define the size and relative field offsets for the
 * Device Mode Registers.
 *
 * These registers are visible only in Device mode and must not be accessed in
 * Host mode, as the results are unknown.
 */
struct device_global_regs {		/* CONFIG_DWC_OTG_REG_LE */
	/* Device Configuration Register.			Offset: 800h */
	u32 dcfg;
	/* Device Control Register.				Offset: 804h */
	u32 dctl;
	/* Device Status Register (Read Only).			Offset: 808h */
	u32 dsts;
	/* Reserved.						Offset: 80Ch */
	u32 unused;
	/* Device IN Endpoint Common Interrupt Mask Register.	Offset: 810h */
	u32 diepmsk;
	/* Device OUT Endpoint Common Interrupt Mask Register.	Offset: 814h */
	u32 doepmsk;
	/* Device All Endpoints Interrupt Register.		Offset: 818h */
	u32 daint;
	/* Device All Endpoints Interrupt Mask Register.	Offset: 81Ch */
	u32 daintmsk;
	/* Device IN Token Queue Read Register-1 (Read Only).	Offset: 820h */
	u32 dtknqr1;
	/* Device IN Token Queue Read Register-2 (Read Only).	Offset: 824h */
	u32 dtknqr2;
	/* Device VBUS  discharge Register.			Offset: 828h */
	u32 dvbusdis;
	/* Device VBUS Pulse Register.				Offset: 82Ch */
	u32 dvbuspulse;
	/* Device IN Token Queue Read Register-3 (Read Only).	Offset: 830h */
	u32 dtknqr3_dthrctl;
	/* Device IN Token Queue Read Register-4 (Read Only).	Offset: 834h */
	u32 dtknqr4_fifoemptymsk;
};

/*
 * This union represents the bit fields in the Device Configuration
 * Register.  Read the register into the d32 member then
 * set/clear the bits using the bit elements.  Write the
 * d32 member to the dcfg register.
 */
union dcfg_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved0_8:9;
		unsigned epmscnt:5;
		/* In Endpoint Mis-match count */
		unsigned reserved17_13:5;
		/* Periodic Frame Interval */
		unsigned perfrint:2;
#define DWC_DCFG_FRAME_INTERVAL_80		0
#define DWC_DCFG_FRAME_INTERVAL_85		1
#define DWC_DCFG_FRAME_INTERVAL_90		2
#define DWC_DCFG_FRAME_INTERVAL_95		3

		/* Device Addresses */
		unsigned devaddr:7;
		unsigned reserved3:1;
		/* Non Zero Length Status OUT Handshake */
		unsigned nzstsouthshk:1;
#define DWC_DCFG_SEND_STALL			1

		/* Device Speed */
		unsigned devspd:2;
	} b;
};

/*
 * This union represents the bit fields in the Device Control Register.  Read
 * the register into the d32 member then set/clear the bits using the bit
 * elements.
 */
union dctl_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved31_12:21;
		/* Clear Global OUT NAK */
		unsigned cgoutnak:1;
		/* Set Global OUT NAK */
		unsigned sgoutnak:1;
		/* Clear Global Non-Periodic IN NAK */
		unsigned cgnpinnak:1;
		/* Set Global Non-Periodic IN NAK */
		unsigned sgnpinnak:1;
		/* Test Control */
		unsigned tstctl:3;
		/* Global OUT NAK Status */
		unsigned goutnaksts:1;
		/* Global Non-Periodic IN NAK Status */
		unsigned gnpinnaksts:1;
		/* Soft Disconnect */
		unsigned sftdiscon:1;
		/* Remote Wakeup */
		unsigned rmtwkupsig:1;
	} b;
};

/*
 * This union represents the bit fields in the Device Status Register.  Read the
 * register into the d32 member then set/clear the bits using the bit elements.
 */
union dsts_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved31_22:10;
		/* Frame or Microframe Number of the received SOF */
		unsigned soffn:14;
		unsigned reserved07_04:4;
		/* Erratic Error */
		unsigned errticerr:1;
		/* Enumerated Speed */
		unsigned enumspd:2;
#define DWC_DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ		0
#define DWC_DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ		1
#define DWC_DSTS_ENUMSPD_LS_PHY_6MHZ			2
#define DWC_DSTS_ENUMSPD_FS_PHY_48MHZ			3
		/* Suspend Status */
		unsigned suspsts:1;
	} b;
};

/*
 * This union represents the bit fields in the Device IN EP Interrupt Register
 * and the Device IN EP Common Mask Register.
 *
 * Read the register into the d32 member then set/clear the bits using the bit
 * elements.
 */
union diepint_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved31_08:23;
		unsigned txfifoundrn:1;
		/* IN Endpoint HAK Effective mask */
		unsigned emptyintr:1;
		/* IN Endpoint NAK Effective mask */
		unsigned inepnakeff:1;
		/* IN Token Received with EP mismatch mask */
		unsigned intknepmis:1;
		/* IN Token received with TxF Empty mask */
		unsigned intktxfemp:1;
		/* TimeOUT Handshake mask (non-ISOC EPs) */
		unsigned timeout:1;
		/* AHB Error mask */
		unsigned ahberr:1;
		/* Endpoint disable mask */
		unsigned epdisabled:1;
		/* Transfer complete mask */
		unsigned xfercompl:1;
	} b;
};

/*
 * This union represents the bit fields in the Device OUT EP Interrupt Register
 * and Device OUT EP Common Interrupt Mask Register.
 *
 * Read the register into the d32 member then set/clear the bits using the bit
 * elements.
 */
union doepint_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved31_04:28; /* Docs say reserved is 27 bits */

		/* There is 1 bit missing here, not used? */

		/* Setup Phase Done (control EPs) */
		unsigned setup:1;
		/* AHB Error */
		unsigned ahberr:1;
		/* Endpoint disable  */
		unsigned epdisabled:1;
		/* Transfer complete */
		unsigned xfercompl:1;
	} b;
};

/*
 * This union represents the bit fields in the Device All EP Interrupt and Mask
 * Registers.  Read the register into the d32 member then set/clear the bits
 * using the bit elements.
 */
union daint_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		/* OUT Endpoint bits */
		unsigned out:16;
		/* IN Endpoint bits */
		unsigned in:16;
	} ep;
	struct {
		/* OUT Endpoint bits */
		unsigned outep15:1;
		unsigned outep14:1;
		unsigned outep13:1;
		unsigned outep12:1;
		unsigned outep11:1;
		unsigned outep10:1;
		unsigned outep9:1;
		unsigned outep8:1;
		unsigned outep7:1;
		unsigned outep6:1;
		unsigned outep5:1;
		unsigned outep4:1;
		unsigned outep3:1;
		unsigned outep2:1;
		unsigned outep1:1;
		unsigned outep0:1;
		/* IN Endpoint bits */
		unsigned inep15:1;
		unsigned inep14:1;
		unsigned inep13:1;
		unsigned inep12:1;
		unsigned inep11:1;
		unsigned inep10:1;
		unsigned inep9:1;
		unsigned inep8:1;
		unsigned inep7:1;
		unsigned inep6:1;
		unsigned inep5:1;
		unsigned inep4:1;
		unsigned inep3:1;
		unsigned inep2:1;
		unsigned inep1:1;
		unsigned inep0:1;
	} b;
};

/*
 * This union represents the bit fields in the Device IN Token Queue Read
 * Registers.  Read the register into the d32 member. READ-ONLY Register
 */
union dtknq1_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		/* EP Numbers of IN Tokens 0 ... 4 */
		unsigned epnums0_5:24;
		/* write pointer has wrapped. */
		unsigned wrap_bit:1;
		/* Reserved */
		unsigned reserved05_06:2;
		/* In Token Queue Write Pointer */
		unsigned intknwptr:5;
	} b;
};

/*
 * This union represents Threshold control Register. Read and write the register
 * into the d32 member.  READ-WRITABLE Register
 */
union dthrctl_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		/* Reserved */
		unsigned reserved26_31:6;
		/* Rx Thr. Length */
		unsigned rx_thr_len:9;
		/* Rx Thr. Enable */
		unsigned rx_thr_en:1;
		/* Reserved */
		unsigned reserved11_15:5;
		/* Tx Thr. Length */
		unsigned tx_thr_len:9;
		/* ISO Tx Thr. Enable */
		unsigned iso_thr_en:1;
		/* non ISO Tx Thr. Enable */
		unsigned non_iso_thr_en:1;
	} b;
};

/*
 * Device Logical IN Endpoint-Specific Registers. Offsets 900h-AFCh
 *
 * There will be one set of endpoint registers per logical endpoint implemented.
 *
 * These registers are visible only in Device mode and must not be accessed in
 * Host mode, as the results are unknown.
 */
struct device_in_ep_regs {
	/*
	 * Device IN Endpoint Control Register.
	 * Offset:900h + (ep_num * 20h) + 00h
	 */
	u32 diepctl;
	/* Reserved. Offset:900h + (ep_num * 20h) + 04h */
	u32 reserved04;
	/*
	 * Device IN Endpoint Interrupt Register.
	 * Offset:900h + (ep_num * 20h) + 08h
	 */
	u32 diepint;
	/* Reserved. Offset:900h + (ep_num * 20h) + 0Ch */
	u32 reserved0C;
	/* Device IN Endpoint Transfer Size Register.
	 * Offset:900h + (ep_num * 20h) + 10h
	 */
	u32 dieptsiz;
	/*
	 * Device IN Endpoint DMA Address Register.
	 * Offset:900h + (ep_num * 20h) + 14h
	 */
	u32 diepdma;
	/* Reserved.
	 * Offset:900h + (ep_num * 20h) + 18h - 900h + (ep_num * 20h) + 1Ch
	 */
	u32 dtxfsts;
	/*
	 * Reserved.
	 * Offset:900h + (ep_num * 20h) + 1Ch - 900h + (ep_num * 20h) + 1Ch
	 */
	u32 reserved18;
};

/*
 * Device Logical OUT Endpoint-Specific Registers. Offsets: B00h-CFCh
 *
 * There will be one set of endpoint registers per logical endpoint implemented.
 *
 * These registers are visible only in Device mode and must not be accessed in
 * Host mode, as the results are unknown.
 */
struct device_out_ep_regs {
	/*
	 * Device OUT Endpoint Control Register.
	 * Offset:B00h + (ep_num * 20h) + 00h
	 */
	u32 doepctl;
	/*
	 * Device OUT Endpoint Frame number Register.
	 * Offset: B00h + (ep_num * 20h) + 04h
	 */
	u32 doepfn;
	/*
	 * Device OUT Endpoint Interrupt Register.
	 * Offset:B00h + (ep_num * 20h) + 08h
	 */
	u32 doepint;
	/* Reserved. Offset:B00h + (ep_num * 20h) + 0Ch */
	u32 reserved0C;
	/*
	 * Device OUT Endpoint Transfer Size Register.
	 * Offset: B00h + (ep_num * 20h) + 10h
	 */
	u32 doeptsiz;
	/*
	 * Device OUT Endpoint DMA Address Register.
	 * Offset:B00h + (ep_num * 20h) + 14h
	 */
	u32 doepdma;
	/*
	 * Reserved.
	 * Offset:B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch
	 */
	u32 unused[2];
};

/*
 * This union represents the bit fields in the Device EP Control Register.  Read
 * the register into the d32 member then set/clear the bits using the bit
 * elements.
 */
union depctl_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		/* Endpoint Enable */
		unsigned epena:1;
		/* Endpoint Disable */
		unsigned epdis:1;

		/*
		 * Set DATA1 PID (INTR/Bulk IN and OUT endpoints) Writing to
		 * this field sets the Endpoint DPID (DPID) field in this
		 * register to DATA1 Set Odd (micro)frame (SetOddFr) (ISO IN and
		 * OUT Endpoints) Writing to this field sets the Even/Odd
		 * (micro)frame (EO_FrNum) field to odd (micro) frame.
		 */
		unsigned setd1pid:1;
		/*
		 * Set DATA0 PID (INTR/Bulk IN and OUT endpoints)  Writing to
		 * this field sets the Endpoint DPID (DPID) field in this
		 * register to DATA0. Set Even (micro)frame (SetEvenFr) (ISO IN
		 * and OUT Endpoints) Writing to this field sets the Even/Odd
		 * (micro)frame (EO_FrNum) field to even (micro) frame.
		 */
		unsigned setd0pid:1;

		/* Set NAK */
		unsigned snak:1;
		/* Clear NAK */
		unsigned cnak:1;

		/*
		 * Tx Fifo Number
		 * IN EPn/IN EP0
		 * OUT EPn/OUT EP0 - reserved
		 */
		unsigned txfnum:4;

		/* Stall Handshake */
		unsigned stall:1;

		/* Snoop Mode
		 * OUT EPn/OUT EP0
		 * IN EPn/IN EP0 - reserved
		 */
		unsigned snp:1;

		/* Endpoint Type
		 *  2'b00: Control
		 *  2'b01: Isochronous
		 *  2'b10: Bulk
		 *  2'b11: Interrupt
		 */
		unsigned eptype:2;

		/* NAK Status */
		unsigned naksts:1;

		/*
		 * Endpoint DPID (INTR/Bulk IN and OUT endpoints) This field
		 * contains the PID of the packet going to be received or
		 * transmitted on this endpoint. The application should program
		 * the PID of the first packet going to be received or
		 * transmitted on this endpoint, after the endpoint is
		 * activated. Applications use the SetD1PID and SetD0PID fields
		 * of this register to program either D0 or D1 PID.
		 *
		 * The encoding for this field is
		 *   - 0: D0
		 *   - 1: D1
		 */
		unsigned dpid:1;

		/* USB Active Endpoint */
		unsigned usbactep:1;

		/*
		 * Next Endpoint
		 * IN EPn/IN EP0
		 * OUT EPn/OUT EP0 - reserved
		 */
		unsigned nextep:4;

		/*
		 * Maximum Packet Size
		 * IN/OUT EPn
		 * IN/OUT EP0 - 2 bits
		 *   2'b00: 64 Bytes
		 *   2'b01: 32
		 *   2'b10: 16
		 *   2'b11: 8
		 */
		unsigned mps:11;
#define DWC_DEP0CTL_MPS_64			0
#define DWC_DEP0CTL_MPS_32			1
#define DWC_DEP0CTL_MPS_16			2
#define DWC_DEP0CTL_MPS_8			3
	} b;
};

/*
 * This union represents the bit fields in the Device EP Transfer Size Register.
 * Read the register into the d32 member then set/clear the bits using the bit
 * elements.
 */
union deptsiz_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;

	/*
	 * Added-sr: 2007-07-26
	 *
	 * Correct ther register layout for the 405EZ Ultra
	 * USB device implementation.
	 */
#ifdef CONFIG_DWC_LIMITED_XFER_SIZE
	struct {
		unsigned reserved:1;
		/* Multi Count - Periodic IN endpoints */
		unsigned mc:2;
		unsigned reserved1:5;
		/* Packet Count */
		unsigned pktcnt:5;
		unsigned reserved2:8;
		/* Transfer size */
		unsigned xfersize:11;
	} b;
#else
	struct {
		unsigned reserved:1;
		/* Multi Count - Periodic IN endpoints */
		unsigned mc:2;
		/* Packet Count */
		unsigned pktcnt:10;
		/* Transfer size */
		unsigned xfersize:19;
	} b;
#endif
};

/*
 * This union represents the bit fields in the Device EP 0 Transfer Size
 * Register.  Read the register into the d32 member then set/clear the bits
 * using the bit elements.
 */
union deptsiz0_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved31:1; /* device*/
		/*Setup Packet Count (DOEPTSIZ0 Only) */
		unsigned supcnt:2;
		/* Reserved */
		unsigned reserved28_20:9;
		/* Packet Count */
		unsigned pktcnt:1;
		/* Reserved */
		unsigned reserved18_7:12;
		/* Transfer size */
		unsigned xfersize:7;
	} b;
};

#define MAX_PERIO_FIFOS			15	/* Max periodic FIFOs */
#define MAX_TX_FIFOS			15	/* Max non-periodic FIFOs */

/* Maximum number of Endpoints/HostChannels */
#define MAX_EPS_CHANNELS		16

/*
 * The device_if structure contains information needed to manage the DWC_otg
 * controller acting in device mode. It represents the programming view of the
 * device-specific aspects of the controller.
 */
struct device_if {
	/* Device Global Registers starting at offset 800h */
	struct device_global_regs *dev_global_regs;
#define DWC_DEV_GLOBAL_REG_OFFSET		0x800

	/* Device Logical IN Endpoint-Specific Registers 900h-AFCh */
	struct device_in_ep_regs *in_ep_regs[MAX_EPS_CHANNELS];
#define DWC_DEV_IN_EP_REG_OFFSET		0x900
#define DWC_EP_REG_OFFSET			0x20

	/* Device Logical OUT Endpoint-Specific Registers B00h-CFCh */
	struct device_out_ep_regs *out_ep_regs[MAX_EPS_CHANNELS];
#define DWC_DEV_OUT_EP_REG_OFFSET		0xB00

	/* Device configuration information */
	/* Device Speed  0: Unknown, 1: LS, 2:FS, 3: HS */
	u8  speed;
	/*  Number # of Tx EP range: 0-15 exept ep0 */
	u8  num_in_eps;
	/*  Number # of Rx EP range: 0-15 exept ep 0*/
	u8  num_out_eps;

	/* Size of periodic FIFOs (Bytes) */
	u16 perio_tx_fifo_size[MAX_PERIO_FIFOS];

	/* Size of Tx FIFOs (Bytes) */
	u16 tx_fifo_size[MAX_TX_FIFOS];

	/* Thresholding enable flags and length varaiables */
	u16 rx_thr_en;
	u16 iso_tx_thr_en;
	u16 non_iso_tx_thr_en;
	u16 rx_thr_length;
	u16 tx_thr_length;
};

/*
 * This union represents the bit fields in the Power and Clock Gating Control
 * Register. Read the register into the d32 member then set/clear the
 * bits using the bit elements.
 */
union pcgcctl_data {
	u32 d32;
	struct {
		unsigned reserved31_05:27;
		/* PHY Suspended */
		unsigned physuspended:1;
		/* Reset Power Down Modules */
		unsigned rstpdwnmodule:1;
		/* Power Clamp */
		unsigned pwrclmp:1;
		/* Gate Hclk */
		unsigned gatehclk:1;
		/* Stop Pclk */
		unsigned stoppclk:1;
	} b;
};

/*
 * Host Mode Register Structures
 */

/*
 * The Host Global Registers structure defines the size and relative field
 * offsets for the Host Mode Global Registers.  Host Global Registers offsets
 * 400h-7FFh.
*/
struct host_global_regs {
	/* Host Configuration Register.				Offset: 400h */
	u32 hcfg;
	/* Host Frame Interval Register.			Offset: 404h */
	u32 hfir;
	/* Host Frame Number / Frame Remaining Register.	Offset: 408h */
	u32 hfnum;
	/* Reserved.						Offset: 40Ch */
	u32 reserved40C;
	/* Host Periodic Transmit FIFO/ Queue Status Register.	Offset: 410h */
	u32 hptxsts;
	/* Host All Channels Interrupt Register.		Offset: 414h */
	u32 haint;
	/* Host All Channels Interrupt Mask Register.		Offset: 418h */
	u32 haintmsk;
};

/*
 * This union represents the bit fields in the Host Configuration Register. Read
 * the register into the d32 member then set/clear the bits using the bit
 * elements. Write the d32 member to the hcfg register.
 */
union hcfg_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
#define DWC_HCFG_30_60_MHZ			0
#define DWC_HCFG_48_MHZ				1
#define DWC_HCFG_6_MHZ				2
		/* FS/LS Only Support */
		unsigned fslssupp:1;
		/* FS/LS Phy Clock Select */
		unsigned fslspclksel:2;
	} b;
};

/*
 * This union represents the bit fields in the Host Frame Remaing/Number
 * Register.
 */
union hfir_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved:16;
		unsigned frint:16;
	} b;
};

/*
 * This union represents the bit fields in the Host Frame Remaing/Number
 * Register.
 */
union hfnum_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
#define DWC_HFNUM_MAX_FRNUM			0x3FFF
		unsigned frrem:16;
		unsigned frnum:16;
	} b;
};

union hptxsts_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned ptxqtop_odd:1;
		unsigned ptxqtop_chnum:4;
		unsigned ptxqtop_token:2;
		unsigned ptxqtop_terminate:1;
		unsigned ptxqspcavail:8;
		unsigned ptxfspcavail:16;
		/*
		 * Top of the Periodic Transmit Request Queue
		 *  - bit 24 - Terminate (last entry for the selected channel)
		 *  - bits 26:25 - Token Type
		 *    - 2'b00 - Zero length
		 *    - 2'b01 - Ping
		 *    - 2'b10 - Disable
		 *  - bits 30:27 - Channel Number
		 *  - bit 31 - Odd/even microframe
		 */
	} b;
};

/*
 * This union represents the bit fields in the Host Port Control and Status
 * Register. Read the register into the d32 member then set/clear the bits using
 * the bit elements. Write the d32 member to the hprt0 register.
 */
union hprt0_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
#define DWC_HPRT0_PRTSPD_HIGH_SPEED		0
#define DWC_HPRT0_PRTSPD_FULL_SPEED		1
#define DWC_HPRT0_PRTSPD_LOW_SPEED		2
		unsigned reserved19_31:13;
		unsigned prtspd:2;
		unsigned prttstctl:4;
		unsigned prtpwr:1;
		unsigned prtlnsts:2;
		unsigned reserved9:1;
		unsigned prtrst:1;
		unsigned prtsusp:1;
		unsigned prtres:1;
		unsigned prtovrcurrchng:1;
		unsigned prtovrcurract:1;
		unsigned prtenchng:1;
		unsigned prtena:1;
		unsigned prtconndet:1;
		unsigned prtconnsts:1;
	} b;
};

/*
 * This union represents the bit fields in the Host All Interrupt Register.
 */
union haint_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved:16;
		unsigned ch15:1;
		unsigned ch14:1;
		unsigned ch13:1;
		unsigned ch12:1;
		unsigned ch11:1;
		unsigned ch10:1;
		unsigned ch9:1;
		unsigned ch8:1;
		unsigned ch7:1;
		unsigned ch6:1;
		unsigned ch5:1;
		unsigned ch4:1;
		unsigned ch3:1;
		unsigned ch2:1;
		unsigned ch1:1;
		unsigned ch0:1;
	} b;
	struct {
		unsigned reserved:16;
		unsigned chint:16;
	} b2;
};

/*
 * This union represents the bit fields in the Host All Interrupt Register.
 */
union haintmsk_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved:16;
		unsigned ch15:1;
		unsigned ch14:1;
		unsigned ch13:1;
		unsigned ch12:1;
		unsigned ch11:1;
		unsigned ch10:1;
		unsigned ch9:1;
		unsigned ch8:1;
		unsigned ch7:1;
		unsigned ch6:1;
		unsigned ch5:1;
		unsigned ch4:1;
		unsigned ch3:1;
		unsigned ch2:1;
		unsigned ch1:1;
		unsigned ch0:1;
	} b;
	struct {
		unsigned reserved:16;
		unsigned chint:16;
	} b2;
};

/*
 * Host Channel Specific Registers. 500h-5FCh
 */
struct dwc_hc_regs {			/* CONFIG_DWC_OTG_REG_LE */
	/*
	 * Host Channel 0 Characteristic Register.
	 * Offset: 500h + (chan_num * 20h) + 00h
	 */
	u32 hcchar;
	/*
	 * Host Channel 0 Split Control Register.
	 * Offset: 500h + (chan_num * 20h) + 04h
	 */
	u32 hcsplt;
	/*
	 * Host Channel 0 Interrupt Register.
	 * Offset: 500h + (chan_num * 20h) + 08h
	 */
	u32 hcint;
	/*
	 * Host Channel 0 Interrupt Mask Register.
	 * Offset: 500h + (chan_num * 20h) + 0Ch
	 */
	u32 hcintmsk;
	/*
	 * Host Channel 0 Transfer Size Register.
	 * Offset: 500h + (chan_num * 20h) + 10h
	 */
	u32 hctsiz;
	/*
	 * Host Channel 0 DMA Address Register.
	 * Offset: 500h + (chan_num * 20h) + 14h
	 */
	u32 hcdma;
	/*
	 * Reserved.
	 * Offset: 500h + (chan_num * 20h) + 18h - 500h + (chan_num * 20h) + 1Ch
	  */
	u32 reserved[2];
};

/*
 * This union represents the bit fields in the Host Channel Characteristics
 * Register. Read the register into the d32 member then set/clear the bits using
 * the bit elements. Write the d32 member to the hcchar register.
 */
union hcchar_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		/* Channel enable */
		unsigned chen:1;
		/* Channel disable */
		unsigned chdis:1;
		/*
		 * Frame to transmit periodic transaction.
		 * 0: even, 1: odd
		 */
		unsigned oddfrm:1;
		/* Device address */
		unsigned devaddr:7;
		/* Packets per frame for periodic transfers. 0 is reserved. */
		unsigned multicnt:2;
		/* 0: Control, 1: Isoc, 2: Bulk, 3: Intr */
		unsigned eptype:2;
		/* 0: Full/high speed device, 1: Low speed device */
		unsigned lspddev:1;
		unsigned reserved:1;
		/* 0: OUT, 1: IN */
		unsigned epdir:1;
		/* Endpoint number */
		unsigned epnum:4;
		/* Maximum packet size in bytes */
		unsigned mps:11;
	} b;
};

union hcsplt_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		/* Split Enble */
		unsigned spltena:1;
		/* Reserved */
		unsigned reserved:14;
		/* Do Complete Split */
		unsigned compsplt:1;
		/* Transaction Position */
		unsigned xactpos:2;
#define DWC_HCSPLIT_XACTPOS_MID			0
#define DWC_HCSPLIT_XACTPOS_END			1
#define DWC_HCSPLIT_XACTPOS_BEGIN		2
#define DWC_HCSPLIT_XACTPOS_ALL			3

		/* Hub Address */
		unsigned hubaddr:7;
		/* Port Address */
		unsigned prtaddr:7;
	} b;
};

/*
 * This union represents the bit fields in the Host All Interrupt
 * Register.
 */
union hcint_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		/* Reserved */
		unsigned reserved:21;
		/* Data Toggle Error */
		unsigned datatglerr:1;
		/* Frame Overrun */
		unsigned frmovrun:1;
		/* Babble Error */
		unsigned bblerr:1;
		/* Transaction Err */
		unsigned xacterr:1;
		/* NYET Response Received */
		unsigned nyet:1;
		/* ACK Response Received */
		unsigned ack:1;
		/* NAK Response Received */
		unsigned nak:1;
		/* STALL Response Received */
		unsigned stall:1;
		/* AHB Error */
		unsigned ahberr:1;
		/* Channel Halted */
		unsigned chhltd:1;
		/* Transfer Complete */
		unsigned xfercomp:1;
	} b;
};

/*
 * This union represents the bit fields in the Host Channel Transfer Size
 * Register. Read the register into the d32 member then set/clear the  bits
 * using the bit elements. Write the d32 member to the hcchar register.
 */
union hctsiz_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
#define DWC_HCTSIZ_DATA0			0
#define DWC_HCTSIZ_DATA1			2
#define DWC_HCTSIZ_DATA2			1
#define DWC_HCTSIZ_MDATA			3
#define DWC_HCTSIZ_SETUP			3

		/* Do PING protocol when 1 */
		unsigned dopng:1;
		/*
		 * Packet ID for next data packet
		 * 0: DATA0
		 * 1: DATA2
		 * 2: DATA1
		 * 3: MDATA (non-Control), SETUP (Control)
		 */
		unsigned pid:2;
		/* Data packets to transfer */
		unsigned pktcnt:10;
		/* Total transfer size in bytes */
		unsigned xfersize:19;
	} b;
};

/*
 * This union represents the bit fields in the Host Channel Interrupt Mask
 * Register. Read the register into the d32 member then set/clear the bits using
 * the bit elements. Write the d32 member to the hcintmsk register.
 */
union hcintmsk_data {				/* CONFIG_DWC_OTG_REG_LE */
	u32 d32;
	struct {
		unsigned reserved:21;
		unsigned datatglerr:1;
		unsigned frmovrun:1;
		unsigned bblerr:1;
		unsigned xacterr:1;
		unsigned nyet:1;
		unsigned ack:1;
		unsigned nak:1;
		unsigned stall:1;
		unsigned ahberr:1;
		unsigned chhltd:1;
		unsigned xfercompl:1;
	} b;
};

/*
 * OTG Host Interface Structure.
 *
 * The OTG Host Interface Structure structure contains information needed to
 * manage the DWC_otg controller acting in host mode. It represents the
 * programming view of the host-specific aspects of the controller.
 */
struct dwc_host_if {			/* CONFIG_DWC_OTG_REG_LE */
	/* Host Global Registers starting at offset 400h.*/
	struct host_global_regs *host_global_regs;
#define DWC_OTG_HOST_GLOBAL_REG_OFFSET		0x400

	/* Host Port 0 Control and Status Register */
	u32 *hprt0;
#define DWC_OTG_HOST_PORT_REGS_OFFSET		0x440

	/* Host Channel Specific Registers at offsets 500h-5FCh. */
	struct dwc_hc_regs *hc_regs[MAX_EPS_CHANNELS];
#define DWC_OTG_HOST_CHAN_REGS_OFFSET		0x500
#define DWC_OTG_CHAN_REGS_OFFSET		0x20

	/* Host configuration information */
	/* Number of Host Channels (range: 1-16) */
	u8  num_host_channels;
	/* Periodic EPs supported (0: no, 1: yes) */
	u8  perio_eps_supported;
	/* Periodic Tx FIFO Size (Only 1 host periodic Tx FIFO) */
	u16 perio_tx_fifo_size;
};

#else  /* CONFIG_DWC_OTG_REG_LE not defined */

/*
 * This union represents the bit fields of the Core OTG Control
 * and Status Register (GOTGCTL).  Set the bits using the bit
 * fields then write the d32 value to the register.
 */
union gotgctl_data {
	u32 d32;
	struct {
		unsigned sesreqscs:1;
		unsigned sesreq:1;
		unsigned reserved2_7:6;
		unsigned hstnegscs:1;
		unsigned hnpreq:1;
		unsigned hstsethnpen:1;
		unsigned devhnpen:1;
		unsigned reserved12_15:4;
		unsigned conidsts:1;
		unsigned reserved17:1;
		unsigned asesvld:1;
		unsigned bsesvld:1;
		unsigned currmod:1;
		unsigned reserved21_31:11;
	} b;
};

/*
 * This union represents the bit fields of the Core OTG Interrupt Register
 * (GOTGINT).  Set/clear the bits using the bit fields then write the d32
 * value to the register.
 */
union gotgint_data {
	u32 d32;
	struct {
		/* Current Mode */
		unsigned reserved0_1:2;

		/* Session End Detected */
		unsigned sesenddet:1;

		unsigned reserved3_7:5;

		/* Session Request Success Status Change */
		unsigned sesreqsucstschng:1;
		/* Host Negotiation Success Status Change */
		unsigned hstnegsucstschng:1;

		unsigned reserver10_16:7;

		/* Host Negotiation Detected */
		unsigned hstnegdet:1;
		/* A-Device Timeout Change */
		unsigned adevtoutchng:1;
		/* Debounce Done */
		unsigned debdone:1;

		unsigned reserved31_20:12;

	} b;
};

/*
 * This union represents the bit fields of the Core AHB Configuration Register
 * (GAHBCFG).  Set/clear the bits using the bit fields then write the d32 value
 * to the register.
 */
union gahbcfg_data {
	u32 d32;
	struct {
		unsigned glblintrmsk:1;
#define DWC_GAHBCFG_GLBINT_ENABLE		1

		unsigned hburstlen:4;
#define DWC_GAHBCFG_INT_DMA_BURST_SINGLE	0
#define DWC_GAHBCFG_INT_DMA_BURST_INCR		1
#define DWC_GAHBCFG_INT_DMA_BURST_INCR4		3
#define DWC_GAHBCFG_INT_DMA_BURST_INCR8		5
#define DWC_GAHBCFG_INT_DMA_BURST_INCR16	7

		unsigned dmaenable:1;
#define DWC_GAHBCFG_DMAENABLE			1
		unsigned reserved:1;
		unsigned nptxfemplvl_txfemplvl:1;
		unsigned ptxfemplvl:1;
#define DWC_GAHBCFG_TXFEMPTYLVL_EMPTY		1
#define DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY	0
		unsigned reserved9_31:23;
	} b;
};

/*
 * This union represents the bit fields of the Core USB Configuration Register
 * (GUSBCFG).  Set the bits using the bit fields then write the d32 value to
 * the register.
 */
union gusbcfg_data {
	u32 d32;
	struct {
		unsigned toutcal:3;
		unsigned phyif:1;
		unsigned ulpi_utmi_sel:1;
		unsigned fsintf:1;
		unsigned physel:1;
		unsigned ddrsel:1;
		unsigned srpcap:1;
		unsigned hnpcap:1;
		unsigned usbtrdtim:4;
		unsigned nptxfrwnden:1;
		unsigned phylpwrclksel:1;
		unsigned otgutmifssel:1;
		unsigned ulpi_fsls:1;
		unsigned ulpi_auto_res:1;
		unsigned ulpi_clk_sus_m:1;
		unsigned ulpi_ext_vbus_drv:1;
		unsigned ulpi_int_vbus_indicator:1;
		unsigned term_sel_dl_pulse:1;
		unsigned reserved23_28:6;
		unsigned force_host_mode:1;
		unsigned force_device_mode:1;
		unsigned corrupt_tx_packet:1;
	} b;
};

/*
 * This union represents the bit fields of the Core Reset Register (GRSTCTL).
 * Set/clear the bits using the bit fields then write the d32 value to the
 * register.
 */
union grstctl_data {
	u32 d32;
	struct {
		/*
		 * Core Soft Reset (CSftRst) (Device and Host)
		 *
		 * The application can flush the control logic in the entire
		 * core using this bit. This bit resets the pipelines in the AHB
		 * Clock domain as well as the PHY Clock domain.
		 *
		 * The state machines are reset to an IDLE state, the control
		 * bits in the CSRs are cleared, all the transmit FIFOs and the
		 * receive FIFO are flushed.
		 *
		 * The status mask bits that control the generation of the
		 * interrupt, are cleared, to clear the interrupt. The interrupt
		 * status bits are not cleared, so the application can get the
		 * status of any events that occurred in the core after it has
		 * set this bit.
		 *
		 * Any transactions on the AHB are terminated as soon as
		 * possible following the protocol. Any transactions on the USB
		 * are terminated immediately.
		 *
		 * The configuration settings in the CSRs are unchanged, so the
		 * software doesn't have to reprogram these registers (Device
		 * Configuration/Host Configuration/Core System
		 * Configuration/Core PHY Configuration).
		 *
		 * The application can write to this bit, any time it wants to
		 * reset the core. This is a self clearing bit and the core
		 * clears this bit after all the necessary logic is reset in the
		 * core, which may take several clocks, depending on the current
		 * state of the core.
		 */
		unsigned csftrst:1;
		/*
		 * Hclk Soft Reset
		 *
		 * The application uses this bit to reset the control logic in
		 * the AHB clock domain. Only AHB clock domain pipelines are
		 * reset.
		 */
		unsigned hsftrst:1;
		/*
		 * Host Frame Counter Reset (Host Only)<br>
		 *
		 * The application can reset the (micro)frame number counter
		 * inside the core, using this bit. When the (micro)frame
		 * counter is reset, the subsequent SOF sent out by the core,
		 * will have a (micro)frame number of 0.
		 */
		unsigned hstfrm:1;
		/*
		 * In Token Sequence Learning Queue Flush (INTknQFlsh) (Device
		 * Only)
		 */
		unsigned intknqflsh:1;
		/*
		 * RxFIFO Flush (RxFFlsh) (Device and Host)
		 *
		 * The application can flush the entire Receive FIFO using this
		 * bit.
		 *
		 * The application must first ensure that the core is not in the
		 * middle of a transaction.
		 *
		 * The application should write into this bit, only after making
		 * sure that neither the DMA engine is reading from the RxFIFO
		 * nor the MAC is writing the data in to the FIFO.
		 *
		 * The application should wait until the bit is cleared before
		 * performing any other operations. This bit will takes 8 clocks
		 * (slowest of PHY or AHB clock) to clear.
		 */
		unsigned rxfflsh:1;
		/*
		 * TxFIFO Flush (TxFFlsh) (Device and Host).
		 *
		 * This bit is used to selectively flush a single or all
		 * transmit FIFOs.  The application must first ensure that the
		 * core is not in the middle of a transaction.
		 *
		 * The application should write into this bit, only after making
		 * sure that neither the DMA engine is writing into the TxFIFO
		 * nor the MAC is reading the data out of the FIFO.
		 *
		 * The application should wait until the core clears this bit,
		 * before performing any operations. This bit will takes 8
		 * clocks (slowest of PHY or AHB clock) to clear.
		 */
		unsigned txfflsh:1;

		/*
		 * TxFIFO Number (TxFNum) (Device and Host).
		 *
		 * This is the FIFO number which needs to be flushed, using the
		 * TxFIFO Flush bit. This field should not be changed until the
		 * TxFIFO Flush bit is cleared by the core.
		 *	 - 0x0 : Non Periodic TxFIFO Flush
		 *	 - 0x1 : Periodic TxFIFO #1 Flush in device mode
		 *	   or Periodic TxFIFO in host mode
		 *	 - 0x2 : Periodic TxFIFO #2 Flush in device mode.
		 *	 - ...
		 *	 - 0xF : Periodic TxFIFO #15 Flush in device mode
		 *	 - 0x10: Flush all the Transmit NonPeriodic and
		 *	   Transmit Periodic FIFOs in the core
		 */
		unsigned txfnum:5;
#define DWC_GRSTCTL_TXFNUM_ALL			0x10

		/* Reserved */
		unsigned reserved11_29:19;
		/*
		 * DMA Request Signal.  Indicated DMA request is in progress.
		 * Used for debug purpose.
		 */
		unsigned dmareq:1;
		/*
		 * AHB Master Idle.  Indicates the AHB Master State Machine is
		 * in IDLE condition.
		 */
		unsigned ahbidle:1;
	} b;
};


/*
 * This union represents the bit fields of the Core Interrupt Mask Register
 * (GINTMSK). Set/clear the bits using the bit fields then write the d32 value
 * to the register.
 */
union gintmsk_data {
	u32 d32;
	struct {
		unsigned reserved0:1;
		unsigned modemismatch:1;
		unsigned otgintr:1;
		unsigned sofintr:1;
		unsigned rxstsqlvl:1;
		unsigned nptxfempty:1;
		unsigned ginnakeff:1;
		unsigned goutnakeff:1;
		unsigned reserved8:1;
		unsigned i2cintr:1;
		unsigned erlysuspend:1;
		unsigned usbsuspend:1;
		unsigned usbreset:1;
		unsigned enumdone:1;
		unsigned isooutdrop:1;
		unsigned eopframe:1;
		unsigned reserved16:1;
		unsigned epmismatch:1;
		unsigned inepintr:1;
		unsigned outepintr:1;
		unsigned incomplisoin:1;
		unsigned incomplisoout:1;
		unsigned reserved22_23:2;
		unsigned portintr:1;
		unsigned hcintr:1;
		unsigned ptxfempty:1;
		unsigned reserved27:1;
		unsigned conidstschng:1;
		unsigned disconnect:1;
		unsigned sessreqintr:1;
		unsigned wkupintr:1;
	} b;
};

/*
 * This union represents the bit fields of the Core Interrupt Register
 * (GINTSTS).  Set/clear the bits using the bit fields then write the d32 value
 * to the register.
 */
union gintsts_data {
	u32 d32;
#define DWC_SOF_INTR_MASK			0x0008

	struct {
#define DWC_HOST_MODE 1
		unsigned curmode:1;
		unsigned modemismatch:1;
		unsigned otgintr:1;
		unsigned sofintr:1;
		unsigned rxstsqlvl:1;
		unsigned nptxfempty:1;
		unsigned ginnakeff:1;
		unsigned goutnakeff:1;
		unsigned reserved8:1;
		unsigned i2cintr:1;
		unsigned erlysuspend:1;
		unsigned usbsuspend:1;
		unsigned usbreset:1;
		unsigned enumdone:1;
		unsigned isooutdrop:1;
		unsigned eopframe:1;
		unsigned intokenrx:1;
		unsigned epmismatch:1;
		unsigned inepint:1;
		unsigned outepintr:1;
		unsigned incomplisoin:1;
		unsigned incomplisoout:1;
		unsigned reserved22_23:2;
		unsigned portintr:1;
		unsigned hcintr:1;
		unsigned ptxfempty:1;
		unsigned reserved27:1;
		unsigned conidstschng:1;
		unsigned disconnect:1;
		unsigned sessreqintr:1;
		unsigned wkupintr:1;
	} b;
};

/*
 * This union represents the bit fields in the Device Receive Status Read and
 * Pop Registers (GRXSTSR, GRXSTSP) Read the register into the d32 element then
 * read out the bits using the bit elements.
 */
union device_grxsts_data {
	u32 d32;
	struct {
		unsigned epnum:4;
		unsigned bcnt:11;
		unsigned dpid:2;

#define DWC_STS_DATA_UPDT		0x2	/* OUT Data Packet */
#define DWC_STS_XFER_COMP		0x3	/* OUT Data Transfer Complete */
#define DWC_DSTS_GOUT_NAK		0x1	/* Global OUT NAK */
#define DWC_DSTS_SETUP_COMP		0x4	/* Setup Phase Complete */
#define DWC_DSTS_SETUP_UPDT		0x6	/* SETUP Packet */
		unsigned pktsts:4;
		unsigned fn:4;
		unsigned reserved:7;
	} b;
};

/*
 * This union represents the bit fields in the Host Receive Status Read and
 * Pop Registers (GRXSTSR, GRXSTSP) Read the register into the d32 element then
 * read out the bits using the bit elements.
 */
union host_grxsts_data {
	u32 d32;
	struct {
		unsigned chnum:4;
		unsigned bcnt:11;
		unsigned dpid:2;

		unsigned pktsts:4;
#define DWC_GRXSTS_PKTSTS_IN			0x2
#define DWC_GRXSTS_PKTSTS_IN_XFER_COMP		0x3
#define DWC_GRXSTS_PKTSTS_DATA_TOGGLE_ERR	0x5
#define DWC_GRXSTS_PKTSTS_CH_HALTED		0x7

		unsigned reserved:11;
	} b;
};

/*
 * This union represents the bit fields in the FIFO Size Registers (HPTXFSIZ,
 * GNPTXFSIZ, DPTXFSIZn, DIEPTXFn). Read the register into the d32 element then
 * read out the bits using the bit elements.
 */
union fifosize_data {
	u32 d32;
	struct {
		unsigned startaddr:16;
		unsigned depth:16;
	} b;
};

/*
 * This union represents the bit fields in the Non-Periodic Transmit FIFO/Queue
 * Status Register (GNPTXSTS). Read the register into the d32 element then read
 * out the bits using the bit elements.
 */
union gnptxsts_data {
	u32 d32;
	struct {
		unsigned nptxfspcavail:16;
		unsigned nptxqspcavail:8;
		/*
		 * Top of the Non-Periodic Transmit Request Queue
		 *	- bit 24 - Terminate (Last entry for the selected
		 *	  channel/EP)
		 *	- bits 26:25 - Token Type
		 *	  - 2'b00 - IN/OUT
		 *	  - 2'b01 - Zero Length OUT
		 *	  - 2'b10 - PING/Complete Split
		 *	  - 2'b11 - Channel Halt
		 *	- bits 30:27 - Channel/EP Number
		 */
		unsigned nptxqtop_terminate:1;
		unsigned nptxqtop_token:2;
		unsigned nptxqtop_chnep:4;
		unsigned reserved:1;
	} b;
};

/*
 * This union represents the bit fields in the Transmit	FIFO Status Register
 * (DTXFSTS). Read the register into the d32 element then read out the bits
 * using the bit elements.
 */
union dtxfsts_data {
	u32 d32;
	struct {
		unsigned txfspcavail:16;
		unsigned reserved:16;
	} b;
};

/*
 * This union represents the bit fields in the I2C Control Register (I2CCTL).
 * Read the register into the d32 element then read out the bits using the bit
 * elements.
 */
union gi2cctl_data {
	u32 d32;
	struct {
		unsigned rwdata:8;
		unsigned regaddr:8;
		unsigned addr:7;
		unsigned i2cen:1;
		unsigned ack:1;
		unsigned i2csuspctl:1;
		unsigned i2cdevaddr:2;
		unsigned reserved:2;
		unsigned rw:1;
		unsigned bsydne:1;
	} b;
};

/*
 * This union represents the bit fields in the User HW Config1 Register.  Read
 * the register into the d32 element then read out the bits using the bit
 * elements.
 */
union hwcfg1_data {
	u32 d32;
	struct {
		unsigned ep_dir0:2;
		unsigned ep_dir1:2;
		unsigned ep_dir2:2;
		unsigned ep_dir3:2;
		unsigned ep_dir4:2;
		unsigned ep_dir5:2;
		unsigned ep_dir6:2;
		unsigned ep_dir7:2;
		unsigned ep_dir8:2;
		unsigned ep_dir9:2;
		unsigned ep_dir10:2;
		unsigned ep_dir11:2;
		unsigned ep_dir12:2;
		unsigned ep_dir13:2;
		unsigned ep_dir14:2;
		unsigned ep_dir15:2;
	} b;
};

/*
 * This union represents the bit fields in the User HW Config2 Register.  Read
 * the register into the d32 element then read out the bits using the bit
 * elements.
 */
union hwcfg2_data {
	u32 d32;
	struct {
		/* GHWCFG2 */
		unsigned op_mode:3;
#define DWC_HWCFG2_OP_MODE_HNP_SRP_CAPABLE_OTG		0
#define DWC_HWCFG2_OP_MODE_SRP_ONLY_CAPABLE_OTG		1
#define DWC_HWCFG2_OP_MODE_NO_HNP_SRP_CAPABLE_OTG	2
#define DWC_HWCFG2_OP_MODE_SRP_CAPABLE_DEVICE		3
#define DWC_HWCFG2_OP_MODE_NO_SRP_CAPABLE_DEVICE	4
#define DWC_HWCFG2_OP_MODE_SRP_CAPABLE_HOST		5
#define DWC_HWCFG2_OP_MODE_NO_SRP_CAPABLE_HOST		6

		unsigned architecture:2;
		unsigned point2point:1;
		unsigned hs_phy_type:2;
#define DWC_HWCFG2_HS_PHY_TYPE_NOT_SUPPORTED		0
#define DWC_HWCFG2_HS_PHY_TYPE_UTMI			1
#define DWC_HWCFG2_HS_PHY_TYPE_ULPI			2
#define DWC_HWCFG2_HS_PHY_TYPE_UTMI_ULPI		3

		unsigned fs_phy_type:2;
		unsigned num_dev_ep:4;
		unsigned num_host_chan:4;
		unsigned perio_ep_supported:1;
		unsigned dynamic_fifo:1;
		unsigned rx_status_q_depth:2;
		unsigned nonperio_tx_q_depth:2;
		unsigned host_perio_tx_q_depth:2;
		unsigned dev_token_q_depth:5;
		unsigned reserved31:1;
	} b;
};

/*
 * This union represents the bit fields in the User HW Config3 Register.  Read
 * the register into the d32 element then read out the bits using the bit
 * elements.
 */
union hwcfg3_data {
	u32 d32;
	struct {
		/* GHWCFG3 */
		unsigned xfer_size_cntr_width:4;
		unsigned packet_size_cntr_width:3;
		unsigned otg_func:1;
		unsigned i2c:1;
		unsigned vendor_ctrl_if:1;
		unsigned optional_features:1;
		unsigned synch_reset_type:1;
		unsigned reserved15_12:4;
		unsigned dfifo_depth:16;
	} b;
};

/*
 * This union represents the bit fields in the User HW Config4 Register.  Read
 * the register into the d32 element then read out the bits using the bit
 * elements.
 */
union hwcfg4_data {
	u32 d32;
	struct {
		unsigned num_dev_perio_in_ep:4;
		unsigned power_optimiz:1;
		unsigned min_ahb_freq:9;
		unsigned utmi_phy_data_width:2;
		unsigned num_dev_mode_ctrl_ep:4;
		unsigned iddig_filt_en:1;
		unsigned vbus_valid_filt_en:1;
		unsigned a_valid_filt_en:1;
		unsigned b_valid_filt_en:1;
		unsigned session_end_filt_en:1;
		unsigned ded_fifo_en:1;
		unsigned num_in_eps:4;
		unsigned reserved31_30:2;
	} b;
};

/*
 * Device Global Registers. Offsets 800h-BFFh
 *
 * The following structures define the size and relative field offsets for the
 * Device Mode Registers.
 *
 * These registers are visible only in Device mode and must not be accessed in
 * Host mode, as the results are unknown.
 */
struct device_global_regs {
	/* Device Configuration Register.			Offset 800h */
	u32 dcfg;
	/* Device Control Register.				Offset: 804h */
	u32 dctl;
	/* Device Status Register (Read Only).			Offset: 808h */
	u32 dsts;
	/* Reserved.						Offset: 80Ch */
	u32 unused;
	/* Device IN Endpoint Common Interrupt Mask Register.	Offset: 810h */
	u32 diepmsk;
	/* Device OUT Endpoint Common Interrupt MaskRegister.	Offset: 814h */
	u32 doepmsk;
	/* Device All Endpoints Interrupt Register.		Offset: 818h */
	u32 daint;
	/* Device All Endpoints Interrupt Mask Register.	Offset:	81Ch */
	u32 daintmsk;
	/* Device IN Token Queue Read Register-1 (Read Only).	Offset: 820h */
	u32 dtknqr1;
	/* Device IN Token Queue Read Register-2 (Read Only).	Offset: 824h */
	u32 dtknqr2;
	/* Device VBUS	 discharge Register.			Offset: 828h */
	u32 dvbusdis;
	/* Device VBUS Pulse Register.				Offset: 82Ch */
	u32 dvbuspulse;
	/*
	 * Device IN Token Queue Read Register-3 (Read Only).
	 * Device Thresholding control register (Read/Write)
	 *							Offset: 830h
	 */
	u32 dtknqr3_dthrctl;
	/*
	 * Device IN Token Queue Read Register-4 (Read Only).
	 * Device IN EPs empty Inr. Mask Register (Read/Write)
	 *							Offset: 834h
	 */
	u32 dtknqr4_fifoemptymsk;
};

/*
 * This union represents the bit fields in the Device Configuration Register.
 * Read the register into the d32 member then  set/clear the bits using the bit
 * elements.  Write the d32 member to the dcfg register.
 */
union dcfg_data {
	u32 d32;
	struct {
		/* Device Speed */
		unsigned devspd:2;
		/* Non Zero Length Status OUT Handshake */
		unsigned nzstsouthshk:1;
#define DWC_DCFG_SEND_STALL			1

		unsigned reserved3:1;
		/* Device Addresses */
		unsigned devaddr:7;
		/* Periodic Frame Interval */
		unsigned perfrint:2;
#define DWC_DCFG_FRAME_INTERVAL_80		0
#define DWC_DCFG_FRAME_INTERVAL_85		1
#define DWC_DCFG_FRAME_INTERVAL_90		2
#define DWC_DCFG_FRAME_INTERVAL_95		3

		unsigned reserved13_17:5;
		/* In Endpoint Mis-match count */
		unsigned epmscnt:4;
	} b;
};

/*
 * This union represents the bit fields in the Device Control Register.  Read
 * the register into the d32 member then set/clear the bits using the bit
 * elements.
 */
union dctl_data {
	u32 d32;
	struct {
		/* Remote Wakeup */
		unsigned rmtwkupsig:1;
		/* Soft Disconnect */
		unsigned sftdiscon:1;
		/* Global Non-Periodic IN NAK Status */
		unsigned gnpinnaksts:1;
		/* Global OUT NAK Status */
		unsigned goutnaksts:1;
		/* Test Control */
		unsigned tstctl:3;
		/* Set Global Non-Periodic IN NAK */
		unsigned sgnpinnak:1;
		/* Clear Global Non-Periodic IN NAK */
		unsigned cgnpinnak:1;
		/* Set Global OUT NAK */
		unsigned sgoutnak:1;
		/* Clear Global OUT NAK */
		unsigned cgoutnak:1;
		unsigned reserved:21;
	} b;
};

/*
 * This union represents the bit fields in the Device Status Register.  Read the
 * register into the d32 member then set/clear the bits using the bit elements.
 */
union dsts_data	{
	u32 d32;
	struct {
		/* Suspend Status */
		unsigned suspsts:1;
		/* Enumerated Speed */
		unsigned enumspd:2;
#define DWC_DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ		0
#define DWC_DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ		1
#define DWC_DSTS_ENUMSPD_LS_PHY_6MHZ			2
#define DWC_DSTS_ENUMSPD_FS_PHY_48MHZ			3

		/* Erratic Error */
		unsigned errticerr:1;
		unsigned reserved4_7:4;
		/* Frame or Microframe Number of the received SOF */
		unsigned soffn:14;
		unsigned reserved22_31:10;
	} b;
};

/*
 * This union represents the bit fields in the Device IN EP Interrupt Register
 * and the Device IN EP Common Mask Register. Read the register into the d32
 * member then set/clear the bits using the bit elements.
 */
union diepint_data {
	u32 d32;
	struct {
		/* Transfer complete mask */
		unsigned xfercompl:1;
		/* Endpoint disable mask */
		unsigned epdisabled:1;
		/* AHB Error mask */
		unsigned ahberr:1;
		/* TimeOUT Handshake mask (non-ISOC EPs) */
		unsigned timeout:1;
		/* IN Token received with TxF Empty mask */
		unsigned intktxfemp:1;
		/* IN Token Received with EP mismatch mask */
		unsigned intknepmis:1;
		/* IN Endpoint HAK Effective mask */
		unsigned inepnakeff:1;
		/* IN Endpoint HAK Effective mask */
		unsigned emptyintr:1;
		unsigned txfifoundrn:1;
		unsigned reserved08_31:23;
		} b;
};

/*
 * This union represents the bit fields in the Device OUT EP Interrupt
 * Registerand Device OUT EP Common Interrupt Mask Register.  Read the register
 * into the d32 member then set/clear the  bits using the bit elements.
 */
union doepint_data {
	u32 d32;
	struct {
		/* Transfer complete */
		unsigned xfercompl:1;
		/* Endpoint disable  */
		unsigned epdisabled:1;
		/* AHB Error */
		unsigned ahberr:1;
		/* Setup Phase Done (contorl EPs) */
		unsigned setup:1;
		unsigned reserved04_31:28;
	} b;
};

/*
 * This union represents the bit fields in the Device All EP Interrupt and Mask
 * Registers.  Read the register into the d32 member then set/clear the bits
 * using the bit elements.
 */
union daint_data {
	u32 d32;
	struct {
		/* IN Endpoint bits */
		unsigned in:16;
		/* OUT Endpoint bits */
		unsigned out:16;
	} ep;
	struct {
		/* IN Endpoint bits */
		unsigned inep0:1;
		unsigned inep1:1;
		unsigned inep2:1;
		unsigned inep3:1;
		unsigned inep4:1;
		unsigned inep5:1;
		unsigned inep6:1;
		unsigned inep7:1;
		unsigned inep8:1;
		unsigned inep9:1;
		unsigned inep10:1;
		unsigned inep11:1;
		unsigned inep12:1;
		unsigned inep13:1;
		unsigned inep14:1;
		unsigned inep15:1;
		/* OUT Endpoint bits */
		unsigned outep0:1;
		unsigned outep1:1;
		unsigned outep2:1;
		unsigned outep3:1;
		unsigned outep4:1;
		unsigned outep5:1;
		unsigned outep6:1;
		unsigned outep7:1;
		unsigned outep8:1;
		unsigned outep9:1;
		unsigned outep10:1;
		unsigned outep11:1;
		unsigned outep12:1;
		unsigned outep13:1;
		unsigned outep14:1;
		unsigned outep15:1;
	} b;
};

/*
 * This union represents the bit fields in the Device IN Token Queue Read
 * Registers.  Read the register into the d32 member.  READ-ONLY Register
 */
union dtknq1_data {
	u32 d32;
	struct {
		/* In Token Queue Write Pointer */
		unsigned intknwptr:5;
		/* Reserved */
		unsigned reserved05_06:2;
		/* write pointer has wrapped. */
		unsigned wrap_bit:1;
		/* EP Numbers of IN Tokens 0 ... 4 */
		unsigned epnums0_5:24;
	} b;
};

/*
 * This union represents Threshold control Register Read and write the register
 * into the d32 member.  READ-WRITABLE Register
 */
union dthrctl_data {
	u32 d32;
	struct {
		/* non ISO Tx Thr. Enable */
		unsigned non_iso_thr_en:1;
		/* ISO Tx Thr. Enable */
		unsigned iso_thr_en:1;
		/* Tx Thr. Length */
		unsigned tx_thr_len:9;
		/* Reserved */
		unsigned reserved11_15:5;
		/* Rx Thr. Enable */
		unsigned rx_thr_en:1;
		/* Rx Thr. Length */
		unsigned rx_thr_len:9;
		/* Reserved */
		unsigned reserved26_31:6;
	} b;
};

/*
 * Device Logical IN Endpoint-Specific Registers. Offsets 900h-AFCh
 *
 * There will be one set of endpoint registers per logical endpoint implemented.
 *
 * These registers are visible only in Device mode and must not be accessed in
 * Host mode, as the results are unknown.
 */
struct device_in_ep_regs {
	/*
	 * Device IN Endpoint Control Register.
	 * Offset: 900h + (ep_num * 20h) + 00h
	 */
	u32 diepctl;
	/* Reserved. Offset:900h + (ep_num * 20h) + 04h */
	u32 reserved04;
	/*
	 * Device IN Endpoint Interrupt Register.
	 * Offset: 900h + (ep_num * 20h) + 08h
	 */
	u32 diepint;
	/* Reserved. Offset:900h + (ep_num * 20h) + 0Ch */
	u32 reserved0C;
	/*
	 * Device IN Endpoint Transfer Size Register.
	 * Offset: 900h + (ep_num * 20h) + 10h
	 */
	u32 dieptsiz;
	/*
	 * Device IN Endpoint DMA Address Register.
	 * Offset: 900h + (ep_num * 20h) + 14h
	 */
	u32 diepdma;
	/*
	 * Device IN Endpoint Transmit FIFO Status Register.
	 * Offset: 900h + (ep_num * 20h) + 18h
	 */
	u32 dtxfsts;
	/*
	 * Reserved.
	 * Offset: 900h + (ep_num * 20h) + 1Ch - 900h + (ep_num * 20h) + 1Ch
	 */
	u32 reserved18;
};

/*
 * Device Logical OUT Endpoint-Specific Registers. Offsets: B00h-CFCh
 *
 * There will be one set of endpoint registers per logical endpoint implemented.
 *
 * These registers are visible only in Device mode and must not be accessed in
 * Host mode, as the results are unknown.
 */
struct device_out_ep_regs {
	/*
	 * Device OUT Endpoint Control Register.
	 * Offset: B00h + (ep_num * 20h) + 00h
	 */
	u32 doepctl;
	/*
	 * Device OUT Endpoint Frame number Register.
	 * Offset: B00h + (ep_num * 20h) + 04h
	 */
	u32 doepfn;
	/*
	 * Device OUT Endpoint Interrupt Register.
	 * Offset: B00h + (ep_num * 20h) + 08h
	 */
	u32 doepint;
	/* Reserved. Offset:B00h + (ep_num * 20h) + 0Ch */
	u32 reserved0C;
	/*
	 * Device OUT Endpoint Transfer Size Register.
	 * Offset: B00h + (ep_num * 20h) + 10h
	 */
	u32 doeptsiz;
	/*
	 * Device OUT Endpoint DMA Address Register.
	 * Offset: B00h + (ep_num * 20h) + 14h
	 */
	u32 doepdma;
	/*
	 * Reserved.
	 * Offset:B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch
	 */
	u32 unused[2];
};

/*
 * This union represents the bit fields in the Device EP Control Register.  Read
 * the register into the d32 member then set/clear the bits using the bit
 * elements.
 */
union depctl_data {
	u32 d32;
	struct {
		/* Maximum Packet Size
		 * IN/OUT EPn
		 * IN/OUT EP0 - 2 bits
		 *	 2'b00: 64 Bytes
		 *	 2'b01: 32
		 *	 2'b10: 16
		 *	 2'b11: 8
		 */
		unsigned mps:11;
#define DWC_DEP0CTL_MPS_64			0
#define DWC_DEP0CTL_MPS_32			1
#define DWC_DEP0CTL_MPS_16			2
#define DWC_DEP0CTL_MPS_8			3

		/*
		 * Next Endpoint
		 * IN EPn/IN EP0
		 * OUT EPn/OUT EP0 - reserved
		 */
		unsigned nextep:4;
		/* USB Active Endpoint */
		unsigned usbactep:1;
		/*
		 * Endpoint DPID (INTR/Bulk IN and OUT endpoints) This field
		 * contains the PID of the packet going to be received or
		 * transmitted on this endpoint. The application should program
		 * the PID of the first packet going to be received or
		 * transmitted on this endpoint , after the endpoint is
		 * activated. Application use the SetD1PID and SetD0PID fields
		 * of this register to program either D0 or D1 PID.
		 *
		 * The encoding for this field is
		 *	 - 0: D0
		 *	 - 1: D1
		 */
		unsigned dpid:1;
		/* NAK Status */
		unsigned naksts:1;
		/* Endpoint Type
		 *	2'b00: Control
		 *	2'b01: Isochronous
		 *	2'b10: Bulk
		 *	2'b11: Interrupt
		 */
		unsigned eptype:2;
		/*
		 * Snoop Mode
		 * OUT EPn/OUT EP0
		 * IN EPn/IN EP0 - reserved
		 */
		unsigned snp:1;
		/* Stall Handshake */
		unsigned stall:1;
		/*
		 * Tx Fifo Number
		 * IN EPn/IN EP0
		 * OUT EPn/OUT EP0 - reserved
		 */
		unsigned txfnum:4;
		/* Clear NAK */
		unsigned cnak:1;
		/* Set NAK */
		unsigned snak:1;
		/*
		 * Set DATA0 PID (INTR/Bulk IN and OUT endpoints)
		 *
		 * Writing to this field sets the Endpoint DPID (DPID) field in
		 * this register to DATA0. Set Even (micro)frame (SetEvenFr)
		 * (ISO IN and OUT Endpoints)
		 *
		 * Writing to this field sets the Even/Odd (micro)frame
		 * (EO_FrNum) field to even (micro) frame.
		 */
		unsigned setd0pid:1;
		/*
		 * Set DATA1 PID (INTR/Bulk IN and OUT endpoints)
		 *
		 * Writing to this field sets the Endpoint DPID (DPID) field in
		 * this register to DATA1 Set Odd (micro)frame (SetOddFr) (ISO
		 * IN and OUT Endpoints)
		 *
		 * Writing to this field sets the Even/Odd (micro)frame
		 * (EO_FrNum) field to odd (micro) frame.
		 */
		unsigned setd1pid:1;
		/* Endpoint Disable */
		unsigned epdis:1;
		/* Endpoint Enable */
		unsigned epena:1;
		} b;
};

/*
 * This union represents the bit fields in the Device EP Transfer Size Register.
 * Read the register into the d32 member then set/clear the bits using the bit
 * elements.
 */
union deptsiz_data {
	u32 d32;
	struct {
		/* Transfer size */
		unsigned xfersize:19;
		/* Packet Count */
		unsigned pktcnt:10;
		/* Multi Count - Periodic IN endpoints */
		unsigned mc:2;
		unsigned reserved:1;
	} b;
};

/*
 * This union represents the bit fields in the Device EP 0 Transfer Size
 * Register.  Read the register into the d32 member then set/clear the bits
 * using the bit elements.
 */
union deptsiz0_data {
	u32 d32;
	struct {
		/* Transfer size */
		unsigned xfersize:7;
		/* Reserved */
		unsigned reserved7_18:12;
		/* Packet Count */
		unsigned pktcnt:2;
		/* Reserved */
		unsigned reserved21_28:9;
		/* Setup Packet Count (DOEPTSIZ0 Only) */
		unsigned supcnt:2;
		unsigned reserved31;
	} b;
};

#define MAX_PERIO_FIFOS			15	/* Max periodic FIFOs */
#define MAX_TX_FIFOS			15	/* Max non-periodic FIFOs */
#define MAX_EPS_CHANNELS		16	/* Max Endpoints/HostChannels */

/*
 * The device_if structure contains information needed to manage the
 * DWC_otg controller acting in device mode. It represents the programming view
 * of the device-specific aspects of the controller.
 */
struct device_if {
	/* Device Global Registers starting at offset 800h */
	struct device_global_regs *dev_global_regs;
#define DWC_DEV_GLOBAL_REG_OFFSET		0x800

	/* Device Logical IN Endpoint-Specific Registers 900h-AFCh */
	struct device_in_ep_regs *in_ep_regs[MAX_EPS_CHANNELS/2];
#define DWC_DEV_IN_EP_REG_OFFSET		0x900
#define DWC_EP_REG_OFFSET			0x20

	/* Device Logical OUT Endpoint-Specific Registers B00h-CFCh */
	struct device_out_ep_regs *out_ep_regs[MAX_EPS_CHANNELS/2];
#define DWC_DEV_OUT_EP_REG_OFFSET		0xB00

	/* Device Speed		0: Unknown, 1: LS, 2:FS, 3: HS */
	u8 speed;
	/* Number # of Tx EP range: 0-15 exept ep0 */
	u8 num_in_eps;
	/* Number # of Rx EP range: 0-15 exept ep0 */
	u8 num_out_eps;

	/* Size of periodic FIFOs (Bytes) */
	u16 perio_tx_fifo_size[MAX_PERIO_FIFOS];

	/* Size of Tx FIFOs (Bytes) */
	u16 tx_fifo_size[MAX_TX_FIFOS];

	/* Thresholding enable flags and length varaiables */
	u16 rx_thr_en;
	u16 iso_tx_thr_en;
	u16 non_iso_tx_thr_en;
	u16 rx_thr_length;
	u16 tx_thr_length;
};

/*
 * The Host Global Registers structure defines the size and relative
 * field offsets for the Host Mode Global Registers.  Host Global
 * Registers offsets 400h-7FFh.
*/
struct host_global_regs {
	/* Host Configuration Register.   Offset: 400h */
	u32 hcfg;
	/* Host Frame Interval Register.	Offset: 404h */
	u32 hfir;
	/* Host Frame Number / Frame Remaining Register. Offset: 408h */
	u32 hfnum;
	/* Reserved.	Offset: 40Ch */
	u32 reserved40C;
	/* Host Periodic Transmit FIFO/ Queue Status Register. Offset: 410h */
	u32 hptxsts;
	/* Host All Channels Interrupt Register. Offset: 414h */
	u32 haint;
	/* Host All Channels Interrupt Mask Register. Offset: 418h */
	u32 haintmsk;
};

/*
 * This union represents the bit fields in the Host Configuration Register.
 * Read the register into the d32 member then set/clear the bits using
 * the bit elements. Write the d32 member to the hcfg register.
 */
union hcfg_data {
	u32 d32;
	struct {
		/* FS/LS Phy Clock Select */
		unsigned fslspclksel:2;
#define DWC_HCFG_30_60_MHZ			0
#define DWC_HCFG_48_MHZ				1
#define DWC_HCFG_6_MHZ				2

		/* FS/LS Only Support */
		unsigned fslssupp:1;
	} b;
};

/*
 * This union represents the bit fields in the Host Frame Remaing/Number
 * Register.
 */
union hfir_data {
	u32 d32;
	struct {
		unsigned frint:16;
		unsigned reserved:16;
	} b;
};

/*
 * This union represents the bit fields in the Host Frame Remaing/Number
 * Register.
 */
union hfnum_data {
	u32 d32;
	struct {
		unsigned frnum:16;
#define DWC_HFNUM_MAX_FRNUM			0x3FFF
		unsigned frrem:16;
	} b;
};

union hptxsts_data {
	u32 d32;
	struct {
		unsigned ptxfspcavail:16;
		unsigned ptxqspcavail:8;
		/*
		 * Top of the Periodic Transmit Request Queue
		 *	- bit 24 - Terminate (last entry of selected channel)
		 *	- bits 26:25 - Token Type
		 *	  - 2'b00 - Zero length
		 *	  - 2'b01 - Ping
		 *	  - 2'b10 - Disable
		 *	- bits 30:27 - Channel Number
		 *	- bit 31 - Odd/even microframe
		 */
		unsigned ptxqtop_terminate:1;
		unsigned ptxqtop_token:2;
		unsigned ptxqtop_chnum:4;
		unsigned ptxqtop_odd:1;
	} b;
};

/*
 * This union represents the bit fields in the Host Port Control and Status
 * Register. Read the register into the d32 member then set/clear the bits using
 * the bit elements. Write the d32 member to the hprt0 register.
 */
union hprt0_data {
	u32 d32;
	struct {
		unsigned prtconnsts:1;
		unsigned prtconndet:1;
		unsigned prtena:1;
		unsigned prtenchng:1;
		unsigned prtovrcurract:1;
		unsigned prtovrcurrchng:1;
		unsigned prtres:1;
		unsigned prtsusp:1;
		unsigned prtrst:1;
		unsigned reserved9:1;
		unsigned prtlnsts:2;
		unsigned prtpwr:1;
		unsigned prttstctl:4;
		unsigned prtspd:2;
#define DWC_HPRT0_PRTSPD_HIGH_SPEED		0
#define DWC_HPRT0_PRTSPD_FULL_SPEED		1
#define DWC_HPRT0_PRTSPD_LOW_SPEED		2
		unsigned reserved19_31:13;
	} b;
};

/*
 * This union represents the bit fields in the Host All Interrupt Register.
 */
union haint_data {
	u32 d32;
	struct {
		unsigned ch0:1;
		unsigned ch1:1;
		unsigned ch2:1;
		unsigned ch3:1;
		unsigned ch4:1;
		unsigned ch5:1;
		unsigned ch6:1;
		unsigned ch7:1;
		unsigned ch8:1;
		unsigned ch9:1;
		unsigned ch10:1;
		unsigned ch11:1;
		unsigned ch12:1;
		unsigned ch13:1;
		unsigned ch14:1;
		unsigned ch15:1;
		unsigned reserved:16;
	} b;

	struct {
		unsigned chint:16;
		unsigned reserved:16;
	} b2;
};

/*
 * This union represents the bit fields in the Host All Interrupt Register.
 */
union haintmsk_data {
	u32 d32;
	struct {
		unsigned ch0:1;
		unsigned ch1:1;
		unsigned ch2:1;
		unsigned ch3:1;
		unsigned ch4:1;
		unsigned ch5:1;
		unsigned ch6:1;
		unsigned ch7:1;
		unsigned ch8:1;
		unsigned ch9:1;
		unsigned ch10:1;
		unsigned ch11:1;
		unsigned ch12:1;
		unsigned ch13:1;
		unsigned ch14:1;
		unsigned ch15:1;
		unsigned reserved:16;
	} b;

	struct {
		unsigned chint:16;
		unsigned reserved:16;
	} b2;
};

/*
 * Host Channel Specific Registers. 500h-5FCh
 */
struct dwc_hc_regs {
	/*
	 * Host Channel 0 Characteristic Register.
	 * Offset: 500h + (chan_num * 20h) + 00h
	 */
	u32 hcchar;
	/*
	 * Host Channel 0 Split Control Register.
	 * Offset: 500h + (chan_num * 20h) + 04h
	 */
	u32 hcsplt;
	/*
	 * Host Channel 0 Interrupt Register.
	 * Offset: 500h + (chan_num * 20h) + 08h
	 */
	u32 hcint;
	/*
	 * Host Channel 0 Interrupt Mask Register.
	 * Offset: 500h + (chan_num * 20h) + 0Ch
	 */
	u32 hcintmsk;
	/*
	 * Host Channel 0 Transfer Size Register.
	 * Offset: 500h + (chan_num * 20h) + 10h
	 */
	u32 hctsiz;
	/*
	 * Host Channel 0 DMA Address Register.
	 * Offset: 500h + (chan_num * 20h) + 14h
	 */
	u32 hcdma;
	/* Reserved.
	 * Offset: 500h + (chan_num * 20h) + 18h - 500h + (chan_num * 20h) + 1Ch
	 */
	u32 reserved[2];
};

/*
 * This union represents the bit fields in the Host Channel Characteristics
 * Register. Read the register into the d32 member then set/clear the bits using
 * the bit elements. Write the d32 member to the hcchar register.
 */
union hcchar_data {
	u32 d32;
	struct {
		/* Maximum packet size in bytes */
		unsigned mps:11;
		/* Endpoint number */
		unsigned epnum:4;
		/* 0: OUT, 1: IN */
		unsigned epdir:1;
		unsigned reserved:1;
		/* 0: Full/high speed device, 1: Low speed device */
		unsigned lspddev:1;
		/* 0: Control, 1: Isoc, 2: Bulk, 3: Intr */
		unsigned eptype:2;
		/* Packets per frame for periodic transfers. 0 is reserved. */
		unsigned multicnt:2;
		/* Device address */
		unsigned devaddr:7;
		/*
		 * Frame to transmit periodic transaction.
		 * 0: even, 1: odd
		 */
		unsigned oddfrm:1;
		/* Channel disable */
		unsigned chdis:1;
		/* Channel enable */
		unsigned chen:1;
	} b;
};

union hcsplt_data {
	u32 d32;
	struct {
		/* Port Address */
		unsigned prtaddr:7;
		/* Hub Address */
		unsigned hubaddr:7;
		/* Transaction Position */
		unsigned xactpos:2;
#define DWC_HCSPLIT_XACTPOS_MID			0
#define DWC_HCSPLIT_XACTPOS_END			1
#define DWC_HCSPLIT_XACTPOS_BEGIN		2
#define DWC_HCSPLIT_XACTPOS_ALL			3

		/* Do Complete Split */
		unsigned compsplt:1;
		/* Reserved */
		unsigned reserved:14;
		/* Split Enble */
		unsigned spltena:1;
	} b;
};


/*
 * This union represents the bit fields in the Host All Interrupt Register.
 */
union hcint_data {
	u32 d32;
	struct {
		/* Transfer Complete */
		unsigned xfercomp:1;
		/* Channel Halted */
		unsigned chhltd:1;
		/* AHB Error */
		unsigned ahberr:1;
		/* STALL Response Received */
		unsigned stall:1;
		/* NAK Response Received */
		unsigned nak:1;
		/* ACK Response Received */
		unsigned ack:1;
		/* NYET Response Received */
		unsigned nyet:1;
		/* Transaction Err */
		unsigned xacterr:1;
		/* Babble Error */
		unsigned bblerr:1;
		/* Frame Overrun */
		unsigned frmovrun:1;
		/* Data Toggle Error */
		unsigned datatglerr:1;
		/* Reserved */
		unsigned reserved:21;
	} b;
};

/*
 * This union represents the bit fields in the Host Channel Transfer Size
 * Register. Read the register into the d32 member then set/clear the bits using
 * the bit elements. Write the d32 member to the hcchar register.
 */
union hctsiz_data {
	u32 d32;
	struct {
		/* Total transfer size in bytes */
		unsigned xfersize:19;
		/* Data packets to transfer */
		unsigned pktcnt:10;
		/*
		 * Packet ID for next data packet
		 * 0: DATA0
		 * 1: DATA2
		 * 2: DATA1
		 * 3: MDATA (non-Control), SETUP (Control)
		 */
		unsigned pid:2;
#define DWC_HCTSIZ_DATA0			0
#define DWC_HCTSIZ_DATA1			2
#define DWC_HCTSIZ_DATA2			1
#define DWC_HCTSIZ_MDATA			3
#define DWC_HCTSIZ_SETUP			3

		/* Do PING protocol when 1 */
		unsigned dopng:1;
	} b;
};

/*
 * This union represents the bit fields in the Host Channel Interrupt Mask
 * Register. Read the register into the d32 member then set/clear the bits using
 * the bit elements. Write the d32 member to the hcintmsk register.
 */
union hcintmsk_data {
	u32 d32;
	struct {
		unsigned xfercompl:1;
		unsigned chhltd:1;
		unsigned ahberr:1;
		unsigned stall:1;
		unsigned nak:1;
		unsigned ack:1;
		unsigned nyet:1;
		unsigned xacterr:1;
		unsigned bblerr:1;
		unsigned frmovrun:1;
		unsigned datatglerr:1;
		unsigned reserved:21;
	} b;
};

/* OTG Host Interface Structure.
 *
 * The OTG Host Interface Structure structure contains information needed to
 * manage the DWC_otg controller acting in host mode. It represents the
 * programming view of the host-specific aspects of the controller.
 */
struct dwc_host_if {
	/* Host Global Registers starting at offset 400h.*/
	struct host_global_regs *host_global_regs;
#define DWC_OTG_HOST_GLOBAL_REG_OFFSET			0x400

	/* Host Port 0 Control and Status Register */
	u32 *hprt0;
#define DWC_OTG_HOST_PORT_REGS_OFFSET			0x440

	/* Host Channel Specific Registers at offsets 500h-5FCh. */
	struct dwc_hc_regs *hc_regs[MAX_EPS_CHANNELS];
#define DWC_OTG_HOST_CHAN_REGS_OFFSET			0x500
#define DWC_OTG_CHAN_REGS_OFFSET			0x20

	/* Host configuration information */
	/* Number of Host Channels (range: 1-16) */
	u8 num_host_channels;
	/* Periodic EPs supported (0: no, 1: yes) */
	u8 perio_eps_supported;
	/* Periodic Tx FIFO Size (Only 1 host periodic Tx FIFO) */
	u16 perio_tx_fifo_size;
};

/*
 * This union represents the bit fields in the Power and Clock Gating Control
 * Register. Read the register into the d32 member then set/clear the bits using
 * the bit elements.
 */
union pcgcctl_data {
	u32 d32;
	struct {
		/* Stop Pclk */
		unsigned stoppclk:1;
		/* Gate Hclk */
		unsigned gatehclk:1;
		/* Power Clamp */
		unsigned pwrclmp:1;
		/* Reset Power Down Modules */
		unsigned rstpdwnmodule:1;
		/* PHY Suspended */
		unsigned physuspended:1;
		unsigned reserved:27;
	} b;
};
#endif /* CONFIG_DWC_OTG_REG_LE */
#endif
