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
 * Modified by Chuck Meade <chuck at theptrgroup.com>
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

#include "dwc_otg_hcd.h"

/* This file contains the implementation of the HCD Interrupt handlers.	*/
static const int erratum_usb09_patched;
static const int deferral_on = 1;
static const int nak_deferral_delay = 8;
static const int nyet_deferral_delay = 1;

/**
 * Handles the start-of-frame interrupt in host mode. Non-periodic
 * transactions may be queued to the DWC_otg controller for the current
 * (micro)frame. Periodic transactions may be queued to the controller for the
 * next (micro)frame.
 */
static int dwc_otg_hcd_handle_sof_intr(struct dwc_hcd *hcd)
{
	union hfnum_data hfnum;
	struct list_head *qh_entry;
	struct dwc_qh *qh;
	enum dwc_transaction_type tr_type;
	union gintsts_data gintsts = {.d32 = 0};

	hfnum.d32 =
		dwc_read_reg32(&hcd->core_if->host_if->host_global_regs->hfnum);

	hcd->frame_number = hfnum.b.frnum;

	/* Determine whether any periodic QHs should be executed. */
	qh_entry = hcd->periodic_sched_inactive.next;
	while (qh_entry != &hcd->periodic_sched_inactive) {
		qh = list_entry(qh_entry, struct dwc_qh, qh_list_entry);
		qh_entry = qh_entry->next;

		/*
		 * If needed, move QH to the ready list to be executed next
		 * (micro)frame.
		 */
		if (dwc_frame_num_le(qh->sched_frame, hcd->frame_number))
			list_move(&qh->qh_list_entry,
				&hcd->periodic_sched_ready);
	}

	tr_type = dwc_otg_hcd_select_transactions(hcd);
	if (tr_type != DWC_OTG_TRANSACTION_NONE)
		dwc_otg_hcd_queue_transactions(hcd, tr_type);

	/* Clear interrupt */
	gintsts.b.sofintr = 1;
	dwc_write_reg32(gintsts_reg(hcd), gintsts.d32);
	return 1;
}

/**
 * Handles the Rx Status Queue Level Interrupt, which indicates that there is at
 * least one packet in the Rx FIFO.  The packets are moved from the FIFO to
 * memory if the DWC_otg controller is operating in Slave mode.
 */
static int dwc_otg_hcd_handle_rx_status_q_level_intr(struct dwc_hcd *hcd)
{
	union host_grxsts_data grxsts;
	struct dwc_hc *hc = NULL;

	grxsts.d32 = dwc_read_reg32(&hcd->core_if->core_global_regs->grxstsp);
	hc = hcd->hc_ptr_array[grxsts.b.chnum];

	/* Packet Status */
	switch (grxsts.b.pktsts) {
	case DWC_GRXSTS_PKTSTS_IN:
		/* Read the data into the host buffer. */
		if (grxsts.b.bcnt > 0) {
			dwc_otg_read_packet(hcd->core_if, hc->xfer_buff,
						grxsts.b.bcnt);
			/* Update the HC fields for the next packet received. */
			hc->xfer_count += grxsts.b.bcnt;
			hc->xfer_buff += grxsts.b.bcnt;
		}
	case DWC_GRXSTS_PKTSTS_IN_XFER_COMP:
	case DWC_GRXSTS_PKTSTS_DATA_TOGGLE_ERR:
	case DWC_GRXSTS_PKTSTS_CH_HALTED:
		/* Handled in interrupt, just ignore data */
		break;
	default:
		printk(KERN_ERR "RX_STS_Q Interrupt: Unknown status %d\n",
					grxsts.b.pktsts);
		break;
	}
	return 1;
}

/**
 * This interrupt occurs when the non-periodic Tx FIFO is half-empty. More
 * data packets may be written to the FIFO for OUT transfers. More requests
 * may be written to the non-periodic request queue for IN transfers. This
 * interrupt is enabled only in Slave mode.
 */
static int dwc_otg_hcd_handle_np_tx_fifo_empty_intr(struct dwc_hcd *hcd)
{
	dwc_otg_hcd_queue_transactions(hcd, DWC_OTG_TRANSACTION_NON_PERIODIC);
	return 1;
}

/**
 * This interrupt occurs when the periodic Tx FIFO is half-empty. More data
 * packets may be written to the FIFO for OUT transfers. More requests may be
 * written to the periodic request queue for IN transfers. This interrupt is
 * enabled only in Slave mode.
 */
static int dwc_otg_hcd_handle_perio_tx_fifo_empty_intr(struct dwc_hcd *hcd)
{
	dwc_otg_hcd_queue_transactions(hcd, DWC_OTG_TRANSACTION_PERIODIC);
	return 1;
}

/**
 * When the port changes to enabled it may be necessary to adjust the phy clock
 * speed.
 */
static int adjusted_phy_clock_speed(struct dwc_hcd *hcd, union hprt0_data hprt0)
{
	int adjusted = 0;
	union gusbcfg_data usbcfg;
	struct core_params *params = hcd->core_if->core_params;
	struct core_global_regs *g_regs = hcd->core_if->core_global_regs;
	struct host_global_regs *h_regs =
		hcd->core_if->host_if->host_global_regs;

	usbcfg.d32 = dwc_read_reg32(&g_regs->gusbcfg);

	if (hprt0.b.prtspd == DWC_HPRT0_PRTSPD_LOW_SPEED ||
		hprt0.b.prtspd == DWC_HPRT0_PRTSPD_FULL_SPEED) {
		/* Low power */
		union hcfg_data hcfg;

		if (usbcfg.b.phylpwrclksel == 0) {
			/* Set PHY low power clock select for FS/LS devices */
			usbcfg.b.phylpwrclksel = 1;
			dwc_write_reg32(&g_regs->gusbcfg, usbcfg.d32);
			adjusted = 1;
		}

		hcfg.d32 = dwc_read_reg32(&h_regs->hcfg);
		if (hprt0.b.prtspd == DWC_HPRT0_PRTSPD_LOW_SPEED &&
				params->host_ls_low_power_phy_clk ==
				DWC_HOST_LS_LOW_POWER_PHY_CLK_PARAM_6MHZ) {
			/* 6 MHZ, check for 6 MHZ clock select */
			if (hcfg.b.fslspclksel != DWC_HCFG_6_MHZ) {
				hcfg.b.fslspclksel = DWC_HCFG_6_MHZ;
				dwc_write_reg32(&h_regs->hcfg, hcfg.d32);
				adjusted = 1;
			}
		} else if (hcfg.b.fslspclksel != DWC_HCFG_48_MHZ) {
			/* 48 MHZ and clock select is not 48 MHZ */
			hcfg.b.fslspclksel = DWC_HCFG_48_MHZ;
			dwc_write_reg32(&h_regs->hcfg, hcfg.d32);
			adjusted = 1;
		}
	} else if (usbcfg.b.phylpwrclksel == 1) {
		usbcfg.b.phylpwrclksel = 0;
		dwc_write_reg32(&g_regs->gusbcfg, usbcfg.d32);
		adjusted = 1;
	}
	if (adjusted)
		schedule_work(&hcd->usb_port_reset);

	return adjusted;
}

/**
 * Helper function to handle the port enable changed interrupt when the port
 * becomes enabled.  Checks if we need to adjust the PHY clock speed for low
 * power and  adjusts it if needed.
 */
static void port_enabled(struct dwc_hcd *hcd, union hprt0_data hprt0)
{
	if (hcd->core_if->core_params->host_support_fs_ls_low_power)
		if (!adjusted_phy_clock_speed(hcd, hprt0))
			hcd->flags.b.port_reset_change = 1;
}

/**
 * There are multiple conditions that can cause a port interrupt. This function
 * determines which interrupt conditions have occurred and handles them
 * appropriately.
 */
static int dwc_otg_hcd_handle_port_intr(struct dwc_hcd *hcd)
{
	int retval = 0;
	union hprt0_data hprt0;
	union hprt0_data hprt0_modify;

	hprt0.d32 = dwc_read_reg32(hcd->core_if->host_if->hprt0);
	hprt0_modify.d32 = dwc_read_reg32(hcd->core_if->host_if->hprt0);

	/*
	 * Clear appropriate bits in HPRT0 to clear the interrupt bit in
	 * GINTSTS
	 */
	hprt0_modify.b.prtena = 0;
	hprt0_modify.b.prtconndet = 0;
	hprt0_modify.b.prtenchng = 0;
	hprt0_modify.b.prtovrcurrchng = 0;

	/* Port connect detected interrupt */
	if (hprt0.b.prtconndet) {
		/* Set the status flags and clear interrupt*/
		hcd->flags.b.port_connect_status_change = 1;
		hcd->flags.b.port_connect_status = 1;
		hprt0_modify.b.prtconndet = 1;

		/* B-Device has connected, Delete the connection timer. */
		del_timer_sync(&hcd->conn_timer);

		/*
		 * The Hub driver asserts a reset when it sees port connect
		 * status change flag
		 */
		retval |= 1;
	}

	/* Port enable changed interrupt */
	if (hprt0.b.prtenchng) {
		/* Set the internal flag if the port was disabled */
		if (hprt0.b.prtena)
			port_enabled(hcd, hprt0);
		else
			hcd->flags.b.port_enable_change = 1;

		/* Clear the interrupt */
		hprt0_modify.b.prtenchng = 1;
		retval |= 1;
	}

	/* Overcurrent change interrupt	*/
	if (hprt0.b.prtovrcurrchng) {
		hcd->flags.b.port_over_current_change = 1;
		hprt0_modify.b.prtovrcurrchng = 1;
		retval |= 1;
	}

	/* Clear the port interrupts */
	dwc_write_reg32(hcd->core_if->host_if->hprt0, hprt0_modify.d32);
	return retval;
}

/**
 * Gets the actual length of a transfer after the transfer halts. halt_status
 * holds the reason for the halt.
 *
 * For IN transfers where halt_status is DWC_OTG_HC_XFER_COMPLETE, _short_read
 * is set to 1 upon return if less than the requested number of bytes were
 * transferred. Otherwise, _short_read is set to 0 upon return. _short_read may
 * also be NULL on entry, in which case it remains unchanged.
 */
static u32 get_actual_xfer_length(struct dwc_hc *hc, struct dwc_hc_regs *regs,
			struct dwc_qtd *qtd, enum dwc_halt_status halt_status,
			int *_short_read)
{
	union hctsiz_data hctsiz;
	u32 length;

	if (_short_read)
		*_short_read = 0;

	hctsiz.d32 = dwc_read_reg32(&regs->hctsiz);
	if (halt_status == DWC_OTG_HC_XFER_COMPLETE) {
		if (hc->ep_is_in) {
			length = hc->xfer_len - hctsiz.b.xfersize;
			if (_short_read)
				*_short_read = (hctsiz.b.xfersize != 0);
		} else if (hc->qh->do_split) {
			length = qtd->ssplit_out_xfer_count;
		} else {
			length = hc->xfer_len;
		}
	} else {
		/*
		 * Must use the hctsiz.pktcnt field to determine how much data
		 * has been transferred. This field reflects the number of
		 * packets that have been transferred via the USB. This is
		 * always an integral number of packets if the transfer was
		 * halted before its normal completion. (Can't use the
		 * hctsiz.xfersize field because that reflects the number of
		 * bytes transferred via the AHB, not the USB).
		 */
		length = (hc->start_pkt_count - hctsiz.b.pktcnt) *
				hc->max_packet;
	}
	return length;
}

/**
 * Updates the state of the URB after a Transfer Complete interrupt on the
 * host channel. Updates the actual_length field of the URB based on the
 * number of bytes transferred via the host channel. Sets the URB status
 * if the data transfer is finished.
 */
static int update_urb_state_xfer_comp(struct dwc_hc *hc,
			struct dwc_hc_regs *regs, struct urb *urb,
			struct dwc_qtd *qtd, int *status)
{
	int xfer_done = 0;
	int short_read = 0;

	urb->actual_length += get_actual_xfer_length(hc, regs, qtd,
			DWC_OTG_HC_XFER_COMPLETE, &short_read);

	if (short_read || urb->actual_length == urb->transfer_buffer_length) {
		xfer_done = 1;
		if (short_read && (urb->transfer_flags & URB_SHORT_NOT_OK))
			*status = -EREMOTEIO;
		else
			*status = 0;
	}
	return xfer_done;
}

/*
 * Save the starting data toggle for the next transfer. The data toggle is
 * saved in the QH for non-control transfers and it's saved in the QTD for
 * control transfers.
 */
static void save_data_toggle(struct dwc_hc *hc, struct dwc_hc_regs *regs,
				struct dwc_qtd *qtd)
{
	union hctsiz_data hctsiz;
	hctsiz.d32 = dwc_read_reg32(&regs->hctsiz);

	if (hc->ep_type != DWC_OTG_EP_TYPE_CONTROL) {
		struct dwc_qh *qh = hc->qh;
		if (hctsiz.b.pid == DWC_HCTSIZ_DATA0)
			qh->data_toggle = DWC_OTG_HC_PID_DATA0;
		else
			qh->data_toggle = DWC_OTG_HC_PID_DATA1;
	} else {
		if (hctsiz.b.pid == DWC_HCTSIZ_DATA0)
			qtd->data_toggle = DWC_OTG_HC_PID_DATA0;
		else
			qtd->data_toggle = DWC_OTG_HC_PID_DATA1;
	}
}

/**
 * Frees the first QTD in the QH's list if free_qtd is 1. For non-periodic
 * QHs, removes the QH from the active non-periodic schedule. If any QTDs are
 * still linked to the QH, the QH is added to the end of the inactive
 * non-periodic schedule. For periodic QHs, removes the QH from the periodic
 * schedule if no more QTDs are linked to the QH.
 */
static void deactivate_qh(struct dwc_hcd *hcd, struct dwc_qh *qh, int free_qtd)
{
	int continue_split = 0;
	struct dwc_qtd *qtd;

	qtd = list_entry(qh->qtd_list.next, struct dwc_qtd, qtd_list_entry);
	if (qtd->complete_split)
		continue_split = 1;
	else if (qtd->isoc_split_pos == DWC_HCSPLIT_XACTPOS_MID ||
			qtd->isoc_split_pos == DWC_HCSPLIT_XACTPOS_END)
		continue_split = 1;

	if (free_qtd) {
		dwc_otg_hcd_qtd_remove(qtd);
		continue_split = 0;
	}

	qh->channel = NULL;
	qh->qtd_in_process = NULL;
	dwc_otg_hcd_qh_deactivate(hcd, qh, continue_split);
}

/**
 * Updates the state of an Isochronous URB when the transfer is stopped for
 * any reason. The fields of the current entry in the frame descriptor array
 * are set based on the transfer state and the input status. Completes the
 * Isochronous URB if all the URB frames have been completed.
 */
static enum dwc_halt_status update_isoc_urb_state(struct dwc_hcd *hcd,
		struct dwc_hc *hc, struct dwc_hc_regs *regs,
		struct dwc_qtd *qtd, enum dwc_halt_status status)
{
	struct urb *urb = qtd->urb;
	enum dwc_halt_status ret_val = status;
	struct usb_iso_packet_descriptor *frame_desc;
	frame_desc = &urb->iso_frame_desc[qtd->isoc_frame_index];

	switch (status) {
	case DWC_OTG_HC_XFER_COMPLETE:
		frame_desc->status = 0;
		frame_desc->actual_length =
			get_actual_xfer_length(hc, regs, qtd, status, NULL);
		break;
	case DWC_OTG_HC_XFER_FRAME_OVERRUN:
		urb->error_count++;
		if (hc->ep_is_in)
			frame_desc->status = -ENOSR;
		else
			frame_desc->status = -ECOMM;

		frame_desc->actual_length = 0;
		break;
	case DWC_OTG_HC_XFER_BABBLE_ERR:
		/* Don't need to update actual_length in this case. */
		urb->error_count++;
		frame_desc->status = -EOVERFLOW;
		break;
	case DWC_OTG_HC_XFER_XACT_ERR:
		urb->error_count++;
		frame_desc->status = -EPROTO;
		frame_desc->actual_length =
			get_actual_xfer_length(hc, regs, qtd, status, NULL);
	default:
		printk(KERN_ERR "%s: Unhandled halt_status (%d)\n", __func__,
				status);
		BUG();
		break;
	}

	if (++qtd->isoc_frame_index == urb->number_of_packets) {
		/*
		 * urb->status is not used for isoc transfers.
		 * The individual frame_desc statuses are used instead.
		 */
		dwc_otg_hcd_complete_urb(hcd, urb, 0);
		ret_val = DWC_OTG_HC_XFER_URB_COMPLETE;
	} else {
		ret_val = DWC_OTG_HC_XFER_COMPLETE;
	}
	return ret_val;
}

/**
 * Releases a host channel for use by other transfers. Attempts to select and
 * queue more transactions since at least one host channel is available.
 */
static void release_channel(struct dwc_hcd *hcd, struct dwc_hc *hc,
			struct dwc_qtd *qtd, enum dwc_halt_status halt_status,
			int *must_free)
{
	enum dwc_transaction_type tr_type;
	int free_qtd;
	int deact = 1;
	struct dwc_qh *qh;
	int retry_delay = 1;

	switch (halt_status) {
	case DWC_OTG_HC_XFER_NYET:
	case DWC_OTG_HC_XFER_NAK:
		if (halt_status == DWC_OTG_HC_XFER_NYET)
			retry_delay = nyet_deferral_delay;
		else
			retry_delay = nak_deferral_delay;
		free_qtd = 0;
		if (deferral_on && hc->do_split) {
			qh = hc->qh;
			if (qh)
				deact = dwc_otg_hcd_qh_deferr(hcd, qh,
						retry_delay);
		}
		break;
	case DWC_OTG_HC_XFER_URB_COMPLETE:
		free_qtd = 1;
		break;
	case DWC_OTG_HC_XFER_AHB_ERR:
	case DWC_OTG_HC_XFER_STALL:
	case DWC_OTG_HC_XFER_BABBLE_ERR:
		free_qtd = 1;
		break;
	case DWC_OTG_HC_XFER_XACT_ERR:
		if (qtd->error_count >= 3) {
			free_qtd = 1;
			dwc_otg_hcd_complete_urb(hcd, qtd->urb, -EPROTO);
		} else {
			free_qtd = 0;
		}
		break;
	case DWC_OTG_HC_XFER_URB_DEQUEUE:
		/*
		 * The QTD has already been removed and the QH has been
		 * deactivated. Don't want to do anything except release the
		 * host channel and try to queue more transfers.
		 */
		goto cleanup;
	case DWC_OTG_HC_XFER_NO_HALT_STATUS:
		printk(KERN_ERR "%s: No halt_status, channel %d\n", __func__,
				hc->hc_num);
		free_qtd = 0;
		break;
	default:
		free_qtd = 0;
		break;
	}
	if (free_qtd)
		/* must_free pre-initialized to zero */
		*must_free = 1;
	if (deact)
		deactivate_qh(hcd, hc->qh, free_qtd);

cleanup:
	/*
	 * Release the host channel for use by other transfers. The cleanup
	 * function clears the channel interrupt enables and conditions, so
	 * there's no need to clear the Channel Halted interrupt separately.
	 */
	dwc_otg_hc_cleanup(hcd->core_if, hc);
	list_add_tail(&hc->hc_list_entry, &hcd->free_hc_list);
	hcd->available_host_channels++;
	/* Try to queue more transfers now that there's a free channel. */
	if (!erratum_usb09_patched) {
		tr_type = dwc_otg_hcd_select_transactions(hcd);
		if (tr_type != DWC_OTG_TRANSACTION_NONE)
			dwc_otg_hcd_queue_transactions(hcd, tr_type);
	}
}

/**
 * Halts a host channel. If the channel cannot be halted immediately because
 * the request queue is full, this function ensures that the FIFO empty
 * interrupt for the appropriate queue is enabled so that the halt request can
 * be queued when there is space in the request queue.
 *
 * This function may also be called in DMA mode. In that case, the channel is
 * simply released since the core always halts the channel automatically in
 * DMA mode.
 */
static void halt_channel(struct dwc_hcd *hcd, struct dwc_hc *hc,
	 struct dwc_qtd *qtd, enum dwc_halt_status halt_status, int *must_free)
{
	if (hcd->core_if->dma_enable) {
		release_channel(hcd, hc, qtd, halt_status, must_free);
		return;
	}

	/* Slave mode processing... */
	dwc_otg_hc_halt(hcd->core_if, hc, halt_status);
	if (hc->halt_on_queue) {
		union gintmsk_data gintmsk = {.d32 = 0};

		if (hc->ep_type == DWC_OTG_EP_TYPE_CONTROL ||
				hc->ep_type == DWC_OTG_EP_TYPE_BULK) {
			/*
			 * Make sure the Non-periodic Tx FIFO empty interrupt
			 * is enabled so that the non-periodic schedule will
			 * be processed.
			 */
			gintmsk.b.nptxfempty = 1;
			dwc_modify_reg32(gintmsk_reg(hcd), 0, gintmsk.d32);
		} else {
			/*
			 * Move the QH from the periodic queued schedule to
			 * the periodic assigned schedule. This allows the
			 * halt to be queued when the periodic schedule is
			 * processed.
			 */
			list_move(&hc->qh->qh_list_entry,
				&hcd->periodic_sched_assigned);

			/*
			 * Make sure the Periodic Tx FIFO Empty interrupt is
			 * enabled so that the periodic schedule will be
			 * processed.
			 */
			gintmsk.b.ptxfempty = 1;
			dwc_modify_reg32(gintmsk_reg(hcd), 0, gintmsk.d32);
		}
	}
}

/**
 * Performs common cleanup for non-periodic transfers after a Transfer
 * Complete interrupt. This function should be called after any endpoint type
 * specific handling is finished to release the host channel.
 */
static void complete_non_periodic_xfer(struct dwc_hcd *hcd, struct dwc_hc *hc,
			struct dwc_hc_regs *regs, struct dwc_qtd *qtd,
			enum dwc_halt_status halt_status, int *must_free)
{
	union hcint_data hcint;

	qtd->error_count = 0;
	hcint.d32 = dwc_read_reg32(&regs->hcint);
	if (hcint.b.nyet) {
		union hcint_data hcint_clear = { .d32 = 0};

		hcint_clear.b.nyet = 1;
		/*
		 * Got a NYET on the last transaction of the transfer. This
		 * means that the endpoint should be in the PING state at the
		 * beginning of the next transfer.
		 */
		hc->qh->ping_state = 1;
		dwc_write_reg32(&(regs->hcint), hcint_clear.d32);
	}

	/*
	 * Always halt and release the host channel to make it available for
	 * more transfers. There may still be more phases for a control
	 * transfer or more data packets for a bulk transfer at this point,
	 * but the host channel is still halted. A channel will be reassigned
	 * to the transfer when the non-periodic schedule is processed after
	 * the channel is released. This allows transactions to be queued
	 * properly via dwc_otg_hcd_queue_transactions, which also enables the
	 * Tx FIFO Empty interrupt if necessary.
	 *
	 * IN transfers in Slave mode require an explicit disable to
	 * halt the channel. (In DMA mode, this call simply releases
	 * the channel.)
	 *
	 * The channel is automatically disabled by the core for OUT
	 * transfers in Slave mode.
	 */
	if (hc->ep_is_in)
		halt_channel(hcd, hc, qtd, halt_status, must_free);
	else
		release_channel(hcd, hc, qtd, halt_status, must_free);
}

/**
 * Performs common cleanup for periodic transfers after a Transfer Complete
 * interrupt. This function should be called after any endpoint type specific
 * handling is finished to release the host channel.
 */
static void complete_periodic_xfer(struct dwc_hcd *hcd, struct dwc_hc *hc,
		struct dwc_hc_regs *regs, struct dwc_qtd *qtd,
		enum dwc_halt_status halt_status, int *must_free)
{
	union hctsiz_data hctsiz;

	hctsiz.d32 = dwc_read_reg32(&regs->hctsiz);
	qtd->error_count = 0;

	/*
	 * For OUT transfers and 0 packet count, the Core halts the channel,
	 * otherwise, Flush any outstanding requests from the Tx queue.
	 */
	if (!hc->ep_is_in || hctsiz.b.pktcnt == 0)
		release_channel(hcd, hc, qtd, halt_status, must_free);
	else
		halt_channel(hcd, hc, qtd, halt_status, must_free);
}

/**
 * Handles a host channel Transfer Complete interrupt. This handler may be
 * called in either DMA mode or Slave mode.
 */
static int handle_hc_xfercomp_intr(struct dwc_hcd *hcd, struct dwc_hc *hc,
		struct dwc_hc_regs *regs, struct dwc_qtd *qtd, int  *must_free)
{
	int urb_xfer_done;
	enum dwc_halt_status halt_status = DWC_OTG_HC_XFER_COMPLETE;
	struct urb *urb = qtd->urb;
	int pipe_type = usb_pipetype(urb->pipe);
	int status = -EINPROGRESS;
	union hcintmsk_data hcintmsk = {.d32 = 0};

	/* Handle xfer complete on CSPLIT. */
	if (hc->qh->do_split)
		qtd->complete_split = 0;

	/* Update the QTD and URB states. */
	switch (pipe_type) {
	case PIPE_CONTROL:
		switch (qtd->control_phase) {
		case DWC_OTG_CONTROL_SETUP:
			if (urb->transfer_buffer_length > 0)
				qtd->control_phase = DWC_OTG_CONTROL_DATA;
			else
				qtd->control_phase = DWC_OTG_CONTROL_STATUS;
			halt_status = DWC_OTG_HC_XFER_COMPLETE;
			break;
		case DWC_OTG_CONTROL_DATA:
			urb_xfer_done = update_urb_state_xfer_comp(hc, regs,
							urb, qtd, &status);
			if (urb_xfer_done)
				qtd->control_phase = DWC_OTG_CONTROL_STATUS;
			else
				save_data_toggle(hc, regs, qtd);
			halt_status = DWC_OTG_HC_XFER_COMPLETE;
			break;
		case DWC_OTG_CONTROL_STATUS:
			if (status == -EINPROGRESS)
				status = 0;
			dwc_otg_hcd_complete_urb(hcd, urb, status);
			halt_status = DWC_OTG_HC_XFER_URB_COMPLETE;
			break;
		}
		complete_non_periodic_xfer(hcd, hc, regs, qtd,
			halt_status, must_free);
		break;
	case PIPE_BULK:
		urb_xfer_done = update_urb_state_xfer_comp(hc, regs, urb, qtd,
								&status);
		if (urb_xfer_done) {
			dwc_otg_hcd_complete_urb(hcd, urb, status);
			halt_status = DWC_OTG_HC_XFER_URB_COMPLETE;
		} else {
			halt_status = DWC_OTG_HC_XFER_COMPLETE;
		}

		save_data_toggle(hc, regs, qtd);
		complete_non_periodic_xfer(hcd, hc, regs, qtd,
			halt_status, must_free);
		break;
	case PIPE_INTERRUPT:
		update_urb_state_xfer_comp(hc, regs, urb, qtd, &status);
		/*
		 * Interrupt URB is done on the first transfer complete
		 * interrupt.
		 */
		dwc_otg_hcd_complete_urb(hcd, urb, status);
		save_data_toggle(hc, regs, qtd);
		complete_periodic_xfer(hcd, hc, regs, qtd,
			DWC_OTG_HC_XFER_URB_COMPLETE, must_free);
		break;
	case PIPE_ISOCHRONOUS:
		if (qtd->isoc_split_pos == DWC_HCSPLIT_XACTPOS_ALL) {
			halt_status = update_isoc_urb_state(hcd, hc, regs, qtd,
					DWC_OTG_HC_XFER_COMPLETE);
		}
		complete_periodic_xfer(hcd, hc, regs, qtd,
			halt_status, must_free);
		break;
	}

	/* disable xfercompl */
	hcintmsk.b.xfercompl = 1;
	dwc_modify_reg32(&regs->hcintmsk, hcintmsk.d32, 0);

	return 1;
}

/**
 * Handles a host channel STALL interrupt. This handler may be called in
 * either DMA mode or Slave mode.
 */
static int handle_hc_stall_intr(struct dwc_hcd *hcd, struct dwc_hc *hc,
		struct dwc_hc_regs *regs, struct dwc_qtd *qtd, int *must_free)
{
	struct urb *urb = qtd->urb;
	int pipe_type = usb_pipetype(urb->pipe);
	union hcintmsk_data hcintmsk = {.d32 = 0};

	if (pipe_type == PIPE_CONTROL)
		dwc_otg_hcd_complete_urb(hcd, qtd->urb, -EPIPE);

	if (pipe_type == PIPE_BULK || pipe_type == PIPE_INTERRUPT) {
		dwc_otg_hcd_complete_urb(hcd, qtd->urb, -EPIPE);
		/*
		 * USB protocol requires resetting the data toggle for bulk
		 * and interrupt endpoints when a CLEAR_FEATURE(ENDPOINT_HALT)
		 * setup command is issued to the endpoint. Anticipate the
		 * CLEAR_FEATURE command since a STALL has occurred and reset
		 * the data toggle now.
		 */
		hc->qh->data_toggle = 0;
	}

	halt_channel(hcd, hc, qtd, DWC_OTG_HC_XFER_STALL, must_free);
	/* disable stall */
	hcintmsk.b.stall = 1;
	dwc_modify_reg32(&regs->hcintmsk, hcintmsk.d32, 0);

	return 1;
}

/**
 * Updates the state of the URB when a transfer has been stopped due to an
 * abnormal condition before the transfer completes. Modifies the
 * actual_length field of the URB to reflect the number of bytes that have
 * actually been transferred via the host channel.
 */
static void update_urb_state_xfer_intr(struct dwc_hc *hc,
		 struct dwc_hc_regs *regs, struct urb *urb, struct dwc_qtd *qtd,
		enum dwc_halt_status sts)
{
	u32 xfr_len = get_actual_xfer_length(hc, regs, qtd, sts, NULL);
	urb->actual_length += xfr_len;
}

/**
 * Handles a host channel NAK interrupt. This handler may be called in either
 * DMA mode or Slave mode.
 */
static int handle_hc_nak_intr(struct dwc_hcd *hcd, struct dwc_hc *hc,
		struct dwc_hc_regs *regs, struct dwc_qtd *qtd, int *must_free)
{
	union hcintmsk_data hcintmsk = {.d32 = 0};

	/*
	 * Handle NAK for IN/OUT SSPLIT/CSPLIT transfers, bulk, control, and
	 * interrupt.  Re-start the SSPLIT transfer.
	 */
	if (hc->do_split) {
		if (hc->complete_split)
			qtd->error_count = 0;

		qtd->complete_split = 0;
		halt_channel(hcd, hc, qtd, DWC_OTG_HC_XFER_NAK, must_free);
		goto handle_nak_done;
	}
	switch (usb_pipetype(qtd->urb->pipe)) {
	case PIPE_CONTROL:
	case PIPE_BULK:
		if (hcd->core_if->dma_enable && hc->ep_is_in) {
			/*
			 * NAK interrupts are enabled on bulk/control IN
			 * transfers in DMA mode for the sole purpose of
			 * resetting the error count after a transaction error
			 * occurs. The core will continue transferring data.
			 */
			qtd->error_count = 0;
			goto handle_nak_done;
		}

		/*
		 * NAK interrupts normally occur during OUT transfers in DMA
		 * or Slave mode. For IN transfers, more requests will be
		 * queued as request queue space is available.
		 */
		qtd->error_count = 0;
		if (!hc->qh->ping_state) {
			update_urb_state_xfer_intr(hc, regs, qtd->urb, qtd,
							DWC_OTG_HC_XFER_NAK);

			save_data_toggle(hc, regs, qtd);
			if (qtd->urb->dev->speed == USB_SPEED_HIGH)
				hc->qh->ping_state = 1;
		}

		/*
		 * Halt the channel so the transfer can be re-started from
		 * the appropriate point or the PING protocol will
		 * start/continue.
		 */
		halt_channel(hcd, hc, qtd, DWC_OTG_HC_XFER_NAK, must_free);
		break;
	case PIPE_INTERRUPT:
		qtd->error_count = 0;
		halt_channel(hcd, hc, qtd, DWC_OTG_HC_XFER_NAK, must_free);
		break;
	case PIPE_ISOCHRONOUS:
		/* Should never get called for isochronous transfers. */
		BUG();
		break;
	}

handle_nak_done:
	/* disable nak */
	hcintmsk.b.nak = 1;
	dwc_modify_reg32(&regs->hcintmsk, hcintmsk.d32, 0);

	return 1;
}

/**
 * Helper function for handle_hc_ack_intr().  Sets the split values for an ACK
 * on SSPLIT for ISOC OUT.
 */
static void set_isoc_out_vals(struct dwc_hc *hc, struct dwc_qtd *qtd)
{
	struct usb_iso_packet_descriptor *frame_desc;

	switch (hc->xact_pos) {
	case DWC_HCSPLIT_XACTPOS_ALL:
		break;
	case DWC_HCSPLIT_XACTPOS_END:
		qtd->isoc_split_pos = DWC_HCSPLIT_XACTPOS_ALL;
		qtd->isoc_split_offset = 0;
		break;
	case DWC_HCSPLIT_XACTPOS_BEGIN:
	case DWC_HCSPLIT_XACTPOS_MID:
		/*
		 * For BEGIN or MID, calculate the length for the next
		 * microframe to determine the correct SSPLIT token, either MID
		 * or END.
		 */
		frame_desc = &qtd->urb->iso_frame_desc[qtd->isoc_frame_index];
		qtd->isoc_split_offset += 188;

		if ((frame_desc->length - qtd->isoc_split_offset) <= 188)
			qtd->isoc_split_pos = DWC_HCSPLIT_XACTPOS_END;
		else
			qtd->isoc_split_pos = DWC_HCSPLIT_XACTPOS_MID;

		break;
	}
}

/**
 * Handles a host channel ACK interrupt. This interrupt is enabled when
 * performing the PING protocol in Slave mode, when errors occur during
 * either Slave mode or DMA mode, and during Start Split transactions.
 */
static int handle_hc_ack_intr(struct dwc_hcd *hcd, struct dwc_hc *hc,
			struct dwc_hc_regs *regs, struct dwc_qtd *qtd,
			int *must_free)
{
	union hcintmsk_data hcintmsk = {.d32 = 0};

	if (hc->do_split) {
		/* Handle ACK on SSPLIT. ACK should not occur in CSPLIT. */
		if (!hc->ep_is_in && hc->data_pid_start != DWC_OTG_HC_PID_SETUP)
			qtd->ssplit_out_xfer_count = hc->xfer_len;

		/* Don't need complete for isochronous out transfers. */
		if (!(hc->ep_type == DWC_OTG_EP_TYPE_ISOC && !hc->ep_is_in))
			qtd->complete_split = 1;

		if (hc->ep_type == DWC_OTG_EP_TYPE_ISOC && !hc->ep_is_in)
			set_isoc_out_vals(hc, qtd);
		else
			halt_channel(hcd, hc, qtd, DWC_OTG_HC_XFER_ACK,
				must_free);
	} else {
		qtd->error_count = 0;
		if (hc->qh->ping_state) {
			hc->qh->ping_state = 0;

			/*
			 * Halt the channel so the transfer can be re-started
			 * from the appropriate point. This only happens in
			 * Slave mode. In DMA mode, the ping_state is cleared
			 * when the transfer is started because the core
			 * automatically executes the PING, then the transfer.
			 */
			halt_channel(hcd, hc, qtd, DWC_OTG_HC_XFER_ACK,
				must_free);
		}
	}

	/*
	 * If the ACK occurred when _not_ in the PING state, let the channel
	 * continue transferring data after clearing the error count.
	 */
	/* disable ack */
	hcintmsk.b.ack = 1;
	dwc_modify_reg32(&regs->hcintmsk, hcintmsk.d32, 0);

	return 1;
}

/**
 * Handles a host channel NYET interrupt. This interrupt should only occur on
 * Bulk and Control OUT endpoints and for complete split transactions. If a
 * NYET occurs at the same time as a Transfer Complete interrupt, it is
 * handled in the xfercomp interrupt handler, not here. This handler may be
 * called in either DMA mode or Slave mode.
 */
static int handle_hc_nyet_intr(struct dwc_hcd *hcd, struct dwc_hc *hc,
		struct dwc_hc_regs *regs, struct dwc_qtd *qtd, int *must_free)
{
	union hcintmsk_data hcintmsk = {.d32 = 0};
	union hcint_data hcint_clear = {.d32 = 0};

	/*
	 * NYET on CSPLIT
	 * re-do the CSPLIT immediately on non-periodic
	 */
	if (hc->do_split && hc->complete_split) {
		if (hc->ep_type == DWC_OTG_EP_TYPE_INTR ||
				hc->ep_type == DWC_OTG_EP_TYPE_ISOC) {
			int frnum = dwc_otg_hcd_get_frame_number(
					dwc_otg_hcd_to_hcd(hcd));
			if (dwc_full_frame_num(frnum) !=
				dwc_full_frame_num(hc->qh->sched_frame)) {
				qtd->complete_split = 0;
				halt_channel(hcd, hc, qtd,
					DWC_OTG_HC_XFER_XACT_ERR, must_free);
				goto handle_nyet_done;
			}
		}
		halt_channel(hcd, hc, qtd, DWC_OTG_HC_XFER_NYET, must_free);
		goto handle_nyet_done;
	}
	hc->qh->ping_state = 1;
	qtd->error_count = 0;
	update_urb_state_xfer_intr(hc, regs, qtd->urb, qtd,
				DWC_OTG_HC_XFER_NYET);
	save_data_toggle(hc, regs, qtd);
	/*
	 * Halt the channel and re-start the transfer so the PING
	 * protocol will start.
	 */
	halt_channel(hcd, hc, qtd, DWC_OTG_HC_XFER_NYET, must_free);

handle_nyet_done:
	/* disable nyet */
	hcintmsk.b.nyet = 1;
	dwc_modify_reg32(&regs->hcintmsk, hcintmsk.d32, 0);
	/* clear nyet */
	hcint_clear.b.nyet = 1;
	dwc_write_reg32(&(regs->hcint), hcint_clear.d32);
	return 1;
}

/**
 * Handles a host channel babble interrupt. This handler may be called in
 * either DMA mode or Slave mode.
 */
static int handle_hc_babble_intr(struct dwc_hcd *hcd, struct dwc_hc *hc,
		struct dwc_hc_regs *regs, struct dwc_qtd *qtd, int *must_free)
{
	union hcintmsk_data hcintmsk = {.d32 = 0};

	if (hc->ep_type != DWC_OTG_EP_TYPE_ISOC) {
		dwc_otg_hcd_complete_urb(hcd, qtd->urb, -EOVERFLOW);
		halt_channel(hcd, hc, qtd, DWC_OTG_HC_XFER_BABBLE_ERR,
			must_free);
	} else {
		enum dwc_halt_status halt_status;
		halt_status = update_isoc_urb_state(hcd, hc, regs, qtd,
				DWC_OTG_HC_XFER_BABBLE_ERR);
		halt_channel(hcd, hc, qtd, halt_status, must_free);
	}
	/* disable bblerr */
	hcintmsk.b.bblerr = 1;
	dwc_modify_reg32(&regs->hcintmsk, hcintmsk.d32, 0);
	return 1;
}

/**
 * Handles a host channel AHB error interrupt. This handler is only called in
 * DMA mode.
 */
static int handle_hc_ahberr_intr(struct dwc_hcd *hcd, struct dwc_hc *hc,
			struct dwc_hc_regs *regs, struct dwc_qtd *qtd)
{
	union hcchar_data hcchar;
	union hcsplt_data hcsplt;
	union hctsiz_data hctsiz;
	u32 hcdma;
	struct urb *urb = qtd->urb;
	union hcintmsk_data hcintmsk = {.d32 = 0};

	hcchar.d32 = dwc_read_reg32(&regs->hcchar);
	hcsplt.d32 = dwc_read_reg32(&regs->hcsplt);
	hctsiz.d32 = dwc_read_reg32(&regs->hctsiz);
	hcdma = dwc_read_reg32(&regs->hcdma);

	printk(KERN_ERR "AHB ERROR, Channel %d\n", hc->hc_num);
	printk(KERN_ERR "  hcchar 0x%08x, hcsplt 0x%08x\n", hcchar.d32,
				hcsplt.d32);
	printk(KERN_ERR "  hctsiz 0x%08x, hcdma 0x%08x\n", hctsiz.d32, hcdma);

	printk(KERN_ERR "  Device address: %d\n", usb_pipedevice(urb->pipe));
	printk(KERN_ERR "  Endpoint: %d, %s\n", usb_pipeendpoint(urb->pipe),
			(usb_pipein(urb->pipe) ? "IN" : "OUT"));

	printk(KERN_ERR "  Endpoint type: %s\n", pipetype_str(urb->pipe));
	printk(KERN_ERR "  Speed: %s\n", dev_speed_str(urb->dev->speed));
	printk(KERN_ERR "  Max packet size: %d\n",
		usb_maxpacket(urb->dev, urb->pipe, usb_pipeout(urb->pipe)));
	printk(KERN_ERR "  Data buffer length: %d\n",
		urb->transfer_buffer_length);
	printk(KERN_ERR "  Transfer buffer: %p, Transfer DMA: %p\n",
		urb->transfer_buffer, (void *) (u32) urb->transfer_dma);
	printk(KERN_ERR "  Setup buffer: %p, Setup DMA: %p\n",
		urb->setup_packet, (void *) (u32) urb->setup_dma);
	printk(KERN_ERR "  Interval: %d\n", urb->interval);

	dwc_otg_hcd_complete_urb(hcd, urb, -EIO);

	/*
	 * Force a channel halt. Don't call halt_channel because that won't
	 * write to the HCCHARn register in DMA mode to force the halt.
	 */
	dwc_otg_hc_halt(hcd->core_if, hc, DWC_OTG_HC_XFER_AHB_ERR);
	/* disable ahberr */
	hcintmsk.b.ahberr = 1;
	dwc_modify_reg32(&regs->hcintmsk, hcintmsk.d32, 0);

	return 1;
}

/**
 * Handles a host channel transaction error interrupt. This handler may be
 * called in either DMA mode or Slave mode.
 */
static int handle_hc_xacterr_intr(struct dwc_hcd *hcd, struct dwc_hc *hc,
		struct dwc_hc_regs *regs, struct dwc_qtd *qtd, int *must_free)
{
	enum dwc_halt_status status = DWC_OTG_HC_XFER_XACT_ERR;
	union hcintmsk_data hcintmsk = {.d32 = 0};

	switch (usb_pipetype(qtd->urb->pipe)) {
	case PIPE_CONTROL:
	case PIPE_BULK:
		qtd->error_count++;
		if (!hc->qh->ping_state) {
			update_urb_state_xfer_intr(hc, regs, qtd->urb, qtd,
							status);
			save_data_toggle(hc, regs, qtd);

			if (!hc->ep_is_in && qtd->urb->dev->speed ==
					USB_SPEED_HIGH)
				hc->qh->ping_state = 1;
		}
		/*
		 * Halt the channel so the transfer can be re-started from
		 * the appropriate point or the PING protocol will start.
		 */
		halt_channel(hcd, hc, qtd, status, must_free);
		break;
	case PIPE_INTERRUPT:
		qtd->error_count++;
		if (hc->do_split && hc->complete_split)
			qtd->complete_split = 0;

		halt_channel(hcd, hc, qtd, status, must_free);
		break;
	case PIPE_ISOCHRONOUS:
		status = update_isoc_urb_state(hcd, hc, regs, qtd, status);
		halt_channel(hcd, hc, qtd, status, must_free);
		break;
	}
	/* Disable xacterr */
	hcintmsk.b.xacterr = 1;
	dwc_modify_reg32(&regs->hcintmsk, hcintmsk.d32, 0);

	return 1;
}

/**
 * Handles a host channel frame overrun interrupt. This handler may be called
 * in either DMA mode or Slave mode.
 */
static int handle_hc_frmovrun_intr(struct dwc_hcd *hcd, struct dwc_hc *hc,
		struct dwc_hc_regs *regs, struct dwc_qtd *qtd, int *must_free)
{
	enum dwc_halt_status status = DWC_OTG_HC_XFER_FRAME_OVERRUN;
	union hcintmsk_data hcintmsk = {.d32 = 0};

	switch (usb_pipetype(qtd->urb->pipe)) {
	case PIPE_CONTROL:
	case PIPE_BULK:
		break;
	case PIPE_INTERRUPT:
		halt_channel(hcd, hc, qtd, status, must_free);
		break;
	case PIPE_ISOCHRONOUS:
		status = update_isoc_urb_state(hcd, hc, regs, qtd, status);
		halt_channel(hcd, hc, qtd, status, must_free);
		break;
	}
	/* Disable frmovrun */
	hcintmsk.b.frmovrun = 1;
	dwc_modify_reg32(&regs->hcintmsk, hcintmsk.d32, 0);

	return 1;
}

/**
 * Handles a host channel data toggle error interrupt. This handler may be
 * called in either DMA mode or Slave mode.
 */
static int handle_hc_datatglerr_intr(struct dwc_hcd *hcd, struct dwc_hc *hc,
			struct dwc_hc_regs *regs, struct dwc_qtd *qtd)
{
	union hcintmsk_data hcintmsk = {.d32 = 0};

	if (hc->ep_is_in)
		qtd->error_count = 0;
	else
		printk(KERN_ERR "Data Toggle Error on OUT transfer, channel "
				"%d\n", hc->hc_num);

	/* disable datatglerr */
	hcintmsk.b.datatglerr = 1;
	dwc_modify_reg32(&regs->hcintmsk, hcintmsk.d32, 0);

	return 1;
}

/**
 * Handles a host Channel Halted interrupt in DMA mode. This handler
 * determines the reason the channel halted and proceeds accordingly.
 */
static void handle_hc_chhltd_intr_dma(struct dwc_hcd *hcd, struct dwc_hc *hc,
		struct dwc_hc_regs *regs, struct dwc_qtd *qtd, int *must_free)
{
	union hcint_data hcint;
	union hcintmsk_data hcintmsk;

	if (hc->halt_status == DWC_OTG_HC_XFER_URB_DEQUEUE ||
			hc->halt_status == DWC_OTG_HC_XFER_AHB_ERR) {
		/*
		 * Just release the channel. A dequeue can happen on a
		 * transfer timeout. In the case of an AHB Error, the channel
		 * was forced to halt because there's no way to gracefully
		 * recover.
		 */
		release_channel(hcd, hc, qtd, hc->halt_status, must_free);
		return;
	}

	/* Read the HCINTn register to determine the cause for the halt. */
	hcint.d32 = dwc_read_reg32(&regs->hcint);
	hcintmsk.d32 = dwc_read_reg32(&regs->hcintmsk);
	if (hcint.b.xfercomp) {
		/*
		 * This is here because of a possible hardware bug.  Spec
		 * says that on SPLIT-ISOC OUT transfers in DMA mode that a HALT
		 * interrupt w/ACK bit set should occur, but I only see the
		 * XFERCOMP bit, even with it masked out.  This is a workaround
		 * for that behavior.  Should fix this when hardware is fixed.
		 */
		if (hc->ep_type == DWC_OTG_EP_TYPE_ISOC && !hc->ep_is_in)
			handle_hc_ack_intr(hcd, hc, regs, qtd, must_free);

		handle_hc_xfercomp_intr(hcd, hc, regs, qtd, must_free);
	} else if (hcint.b.stall) {
		handle_hc_stall_intr(hcd, hc, regs, qtd, must_free);
	} else if (hcint.b.xacterr) {
		/*
		 * Must handle xacterr before nak or ack. Could get a xacterr
		 * at the same time as either of these on a BULK/CONTROL OUT
		 * that started with a PING. The xacterr takes precedence.
		 */
		handle_hc_xacterr_intr(hcd, hc, regs, qtd, must_free);
	} else if (hcint.b.nyet) {
		/*
		 * Must handle nyet before nak or ack. Could get a nyet at the
		 * same time as either of those on a BULK/CONTROL OUT that
		 * started with a PING. The nyet takes precedence.
		 */
		handle_hc_nyet_intr(hcd, hc, regs, qtd, must_free);
	} else if (hcint.b.bblerr) {
		handle_hc_babble_intr(hcd, hc, regs, qtd, must_free);
	} else if (hcint.b.frmovrun) {
		handle_hc_frmovrun_intr(hcd, hc, regs, qtd, must_free);
	} else if (hcint.b.datatglerr) {
		handle_hc_datatglerr_intr(hcd, hc, regs, qtd);
		hc->qh->data_toggle = 0;
		halt_channel(hcd, hc, qtd, hc->halt_status, must_free);
	} else if (hcint.b.nak && !hcintmsk.b.nak) {
		/*
		 * If nak is not masked, it's because a non-split IN transfer
		 * is in an error state. In that case, the nak is handled by
		 * the nak interrupt handler, not here. Handle nak here for
		 * BULK/CONTROL OUT transfers, which halt on a NAK to allow
		 * rewinding the buffer pointer.
		 */
		handle_hc_nak_intr(hcd, hc, regs, qtd, must_free);
	} else if (hcint.b.ack && !hcintmsk.b.ack) {
		/*
		 * If ack is not masked, it's because a non-split IN transfer
		 * is in an error state. In that case, the ack is handled by
		 * the ack interrupt handler, not here. Handle ack here for
		 * split transfers. Start splits halt on ACK.
		 */
		handle_hc_ack_intr(hcd, hc, regs, qtd, must_free);
	} else {
		if (hc->ep_type == DWC_OTG_EP_TYPE_INTR ||
				hc->ep_type == DWC_OTG_EP_TYPE_ISOC) {
			/*
			 * A periodic transfer halted with no other channel
			 * interrupts set. Assume it was halted by the core
			 * because it could not be completed in its scheduled
			 * (micro)frame.
			 */
			halt_channel(hcd, hc, qtd,
				DWC_OTG_HC_XFER_PERIODIC_INCOMPLETE,
				must_free);
		} else {
			printk(KERN_ERR "%s: Channel %d, DMA Mode -- ChHltd "
				"set, but reason for halting is unknown, "
				"hcint 0x%08x, intsts 0x%08x\n",
				__func__, hc->hc_num, hcint.d32,
				dwc_read_reg32(gintsts_reg(hcd)));
		}
	}
}

/**
 * Handles a host channel Channel Halted interrupt.
 *
 * In slave mode, this handler is called only when the driver specifically
 * requests a halt. This occurs during handling other host channel interrupts
 * (e.g. nak, xacterr, stall, nyet, etc.).
 *
 * In DMA mode, this is the interrupt that occurs when the core has finished
 * processing a transfer on a channel. Other host channel interrupts (except
 * ahberr) are disabled in DMA mode.
 */
static int handle_hc_chhltd_intr(struct dwc_hcd *hcd, struct dwc_hc *hc,
		struct dwc_hc_regs *regs, struct dwc_qtd *qtd, int *must_free)
{
	if (hcd->core_if->dma_enable)
		handle_hc_chhltd_intr_dma(hcd, hc, regs, qtd, must_free);
	else
		release_channel(hcd, hc, qtd, hc->halt_status, must_free);

	return 1;
}

/* Handles interrupt for a specific Host Channel */
static int dwc_otg_hcd_handle_hc_n_intr(struct dwc_hcd *hcd, u32 num)
{
	int must_free = 0;
	int retval = 0;
	union hcint_data hcint;
	union hcintmsk_data hcintmsk;
	struct dwc_hc *hc;
	struct dwc_hc_regs *hc_regs;
	struct dwc_qtd *qtd;

	hc = hcd->hc_ptr_array[num];
	hc_regs = hcd->core_if->host_if->hc_regs[num];
	qtd = list_entry(hc->qh->qtd_list.next, struct dwc_qtd, qtd_list_entry);

	hcint.d32 = dwc_read_reg32(&hc_regs->hcint);
	hcintmsk.d32 = dwc_read_reg32(&hc_regs->hcintmsk);

	hcint.d32 = hcint.d32 & hcintmsk.d32;
	if (!hcd->core_if->dma_enable && hcint.b.chhltd && hcint.d32 != 0x2)
		hcint.b.chhltd = 0;

	if (hcint.b.xfercomp) {
		retval |= handle_hc_xfercomp_intr(hcd, hc, hc_regs,
			qtd, &must_free);
		/*
		 * If NYET occurred at same time as Xfer Complete, the NYET is
		 * handled by the Xfer Complete interrupt handler. Don't want
		 * to call the NYET interrupt handler in this case.
		 */
		hcint.b.nyet = 0;
	}

	if (hcint.b.chhltd)
		retval |= handle_hc_chhltd_intr(hcd, hc, hc_regs,
			qtd, &must_free);
	if (hcint.b.ahberr)
		retval |= handle_hc_ahberr_intr(hcd, hc, hc_regs, qtd);
	if (hcint.b.stall)
		retval |= handle_hc_stall_intr(hcd, hc, hc_regs,
			qtd, &must_free);
	if (hcint.b.nak)
		retval |= handle_hc_nak_intr(hcd, hc, hc_regs,
			qtd, &must_free);
	if (hcint.b.ack)
		retval |= handle_hc_ack_intr(hcd, hc, hc_regs,
			qtd, &must_free);
	if (hcint.b.nyet)
		retval |= handle_hc_nyet_intr(hcd, hc, hc_regs,
			qtd, &must_free);
	if (hcint.b.xacterr)
		retval |= handle_hc_xacterr_intr(hcd, hc, hc_regs,
			qtd, &must_free);
	if (hcint.b.bblerr)
		retval |= handle_hc_babble_intr(hcd, hc, hc_regs,
			qtd, &must_free);
	if (hcint.b.frmovrun)
		retval |= handle_hc_frmovrun_intr(hcd, hc, hc_regs,
			qtd, &must_free);
	if (hcint.b.datatglerr)
		retval |= handle_hc_datatglerr_intr(hcd, hc, hc_regs, qtd);

	if (must_free)
		/* Free the qtd here now that we are done using it. */
		dwc_otg_hcd_qtd_free(qtd);
	return retval;
}

/**
 * This function returns the Host All Channel Interrupt register
 */
static inline u32 dwc_otg_read_host_all_channels_intr(struct core_if
						*core_if)
{
	return dwc_read_reg32(&core_if->host_if->host_global_regs->haint);
}

/**
 * This interrupt indicates that one or more host channels has a pending
 * interrupt. There are multiple conditions that can cause each host channel
 * interrupt. This function determines which conditions have occurred for each
 * host channel interrupt and handles them appropriately.
 */
static int dwc_otg_hcd_handle_hc_intr(struct dwc_hcd *hcd)
{
	u32 i;
	int retval = 0;
	union haint_data haint;

	/*
	 * Clear appropriate bits in HCINTn to clear the interrupt bit in
	 *  GINTSTS
	 */
	haint.d32 = dwc_otg_read_host_all_channels_intr(hcd->core_if);
	for (i = 0; i < hcd->core_if->core_params->host_channels; i++)
		if (haint.b2.chint & (1 << i))
			retval |= dwc_otg_hcd_handle_hc_n_intr(hcd, i);

	return retval;
}

/* This function handles interrupts for the HCD.*/
int dwc_otg_hcd_handle_intr(struct dwc_hcd *hcd)
{
	int ret = 0;
	struct core_if *core_if = hcd->core_if;
	union gintsts_data gintsts;

	/* Check if HOST Mode */
	if (dwc_otg_is_host_mode(core_if)) {
		spin_lock(&hcd->lock);
		gintsts.d32 = dwc_otg_read_core_intr(core_if);
		if (!gintsts.d32) {
			spin_unlock(&hcd->lock);
			return IRQ_NONE;
		}

		if (gintsts.b.sofintr)
			ret |= dwc_otg_hcd_handle_sof_intr(hcd);
		if (gintsts.b.rxstsqlvl)
			ret |= dwc_otg_hcd_handle_rx_status_q_level_intr(hcd);
		if (gintsts.b.nptxfempty)
			ret |= dwc_otg_hcd_handle_np_tx_fifo_empty_intr(hcd);
		if (gintsts.b.portintr)
			ret |= dwc_otg_hcd_handle_port_intr(hcd);
		if (gintsts.b.hcintr)
			ret |= dwc_otg_hcd_handle_hc_intr(hcd);
		if (gintsts.b.ptxfempty)
			ret |= dwc_otg_hcd_handle_perio_tx_fifo_empty_intr(hcd);

		spin_unlock(&hcd->lock);
	}
	return ret;
}
