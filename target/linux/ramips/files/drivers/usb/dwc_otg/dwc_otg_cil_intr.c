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
 * This file contains the Common Interrupt handlers.
 */
#include <linux/delay.h>

#include "dwc_otg_cil.h"

/**
 *  This function will log a debug message
 */
static int dwc_otg_handle_mode_mismatch_intr(struct core_if *core_if)
{
	union gintsts_data gintsts;

	printk(KERN_WARNING "Mode Mismatch Interrupt: currently in %s mode\n",
		dwc_otg_mode(core_if) ? "Host" : "Device");

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.modemismatch = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);

	return 1;
}

/**
 *  Start the HCD.  Helper function for using the HCD callbacks.
 */
static inline void hcd_start(struct core_if *core_if)
{
	if (core_if->hcd_cb && core_if->hcd_cb->start)
		core_if->hcd_cb->start(core_if->hcd_cb->p);
}

/**
 *  Stop the HCD.  Helper function for using the HCD callbacks.
 */
static inline void hcd_stop(struct core_if *core_if)
{
	if (core_if->hcd_cb && core_if->hcd_cb->stop)
		core_if->hcd_cb->stop(core_if->hcd_cb->p);
}

/**
 *  Disconnect the HCD.  Helper function for using the HCD callbacks.
 */
static inline void hcd_disconnect(struct core_if *core_if)
{
	if (core_if->hcd_cb && core_if->hcd_cb->disconnect)
		core_if->hcd_cb->disconnect(core_if->hcd_cb->p);
}

/**
 *  Inform the HCD the a New Session has begun.  Helper function for using the
 *  HCD callbacks.
 */
static inline void hcd_session_start(struct core_if *core_if)
{
	if (core_if->hcd_cb && core_if->hcd_cb->session_start)
		core_if->hcd_cb->session_start(core_if->hcd_cb->p);
}

/**
 *  Start the PCD.  Helper function for using the PCD callbacks.
 */
static inline void pcd_start(struct core_if *core_if)
{
	if (core_if->pcd_cb && core_if->pcd_cb->start) {
		struct dwc_pcd *pcd;

		pcd = (struct dwc_pcd *)core_if->pcd_cb->p;
		spin_lock(&pcd->lock);
		core_if->pcd_cb->start(core_if->pcd_cb->p);
		spin_unlock(&pcd->lock);
	}
}

/**
 *  Stop the PCD.  Helper function for using the PCD callbacks.
 */
static inline void pcd_stop(struct core_if *core_if)
{
	if (core_if->pcd_cb && core_if->pcd_cb->stop) {
		struct dwc_pcd *pcd;

		pcd = (struct dwc_pcd *)core_if->pcd_cb->p;
		spin_lock(&pcd->lock);
		core_if->pcd_cb->stop(core_if->pcd_cb->p);
		spin_unlock(&pcd->lock);
	}
}

/**
 *  Suspend the PCD.  Helper function for using the PCD callbacks.
 */
static inline void pcd_suspend(struct core_if *core_if)
{
	if (core_if->pcd_cb && core_if->pcd_cb->suspend) {
		struct dwc_pcd *pcd;

		pcd = (struct dwc_pcd *)core_if->pcd_cb->p;
		spin_lock(&pcd->lock);
		core_if->pcd_cb->suspend(core_if->pcd_cb->p);
		spin_unlock(&pcd->lock);
	}
}

/**
 *  Resume the PCD.  Helper function for using the PCD callbacks.
 */
static inline void pcd_resume(struct core_if *core_if)
{
	if (core_if->pcd_cb && core_if->pcd_cb->resume_wakeup) {
		struct dwc_pcd *pcd;

		pcd = (struct dwc_pcd *)core_if->pcd_cb->p;
		spin_lock(&pcd->lock);
		core_if->pcd_cb->resume_wakeup(core_if->pcd_cb->p);
		spin_unlock(&pcd->lock);
	}
}

/**
 * This function handles the OTG Interrupts. It reads the OTG
 * Interrupt Register (GOTGINT) to determine what interrupt has
 * occurred.
 */
static int dwc_otg_handle_otg_intr(struct core_if *core_if)
{
	struct core_global_regs *global_regs = core_if->core_global_regs;
	union gotgint_data gotgint;
	union gotgctl_data gotgctl;
	union gintmsk_data gintmsk;

	gotgint.d32 = dwc_read_reg32(&global_regs->gotgint);
	gotgctl.d32 = dwc_read_reg32(&global_regs->gotgctl);

	if (gotgint.b.sesenddet) {
		gotgctl.d32 = dwc_read_reg32(&global_regs->gotgctl);
		if (core_if->xceiv->state == OTG_STATE_B_HOST) {
			pcd_start(core_if);
			core_if->xceiv->state = OTG_STATE_B_PERIPHERAL;
		} else {
			/*
			 * If not B_HOST and Device HNP still set. HNP did not
			 * succeed
			 */
			if (gotgctl.b.devhnpen)
				printk(KERN_ERR "Device Not Connected / "
					"Responding\n");
			/*
			 * If Session End Detected the B-Cable has been
			 * disconnected.  Reset PCD and Gadget driver to a
			 * clean state.
			 */
			pcd_stop(core_if);
		}
		gotgctl.d32 = 0;
		gotgctl.b.devhnpen = 1;
		dwc_modify_reg32(&global_regs->gotgctl, gotgctl.d32, 0);
	}
	if (gotgint.b.sesreqsucstschng) {
		gotgctl.d32 = dwc_read_reg32(&global_regs->gotgctl);
		if (gotgctl.b.sesreqscs) {
			if (core_if->core_params->phy_type ==
					DWC_PHY_TYPE_PARAM_FS &&
					core_if->core_params->i2c_enable) {
				core_if->srp_success = 1;
			} else {
				pcd_resume(core_if);

				/* Clear Session Request */
				gotgctl.d32 = 0;
				gotgctl.b.sesreq = 1;
				dwc_modify_reg32(&global_regs->gotgctl,
							gotgctl.d32, 0);
			}
		}
	}
	if (gotgint.b.hstnegsucstschng) {
		/*
		 * Print statements during the HNP interrupt handling can cause
		 * it to fail.
		 */
		gotgctl.d32 = dwc_read_reg32(&global_regs->gotgctl);
		if (gotgctl.b.hstnegscs) {
			if (dwc_otg_is_host_mode(core_if)) {
				core_if->xceiv->state = OTG_STATE_B_HOST;
				/*
				 * Need to disable SOF interrupt immediately.
				 * When switching from device to host, the PCD
				 * interrupt handler won't handle the
				 * interrupt if host mode is already set. The
				 * HCD interrupt handler won't get called if
				 * the HCD state is HALT. This means that the
				 * interrupt does not get handled and Linux
				 * complains loudly.
				 */
				gintmsk.d32 = 0;
				gintmsk.b.sofintr = 1;
				dwc_modify_reg32(&global_regs->gintmsk,
						gintmsk.d32, 0);
				pcd_stop(core_if);
				/* Initialize the Core for Host mode. */
				hcd_start(core_if);
				core_if->xceiv->state = OTG_STATE_B_HOST;
			}
		} else {
			gotgctl.d32 = 0;
			gotgctl.b.hnpreq = 1;
			gotgctl.b.devhnpen = 1;
			dwc_modify_reg32(&global_regs->gotgctl, gotgctl.d32, 0);

			printk(KERN_ERR "Device Not Connected / Responding\n");
		}
	}
	if (gotgint.b.hstnegdet) {
		/*
		 * The disconnect interrupt is set at the same time as
		 * Host Negotiation Detected.  During the mode
		 * switch all interrupts are cleared so the disconnect
		 * interrupt handler will not get executed.
		 */
		if (dwc_otg_is_device_mode(core_if)) {
			hcd_disconnect(core_if);
			pcd_start(core_if);
			core_if->xceiv->state = OTG_STATE_A_PERIPHERAL;
		} else {
			/*
			 * Need to disable SOF interrupt immediately. When
			 * switching from device to host, the PCD interrupt
			 * handler won't handle the interrupt if host mode is
			 * already set. The HCD interrupt handler won't get
			 * called if the HCD state is HALT. This means that
			 * the interrupt does not get handled and Linux
			 * complains loudly.
			 */
			gintmsk.d32 = 0;
			gintmsk.b.sofintr = 1;
			dwc_modify_reg32(&global_regs->gintmsk, gintmsk.d32, 0);
			pcd_stop(core_if);
			hcd_start(core_if);
			core_if->xceiv->state = OTG_STATE_A_HOST;
		}
	}
	if (gotgint.b.adevtoutchng)
		printk(KERN_INFO  " ++OTG Interrupt: A-Device Timeout "
				"Change++\n");
	if (gotgint.b.debdone)
		printk(KERN_INFO  " ++OTG Interrupt: Debounce Done++\n");

	/* Clear GOTGINT */
	dwc_write_reg32(&core_if->core_global_regs->gotgint, gotgint.d32);
	return 1;
}

/*
 * Wakeup Workqueue implementation
 */
static void port_otg_wqfunc(struct work_struct *work)
{
	struct core_if *core_if = container_of(work, struct core_if,
			usb_port_otg);
	u32 count = 0;
	union gotgctl_data gotgctl = {.d32 = 0};

	printk(KERN_INFO "%s\n", __func__);
	gotgctl.d32 = dwc_read_reg32(&core_if->core_global_regs->gotgctl);
	if (gotgctl.b.conidsts) {
		/*
		 * B-Device connector (device mode) wait for switch to device
		 * mode.
		 */
		while (!dwc_otg_is_device_mode(core_if) && ++count <= 10000) {
			printk(KERN_INFO "Waiting for Peripheral Mode, "
				"Mode=%s\n", dwc_otg_is_host_mode(core_if) ?
				"Host" : "Peripheral");
			msleep(100);
		}
		BUG_ON(count > 10000);
		core_if->xceiv->state = OTG_STATE_B_PERIPHERAL;
		dwc_otg_core_init(core_if);
		dwc_otg_enable_global_interrupts(core_if);
		pcd_start(core_if);
	} else {
		/*
		 * A-Device connector (host mode) wait for switch to host
		 * mode.
		 */
		while (!dwc_otg_is_host_mode(core_if) && ++count <= 10000) {
			printk(KERN_INFO "Waiting for Host Mode, Mode=%s\n",
				dwc_otg_is_host_mode(core_if) ?
				"Host" : "Peripheral");
			msleep(100);
		}
		BUG_ON(count > 10000);
		core_if->xceiv->state = OTG_STATE_A_HOST;
		dwc_otg_core_init(core_if);
		dwc_otg_enable_global_interrupts(core_if);
		hcd_start(core_if);
	}
}

/**
 * This function handles the Connector ID Status Change Interrupt.  It
 * reads the OTG Interrupt Register (GOTCTL) to determine whether this
 * is a Device to Host Mode transition or a Host Mode to Device
 * Transition.
 *
 * This only occurs when the cable is connected/removed from the PHY
 * connector.
 */
static int dwc_otg_handle_conn_id_status_change_intr(struct core_if *core_if)
{
	union gintsts_data gintsts = {.d32 = 0};
	union gintmsk_data gintmsk = {.d32 = 0};

	/*
	 * Need to disable SOF interrupt immediately. If switching from device
	 * to host, the PCD interrupt handler won't handle the interrupt if
	 * host mode is already set. The HCD interrupt handler won't get
	 * called if the HCD state is HALT. This means that the interrupt does
	 * not get handled and Linux complains loudly.
	 */
	gintmsk.b.sofintr = 1;
	dwc_modify_reg32(&core_if->core_global_regs->gintmsk, gintmsk.d32, 0);

	INIT_WORK(&core_if->usb_port_otg, port_otg_wqfunc);
	schedule_work(&core_if->usb_port_otg);

	/* Set flag and clear interrupt */
	gintsts.b.conidstschng = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);
	return 1;
}

/**
 * This interrupt indicates that a device is initiating the Session
 * Request Protocol to request the host to turn on bus power so a new
 * session can begin. The handler responds by turning on bus power. If
 * the DWC_otg controller is in low power mode, the handler brings the
 * controller out of low power mode before turning on bus power.
 */
static int dwc_otg_handle_session_req_intr(struct core_if *core_if)
{
	union gintsts_data gintsts;

	if (!dwc_has_feature(core_if, DWC_HOST_ONLY)) {
		union hprt0_data hprt0;

		if (dwc_otg_is_device_mode(core_if)) {
			printk(KERN_INFO "SRP: Device mode\n");
		} else {
			printk(KERN_INFO "SRP: Host mode\n");

			/* Turn on the port power bit. */
			hprt0.d32 = dwc_otg_read_hprt0(core_if);
			hprt0.b.prtpwr = 1;
			dwc_write_reg32(core_if->host_if->hprt0, hprt0.d32);

			/*
			 * Start the Connection timer.
			 * A message can be displayed,
			 * if connect does not occur within 10 seconds.
			 */
			hcd_session_start(core_if);
		}
	}
	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.sessreqintr = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);
	return 1;
}

/**
 * This interrupt indicates that the DWC_otg controller has detected a
 * resume or remote wakeup sequence. If the DWC_otg controller is in
 * low power mode, the handler must brings the controller out of low
 * power mode. The controller automatically begins resume
 * signaling. The handler schedules a time to stop resume signaling.
 */
static int dwc_otg_handle_wakeup_detected_intr(struct core_if *core_if)
{
	union gintsts_data gintsts;
	struct device_if *dev_if = core_if->dev_if;

	if (dwc_otg_is_device_mode(core_if)) {
		union dctl_data dctl = {.d32 = 0};

		/* Clear the Remote Wakeup Signalling */
		dctl.b.rmtwkupsig = 1;
		dwc_modify_reg32(&dev_if->dev_global_regs->dctl, dctl.d32, 0);

		if (core_if->pcd_cb && core_if->pcd_cb->resume_wakeup)
			core_if->pcd_cb->resume_wakeup(core_if->pcd_cb->p);
	} else {
		union pcgcctl_data pcgcctl = {.d32 = 0};

		/* Restart the Phy Clock */
		pcgcctl.b.stoppclk = 1;
		dwc_modify_reg32(core_if->pcgcctl, pcgcctl.d32, 0);
		schedule_delayed_work(&core_if->usb_port_wakeup, 10);
	}

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.wkupintr = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);
	return 1;
}

/**
 * This interrupt indicates that a device has been disconnected from
 * the root port.
 */
static int dwc_otg_handle_disconnect_intr(struct core_if *core_if)
{
	union gintsts_data gintsts;
	struct core_global_regs *global_regs = core_if->core_global_regs;

	if (!dwc_has_feature(core_if, DWC_HOST_ONLY)) {
		if (core_if->xceiv->state == OTG_STATE_B_HOST) {
			hcd_disconnect(core_if);
			pcd_start(core_if);
			core_if->xceiv->state = OTG_STATE_B_PERIPHERAL;
		} else if (dwc_otg_is_device_mode(core_if)) {
			union gotgctl_data gotgctl = {.d32 = 0};

			gotgctl.d32 =
				dwc_read_reg32(&global_regs->gotgctl);

			/*
			 * If HNP is in process, do nothing.
			 * The OTG "Host Negotiation Detected"
			 * interrupt will do the mode switch.
			 * Otherwise, since we are in device mode,
			 * disconnect and stop the HCD,
			 * then start the PCD.
			 */
			if (!gotgctl.b.devhnpen) {
				hcd_disconnect(core_if);
				pcd_start(core_if);
				core_if->xceiv->state = OTG_STATE_B_PERIPHERAL;
			}
		} else if (core_if->xceiv->state == OTG_STATE_A_HOST) {
			/* A-Cable still connected but device disconnected. */
			hcd_disconnect(core_if);
		}
	}
	gintsts.d32 = 0;
	gintsts.b.disconnect = 1;
	dwc_write_reg32(&global_regs->gintsts, gintsts.d32);
	return 1;
}

/**
 * This interrupt indicates that SUSPEND state has been detected on
 * the USB.
 *
 * For HNP the USB Suspend interrupt signals the change from
 * "a_peripheral" to "a_host".
 *
 * When power management is enabled the core will be put in low power
 * mode.
 */
static int dwc_otg_handle_usb_suspend_intr(struct core_if *core_if)
{
	union dsts_data dsts;
	union gintsts_data gintsts;
	struct device_if *dev_if = core_if->dev_if;

	if (dwc_otg_is_device_mode(core_if)) {
		struct dwc_pcd *pcd;
		/*
		 * Check the Device status register to determine if the Suspend
		 * state is active.
		 */
		dsts.d32 = dwc_read_reg32(&dev_if->dev_global_regs->dsts);
		/* PCD callback for suspend. */
		pcd = (struct dwc_pcd *)core_if->pcd_cb->p;
		pcd_suspend(core_if);
	} else {
		if (core_if->xceiv->state == OTG_STATE_A_PERIPHERAL) {
			/* Clear the a_peripheral flag, back to a_host. */
			pcd_stop(core_if);
			hcd_start(core_if);
			core_if->xceiv->state = OTG_STATE_A_HOST;
		}
	}

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.usbsuspend = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);
	return 1;
}

/**
 * This function returns the Core Interrupt register.
 *
 * Although the Host Port interrupt (portintr) is documented as host mode
 * only, it appears to occur in device mode when Port Enable / Disable Changed
 * bit in HPRT0 is set. The code in dwc_otg_handle_common_intr checks if in
 * device mode and just clears the interrupt.
 */
static inline u32 dwc_otg_read_common_intr(struct core_if *core_if)
{
	union gintsts_data gintsts;
	union gintmsk_data gintmsk;
	union gintmsk_data gintmsk_common = {.d32 = 0};

	gintmsk_common.b.wkupintr = 1;
	gintmsk_common.b.sessreqintr = 1;
	gintmsk_common.b.conidstschng = 1;
	gintmsk_common.b.otgintr = 1;
	gintmsk_common.b.modemismatch = 1;
	gintmsk_common.b.disconnect = 1;
	gintmsk_common.b.usbsuspend = 1;
	gintmsk_common.b.portintr = 1;

	gintsts.d32 = dwc_read_reg32(&core_if->core_global_regs->gintsts);
	gintmsk.d32 = dwc_read_reg32(&core_if->core_global_regs->gintmsk);

	return (gintsts.d32 & gintmsk.d32) & gintmsk_common.d32;
}

/**
 * Common interrupt handler.
 *
 * The common interrupts are those that occur in both Host and Device mode.
 * This handler handles the following interrupts:
 * - Mode Mismatch Interrupt
 * - Disconnect Interrupt
 * - OTG Interrupt
 * - Connector ID Status Change Interrupt
 * - Session Request Interrupt.
 * - Resume / Remote Wakeup Detected Interrupt.
 *
 * - Host Port Interrupt.  Although this interrupt is documented as only
 *   occurring in Host mode, it also occurs in Device mode when Port Enable /
 *   Disable Changed bit in HPRT0 is set. If it is seen here, while in Device
 *   mode, the interrupt is just cleared.
 *
 */
int dwc_otg_handle_common_intr(struct core_if *core_if)
{
	int retval = 0;
	union gintsts_data gintsts;

	gintsts.d32 = dwc_otg_read_common_intr(core_if);

	if (gintsts.b.modemismatch)
		retval |= dwc_otg_handle_mode_mismatch_intr(core_if);
	if (gintsts.b.otgintr)
		retval |= dwc_otg_handle_otg_intr(core_if);
	if (gintsts.b.conidstschng)
		retval |= dwc_otg_handle_conn_id_status_change_intr(core_if);
	if (gintsts.b.disconnect)
		retval |= dwc_otg_handle_disconnect_intr(core_if);
	if (gintsts.b.sessreqintr)
		retval |= dwc_otg_handle_session_req_intr(core_if);
	if (gintsts.b.wkupintr)
		retval |= dwc_otg_handle_wakeup_detected_intr(core_if);
	if (gintsts.b.usbsuspend)
		retval |= dwc_otg_handle_usb_suspend_intr(core_if);

	if (gintsts.b.portintr && dwc_otg_is_device_mode(core_if)) {
		gintsts.d32 = 0;
		gintsts.b.portintr = 1;
		dwc_write_reg32(&core_if->core_global_regs->gintsts,
				gintsts.d32);
		retval |= 1;
		printk(KERN_INFO "RECEIVED PORTINT while in Device mode\n");
	}

	return retval;
}
