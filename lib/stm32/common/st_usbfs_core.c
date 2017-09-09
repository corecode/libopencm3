/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2015 Robin Kreis <r.kreis@uni-bremen.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/assert.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/tools.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/usb/usbd.h>
#include "../../usb/usb_private.h"
#include "st_usbfs_core.h"

#include <libopencm3/stm32/gpio.h>

/* TODO - can't these be inside the impls, not globals from the core? */
uint8_t st_usbfs_force_nak[8];
struct _usbd_device st_usbfs_dev;

void st_usbfs_set_address(usbd_device *dev, uint8_t addr)
{
	(void)dev;
	/* Set device address and enable. */
	SET_REG(USB_DADDR_REG, (addr & USB_DADDR_ADDR) | USB_DADDR_EF);
}

/**
 * Set the receive buffer size for a given USB endpoint.
 *
 * @param ep Index of endpoint to configure.
 * @param size Size in bytes of the RX buffer.
 */
void st_usbfs_set_ep_rx_bufsize(usbd_device *dev, uint8_t ep, uint32_t size)
{
	(void)dev;
	if (size > 62) {
		if (size & 0x1f) {
			size -= 32;
		}
		USB_SET_EP_RX_COUNT(ep, (size << 5) | 0x8000);
	} else {
		if (size & 1) {
			size++;
		}
		USB_SET_EP_RX_COUNT(ep, size << 10);
	}
}

void st_usbfs_ep_setup(usbd_device *dev, uint8_t addr, uint8_t type,
		uint16_t max_size,
		void (*callback) (usbd_device *usbd_dev,
		uint8_t ep))
{
	/* Translate USB standard type codes to STM32. */
	const uint16_t typelookup[] = {
		[USB_ENDPOINT_ATTR_CONTROL] = USB_EP_TYPE_CONTROL,
		[USB_ENDPOINT_ATTR_ISOCHRONOUS] = USB_EP_TYPE_ISO,
		[USB_ENDPOINT_ATTR_BULK] = USB_EP_TYPE_BULK,
		[USB_ENDPOINT_ATTR_INTERRUPT] = USB_EP_TYPE_INTERRUPT,
	};
	uint8_t dir = addr & 0x80;
	addr &= 0x7f;

	/* Assign address. */
	USB_SET_EP_ADDR(addr, addr);
	USB_SET_EP_TYPE(addr, typelookup[type]);

	if (dir || (addr == 0)) {
		USB_SET_EP_TX_ADDR(addr, dev->pm_top);
		if (callback) {
			dev->user_callback_ctr[addr][USB_TRANSACTION_IN] =
			    (void *)callback;
		}
		USB_CLR_EP_TX_DTOG(addr);
		USB_SET_EP_TX_STAT(addr, USB_EP_TX_STAT_NAK);
		dev->pm_top += max_size;
	}

	if (!dir) {
		USB_SET_EP_RX_ADDR(addr, dev->pm_top);
		st_usbfs_set_ep_rx_bufsize(dev, addr, max_size);
		if (callback) {
			dev->user_callback_ctr[addr][USB_TRANSACTION_OUT] =
			    (void *)callback;
		}
		USB_CLR_EP_RX_DTOG(addr);
		USB_SET_EP_RX_STAT(addr, USB_EP_RX_STAT_VALID);
		dev->pm_top += max_size;
	}
}

void st_usbfs_endpoints_reset(usbd_device *dev)
{
	int i;

	/* Reset all endpoints. */
	for (i = 1; i < 8; i++) {
		USB_SET_EP_TX_STAT(i, USB_EP_TX_STAT_DISABLED);
		USB_SET_EP_RX_STAT(i, USB_EP_RX_STAT_DISABLED);
		/* USB_CLR_EP_RX_CTR(i); */
		/* USB_CLR_EP_TX_CTR(i); */
	}
	dev->pm_top = USBD_PM_TOP + (2 * dev->desc->bMaxPacketSize0);
}

void st_usbfs_ep_stall_set(usbd_device *dev, uint8_t addr,
				   uint8_t stall)
{
	(void)dev;
	if (addr == 0) {
		USB_SET_EP_TX_STAT(addr, stall ? USB_EP_TX_STAT_STALL :
				   USB_EP_TX_STAT_NAK);
	}

	if (addr & 0x80) {
		addr &= 0x7F;

		USB_SET_EP_TX_STAT(addr, stall ? USB_EP_TX_STAT_STALL :
				   USB_EP_TX_STAT_NAK);

		/* Reset to DATA0 if clearing stall condition. */
		if (!stall) {
			USB_CLR_EP_TX_DTOG(addr);
		}
	} else {
		/* Reset to DATA0 if clearing stall condition. */
		if (!stall) {
			USB_CLR_EP_RX_DTOG(addr);
		}

		if (stall && addr == 0) {
			USB_CLR_EP_RX_CTR(addr);
		}
		USB_SET_EP_RX_STAT(addr, stall ? USB_EP_RX_STAT_STALL :
				   USB_EP_RX_STAT_VALID);
		if (stall && addr != 0) {
			USB_CLR_EP_RX_CTR(addr);
		}
	}
}

uint8_t st_usbfs_ep_stall_get(usbd_device *dev, uint8_t addr)
{
	(void)dev;
	if (addr & 0x80) {
		if ((*USB_EP_REG(addr & 0x7F) & USB_EP_TX_STAT) ==
		    USB_EP_TX_STAT_STALL) {
			return 1;
		}
	} else {
		if ((*USB_EP_REG(addr) & USB_EP_RX_STAT) ==
		    USB_EP_RX_STAT_STALL) {
			return 1;
		}
	}
	return 0;
}

void st_usbfs_ep_nak_set(usbd_device *dev, uint8_t addr, uint8_t nak)
{
	(void)dev;
	/* It does not make sense to force NAK on IN endpoints. */
	if (addr & 0x80) {
		return;
	}

	st_usbfs_force_nak[addr] = nak;

	if (nak) {
		USB_SET_EP_RX_STAT(addr, USB_EP_RX_STAT_NAK);
	} else {
		/* if (addr == 0) { */
		/* 	if (dev->control_state.state == STATUS_OUT) { */
		/* 		USB_SET_EP_KIND(addr); */
		/* 	} else { */
		/* 		USB_CLR_EP_KIND(addr); */
		/* 	} */
		/* } */
		USB_SET_EP_RX_STAT(addr, USB_EP_RX_STAT_VALID);
	}
}

uint16_t st_usbfs_ep_write_packet(usbd_device *dev, uint8_t addr,
				     const void *buf, uint16_t len)
{
	(void)dev;
	addr &= 0x7F;

	if ((*USB_EP_REG(addr) & USB_EP_TX_STAT) == USB_EP_TX_STAT_VALID) {
		/* return 0; */
		*USB_CNTR_REG = USB_CNTR_PWDN;
		gpio_clear(GPIOB, GPIO1);
		cm3_assert_not_reached();
	}

	st_usbfs_copy_to_pm(USB_GET_EP_TX_BUFF(addr), buf, len);
	USB_SET_EP_TX_COUNT(addr, len);
	USB_SET_EP_TX_STAT(addr, USB_EP_TX_STAT_VALID);

	return len;
}

uint16_t st_usbfs_ep_read_packet(usbd_device *dev, uint8_t addr,
					 void *buf, uint16_t len)
{
	(void)dev;
	// uint16_t epr;
	// uint16_t epr2;

	if (((*USB_EP_REG(addr)) & USB_EP_RX_STAT) == USB_EP_RX_STAT_VALID || /* if it is already marked as ready */
	    !((*USB_EP_REG(addr)) & USB_EP_RX_CTR)) { /* or if the transfer has already been acknowledged */
		cm3_assert_not_reached();
	}

	len = MIN(USB_GET_EP_RX_COUNT(addr) & 0x3ff, len);
	st_usbfs_copy_from_pm(buf, USB_GET_EP_RX_BUFF(addr), len);
	/* USB_SET_EP_RX_COUNT(addr, 0); */

	/**
	 * - check that RX_CTR has not been cleared before we enter here.
	 *
	 * - if we're not dealing with any past or future SETUP packets, then everything is simple.
	 * - if we are about to read a SETUP packet...
	 *   - check for SETUP?
	 *   - clear both RX_STAT and RX_CTR at the same time
	 * - if a SETUP packet is about to arrive after the one we're ACK'ing here...
	 *   - clear RX_CTR and set RX_STAT to VALID at the same time.
	 *   - check for SETUP before doing this?
	 */

	/* uint16_t epr = *USB_EP_REG(addr); */
	/* uint16_t epr_write = epr & ~(USB_EP_RX_DTOG | USB_EP_RX_STAT | USB_EP_TX_DTOG | USB_EP_TX_STAT); */
	/* epr_write |= USB_EP_RX_CTR | USB_EP_TX_CTR; */

	/* /\* clear CTR_RX *\/ */
	/* epr_write &= ~USB_EP_RX_CTR; */

	/* if (!st_usbfs_force_nak[addr]) { */
	/* 	uint16_t cur_rx_stat = epr & USB_EP_RX_STAT_TOG_MSK; */
	/* 	epr_write |= USB_EP_RX_STAT_VALID ^ cur_rx_stat; */
	/* } */
	/* if (addr == 0) { */
	/* 	if (dev->control_state.state == STATUS_OUT) { */
	/* 		epr_write |= USB_EP_KIND; */
	/* 	} else { */
	/* 		epr_write &= ~USB_EP_KIND; */
	/* 	} */
	/* } */

	/* *USB_EP_REG(addr) = epr_write; */

	if (!st_usbfs_force_nak[addr]) {
		USB_SET_EP_RX_STAT(addr, USB_EP_RX_STAT_VALID);
	}
	USB_CLR_EP_RX_CTR(addr);

	return len;
}

void st_usbfs_poll(usbd_device *dev)
{
	uint16_t istr = *USB_ISTR_REG;

	for (; istr & USB_ISTR_CTR; istr = *USB_ISTR_REG) {
		uint8_t ep = istr & USB_ISTR_EP_ID;
		uint8_t type;

		/**
		 * Possible states for a control endpoint:
		 * - DIR, RX_CTR, SETUP, ~tx_ctr: handled SETUP token, trivial
		 * - DIR, RX_CTR, ~setup, ~tx_ctr: handled DATA OUT token, trivial
		 * - ~dir, ~rx_ctr, ~setup, TX_CTR: handled DATA IN token, trivial
		 * - DIR, RX_CTR, SETUP, TX_CTR: handled DATA IN token (potentially previous control transfer confirmation), handled SETUP token
		 * - DIR, RX_CTR, ~setup, TX_CTR: handled DATA IN token and DATA OUT token.  Order is not clear.
		 *   Avoid this situation by only allowing either IN or OUT be VALID at any time and set the other to STALL or NAK.
		 * Impossible:
		 * - DIR, RX_CTR, SETUP, ~tx_ctr: handled DATA OUT token (potentially previous control transfer confirmation), handled SETUP token
		 *   This cannot happen, because while RX_CTR is set, SETUP will not be set, i.e. an incoming SETUP token will be discarded.
		 *
		 * Processing order:
		 * - if TX_CTR and RX_CTR (and SETUP) set, then process TX before SETUP.  If SETUP not set, our state machine made a mistake.
		 */

		volatile uint16_t epr = *USB_EP_REG(ep);
		int rx = !!(epr & USB_EP_RX_CTR);
		int tx = !!(epr & USB_EP_TX_CTR);

		if (rx && (epr & USB_EP_RX_STAT) == USB_EP_RX_STAT_DISABLED) {
			USB_CLR_EP_RX_CTR(ep);
			continue;
		}

		if (tx && (epr & USB_EP_TX_STAT) == USB_EP_TX_STAT_DISABLED) {
			USB_CLR_EP_TX_CTR(ep);
			continue;
		}


		if (rx && (epr & USB_EP_RX_STAT) == USB_EP_RX_STAT_VALID) {
			/* loop until clear */
			continue;
		}

		if (tx && (epr & USB_EP_TX_STAT) == USB_EP_TX_STAT_DISABLED) {
			/* loop until clear */
			continue;
		}


		if (rx && tx) {
			type = USB_TRANSACTION_IN;
		} else if (rx) {
			/* OUT or SETUP? */
			if (epr & USB_EP_SETUP) {
				type = USB_TRANSACTION_SETUP;
			} else {
				type = USB_TRANSACTION_OUT;
			}
		} else if (tx) {
			type = USB_TRANSACTION_IN;
		} else {
			cm3_assert_not_reached();
		}

		if (type == USB_TRANSACTION_IN)
			USB_CLR_EP_TX_CTR(ep);

		/* static volatile int callback_count; */

		if (dev->user_callback_ctr[ep][type]) {
			/* if ((*USB_EP_REG(0) & USB_EP_TX_STAT) == USB_EP_TX_STAT_VALID) { */
			/* 	cm3_assert_not_reached(); */
			/* } */
			/* gpio_toggle(GPIOA, GPIO10); */
			/* callback_count++; */

			/* if (callback_count == 8) { */
			/* 	*USB_CNTR_REG = USB_CNTR_PWDN; */
			/* 	for (volatile int i = 0; i < 10; i++) */
			/* 		i++; */
			/* 	/\* gpio_clear(GPIOB, GPIO1); *\/ */
			/* 	/\* cm3_assert_not_reached(); *\/ */
			/* } */

			dev->user_callback_ctr[ep][type] (dev, ep);
			/* gpio_toggle(GPIOA, GPIO10); */
		} else {
			if (type != USB_TRANSACTION_IN)
				USB_CLR_EP_RX_CTR(ep);
		}
	}

	if (istr & USB_ISTR_RESET) {
		USB_CLR_ISTR_RESET();
		dev->pm_top = USBD_PM_TOP;
		_usbd_reset(dev);
		return;
	}

	if (istr & USB_ISTR_SUSP) {
		USB_CLR_ISTR_SUSP();
		if (dev->user_callback_suspend) {
			dev->user_callback_suspend();
		}
	}

	if (istr & USB_ISTR_WKUP) {
		USB_CLR_ISTR_WKUP();
		if (dev->user_callback_resume) {
			dev->user_callback_resume();
		}
	}

	if (istr & USB_ISTR_SOF) {
		USB_CLR_ISTR_SOF();
		if (dev->user_callback_sof) {
			dev->user_callback_sof();
		}
	}

	if (dev->user_callback_sof) {
		*USB_CNTR_REG |= USB_CNTR_SOFM;
	} else {
		*USB_CNTR_REG &= ~USB_CNTR_SOFM;
	}
}
