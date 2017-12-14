/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Sample app for WebUSB enabled custom class driver.
 *
 * Sample app for WebUSB enabled custom class driver. The received
 * data is echoed back to the WebUSB based application running in
 * the browser at host.
 */

#include <stdio.h>
#include <string.h>
#include <device.h>
#include <zephyr.h>
#include <net/buf.h>

#include "webusb-driver-netbuf.h"
#include "webusb-ide.h"

static K_FIFO_DEFINE(rx_queue);

/* WebUSB Platform Capability Descriptor */
static const u8_t webusb_bos_descriptor[] = {
	/* Binary Object Store descriptor */
	0x05, 0x0F, 0x1D, 0x00, 0x01,

	/* WebUSB Platform Capability Descriptor:
	 * https://wicg.github.io/webusb/#webusb-platform-capability-descriptor
	 */
	0x18, 0x10, 0x05, 0x00,

	/* PlatformCapability UUID */
	0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47,
	0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65,

	/* Version, VendorCode, Landing Page */
	0x00, 0x01, 0x01, 0x01,
};

/* URL Descriptor: https://wicg.github.io/webusb/#url-descriptor */
static const u8_t webusb_origin_url[] = {
	/* Length, DescriptorType, Scheme */
	0x11, 0x03, 0x00,
	'l', 'o', 'c', 'a', 'l', 'h', 'o', 's', 't', ':', '8', '0', '0', '0'
};

/**
 * @brief Custom handler for standard requests in
 *        order to catch the request and return the
 *        WebUSB Platform Capability Descriptor.
 *
 * @param pSetup    Information about the request to execute.
 * @param len       Size of the buffer.
 * @param data      Buffer containing the request result.
 *
 * @return  0 on success, negative errno code on fail
 */
int custom_handle_req(struct usb_setup_packet *pSetup,
		s32_t *len, u8_t **data)
{
	if (GET_DESC_TYPE(pSetup->wValue) == DESCRIPTOR_TYPE_BOS) {
		*data = (u8_t *)(&webusb_bos_descriptor);
		*len = sizeof(webusb_bos_descriptor);

		return 0;
	}

	return -ENOTSUP;
}

/**
 * @brief Handler called for WebUSB vendor specific commands.
 *
 * @param pSetup    Information about the request to execute.
 * @param len       Size of the buffer.
 * @param data      Buffer containing the request result.
 *
 * @return  0 on success, negative errno code on fail.
 */
int vendor_handle_req(struct usb_setup_packet *pSetup,
		s32_t *len, u8_t **data)
{
	/* Get URL request */
	if (pSetup->bRequest == 0x01 && pSetup->wIndex == 0x02) {
		u8_t index = GET_DESC_INDEX(pSetup->wValue);

		if (index != 1)
			return -ENOTSUP;

		*data = (u8_t *)(&webusb_origin_url);
		*len = sizeof(webusb_origin_url);
		return 0;
	}

	return -ENOTSUP;
}

#if 0
static void hexdump(const char *str, const u8_t *packet, size_t length)
{
	int n = 0;

	if (!length) {
		printf("%s zero-length signal packet\n", str);
		return;
	}

	while (length--) {
		if (n % 16 == 0) {
			printf("%s %08X ", str, n);
		}

		printf("%02X ", *packet++);

		n++;
		if (n % 8 == 0) {
			if (n % 16 == 0) {
			    printf("\n");
			} else {
			    printf(" ");
			}
		}
	}

	if (n % 16) {
        printf("\n");
	}
}

static void read_and_echo_data(struct net_buf *buf)
{
	hexdump("<", buf->data, buf->len);

	/* Read all data and echo it back */
	webusb_write(buf);
	net_buf_unref(buf);
}
#endif

static webusb_rx_handler_t rx_handler = NULL;

static void rx_callback(struct net_buf *buf)
{
  if (!rx_handler) {
  	return;
  }
  rx_handler(buf->data, buf->len);
	net_buf_unref(buf);
}

/* Custom and Vendor request handlers */
static struct webusb_req_handlers req_handlers = {
	.custom_handler = custom_handle_req,
	.vendor_handler = vendor_handle_req,
	.rx_cb = rx_callback,
};

void test_main(void)
{
	/* Set the custom and vendor request handlers */
	webusb_register_request_handlers(&req_handlers);
}

void webusb_init(webusb_rx_handler_t receiver)
{
	/* Set the custom and vendor request handlers */
	webusb_register_request_handlers(&req_handlers);
  rx_handler = receiver;
}
