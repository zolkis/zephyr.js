// Copyright (c) 2016-2017, Intel Corporation.

#ifndef __ide_webusb_h__
#define __ide_webusb_h__

typedef void (*webusb_rx_handler_t)(u8_t *data, size_t len);

void webusb_init(webusb_rx_handler_t receiver);

// Provided by the WebUSB driver.
extern int webusb_write(u8_t *data, size_t len);

#endif  // __ide_webusb_h__
