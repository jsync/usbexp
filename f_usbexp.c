/*
 * f_usbexp.c
 *
 * Experiments based on USB composite framework
 *
 * Created on: Jun 6, 2013
 * Author: Jesslyn Abdul Salam <jesslyn.abdulsalam@gmail.com>
 *
 * Derived from Gadget Driver for Android ADB
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/err.h>
#include <linux/interrupt.h>

#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <linux/usb/android_composite.h>
#include <linux/sched.h>

#define BULK_BUFFER_SIZE		4096

/* number of tx request to allocate */
#define TX_REQ_MAX 4

static const char shortname[] = "android_usbexp";

struct f_usbexp {
	struct usb_function function;
	struct usb_composite_dev *cdev;
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_ep *ep_out;

	int online;
	int error;

	atomic_t read_excl;
	atomic_t write_excl;
	atomic_t open_excl;

	struct list_head tx_idle;

	wait_queue_head_t read_wq;
	wait_queue_head_t write_wq;
	struct usb_request *rx_req;
	int rx_done;
};


static struct usb_interface usbexp_interface_desc = {
	.bLength		= USB_DT_INTERFACE_SIZE,
	.bDescriptorType	= USB_DT_INTERFACE,
	.bInterfaceNumber       = 0,
	.bNumEndpoints          = 2,
	.bInterfaceClass        = 0xFF,
	.bInterfaceSubClass     = 0x42,
	.bInterfaceProtocol     = 1,
};

static struct usb_endpoint_descriptor usbexp_highspeed_in_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT, 
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= __constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor usbexp_highspeed_out_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT, 
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize		= __constant_cpu_to_le16(512),

};

static struct usb_endpoint_descriptor usbexp_fullspeed_in_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT, 
	.bEndpointAddress	= USB_DIR_IN,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor usbexp_fullspeed_out_desc = {
	.bLength		= USB_DT_ENDPOINT_SIZE,
	.bDescriptorType	= USB_DT_ENDPOINT, 
	.bEndpointAddress	= USB_DIR_OUT,
	.bmAttributes		= USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_usbexp_desc[] = {
	(struct usb_descriptor_header *) &usbexp_interface_desc,
	(struct usb_descriptor_header *) &usbexp_fullspeed_in_desc,
	(struct usb_descriptor_header *) &usbexp_fullspeed_out_desc,
	NULL,
};

static struct usb_descriptor_header *hs_usbexp_desc[] = {
	(struct usb_descriptor_header *) &usbexp_interface_desc,
	(struct usb_descriptor_header *) &usbexp_highspeed_in_desc,
	(struct usb_descriptor_header *) &usbexp_highspeed_out_desc,
	NULL,
};

static struct f_usbexp *_f_usbexp;

static atomic_t usbexp_enable_excl;
static inline struct f_usbexp *func_to_dev(struct usb_function *f)
{
	return container_of(f, struct f_usbexp, function);
}

static struct usb_request *usbexp_request_new(struct usb_ep *ep, int buffer_size)
{
	static usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if(!req)
		return NULL;

	/* Now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if(!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

static void usbexp_request_free(struct usb_request *req, struct usb_ep *ep) 
{
	if(req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static inline int _lock(atomic_t *excl)
{
	if(atomic_inc_return(excl) == 1) {
		return 0;
	} else { 
		atomic_dec(excl);
		return -1;
	}
}

static inline void _unlock(atomic_t *excl)
{
	atomci_dec(excl);

}

/* Add a request to the tail of a list */
void req_put(struct f_usbexp *dev, struct list_head *head, struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* Remove a request from the head of a list */
struct usb_request *req_get(struct f_usbexp *dev, struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if(list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void usbexp_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	struct f_usbexp *dev = _f_usbexp;

	if(req->status != 0)
		dev->error = 1;

	req_put(dev, &dev->tx_idle, req);

	wake_up(&dev->write_wq);

}

static void usbexp_complete_out(struct usb_ep *ep, struct usb_request *req)
{
	struct f_usbexp *dev = _f_usbexp; 

	dev->rx_done = 1;
	if(req->status != 0)
		dev->error = 1;
	
	wake_up(&dev->read_wq);
}

static int __init create_bulk_endpoints(struct f_usbexp *dev, 
		struct usb_endpoint_descriptor *in_desc,
		struct usb_endpoint_descriptor *out_desc)
{
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	struct usb_ep *ep;
	int i;

	DBG(cdev, "create_bulk_endpoints dev : %p\n", dev);

	ep = usb_ep_autoconfig(cdev->gadget, in_desc);
	if(!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_in failed\n");
		return -ENODEV;
	}

	DBD(cdev, "usb_ep_autoconfig for ep_in got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_in = ep;

	ep = usb_ep_autoconfig(cdev->gadget, out_desc);
	if(!ep) {
		DBG(cdev, "usb_ep_autoconfig for ep_out failed\n");
		return -ENODEV;
	}

	DBG(cdev, "usb_ep_autoconfig for ep_out got %s\n", ep->name);
	ep->driver_data = dev;		/* claim the endpoint */
	dev->ep_out = ep;

	/* now allocate requests for our endpoints */
	req = adb_request_new(dev->ep_out, BULK_BUFFER_SIZE);
	if(!req)
		goto fail;

	req->complete = usbexp_complete_out;


}
static struct android_usb_function usbexp_function = {
		.name			= "usbexp",
		.bind_config	= usbexp_bind_config,
};

static int __init init(void)
{
	printk(KERN_INFO "f_usbexp init\n");
	return 0;
}

module_init(init);



