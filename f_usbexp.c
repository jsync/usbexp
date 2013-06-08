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

static int __initcreate_bulk_endpoints(struct f_usbexp *dev,
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
	dev->rx_req = req;

	for(i = 0; i < TX_REQ_MAX; i++) {
		req = usbexp_request_new(dev->ep_in, BULK_BUFFER_SIZE);
		if(!req)
			goto fail;

		req->complete = usbexp_complete_in;
		req_put(dev, &dev->tx_idle, req);
	}

	return 0;

fail:
	printk(KERN_ERR, "usbexp_bind() could not allocate requests\n");
	return -1;
}

static ssize_t usbexp_read(struct file *fp, char __user *buf,
		size_t count, loff_t *pos)
{
	struct f_usbexp *dev = fp->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req;
	int r = count, xfer;
	int ret;

	DBG(cdev, "usbexp_read(%d)\n", count);

	if(count > BULK_BUFFER_SIZE)
		return -EINVAL;

	if(_lock(&dev->read_excl))
		return -EBUSY;

	/* we will block until we're online */
	while(!(dev->online || dev->error)) {
		DBG(cdev, "usbexp_read: waiting for online state\n");
		ret = wait_event_interruptible(dev->read_wq, (dev->online || dev->error));
		if(ret < 0) {
			_unlock(&dev->read_excl);
			return ret;
		}
	}
	if(dev->error) {
		r = -EIO;
		goto done;
	}

requeue_req:
	/* queue a request */
	req = dev->rx_req;
	req->length = count;
	dev->rx_done = 0;
	ret = usb_ep_queue(dev->ep_out, req, GFP_ATOMIC);
	if (ret < 0) {
		DBG(cdev, "usbexp_read: failed to queue req %p (%d)\n", req, ret);
		r = -EIO;
		dev->error = 1;
		goto done;
	} else {
		DBG(cdev, "rx %p queue\n", req);
	}

	/* wait for a request to complete */
	ret = wait_event_interruptible(dev->read_wq, dev->rx_done);
	if(ret < 0) {
		dev->error = 1;
		r = ret;
		usb_ep_dequeue(dev->ep_out, req);
		goto done;
	}

	if(!dev->error) {
		/* If we got a 0-len packet, throw it back and try again */
		if(req->actual == 0)
			goto requeue_req;

		DBG(cedv, "rx %p %d\n", req, req->acutual);
		xfer = (req->actual < count ) ? req->actual : count ;
		if(copy_to_user(buf, req->buf, xfer))
			r = -EFAULT;
	} else
		r = -EIO;

done:
	_unlock(&dev->read_excl);
	DBG(cdev, "usbexp_read returning %d\n", r);
	return r;
}

static ssize_t usbexp_write(struct file *fp, const char __user *buf,
		size_t count, loff_t *pos)
{
	struct f_usbexp *dev = fp->private_data;
	struct usb_composite_dev *cdev = dev->cdev;
	struct usb_request *req = 0;
	int r = count, xfer;
	int ret;

	DBG(cdev, "usbexp_write (%d)\n", count);

	if(_lock(&dev->write_excl))
		return -EBUSY;

	while(count > 0) {
		if(dev->error) {
			DBG(cdev, "usbexp_write dev->error\n");
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
				((req = req_get(dev, &dev->tx_idle)) || dev->error));

		if(ret < 0) {
			r = ret;
			break;
		}

		if(req != 0 ) {
			if (count > BULK_BUFFER_SIZE)
				xfer = BULK_BUFFER_SIZE;
			else
				xfer = count;
			if(copy_from_user(req->buf, buf, xfer) {
				r = -EFAULT;
				break;
			}
			req->length = xfer;
			ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
			if(ret < 0) {
				DBG(cdev, "usbexp_write: xfer error %d\n", ret);
				dev->error = 1;
				r = -EIO;
				break;
			}

			buf += xfer;
			count -= xfer;

			/* zero this so we don't try it on error exit */
			req = 0;
		}
	}
	if(req)
		req_put(dev, &dev->tx_idle, req);

	_unlock(&dev->write_excl);
	DBG(cdev, "usbexp_write returning %d\n", r);
	return r;
}
static int usbexp_open(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "usbexp_open\n");
	if (_lock(&_f_usbexp->open_excl))
		return -EBUSY;

	fp->private_data = _f_usbexp;

	/* clear the error latch */
	_f_usbexp->error = 0;

	return 0;
}

static int usbexp_release(struct inode *ip, struct file *fp)
{
	printk(KERN_INFO "usbexp_release\n");
	_unlock(&_f_usbexp->open_excl);
	return 0;
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



