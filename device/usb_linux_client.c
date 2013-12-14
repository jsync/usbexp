/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "usbexp.h"

void usb_cleanup()
{
	//nothing to do here
}

static void *usb_open_thread(void *x)
{
	struct usb_handle *usb = (struct usb_handle *)x;
	int fd;

	while(1) {
		//wait untill the USB device needs opening
		pthread_mutex_lock(&usb->lock);
		while(usb->fd != -1)
			pthread_cond_wait(&usb->notify, &usb->lock);
		pthread_mutex_unlock(&usb->lock);
		D("[ usb_thread - opening device ]\n");
		do {
			/* XXX use inotify */
			fd = open("/dev/android_usbexp", O_RDWR);
			if (fd < 0) {
				usleep(1000 * 1000);
			}
		} while(fd < 0);
		D("[Opening device suceeded]\n");

		fcntl(fd, F_SETFD, FD_CLOEXEC);
		usb->fd = fd;

		D("[ usb_thread - registering device ]\n");

		// not sure if this is needed
		register_usb_transport(usb);
	}
	// never gets here
	return 0;	
}

int usb_write(usb_handle *h, const void *data, int len)
{
	int n;

//	D("[ write %d ]\n", len);
	n = write(h->fd, data, len);
	if (n != len) {
		D("[ERROR: n  = %d, errno = %d (%s)]\n", 
				n, errno, strerror(errno));
		return -1;
	}
//	D("[ done ]\n");
	return 0;
}

int usb_read(usb_handle *h, void *data, int len)
{
	int n;

	D("[ read %d ]\n", len);
	n = read(h->fd, data, len);
	if(n != len) {
		D("ERROR: n = %d, errno = %d (%s)\n",
			n, errno, strerror(errno));
		return -1;
	}
	return 0;
}

void usb_init()
{
	usb_handle *h;
	pthread_t tid;
	pthread_attr_t attr;
	
	int fd;

	h = calloc(1, sizeof(usb_handle));
	h->fd = -1;
	pthread_cond_init(&h->notify, 0);
	pthread_mutex_init(&h->lock, 0);
	D(" usb_init \n");

	/*  open the file /dev/android_usbexp_enable to trigger
	    the enabling of the usbexp USB function in the kernel.
	    We never touch this file again - just leave it open
	    indefinitely so the kernel will know when we are running
	    and when we are not. */

	fd = open("/dev/android_usbexp_enable", O_RDWR);
	D(" /dev/android_usbexp_enable opening fd = %d ! \n", fd);
	if(fd < 0) {
		D("Failed to open /dev/android_usbexp_enable\n");
	} else {
		D(" /dev/android_usbexp_enable opening fd = %d ! \n", fd);
		fcntl(fd, F_SETFD, FD_CLOEXEC);
	}
	D(" /dev/android_usbexp_enable opened ! \n");

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

	D("[ usb_init - starting thread ]\n");
	
	if(pthread_create(&tid, &attr, usb_open_thread, h)) {
		D("Cannot create usb thread errno = %d msg = $s\n", errno, strerror(errno));
	}
}

void usb_kick(usb_handle *h)
{
	D(" usb_kick\n");
	pthread_mutex_lock(&h->lock);
	close(h->fd);
	h->fd = -1;

	// notify usb_open_thread that we are disconnected 
	pthread_cond_signal(&h->notify);
	pthread_mutex_lock(&h->lock);
}

int usb_close(usb_handle *h)
{
	// nothing to do here 
	return 0;
}
