/*
 *  usbexp.h
 *
 * Created on: Jun 12, 2013
 * Author: Jesslyn Abdul Salam <jesslyn.abdulsalam@gmail.com>
 *
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <dirent.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <linux/usbdevice_fs.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 20)
#include <linux/usb/ch9.h>
#else
#include <linux/usb_ch9.h>
#endif
#include <asm/byteorder.h>

#include <pthread.h>

typedef struct usb_handle {
	struct usb_handle *prev;
	struct usb_handle *next;

	char fname[64];
	int desc;
	unsigned char ep_in;
	unsigned char ep_out;

	unsigned zero_mask;
	unsigned writeable;

	struct usbdevfs_urb urb_in;
	struct usbdevfs_urb urb_out;

	int urb_in_busy;
	int urb_out_busy;
	int dead;

	pthread_cond_t notify;
	pthread_mutex_t lock;

	//for garbage collecting disconnected devices
	int mark;

	//ID for thread currently in REAPURB
	pthread_t reaper_thread;
}usb_handle;

int usb_read(usb_handle *h, void *data, int len);
void usb_init();

#define D(...)       \
	do {			\
		fprintf(stderr, __VA_ARGS__); \
	} while(0)

