/*
 *  usbexp_main.c
 *
 * Derived from adb.c from adb
 *
 * Created on: Jun 19, 2013
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

#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>

#include <fcntl.h>
#include <pthread.h>
#include "usbexp.h"
usb_handle vUsb;

extern void usb_init();
int usb_ready = 0;

#define XRES		800
#define	YRES		480
#define BPP		2
#define FPS		24
#define MAX_USB_BUF	4096

char *frame_buff;

extern int usb_read(usb_handle *h, void *data, int len);
int is_usbexp_interface(int vid, int pid, int usb_class, int usb_subclass, int usb_protocol) 
{
	if (usb_class == 0xff && usb_subclass == 0x43 && usb_protocol == 0x1)
		return 1;
	else
		return 0;
}
void register_usb_transport(usb_handle *usb)
{
	memcpy((void *)&vUsb, (const void *)usb, sizeof(usb_handle));
	usb_ready = 1;
	D("%s\n", __func__);
}
void unregister_usb_transport(usb_handle *usb)
{
	memset((void *)&vUsb, 0, sizeof(usb_handle));
	usb_ready = 0;
	D("%s\n", __func__);
}
//void sig_handler()
//{
//	trap = 1;
//}
static long long start_time;
static long long NOW()
{
	struct timeval tv;
	gettimeofday(&tv, 0);
	return ((long long) tv.tv_usec) + 
		1000000L *((long long) tv.tv_sec);
}
static void BEGIN()
{
	start_time = NOW();
	fprintf(stderr, "%lld\n", start_time);
}
static void END()
{
	long long t = NOW() - start_time;
//	if (t == 0)
//		t = 1000000;
	fprintf(stderr, "%lld\n", t);
	fprintf(stderr, "%f KB/s (%fK Bytes in %f.%03llds)\n",
	              ((((long long)(XRES * YRES * BPP * FPS)) * 1000000L * 1.0) / t) /(1024L *1024L * 1.0), 
		      ((XRES * YRES * BPP * FPS) / (1024L*1.0)), (t / (1000000LL*1.0)),
	                    (t % 1000000LL) / 1000LL);	      
}
int main ()
{
//	struct sigaction actions;
	int ret;
	long int fb_size_1sec;
	char *fbptr;

//	memset(&actions, 0, sizeof(actions));
//	sigemptyset(&actions.sa_mask);
//	actions.sa_flags = 0;
//	actions.sa_handler = sig_handler;
//	sigaction(SIGINT, &actions, NULL);

	fb_size_1sec = XRES * YRES * BPP * FPS;
	frame_buff = (char *)malloc(fb_size_1sec);
	fbptr = frame_buff;

	usb_init();

	D("%s usb_init finished \n", __func__);
	D("Remaining : %ld\n",fb_size_1sec);
	BEGIN();
	while(fb_size_1sec)
	{
//			D("Remaining : %ld\n",fb_size_1sec);
		if(usb_ready) 
		{
//			D("Remaining : %ld\n",fb_size_1sec);
			ret = usb_read(&vUsb, fbptr, 4096);
			if (ret < 0)
				break;
			else {
				fbptr += 4096;
				fb_size_1sec -= 4096;
			}
		}
	}
	D("Remaining : %ld\n",fb_size_1sec);
	END();
	
	return 0;
}
