
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <pthread.h>

typedef struct usb_handle
{
	int fd;
	pthread_cond_t notify;
	pthread_mutex_t lock;
}usb_handle;



int usb_write(usb_handle *h, const void *data, int len);
void register_usb_transport(usb_handle *h);
void usb_init();

#define D(...)       \
	do {			\
		fprintf(stderr, __VA_ARGS__); \
	} while(0)
