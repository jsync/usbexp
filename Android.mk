# Copyright 2005 The Android Open Source Project
#
# Android.mk 

LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= \
	device/usb_linux_client.c \
	device/usbexp_main.c

LOCAL_SHARED_LIBRARIES := libcutils libc
LOCAL_CFLAGS := -O0 -g
LOCAL_MODULE_TAGS := tests
LOCAL_MODULE:= usbexp
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := \
	host/usb_linux.c \
	host/usbexp_main.c 

LOCAL_CFLAGS := -O0 -g
LOCAL_MODULE:= usbexp
LOCAL_LDLIBS += -lpthread
LOCAL_MODULE_TAGS := tests
include $(BUILD_HOST_EXECUTABLE)
