LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := InfiniTAM
LOCAL_SRC_FILES := InfiniTAMApp.cpp jniExport.cpp
LOCAL_CFLAGS := -Werror
LOCAL_LDLIBS := -landroid -lGLESv2

include $(BUILD_SHARED_LIBRARY)
