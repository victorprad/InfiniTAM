LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libcudart_static
LOCAL_LIB_PATH   += $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/lib/
LOCAL_SRC_FILES  := $(LOCAL_LIB_PATH)/libcudart_static.a 
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)

LOCAL_MODULE    := InfiniTAM
LOCAL_SRC_FILES := InfiniTAMApp.cpp jniExport.cpp
LOCAL_CFLAGS := -Werror #-DCOMPILE_WITHOUT_CUDA
LOCAL_LDLIBS := -landroid -lGLESv1_CM -llog
LOCAL_C_INCLUDES += $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/include
LOCAL_STATIC_LIBRARIES := Engine Utils ITMLib cudart_static

include $(BUILD_SHARED_LIBRARY)

include ../Engine/Android.mk
include ../Utils/Android.mk
include ../ITMLib/Android.mk

