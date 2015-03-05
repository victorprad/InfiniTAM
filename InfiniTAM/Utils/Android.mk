LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := Utils
LOCAL_SRC_FILES := FileUtils.cpp
LOCAL_CFLAGS := -Werror
# -DCOMPILE_WITHOUT_CUDA
LOCAL_C_INCLUDES += $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/include
#LOCAL_LDLIBS := -landroid -lGLESv2

include $(BUILD_STATIC_LIBRARY)

