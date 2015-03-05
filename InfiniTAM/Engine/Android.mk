LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := Engine
LOCAL_SRC_FILES := ImageSourceEngine.cpp IMUSourceEngine.cpp OpenNIEngine.cpp
LOCAL_CFLAGS := -Werror
# -DCOMPILE_WITHOUT_CUDA
LOCAL_C_INCLUDES += $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/include
LOCAL_C_INCLUDES += $(OPENNI2_ROOT)/Include
#LOCAL_LDLIBS := -landroid -lGLESv2

include $(BUILD_STATIC_LIBRARY)

