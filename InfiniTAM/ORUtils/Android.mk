LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := ORUtils
LOCAL_SRC_FILES := KeyValueConfig.cpp SE3Pose.cpp
LOCAL_CFLAGS := -Werror
# -DCOMPILE_WITHOUT_CUDA
LOCAL_C_INCLUDES += $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/include

include $(BUILD_STATIC_LIBRARY)

