LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libcudart_static
LOCAL_SRC_FILES  := $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/lib/libcudart_static.a 
LOCAL_EXPORT_C_INCLUDES := $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/include
include $(PREBUILT_STATIC_LIBRARY)

OPENNI2_ROOT:=/local/olaf/compile/OpenNI2/
ifneq ($(OPENNI2_ROOT),)

MY_OPENNI2_LIBDIR := $(OPENNI2_ROOT)/Packaging/OpenNI-android-2.2/
MY_OPENNI2_MODULE := OpenNI2 OniFile PSLink PS1080 usb

include $(CLEAR_VARS)
LOCAL_MODULE := OpenNI2
LOCAL_SRC_FILES := $(MY_OPENNI2_LIBDIR)libOpenNI2.so
LOCAL_EXPORT_C_INCLUDES := $(OPENNI2_ROOT)/Include
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := OniFile
LOCAL_SRC_FILES := $(MY_OPENNI2_LIBDIR)libOniFile.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := PS1080
LOCAL_SRC_FILES := $(MY_OPENNI2_LIBDIR)libPS1080.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := PSLink
LOCAL_SRC_FILES := $(MY_OPENNI2_LIBDIR)libPSLink.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := usb
LOCAL_SRC_FILES := $(MY_OPENNI2_LIBDIR)libusb.so
include $(PREBUILT_SHARED_LIBRARY)

endif

FFMPEG_ROOT:=/local/olaf/compile/ffmpeg-android/
ifneq ($(FFMPEG_ROOT),)
MY_FFMPEG_BASEDIR := $(FFMPEG_ROOT)/build/armeabi-v7a-neon/
MY_FFMPEG_LIBDIR := $(MY_FFMPEG_BASEDIR)lib/
MY_FFMPEG_MODULE := avformat avfilter avcodec swresample swscale avutil

include $(CLEAR_VARS)
LOCAL_MODULE := avcodec
LOCAL_SRC_FILES := $(MY_FFMPEG_LIBDIR)libavcodec.a
LOCAL_EXPORT_C_INCLUDES := $(MY_FFMPEG_BASEDIR)/include
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := avformat
LOCAL_SRC_FILES := $(MY_FFMPEG_LIBDIR)libavformat.a
LOCAL_EXPORT_C_INCLUDES := $(MY_FFMPEG_BASEDIR)/include
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := avfilter
LOCAL_SRC_FILES := $(MY_FFMPEG_LIBDIR)libavfilter.a
LOCAL_EXPORT_C_INCLUDES := $(MY_FFMPEG_BASEDIR)/include
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := swresample
LOCAL_SRC_FILES := $(MY_FFMPEG_LIBDIR)libswresample.a
LOCAL_EXPORT_C_INCLUDES := $(MY_FFMPEG_BASEDIR)/include
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := swscale
LOCAL_SRC_FILES := $(MY_FFMPEG_LIBDIR)libswscale.a
LOCAL_EXPORT_C_INCLUDES := $(MY_FFMPEG_BASEDIR)/include
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := avutil
LOCAL_SRC_FILES := $(MY_FFMPEG_LIBDIR)libavutil.a
LOCAL_EXPORT_C_INCLUDES := $(MY_FFMPEG_BASEDIR)/include
include $(PREBUILT_STATIC_LIBRARY)

endif

include $(CLEAR_VARS)

LOCAL_MODULE    := InfiniTAM
LOCAL_SRC_FILES := InfiniTAMApp.cpp jniExport.cpp
LOCAL_CFLAGS := -Werror #-DCOMPILE_WITHOUT_CUDA
LOCAL_LDLIBS := -landroid -lGLESv1_CM -llog -lz
LOCAL_STATIC_LIBRARIES := InputSource ITMLib RelocLib ORUtils cudart_static

include $(BUILD_SHARED_LIBRARY)

include ../../InputSource/Android.mk
include ../../ORUtils/Android.mk
include ../../RelocLib/Android.mk
include ../../ITMLib/Android.mk

