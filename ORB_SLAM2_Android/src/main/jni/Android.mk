LOCAL_PATH:= $(call my-dir)

#############DLib Module##################
include $(CLEAR_VARS)
OPENCV_LIB_TYPE:=STATIC
ifeq ("$(wildcard $(OPENCV_MK_PATH))","")  
#try to load OpenCV.mk from default install location  
include $(LOCAL_PATH)/../OpenCV-2.4.13-android-sdk/sdk/native/jni/OpenCV.mk
else  
include $(OPENCV_MK_PATH)  
endif
LOCAL_MODULE:=DLib
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/DBoW2/DLib/include/DUtils
FILE_LIST:=$(wildcard $(LOCAL_PATH)/Thirdparty/DBoW2/DLib/src/DUtils/*.cpp)
LOCAL_SRC_FILES+=$(FILE_LIST:$(LOCAL_PATH)/%=%)
LOCAL_LDLIBS    := -llog
LOCAL_LDLIBS    +=-lz -llog -landroid -lEGL -lGLESv2
LOCAL_CPPFLAGS := -std=c++11 -pthread -frtti -fexceptions
LOCAL_EXPORT_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/DBoW2/DLib/include
LOCAL_EXPORT_C_INCLUDES+=$(LOCAL_PATH)/../OpenCV-2.4.13-android-sdk/sdk/native/jni/include
include $(BUILD_SHARED_LIBRARY)
###############################################################

##############Boost Module############################################
include $(CLEAR_VARS)
LOCAL_MODULE := boost_thread
LOCAL_SRC_FILES := $(LOCAL_PATH)/Thirdparty/boost_1_64_0/android/lib/libboost_thread.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := boost_system
LOCAL_SRC_FILES := $(LOCAL_PATH)/Thirdparty/boost_1_64_0/android/lib/libboost_system.a
include $(PREBUILT_STATIC_LIBRARY)

##############DBoW2 Module#########################################
include $(CLEAR_VARS)
OPENCV_LIB_TYPE:=STATIC
ifeq ("$(wildcard $(OPENCV_MK_PATH))","")  
#try to load OpenCV.mk from default install location  
include $(LOCAL_PATH)/../OpenCV-2.4.13-android-sdk/sdk/native/jni/OpenCV.mk
else  
include $(OPENCV_MK_PATH)  
endif
LOCAL_MODULE:=DBoW2
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/DBoW2/include/DBoW2
FILE_LIST:=$(wildcard $(LOCAL_PATH)/Thirdparty/DBoW2/src/*.cpp)
LOCAL_SRC_FILES+=$(FILE_LIST:$(LOCAL_PATH)/%=%)
LOCAL_SHARED_LIBRARIES+=DLib
LOCAL_EXPORT_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/DBoW2/include
LOCAL_CPPFLAGS := -std=c++11 -pthread -frtti -fexceptions
include $(BUILD_SHARED_LIBRARY)
###############################################################


###################G2O Module##################################
include $(CLEAR_VARS)
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/g2o/g2o/core
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/g2o/g2o/stuff
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/g2o/g2o/solvers
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/g2o/g2o/types
FILE_LIST:=$(wildcard $(LOCAL_PATH)/Thirdparty/g2o/g2o/core/*.cpp)
LOCAL_SRC_FILES+=$(FILE_LIST:$(LOCAL_PATH)/%=%)

FILE_LIST:=$(wildcard $(LOCAL_PATH)/Thirdparty/g2o/g2o/solvers/*.cpp)
LOCAL_SRC_FILES+=$(FILE_LIST:$(LOCAL_PATH)/%=%)

FILE_LIST:=$(wildcard $(LOCAL_PATH)/Thirdparty/g2o/g2o/stuff/*.cpp)
LOCAL_SRC_FILES+=$(FILE_LIST:$(LOCAL_PATH)/%=%)

FILE_LIST:=$(wildcard $(LOCAL_PATH)/Thirdparty/g2o/g2o/types/*.cpp)
LOCAL_SRC_FILES+=$(FILE_LIST:$(LOCAL_PATH)/%=%)
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/eigen3
LOCAL_MODULE:=g2o
LOCAL_EXPORT_LDLIBS := $(LOCAL_LDLIBS)
LOCAL_EXPORT_C_INCLUDES+=LOCAL_C_INCLUDES
LOCAL_CPPFLAGS := -std=c++11 -pthread -frtti -fexceptions
include $(BUILD_SHARED_LIBRARY)
############################################################

##############ORB_SLAM2 Module##################################
include $(CLEAR_VARS)
OPENCV_LIB_TYPE:=STATIC
ifeq ("$(wildcard $(OPENCV_MK_PATH))","")  
#try to load OpenCV.mk from default install location  
include $(LOCAL_PATH)/../OpenCV-2.4.13-android-sdk/sdk/native/jni/OpenCV.mk
else  
include $(OPENCV_MK_PATH)  
endif 
LOCAL_MODULE := ORB_SLAM2
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/eigen3
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/ORB_SLAM2/include
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/Sophus
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/sse2neon
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/boost_1_64_0
FILE_LIST:=$(wildcard $(LOCAL_PATH)/ORB_SLAM2/src/*.cc)
LOCAL_SRC_FILES+=$(FILE_LIST:$(LOCAL_PATH)/%=%)
LOCAL_SHARED_LIBRARIES+=DBoW2
LOCAL_SHARED_LIBRARIES+=DLib
LOCAL_SHARED_LIBRARIES+=g2o
LOCAL_STATIC_LIBRARIES+= boost_thread \
boost_system
LOCAL_LDLIBS += -llog -landroid -lEGL -lGLESv2
LOCAL_EXPORT_C_INCLUDES+=$(LOCAL_PATH)/ORB_SLAM2/include
LOCAL_CPPFLAGS := -std=c++11 -pthread -frtti -fexceptions -mfpu=neon
include $(BUILD_SHARED_LIBRARY)
############################################################

##############ORB_SLAM2 Module###############################
include $(CLEAR_VARS)
OPENCV_LIB_TYPE:=STATIC
ifeq ("$(wildcard $(OPENCV_MK_PATH))","")  
#try to load OpenCV.mk from default install location  
include $(LOCAL_PATH)/../OpenCV-2.4.13-android-sdk/sdk/native/jni/OpenCV.mk
else  
include $(OPENCV_MK_PATH)  
endif 
LOCAL_MODULE := ORB_SLAM2_EXCUTOR
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/eigen3
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/Sophus
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/sse2neon
LOCAL_C_INCLUDES+=$(LOCAL_PATH)/Thirdparty/boost_1_64_0
LOCAL_C_INCLUDES+=orb_slam2_android_nativefunc_OrbNdkHelper.h
LOCAL_SRC_FILES+=orb_slam2_android_nativefunc_OrbNdkHelper.cpp
LOCAL_SHARED_LIBRARIES+=ORB_SLAM2
LOCAL_SHARED_LIBRARIES+=g2o
LOCAL_LDLIBS += -llog -landroid -lEGL -lGLESv2
LOCAL_CPPFLAGS := -std=c++11 -pthread -frtti -fexceptions -mfpu=neon
include $(BUILD_SHARED_LIBRARY)
############################################################
