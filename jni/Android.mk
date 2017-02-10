ifneq ($(TARGET_SIMULATOR),true)

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_CFLAGS += -Wall


LOCAL_LDLIBS := -L$(LOCAL_PATH)/lib -llog -g

LOCAL_C_INCLUDES := bionic
LOCAL_C_INCLUDES += $(LOCAL_PATH)/include

LOCAL_SRC_FILES:= rtcomm_test.c

LOCAL_MODULE := rtcomm_test

include $(BUILD_EXECUTABLE)

endif  # TARGET_SIMULATOR != true
