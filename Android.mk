# Copyright (C) 2012 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_VENDOR_MODULE := true
LOCAL_COPY_HEADERS_TO := cashsvr
LOCAL_COPY_HEADERS := ./cash_ext.h
include $(BUILD_COPY_HEADERS)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := cashsvr.c cash_input_common.c cashsvr_input_tof.c cashsvr_input_rgbc.c expatparser.c
LOCAL_C_INCLUDES := external/expat/lib
LOCAL_SHARED_LIBRARIES := liblog libcutils libexpat libpolyreg
LOCAL_MODULE := cashsvr
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := sony
LOCAL_INIT_RC_64   := vendor/etc/init/cashsvr.rc
LOCAL_PROPRIETARY_MODULE := true
include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_HEADER_LIBRARIES := libhardware_headers
LOCAL_SRC_FILES := cash_ctl.c
LOCAL_SHARED_LIBRARIES := \
    liblog \
    libcutils \

LOCAL_MODULE := libcashctl
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := sony
LOCAL_PROPRIETARY_MODULE := true

include $(BUILD_SHARED_LIBRARY)
