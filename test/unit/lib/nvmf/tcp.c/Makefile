#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) 2015 Intel Corporation.
#  All rights reserved.
#

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../../../../..)

SPDK_LIB_LIST = json
TEST_FILE = tcp_ut.c

SPDK_MOCK_SYMBOLS = spdk_interrupt_mode_is_enabled

include $(SPDK_ROOT_DIR)/mk/spdk.unittest.mk
