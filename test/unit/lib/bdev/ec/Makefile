#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) 2026 Longhorn Authors.
#  All rights reserved.
#

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../../../../..)
include $(SPDK_ROOT_DIR)/mk/spdk.common.mk

DIRS-y = bdev_ec.c bdev_ec_wib.c bdev_ec_rmw.c

.PHONY: all clean $(DIRS-y)

all: $(DIRS-y)
clean: $(DIRS-y)

include $(SPDK_ROOT_DIR)/mk/spdk.subdirs.mk
