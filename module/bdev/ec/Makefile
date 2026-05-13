#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) 2026 Longhorn Authors.
#  All rights reserved.

SPDK_ROOT_DIR := $(abspath $(CURDIR)/../../..)
include $(SPDK_ROOT_DIR)/mk/spdk.common.mk

SO_VER := 1
SO_MINOR := 0

C_SRCS = bdev_ec.c bdev_ec_io.c bdev_ec_rebuild.c bdev_ec_resize.c bdev_ec_rmw.c bdev_ec_wib.c

LIBNAME = bdev_ec

# bdev_ec requires ISA-L for Reed-Solomon coding. The isa-l include path and
# -lisal are provided globally by mk/spdk.common.mk when CONFIG_ISAL=y (the
# default), so they are not repeated here.

SPDK_MAP_FILE = $(SPDK_ROOT_DIR)/mk/spdk_blank.map

include $(SPDK_ROOT_DIR)/mk/spdk.lib.mk
