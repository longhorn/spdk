#!/usr/bin/env bash
#  SPDX-License-Identifier: BSD-3-Clause
#  Copyright (C) 2023 SUSE LLC.
#  All rights reserved.
#
testdir=$(readlink -f $(dirname $0))
rootdir=$(readlink -f $testdir/../..)
source $rootdir/test/common/autotest_common.sh
source $rootdir/test/lvol/common.sh

function is_rfc3339_formatted() {
	# Define the RFC3339 regex pattern
	rfc3339_pattern="^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z$"

	# Check if the input string matches the pattern
	if [[ $1 =~ $rfc3339_pattern ]]; then
		echo "The time string '$1' is in RFC3339 format."
		return 0 # Success
	else
		echo "The time string '$1' is not in RFC3339 format."
		return 1 # Failure
	fi
}

function test_set_xattr() {
	# Create lvs
	malloc_name=$(rpc_cmd bdev_malloc_create 20 $MALLOC_BS)
	lvs_uuid=$(rpc_cmd bdev_lvol_create_lvstore "$malloc_name" lvs_test)

	# Create lvol with 4 cluster
	lvol_size=$((LVS_DEFAULT_CLUSTER_SIZE_MB * 4))
	lvol_uuid=$(rpc_cmd bdev_lvol_create -u "$lvs_uuid" lvol_test "$lvol_size" -t)

	rpc_cmd bdev_lvol_set_xattr "$lvol_uuid" "foo" "bar"
	value=$(rpc_cmd bdev_lvol_get_xattr "$lvol_uuid" "foo")
	[ "\"bar\"" = "$value" ]

	# Snapshot is read-only, but setting xattr is permitted
	snapshot_uuid=$(rpc_cmd bdev_lvol_snapshot lvs_test/lvol_test lvol_snapshot)
	rpc_cmd bdev_lvol_set_xattr "$snapshot_uuid" "foo" "bar"
	rpc_cmd bdev_lvol_set_xattr "$snapshot_uuid" "foo" "bar2"

	# Create snapshot with xattr
	snapshotx_uuid=$(rpc_cmd bdev_lvol_snapshot --xattr snapshot_timestamp=2024-01-16T16:06:46Z lvs_test/lvol_test lvol_snapshotx)
	value=$(rpc_cmd bdev_lvol_get_xattr "$snapshotx_uuid" "snapshot_timestamp")
	[ "\"2024-01-16T16:06:46Z\"" = "$value" ]

	rpc_cmd bdev_lvol_delete_lvstore -u "$lvs_uuid"
	rpc_cmd bdev_malloc_delete "$malloc_name"
	check_leftover_devices
}

function test_creation_time_xattr() {
	# Create lvs
	malloc_name=$(rpc_cmd bdev_malloc_create 20 $MALLOC_BS)
	lvs_uuid=$(rpc_cmd bdev_lvol_create_lvstore "$malloc_name" lvs_test)

	# Create lvol with 4 cluster
	lvol_size=$((LVS_DEFAULT_CLUSTER_SIZE_MB * 4))
	lvol_uuid=$(rpc_cmd bdev_lvol_create -u "$lvs_uuid" lvol_test "$lvol_size" -t)

	value=$(rpc_cmd bdev_lvol_get_xattr "$lvol_uuid" "creation_time")
	value="${value//\"/}"
	is_rfc3339_formatted ${value}

	# Create snapshots of lvol bdev
	snapshot_uuid=$(rpc_cmd bdev_lvol_snapshot lvs_test/lvol_test lvol_snapshot)
	value=$(rpc_cmd bdev_lvol_get_xattr "$snapshot_uuid" "creation_time")
	value="${value//\"/}"
	is_rfc3339_formatted ${value}

	clone_uuid=$(rpc_cmd bdev_lvol_clone "$snapshot_uuid" lvol_clone)
	value=$(rpc_cmd bdev_lvol_get_xattr "$snapshot_uuid" "creation_time")
	value="${value//\"/}"
	is_rfc3339_formatted ${value}

	rpc_cmd bdev_lvol_delete_lvstore -u "$lvs_uuid"
	rpc_cmd bdev_malloc_delete "$malloc_name"
	check_leftover_devices
}

$SPDK_BIN_DIR/spdk_tgt &
spdk_pid=$!
trap 'killprocess "$spdk_pid"; exit 1' SIGINT SIGTERM EXIT
waitforlisten $spdk_pid
modprobe nbd

run_test "test_set_xattr" test_set_xattr
run_test "test_creation_time_xattr" test_creation_time_xattr

trap - SIGINT SIGTERM EXIT
killprocess $spdk_pid
