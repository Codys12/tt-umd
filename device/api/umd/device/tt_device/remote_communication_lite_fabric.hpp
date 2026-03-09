/*
 * SPDX-FileCopyrightText: (c) 2025 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <set>
#include <unordered_set>

#include "umd/device/lite_fabric/lite_fabric.hpp"
#include "umd/device/tt_device/remote_communication.hpp"

namespace tt::umd {

class SysmemManager;

class RemoteCommunicationLiteFabric : public RemoteCommunication {
public:
    RemoteCommunicationLiteFabric(TTDevice* local_tt_device, SysmemManager* sysmem_manager = nullptr);

    void read_non_mmio(
        tt_xy_pair target_core,
        void* dest,
        uint64_t core_src,
        uint32_t size_in_bytes,
        const std::chrono::milliseconds timeout_ms = timeout::NON_MMIO_RW_TIMEOUT) override;

    void write_to_non_mmio(
        tt_xy_pair target_core,
        const void* src,
        uint64_t core_dest,
        uint32_t size_in_bytes,
        bool broadcast = false,
        std::vector<int> broadcast_header = {},
        const std::chrono::milliseconds timeout_ms = timeout::NON_MMIO_RW_TIMEOUT) override;

    void wait_for_non_mmio_flush(const std::chrono::milliseconds timeout_ms = timeout::NON_MMIO_RW_TIMEOUT) override;

    void write_remote_reg(uint32_t reg_addr, uint32_t reg_value) override;
    void write_remote_reg(uint32_t reg_addr, uint32_t reg_value, tt_xy_pair sender_core) override;

    void set_num_hops(uint32_t num_hops) override;

    // Override to reset host_interface counters when bindings change.
    // After a binding change, the new core's d2h may differ from the host's
    // accumulated h2d, causing wait_for_all_writes_consumed to deadlock.
    void set_remote_transfer_ethernet_cores(const std::unordered_set<tt_xy_pair>& cores) override;
    void resync_remote_transfer_ethernet_cores() override;

private:
    void sync_host_interface_state(bool sync_sender_state);

    lite_fabric::HostToLiteFabricInterface<lite_fabric::SENDER_NUM_BUFFERS_ARRAY[0], lite_fabric::CHANNEL_BUFFER_SIZE>
        host_interface;

    // Per-sender-core write index tracking for write_remote_reg with explicit
    // sender cores.  Avoids polluting the main host_interface's h2d counter
    // which is used by regular L1 read/write operations through the default core.
    std::unordered_map<uint64_t, uint8_t> write_reg_sender_indices_;
};

}  // namespace tt::umd
