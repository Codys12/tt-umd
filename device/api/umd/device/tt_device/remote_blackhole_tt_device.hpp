/*
 * SPDX-FileCopyrightText: (c) 2025 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <map>

#include "umd/device/chip/local_chip.hpp"
#include "umd/device/tt_device/blackhole_tt_device.hpp"

namespace tt::umd {

class RemoteBlackholeTTDevice : public BlackholeTTDevice {
public:
    void read_from_device(void* mem_ptr, tt_xy_pair core, uint64_t addr, uint32_t size) override;

    void write_to_device(const void* mem_ptr, tt_xy_pair core, uint64_t addr, uint32_t size) override;

    void read_from_arc_apb(void* mem_ptr, uint64_t arc_addr_offset, size_t size) override;

    void noc_multicast_write(
        void* dst, size_t size, tt_xy_pair core_start, tt_xy_pair core_end, uint64_t addr) override;

    void write_to_arc_apb(const void* mem_ptr, uint64_t arc_addr_offset, size_t size) override;

    void wait_for_non_mmio_flush() override;

    RemoteCommunication* get_remote_communication();

    // Override get_chip_info to avoid reading from telemetry before lite fabric is running.
    // Before upgrade_firmware_info_provider() is called, telemetry is null, so we fall back
    // to the local gateway chip's chip info (which has valid harvesting masks).
    ChipInfo get_chip_info() override;

    // Override init_tt_device to avoid reading from the remote chip's ARC via lite fabric,
    // which is not running during topology discovery. Uses the local gateway chip's firmware
    // info as a proxy so that get_chip_info() and wait_dram_channel_training() do not crash.
    void init_tt_device(
        const std::chrono::milliseconds timeout_ms = std::chrono::milliseconds(30000)) override;

    // Override wait_eth_core_training to return immediately: we cannot query the remote ETH
    // cores until lite fabric is up, and they will have already been trained by the time we
    // reach here (the link was up during topology discovery).
    std::chrono::milliseconds wait_eth_core_training(
        const tt_xy_pair eth_core,
        const std::chrono::milliseconds timeout_ms = std::chrono::milliseconds(15000)) override;

    // Replace the proxy FirmwareInfoProvider (which was borrowed from the local gateway chip
    // in init_tt_device()) with a real one that reads telemetry from the remote ARC via lite
    // fabric.  Must be called after lite fabric is running and before any operation that needs
    // accurate chip info (harvesting masks, DRAM training status, etc.) from the remote chip.
    void upgrade_firmware_info_provider();

protected:
    bool is_arc_available_over_axi() override;

private:
    RemoteBlackholeTTDevice(std::unique_ptr<RemoteCommunication> remote_communication);

    friend std::unique_ptr<TTDevice> TTDevice::create(std::unique_ptr<RemoteCommunication> remote_communication);

    std::unique_ptr<RemoteCommunication> remote_communication_;

    // Separate flag for whether lite fabric is running.  read_from_device /
    // write_to_device previously used `telemetry != nullptr` as a proxy for
    // this, but that creates a circular dependency: upgrade_firmware_info_provider()
    // needs to read from the remote ARC to *create* the telemetry reader, yet
    // reads are gated on telemetry being non-null.  This flag is set at the
    // start of upgrade_firmware_info_provider() so that the ArcTelemetryReader
    // constructor can read through lite fabric.
    bool lite_fabric_running_ = false;

    // Cache of per-core soft reset register values.  NOC reads to the Tensix
    // soft reset register (0xFFB121B0) hang when the target core is in reset
    // because its local bus does not respond.  Instead of reading through lite
    // fabric (which would cause ERISC1's noc_async_read_barrier to hang
    // forever), we track writes and return the cached value on reads.  Cores
    // not in the cache are assumed to be in POR state (all RISCs in reset).
    // Key: (core.x << 16) | core.y
    std::map<uint32_t, uint32_t> soft_reset_reg_cache_;
};

}  // namespace tt::umd
