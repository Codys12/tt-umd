// SPDX-FileCopyrightText: (c) 2025 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0
#include "umd/device/tt_device/remote_blackhole_tt_device.hpp"

#include <cstring>

#include "umd/device/arc/arc_messenger.hpp"
#include "umd/device/arc/arc_telemetry_reader.hpp"
#include "umd/device/arch/blackhole_implementation.hpp"
#include "umd/device/firmware/firmware_info_provider.hpp"

namespace tt::umd {

RemoteBlackholeTTDevice::RemoteBlackholeTTDevice(std::unique_ptr<RemoteCommunication> remote_communication) :
    BlackholeTTDevice(remote_communication->get_local_device()->get_pci_device()),
    remote_communication_(std::move(remote_communication)) {
    is_remote_tt_device = true;
}

void RemoteBlackholeTTDevice::read_from_device(void* mem_ptr, tt_xy_pair core, uint64_t addr, uint32_t size) {
    if (!lite_fabric_running_) {
        // Lite fabric not yet running; reads are not possible.
        std::memset(mem_ptr, 0, size);
        return;
    }
    remote_communication_->read_non_mmio(core, mem_ptr, addr, size);
}

void RemoteBlackholeTTDevice::write_to_device(const void* mem_ptr, tt_xy_pair core, uint64_t addr, uint32_t size) {
    if (!lite_fabric_running_) {
        // Lite fabric not yet running; writes are silently dropped.
        return;
    }
    remote_communication_->write_to_non_mmio(core, mem_ptr, addr, size);
}

void RemoteBlackholeTTDevice::read_from_arc_apb(void* mem_ptr, uint64_t arc_addr_offset, size_t size) {
    read_from_device(
        mem_ptr, get_arc_core(), architecture_impl_->get_arc_apb_noc_base_address() + arc_addr_offset, size);
}

void RemoteBlackholeTTDevice::write_to_arc_apb(const void* mem_ptr, uint64_t arc_addr_offset, size_t size) {
    write_to_device(
        mem_ptr, get_arc_core(), architecture_impl_->get_arc_apb_noc_base_address() + arc_addr_offset, size);
}

void RemoteBlackholeTTDevice::wait_for_non_mmio_flush() { remote_communication_->wait_for_non_mmio_flush(); }

RemoteCommunication* RemoteBlackholeTTDevice::get_remote_communication() { return remote_communication_.get(); }

// ARC tile access over AXI is not supported for remote devices.
bool RemoteBlackholeTTDevice::is_arc_available_over_axi() { return false; }

ChipInfo RemoteBlackholeTTDevice::get_chip_info() {
    if (lite_fabric_running_ && telemetry) {
        // Lite fabric is running; we have real telemetry for this remote chip.
        return BlackholeTTDevice::get_chip_info();
    }
    // Pre-lite-fabric: telemetry is not available for the remote chip.
    // Fall back to the local gateway chip's chip info which has valid
    // harvesting masks, board_type, and noc_translation settings.
    return remote_communication_->get_local_device()->get_chip_info();
}

void RemoteBlackholeTTDevice::init_tt_device(const std::chrono::milliseconds timeout_ms) {
    // Lite fabric is not yet running when the remote chip is created during topology discovery.
    // We cannot read from the remote ARC.  Instead, borrow the local gateway chip's
    // FirmwareInfoProvider so that get_chip_info() and wait_dram_channel_training() return
    // sensible values (same board_type P150, same noc_translation) without crashing.
    TTDevice* local_device = remote_communication_->get_local_device();
    firmware_info_provider = FirmwareInfoProvider::create_firmware_info_provider(local_device);
}

void RemoteBlackholeTTDevice::upgrade_firmware_info_provider() {
    // Lite fabric is now running.  Initialize the ARC telemetry reader, ARC messenger, and
    // FirmwareInfoProvider that read from this remote chip's ARC via lite fabric.  This
    // replaces the proxy provider installed in init_tt_device() with real ones that return
    // correct harvesting masks, DRAM training status, board ID, and other chip-specific
    // information for this remote chip.
    //
    // Note: ArcTelemetryReader construction calls get_noc_translation_enabled(), which reads
    // the local gateway's PCIe BAR (since RemoteBlackholeTTDevice shares pci_device_ with the
    // local chip).  This returns the correct value because all production P150 BH chips have
    // NOC translation enabled.  The subsequent read_from_device() calls inside the telemetry
    // reader use RemoteBlackholeTTDevice::read_from_device(), which routes through lite fabric
    // to the remote ARC.
    //
    // IMPORTANT: Set lite_fabric_running_ BEFORE creating the telemetry reader.  The
    // ArcTelemetryReader constructor reads from the remote ARC, and read_from_device()
    // gates on this flag.  Previously, `telemetry != nullptr` was used as the gate, but
    // telemetry is null during its own construction — a circular dependency that caused
    // all reads to return zeros and produced a broken telemetry reader.
    lite_fabric_running_ = true;
    telemetry = ArcTelemetryReader::create_arc_telemetry_reader(this);
    arc_messenger_ = ArcMessenger::create_arc_messenger(this);
    firmware_info_provider = FirmwareInfoProvider::create_firmware_info_provider(this);
}

std::chrono::milliseconds RemoteBlackholeTTDevice::wait_eth_core_training(
    const tt_xy_pair eth_core, const std::chrono::milliseconds timeout_ms) {
    // ETH link was already observed as trained during topology discovery.
    // We cannot read the remote ETH core's training status without lite fabric running.
    return std::chrono::milliseconds(0);
}

void RemoteBlackholeTTDevice::noc_multicast_write(
    void* dst, size_t size, tt_xy_pair core_start, tt_xy_pair core_end, uint64_t addr) {
    // TODO: implement multicast over remote communication.
    // For now, we fallback to unicast for all cores.
    for (uint32_t x = core_start.x; x <= core_end.x; ++x) {
        for (uint32_t y = core_start.y; y <= core_end.y; ++y) {
            write_to_device(dst, tt_xy_pair(x, y), addr, size);
        }
    }
}

}  // namespace tt::umd
