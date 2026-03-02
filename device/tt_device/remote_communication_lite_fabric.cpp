/*
 * SPDX-FileCopyrightText: (c) 2025 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "umd/device/tt_device/remote_communication_lite_fabric.hpp"

namespace tt::umd {

RemoteCommunicationLiteFabric::RemoteCommunicationLiteFabric(TTDevice* local_tt_device, SysmemManager* sysmem_manager) :
    RemoteCommunication(local_tt_device, sysmem_manager) {
    host_interface = lite_fabric::LiteFabricMemoryMap::make_host_interface(local_tt_device);
}

void RemoteCommunicationLiteFabric::read_non_mmio(
    tt_xy_pair target_core,
    void* dest,
    uint64_t core_src,
    uint32_t size_in_bytes,
    const std::chrono::milliseconds timeout_ms) {
    tt_xy_pair eth_core = get_remote_transfer_ethernet_core();
    CoreCoord core_coord = CoreCoord(eth_core.x, eth_core.y, CoreType::ETH, CoordSystem::NOC0);
    host_interface.read(dest, size_in_bytes, core_coord, target_core, core_src);
}

void RemoteCommunicationLiteFabric::write_to_non_mmio(
    tt_xy_pair target_core,
    const void* src,
    uint64_t core_dest,
    uint32_t size_in_bytes,
    bool broadcast,
    std::vector<int> broadcast_header,
    const std::chrono::milliseconds timeout_ms) {
    // hacking this to be void* from const void*
    // TODO: support const void* properly.
    tt_xy_pair eth_core = get_remote_transfer_ethernet_core();
    CoreCoord core_coord = CoreCoord(eth_core.x, eth_core.y, CoreType::ETH, CoordSystem::NOC0);
    host_interface.write(const_cast<void*>(src), size_in_bytes, core_coord, target_core, core_dest);
}

void RemoteCommunicationLiteFabric::wait_for_non_mmio_flush(const std::chrono::milliseconds timeout_ms) {
    // Block until the lite fabric sender channel on the MMIO ERISC1 has consumed all pending
    // write descriptors that the host has submitted.  This ensures that all prior
    // write_to_non_mmio() calls have been committed to the remote chip's L1/DRAM before
    // this function returns, providing the barrier semantics needed by l1_membar /
    // dram_membar on remote Blackhole chips.
    //
    // After fabric routers are launched, all lite fabric channels for a remote chip
    // may be taken by the fabric.  In that case remote_transfer_eth_cores_ is cleared
    // and there are no pending UMD-level writes to flush, so return immediately.
    if (remote_transfer_eth_cores_.empty()) {
        return;
    }
    tt_xy_pair eth_core = get_remote_transfer_ethernet_core();
    CoreCoord core_coord = CoreCoord(eth_core.x, eth_core.y, CoreType::ETH, CoordSystem::NOC0);
    host_interface.wait_for_all_writes_consumed(core_coord);
}

void RemoteCommunicationLiteFabric::set_remote_transfer_ethernet_cores(
    const std::unordered_set<tt_xy_pair>& cores) {
    RemoteCommunication::set_remote_transfer_ethernet_cores(cores);
    write_reg_sender_indices_.clear();

    if (cores.empty()) {
        host_interface.init();
        return;
    }

    // Sync host-side counters with the device's actual d2h state on the new
    // core.  The firmware's d2h counter may be non-zero if the core was used
    // by a prior binding (e.g. UMD topology discovery) or if the device L1
    // retained stale values.  Setting h2d = d2h means "no pending writes",
    // which prevents wait_for_all_writes_consumed from deadlocking on a
    // stale mismatch.
    tt_xy_pair eth_core = *cores.begin();
    CoreCoord cc(eth_core.x, eth_core.y, CoreType::ETH, CoordSystem::NOC0);
    uint32_t dev_iface_word = 0;
    host_interface.tt_device->read_from_device(
        &dev_iface_word, cc, host_interface.host_interface_on_device_addr, sizeof(dev_iface_word));
    uint8_t dev_d2h_sender = dev_iface_word & 0xFF;
    uint8_t dev_d2h_receiver = (dev_iface_word >> 8) & 0xFF;

    host_interface.h2d.sender_host_write_index = dev_d2h_sender;
    host_interface.h2d.receiver_host_read_index = dev_d2h_receiver;
    host_interface.d2h.fabric_sender_channel_index = dev_d2h_sender;
    host_interface.d2h.fabric_receiver_channel_index = dev_d2h_receiver;
    host_interface.read_event_counter = 0;
}

void RemoteCommunicationLiteFabric::write_remote_reg(uint32_t reg_addr, uint32_t reg_value) {
    tt_xy_pair eth_core = get_remote_transfer_ethernet_core();
    CoreCoord core_coord = CoreCoord(eth_core.x, eth_core.y, CoreType::ETH, CoordSystem::NOC0);
    host_interface.write_reg(reg_addr, reg_value, core_coord);
}

void RemoteCommunicationLiteFabric::write_remote_reg(uint32_t reg_addr, uint32_t reg_value, tt_xy_pair sender_core) {
    // If sender_core is the same as the default remote transfer core, use the
    // regular h2d counter.  The save/restore path below flushes a separate h2d
    // value to the device's L1; when that happens on the same physical channel
    // as regular writes, it desynchronizes the device-side h2d from the host's
    // expectation, causing l1_barrier (wait_for_all_writes_consumed) to timeout.
    if (!remote_transfer_eth_cores_.empty() && sender_core == get_remote_transfer_ethernet_core()) {
        CoreCoord core_coord = CoreCoord(sender_core.x, sender_core.y, CoreType::ETH, CoordSystem::NOC0);
        host_interface.write_reg(reg_addr, reg_value, core_coord);
        return;
    }

    // Save the main host_interface h2d state.  WRITE_REG through a non-default
    // sender core must not pollute the h2d counter used by regular L1 operations
    // through the default sender.
    auto saved_h2d = host_interface.h2d;

    uint64_t key = (uint64_t(sender_core.x) << 16) | sender_core.y;

    // On first use of a sender core, read the device's current d2h to sync
    // with whatever state was left by prior L1 operations through this core
    // (or by a previous process).  Setting h2d = d2h means "all prior writes
    // consumed", which avoids wait_for_empty_write_slot seeing a phantom-full
    // buffer.
    auto it = write_reg_sender_indices_.find(key);
    if (it == write_reg_sender_indices_.end()) {
        CoreCoord cc = CoreCoord(sender_core.x, sender_core.y, CoreType::ETH, CoordSystem::NOC0);
        uint32_t dev_iface_word = 0;
        host_interface.tt_device->read_from_device(
            &dev_iface_word,
            cc,
            host_interface.host_interface_on_device_addr,
            sizeof(dev_iface_word));
        // d2h.fabric_sender_channel_index is the first byte of the host_interface on device.
        uint8_t dev_d2h_sender = dev_iface_word & 0xFF;
        it = write_reg_sender_indices_.emplace(key, dev_d2h_sender).first;
    }

    host_interface.h2d.sender_host_write_index = it->second;

    CoreCoord core_coord = CoreCoord(sender_core.x, sender_core.y, CoreType::ETH, CoordSystem::NOC0);
    host_interface.write_reg(reg_addr, reg_value, core_coord);

    // Save the updated write index for this sender core.
    it->second = host_interface.h2d.sender_host_write_index;

    // Restore the main h2d state so subsequent L1 operations see the correct counter.
    host_interface.h2d = saved_h2d;
}

}  // namespace tt::umd
