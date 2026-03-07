// SPDX-FileCopyrightText: © 2025 Tenstorrent AI ULC
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <fmt/ranges.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <tt-logger/tt-logger.hpp>

#include "umd/device/cluster.hpp"
#include "umd/device/driver_atomics.hpp"
#include "umd/device/lite_fabric/fabric_edm_types.hpp"
#include "umd/device/lite_fabric/lf_dev_mem_map.hpp"
#include "umd/device/lite_fabric/lite_fabric_constants.hpp"
#include "umd/device/lite_fabric/lite_fabric_header.hpp"
#include "umd/device/types/xy_pair.hpp"

namespace tt::umd {

namespace lite_fabric {

#define is_power_of_2(x) (((x) > 0) && (((x) & ((x)-1)) == 0))

template <size_t LIMIT = 0, typename T>
auto wrap_increment(T val) -> T {
    constexpr bool is_pow2 = LIMIT != 0 && is_power_of_2(LIMIT);
    if constexpr (LIMIT == 1) {
        return val;
    } else if constexpr (LIMIT == 2) {
        return 1 - val;
    } else if constexpr (is_pow2) {
        return (val + 1) & (static_cast<T>(LIMIT - 1));
    } else {
        return (val == static_cast<T>(LIMIT - 1)) ? static_cast<T>(0) : static_cast<T>(val + 1);
    }
}

/*
Initialization process for Lite Fabric

    1. Host writes the lite fabric kernel to an arbitrary active ethernet core on MMIO capable chips. This
    is designated as the Primary core with an initial state of ETH_INIT_LOCAL. This core will launch
    lite fabric kernels on other active ethernet cores on the same chip with an initial state of
ETH_INIT_LOCAL_HANDSHAKE.

    2. The primary core will stall for the ETH_INIT_LOCAL_HANDSHAKE cores to be ready

    3. Primary core transitions state to ETH_INIT_NEIGHBOUR. It will launch a primary lite fabric kernel on the eth
device.

    4. Subordinate core transitions state to ETH_INIT_NEIGHBOUR_HANDSHAKE

    5. The primary lite fabric kernel on the eth device will launch lite fabric kernels on other active ethernet cores
on the eth device with an initial state of ETH_INIT_LOCAL_HANDSHAKE
*/

enum class InitState : uint16_t {
    // Unknown initial state.
    UNKNOWN = 0,
    // Indicates that this is written directly from host.
    ETH_INIT_FROM_HOST,
    // Write kernel to local ethernet cores and wait for ack.
    ETH_INIT_LOCAL,
    // Wait for ack from connected ethernet core.
    ETH_HANDSHAKE_NEIGHBOUR,
    // Write primary kernel to connected ethernet core and wait for ack.
    ETH_INIT_NEIGHBOUR,
    // Wait for ack from local ethernet cores.
    ETH_HANDSHAKE_LOCAL,
    // Ready for traffic.
    READY,
    // Terminated.
    TERMINATED,
};

enum class RoutingEnabledState : uint16_t {
    // Stopped.
    STOPPED = 0,
    // Enabled. Call functions to service channels.
    ENABLED = 1,
    // Write to disable routing. This will stop all routing activity and propagate routing enabled state to the
    // connected core.
    STOP = 2,
};

struct LiteFabricConfig {
    // Starting address of the Lite Fabric binary to be copied locally and to the neighbour.
    volatile uint32_t binary_addr = 0;

    // Size of the Lite Fabric binary.
    volatile uint32_t binary_size = 0;

    // Bit N is 1 if channel N is an active ethernet core. Relies on eth_chan_to_noc_xy to
    // get the ethernet core coordinate.
    volatile uint32_t eth_chans_mask = 0;

    uint32_t padding0{};

    // Subordinate cores on the same chip increment this value when they are ready. The primary core
    // will stall until this value shows all eth cores are ready.
    volatile uint32_t primary_local_handshake = 0;

    uint32_t padding1[3]{};

    // Becomes 1 when the neighbour is ready.
    volatile uint32_t neighbour_handshake = 0;

    uint32_t padding2[1]{};

    // This is the local primary core.
    volatile uint16_t is_primary = false;

    volatile uint8_t primary_eth_core_x = 0;

    volatile uint8_t primary_eth_core_y = 0;

    // This is on the MMIO.
    volatile uint16_t is_mmio = false;

    volatile InitState initial_state = InitState::UNKNOWN;

    volatile InitState current_state = InitState::UNKNOWN;

    unsigned char padding3[14]{};

    volatile RoutingEnabledState routing_enabled = RoutingEnabledState::STOPPED;

    unsigned char padding4[14]{};

    // Multi-hop forwarding configuration.  Must match the FW-side
    // FabricLiteConfig::ForwardingConfig in host_interface.hpp so that
    // sizeof(LiteFabricConfig) is identical and the LiteFabricMemoryMap
    // host_interface offset stays in sync between host and device.
    struct ForwardingConfig {
        volatile uint8_t enabled = 0;
        uint8_t downstream_noc_x = 0;
        uint8_t downstream_noc_y = 0;
        uint8_t downstream_num_buffers = 0;
        volatile uint32_t downstream_sender_buf_addr = 0;
        volatile uint32_t downstream_h2d_addr = 0;
        uint32_t downstream_buffer_size = 0;
        // Must match FW-side ForwardingConfig in host_interface.hpp:
        // initial_wr_idx + padding to keep struct sizes identical.
        uint8_t initial_wr_idx = 0;
        uint8_t is_reverse_relay = 0;
        uint8_t _forwarding_pad[14]{};
    } __attribute__((packed)) forwarding;
} __attribute__((packed));

static_assert(sizeof(LiteFabricConfig) % 16 == 0);
static_assert(offsetof(LiteFabricConfig, primary_local_handshake) % 16 == 0);
static_assert(offsetof(LiteFabricConfig, neighbour_handshake) % 16 == 0);

// Deprecated: was a global static counter shared across all devices, which caused a race
// condition when multiple remote devices performed concurrent reads (e.g. during parallel
// build_and_init in Phase 3).  Each HostToLiteFabricInterface now has its own counter.
class HostToLiteFabricReadEvent {
private:
    inline static std::atomic<uint64_t> event{0};

public:
    static uint64_t get() { return event.load(); }

    static void increment() { event.fetch_add(1); }
};

// Interface for Host to MMIO Lite Fabric.
template <uint32_t NUM_BUFFERS, uint32_t CHANNEL_BUFFER_SIZE>
struct HostToLiteFabricInterface {
    // This values are updated by the device and read to the host.
    struct DeviceToHost {
        volatile uint8_t fabric_sender_channel_index = 0;
        volatile uint8_t fabric_receiver_channel_index = 0;
    } __attribute((packed)) d2h;

    // Padding to ensure d2h and h2d occupy separate 4-byte words.
    // Without this, firmware byte writes to d2h (RISC-V SB → word-level
    // RMW on L1) can clobber concurrent host writes to h2d in the same word.
    uint8_t _d2h_h2d_pad[2]{};

    // These values are updated by the host and written to the device.
    struct HostToDevice {
        volatile uint8_t sender_host_write_index = 0;
        volatile uint8_t receiver_host_read_index = 0;
    } __attribute((packed)) h2d;

    // Ch0 host interface device address (for sender h2d flushing).
    uint32_t host_interface_on_device_addr = 0;
    uint32_t sender_channel_base = 0;
    // Ch1 receiver channel base (where read responses arrive).
    uint32_t receiver_channel_base = 0;
    uint32_t eth_barrier_addr = 0;
    uint32_t tensix_barrier_addr = 0;
    uint32_t l1_alignment_bytes = 0;
    // Address of LiteFabricConfig on device, for diagnostic readback.
    uint32_t config_on_device_addr = 0;
    // The core to process requests.
    uint32_t mmio_device_id = 0;
    uint32_t mmio_eth_core_x = 0;
    uint32_t mmio_eth_core_y = 0;
    TTDevice* tt_device = nullptr;

    // Number of lite fabric hops to reach the target chip.  1 = direct ETH link,
    // 2+ = multi-hop forwarding through intermediate chips.  Used by write_reg,
    // write_one_page, and read_one_page to set routing_fields in the header.
    uint32_t num_hops = 1;

    // Per-instance read event counter.  Each HostToLiteFabricInterface (one per
    // lite-fabric tunnel / remote device) tracks its own monotonic event ID so
    // concurrent reads on different tunnels cannot interfere with each other.
    uint64_t read_event_counter = 0;

    // Ch1 host interface device address (for receiver h2d flushing).
    // Read responses are delivered to ch1 receiver buffers; the host must
    // flush the receiver_host_read_index to ch1's h2d on device.
    uint32_t receiver_host_interface_on_device_addr = 0;

    // Ch1 receiver tracking: mirrors ch1's h2d/d2h on device.
    // h2d.sender_host_write_index is always 0 (the NOC_READ handler manages
    // ch1 sender internally); only receiver_host_read_index is used.
    struct ReceiverCh1State {
        uint8_t receiver_host_read_index = 0;
        uint8_t d2h_receiver_index = 0;
    } recv_ch1;

    inline void init() volatile {
        h2d.sender_host_write_index = 0;
        h2d.receiver_host_read_index = 0;
        d2h.fabric_sender_channel_index = 0;
        d2h.fabric_receiver_channel_index = 0;
        read_event_counter = 0;
        recv_ch1.receiver_host_read_index = 0;
        recv_ch1.d2h_receiver_index = 0;
    }

    // Flush ch1 receiver h2d to device.  Ch1 sender on the MMIO side is
    // unused (read responses flow remote→MMIO only), so sender=0 is safe.
    void flush_recv_ch1_h2d(CoreCoord translated_core) {
        tt_driver_atomics::mfence();
        // Pack h2d word: byte 0 = sender (0), byte 1 = receiver
        uint8_t h2d_bytes[4] = {0, recv_ch1.receiver_host_read_index, 0, 0};
        tt_device->write_to_device(
            h2d_bytes,
            translated_core,
            receiver_host_interface_on_device_addr + offsetof(HostToLiteFabricInterface, h2d),
            sizeof(h2d_bytes));
    }

    void read(void* mem_ptr, size_t size, CoreCoord receiver_core, tt_xy_pair src_core, uint64_t src_addr) {
        uint64_t src_noc_addr = (uint64_t(src_core.y) << (36 + 6)) | (uint64_t(src_core.x) << 36) | src_addr;
        read_noc_addr(mem_ptr, size, receiver_core, src_noc_addr);
    }

    void write(void* mem_ptr, size_t size, CoreCoord sender_core, tt_xy_pair dst_core, uint64_t dst_addr) {
        uint64_t dst_noc_addr = (uint64_t(dst_core.y) << (36 + 6)) | (uint64_t(dst_core.x) << 36) | dst_addr;
        write_noc_addr(mem_ptr, size, sender_core, dst_noc_addr);
    }

    // Send a WRITE_REG command through the lite fabric.  The MMIO-side
    // relay calls eth_write_remote_reg() which writes directly to the
    // remote ETH tile's register space via the Ethernet hardware, bypassing
    // the NOC.  This is required for writing to debug registers (0xFFBxxxxx)
    // such as the soft reset register which are not reachable via NOC
    // unicast writes.
    void write_reg(uint32_t reg_addr, uint32_t reg_value, CoreCoord sender_core) {
        FabricLiteHeader header;
        header.to_chip_unicast(num_hops);
        header.to_write_reg(lite_fabric::WriteRegCommandHeader{reg_addr, reg_value});
        header.payload_size_bytes = sizeof(FabricLiteHeader);
        header.unaligned_offset = 0;
        header.debug = 0xcafe0000;
        header.noc_send_type.fields.noc_index = 0;

        wait_for_empty_write_slot(sender_core);

        uint32_t addr = get_next_send_buffer_slot_address(sender_channel_base);
        tt_device->write_to_device(&header, sender_core, addr, sizeof(FabricLiteHeader));

        h2d.sender_host_write_index = lite_fabric::wrap_increment<NUM_BUFFERS>(h2d.sender_host_write_index);
        flush_h2d(sender_core);
    }

    void barrier(CoreCoord translated_core_sender) {
        uint32_t barrier_value = 0xca11ba11;
        const auto do_barrier =
            [&](const CoreCoord& translated_core, const std::string& core_type_name, uint32_t barrier_addr) -> void {
            const uint64_t dest_noc_upper =
                (uint64_t(translated_core.y) << (36 + 6)) | (uint64_t(translated_core.x) << 36);
            uint64_t dest_noc_addr = dest_noc_upper | (uint64_t)barrier_addr;
            write_one_page(&barrier_value, sizeof(uint32_t), translated_core_sender, dest_noc_addr);

            uint32_t read_barrier = 0;
            read_one_page(&read_barrier, sizeof(uint32_t), translated_core_sender, dest_noc_addr);

            if (read_barrier != barrier_value) {
                throw std::runtime_error(fmt::format(
                    "Lite fabric barrier failed. Chip memory corruption on {} translated core ({}, {}): barrier value "
                    "mismatch {:#x} != {:#x}",
                    core_type_name,
                    translated_core.x,
                    translated_core.y,
                    read_barrier,
                    barrier_value));
            }
        };

        CoreCoord barrier_coord = CoreCoord(1, 2, CoreType::TENSIX, CoordSystem::TRANSLATED);
        do_barrier(barrier_coord, "tensix", tensix_barrier_addr);
        barrier_coord = CoreCoord(1, 1, CoreType::ETH, CoordSystem::NOC0);
        do_barrier(barrier_coord, "ethernet", eth_barrier_addr);
    }

    // Block until the device has consumed all pending writes (d2h.fabric_sender_channel_index
    // catches up to h2d.sender_host_write_index).  Used to implement a non-MMIO flush/barrier:
    // callers can guarantee that all prior writes have been committed to the remote chip before
    // this function returns.
    void wait_for_all_writes_consumed(CoreCoord translated_core_sender) {
        static constexpr auto k_Timeout = std::chrono::seconds(10);
        static constexpr auto k_WarnInterval = std::chrono::seconds(2);
        uint32_t offset = offsetof(HostToLiteFabricInterface, d2h);
        auto start = std::chrono::steady_clock::now();
        auto last_warn = start;
        bool self_heal_attempted = false;
        do {
            tt_device->read_from_device(
                (void*)(reinterpret_cast<uintptr_t>(this) + offset),
                translated_core_sender,
                host_interface_on_device_addr + offset,
                sizeof(DeviceToHost));

            // Immediate self-healing: if host thinks there are outstanding
            // writes but the device sender is idle (device h2d == device d2h),
            // the host cached h2d is stale.  Re-sync without waiting 2s.
            if (!self_heal_attempted &&
                d2h.fabric_sender_channel_index != h2d.sender_host_write_index) {
                self_heal_attempted = true;
                uint32_t dev_h2d_word = 0;
                tt_device->read_from_device(
                    &dev_h2d_word,
                    translated_core_sender,
                    host_interface_on_device_addr + offsetof(HostToLiteFabricInterface, h2d),
                    sizeof(uint32_t));
                uint8_t dev_h2d_sender = dev_h2d_word & 0xFF;
                if (dev_h2d_sender == d2h.fabric_sender_channel_index) {
                    log_warning(
                        LogUMD,
                        "wait_for_all_writes_consumed: immediate self-healing on core ({},{}) — "
                        "device h2d==d2h=={}, resyncing host h2d {} -> {}",
                        translated_core_sender.x,
                        translated_core_sender.y,
                        dev_h2d_sender,
                        h2d.sender_host_write_index,
                        dev_h2d_sender);
                    h2d.sender_host_write_index = dev_h2d_sender;
                    d2h.fabric_sender_channel_index = dev_h2d_sender;
                    // Also sync receiver index — d2h was already read above.
                    h2d.receiver_host_read_index = d2h.fabric_receiver_channel_index;
                    flush_h2d(translated_core_sender);
                    break;
                }
            }

            auto now = std::chrono::steady_clock::now();
            if (now - last_warn > k_WarnInterval) {
                last_warn = now;
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);

                // Read d2h and h2d from device separately (they are in different
                // 4-byte words after padding was added between them).
                uint32_t dev_d2h_word = 0;
                tt_device->read_from_device(
                    &dev_d2h_word,
                    translated_core_sender,
                    host_interface_on_device_addr + offsetof(HostToLiteFabricInterface, d2h),
                    sizeof(uint32_t));
                uint8_t dev_d2h_sender = dev_d2h_word & 0xFF;
                uint8_t dev_d2h_receiver = (dev_d2h_word >> 8) & 0xFF;
                uint32_t dev_h2d_word = 0;
                tt_device->read_from_device(
                    &dev_h2d_word,
                    translated_core_sender,
                    host_interface_on_device_addr + offsetof(HostToLiteFabricInterface, h2d),
                    sizeof(uint32_t));
                uint8_t dev_h2d_sender = dev_h2d_word & 0xFF;
                uint8_t dev_h2d_receiver = (dev_h2d_word >> 8) & 0xFF;

                // Read config: routing_enabled and current_state to check firmware health.
                LiteFabricConfig dev_config{};
                if (config_on_device_addr != 0) {
                    tt_device->read_from_device(
                        &dev_config,
                        translated_core_sender,
                        config_on_device_addr,
                        sizeof(LiteFabricConfig));
                }

                // Decode firmware diagnostic from primary_local_handshake:
                //   bits 31-24: num_free_slots (capped at 0xFF)
                //   bits 23-16: raw completion stream register value (capped at 0xFF)
                //   bit 8:      has_unsent_packet
                //   bit 0:      can_send
                uint32_t diag = dev_config.primary_local_handshake;
                uint32_t fw_num_free_slots = (diag >> 24) & 0xFF;
                uint32_t fw_completion_reg = (diag >> 16) & 0xFF;
                uint32_t fw_has_unsent = (diag >> 8) & 0xFF;
                uint32_t fw_can_send = diag & 0xFF;
                uint32_t fw_loop_counter = dev_config.neighbour_handshake;

                log_warning(
                    LogUMD,
                    "wait_for_all_writes_consumed: stuck {}ms on core ({},{}) "
                    "host: d2h.sender={} h2d.sender={} | "
                    "device: d2h.sender={} d2h.recv={} h2d.sender={} h2d.recv={} | "
                    "config: routing_enabled={} current_state={} | "
                    "fw_diag: num_free_slots={} completion_reg={} has_unsent={} can_send={} loop_cnt={}",
                    elapsed.count(),
                    translated_core_sender.x,
                    translated_core_sender.y,
                    d2h.fabric_sender_channel_index,
                    h2d.sender_host_write_index,
                    dev_d2h_sender,
                    dev_d2h_receiver,
                    dev_h2d_sender,
                    dev_h2d_receiver,
                    static_cast<uint32_t>(dev_config.routing_enabled),
                    static_cast<uint32_t>(dev_config.current_state),
                    fw_num_free_slots,
                    fw_completion_reg,
                    fw_has_unsent,
                    fw_can_send,
                    fw_loop_counter);

                // Self-healing: if device h2d == device d2h, FW considers all
                // packets processed.  The mismatch is a host-side counter desync
                // (e.g. from multiple remote chips sharing the same MMIO ETH
                // core channel across teardown/re-init cycles).  Resync host
                // counters to match device state and break out.
                if (dev_h2d_sender == dev_d2h_sender) {
                    log_warning(
                        LogUMD,
                        "wait_for_all_writes_consumed: self-healing on core ({},{}) — "
                        "device h2d==d2h=={}, resyncing host h2d {} -> {} and d2h {} -> {}",
                        translated_core_sender.x,
                        translated_core_sender.y,
                        dev_d2h_sender,
                        h2d.sender_host_write_index,
                        dev_d2h_sender,
                        d2h.fabric_sender_channel_index,
                        dev_d2h_sender);
                    h2d.sender_host_write_index = dev_d2h_sender;
                    d2h.fabric_sender_channel_index = dev_d2h_sender;
                    // Flush resynced h2d to device so FW and host stay in agreement.
                    flush_h2d(translated_core_sender);
                    break;
                }

                if (now - start > k_Timeout) {
                    throw std::runtime_error(fmt::format(
                        "Lite fabric wait_for_all_writes_consumed timed out after {}s on core ({},{}): "
                        "host d2h.sender={} h2d.sender={}, "
                        "device d2h.sender={} h2d.sender={}, "
                        "routing_enabled={} current_state={}, "
                        "fw: num_free_slots={} completion_reg={} has_unsent={} can_send={} loop_cnt={}. "
                        "{}",
                        k_Timeout.count(),
                        translated_core_sender.x,
                        translated_core_sender.y,
                        d2h.fabric_sender_channel_index,
                        h2d.sender_host_write_index,
                        dev_d2h_sender,
                        dev_h2d_sender,
                        static_cast<uint32_t>(dev_config.routing_enabled),
                        static_cast<uint32_t>(dev_config.current_state),
                        fw_num_free_slots,
                        fw_completion_reg,
                        fw_has_unsent,
                        fw_can_send,
                        fw_loop_counter,
                        fw_loop_counter == 0
                            ? "Firmware never ran (loop_cnt=0)."
                            : fw_num_free_slots == 0
                                ? "Firmware alive but num_free_slots=0 — remote receiver completions not arriving."
                                : "h2d flush reached device but firmware is not consuming — unknown cause."));
                }
            }
        } while (d2h.fabric_sender_channel_index != h2d.sender_host_write_index);
    }

private:
    constexpr uint32_t get_max_payload_data_size_bytes() const {
        // Additional 64B to be used only for unaligned reads/writes.
        return CHANNEL_BUFFER_SIZE - sizeof(FabricLiteHeader) - GLOBAL_ALIGNMENT;
    }

    uint32_t get_next_send_buffer_slot_address(uint32_t channel_address) const {
        auto buffer_index = h2d.sender_host_write_index;
        return channel_address + buffer_index * CHANNEL_BUFFER_SIZE;
    }

    uint32_t get_next_receiver_buffer_slot_address(uint32_t channel_address) const {
        auto buffer_index = h2d.receiver_host_read_index;
        return channel_address + buffer_index * CHANNEL_BUFFER_SIZE;
    }

    // Ch1 receiver buffer slot address (for read responses).
    uint32_t get_next_ch1_receiver_buffer_slot_address() const {
        return receiver_channel_base + recv_ch1.receiver_host_read_index * CHANNEL_BUFFER_SIZE;
    }

    void wait_for_empty_write_slot(CoreCoord translated_core_sender) {
        static constexpr auto k_Timeout = std::chrono::seconds(10);
        static constexpr auto k_WarnInterval = std::chrono::seconds(2);
        uint32_t offset = offsetof(HostToLiteFabricInterface, d2h);
        auto start = std::chrono::steady_clock::now();
        auto last_warn = start;
        bool self_heal_attempted = false;
        do {
            tt_device->read_from_device(
                (void*)(reinterpret_cast<uintptr_t>(this) + offset),
                translated_core_sender,
                host_interface_on_device_addr + offset,
                sizeof(DeviceToHost));

            // Self-healing: if the slot appears full but the device sender is
            // actually idle (device h2d == device d2h), the host's cached h2d
            // is stale from another HostToLiteFabricInterface that shared the
            // same MMIO ETH sender during multi-hop discovery.  Re-sync host
            // h2d to match device state so the write can proceed.
            if (!self_heal_attempted &&
                (h2d.sender_host_write_index + 1) % NUM_BUFFERS == d2h.fabric_sender_channel_index) {
                self_heal_attempted = true;
                uint32_t dev_h2d_word = 0;
                tt_device->read_from_device(
                    &dev_h2d_word,
                    translated_core_sender,
                    host_interface_on_device_addr + offsetof(HostToLiteFabricInterface, h2d),
                    sizeof(uint32_t));
                uint8_t dev_h2d_sender = dev_h2d_word & 0xFF;
                if (dev_h2d_sender == d2h.fabric_sender_channel_index) {
                    log_warning(
                        LogUMD,
                        "wait_for_empty_write_slot: self-healing on core ({},{}) — "
                        "device h2d==d2h=={}, resyncing host h2d {} -> {}",
                        translated_core_sender.x,
                        translated_core_sender.y,
                        dev_h2d_sender,
                        h2d.sender_host_write_index,
                        dev_h2d_sender);
                    h2d.sender_host_write_index = dev_h2d_sender;
                    d2h.fabric_sender_channel_index = dev_h2d_sender;
                    // Also sync receiver index — d2h was already read above.
                    // Without this, flush_h2d writes a stale h2d.receiver to the
                    // device, blocking the MMIO receiver completion gate.
                    h2d.receiver_host_read_index = d2h.fabric_receiver_channel_index;
                    flush_h2d(translated_core_sender);
                    break;
                }
            }

            auto now = std::chrono::steady_clock::now();
            if (now - last_warn > k_WarnInterval) {
                last_warn = now;
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
                log_warning(
                    LogUMD,
                    "wait_for_empty_write_slot: stuck {}ms on core ({},{}) "
                    "d2h.sender_idx={} h2d.sender_idx={}",
                    elapsed.count(),
                    translated_core_sender.x,
                    translated_core_sender.y,
                    d2h.fabric_sender_channel_index,
                    h2d.sender_host_write_index);
                if (now - start > k_Timeout) {
                    throw std::runtime_error(fmt::format(
                        "Lite fabric wait_for_empty_write_slot timed out after {}s on core ({},{}): "
                        "d2h.sender_idx={} h2d.sender_idx={}. "
                        "The MMIO-side ERISC1 is not consuming writes.",
                        k_Timeout.count(),
                        translated_core_sender.x,
                        translated_core_sender.y,
                        d2h.fabric_sender_channel_index,
                        h2d.sender_host_write_index));
                }
            }
        } while ((h2d.sender_host_write_index + 1) % NUM_BUFFERS == d2h.fabric_sender_channel_index);
    }

    void wait_for_read_event(CoreCoord translated_core_sender, uint32_t read_event_addr) {
        static constexpr auto k_Timeout = std::chrono::seconds(10);
        tt_driver_atomics::mfence();
        volatile FabricLiteHeader header;
        header.command_fields.noc_read.event = 0;
        const auto expectedOrderId = read_event_counter;
        auto start = std::chrono::steady_clock::now();
        int poll_count = 0;
        while (true) {
            tt_device->read_from_device(
                const_cast<void*>(static_cast<volatile void*>(&header)),
                translated_core_sender,

                read_event_addr,
                sizeof(FabricLiteHeader));
            if (header.command_fields.noc_read.event == expectedOrderId) {
                break;
            } else if (
                header.command_fields.noc_read.event != 0xdeadbeef &&
                header.command_fields.noc_read.event > expectedOrderId) {
                throw std::runtime_error(fmt::format(
                    "Read event out of order: {} > {}", header.command_fields.noc_read.event, expectedOrderId));
            }
            if (++poll_count % 10000 == 0 && std::chrono::steady_clock::now() - start > k_Timeout) {
                // Diagnostic: read firmware state from MMIO ERISC1 via PCIe
                std::string diag_str;
                if (config_on_device_addr != 0) {
                    LiteFabricConfig dev_config{};
                    tt_device->read_from_device(
                        &dev_config, translated_core_sender, config_on_device_addr, sizeof(LiteFabricConfig));
                    // Decode sender diag from primary_local_handshake
                    uint32_t sdiag = dev_config.primary_local_handshake;
                    uint32_t fw_nfs = (sdiag >> 24) & 0xFF;
                    uint32_t fw_comp = (sdiag >> 16) & 0xFF;
                    uint32_t fw_unsent = (sdiag >> 8) & 0xFF;
                    uint32_t fw_can = sdiag & 0xFF;
                    // Decode receiver diag from padding1[0]
                    uint32_t rdiag = dev_config.padding1[0];
                    uint32_t recv_wr_sent = (rdiag >> 24) & 0xFF;
                    uint32_t recv_comp = (rdiag >> 16) & 0xFF;
                    uint32_t recv_d2h_idx = (rdiag >> 8) & 0xFF;
                    uint32_t recv_h2d_idx = rdiag & 0xFF;
                    uint32_t loop_cnt = dev_config.neighbour_handshake;
                    // Fresh read of d2h and h2d from device L1 to distinguish
                    // "FW didn't see h2d" from "FW processed but response lost".
                    // The host-side d2h is stale (last read during wait_for_empty_write_slot).
                    uint32_t dev_d2h_word = 0;
                    tt_device->read_from_device(
                        &dev_d2h_word, translated_core_sender,
                        host_interface_on_device_addr, sizeof(dev_d2h_word));
                    uint8_t dev_d2h_sender = dev_d2h_word & 0xFF;
                    uint8_t dev_d2h_receiver = (dev_d2h_word >> 8) & 0xFF;
                    uint32_t dev_h2d_word = 0;
                    tt_device->read_from_device(
                        &dev_h2d_word, translated_core_sender,
                        host_interface_on_device_addr + offsetof(HostToLiteFabricInterface, h2d),
                        sizeof(dev_h2d_word));
                    uint8_t dev_h2d_sender = dev_h2d_word & 0xFF;
                    uint8_t dev_h2d_receiver = (dev_h2d_word >> 8) & 0xFF;

                    diag_str = fmt::format(
                        " FW: loop_cnt={} routing={} state={} | "
                        "sender: nfs={} comp_reg={} unsent={} can={} | "
                        "receiver: wr_sent={} comp={} d2h_idx={} h2d_idx={} | "
                        "host: d2h.sender={} h2d.sender={} h2d.recv={} | "
                        "dev: d2h.sender={} d2h.recv={} h2d.sender={} h2d.recv={}",
                        loop_cnt,
                        static_cast<uint32_t>(dev_config.routing_enabled),
                        static_cast<uint32_t>(dev_config.current_state),
                        fw_nfs, fw_comp, fw_unsent, fw_can,
                        recv_wr_sent, recv_comp, recv_d2h_idx, recv_h2d_idx,
                        d2h.fabric_sender_channel_index,
                        h2d.sender_host_write_index,
                        h2d.receiver_host_read_index,
                        dev_d2h_sender, dev_d2h_receiver,
                        dev_h2d_sender, dev_h2d_receiver);
                }
                throw std::runtime_error(fmt::format(
                    "Lite fabric wait_for_read_event timed out after {}s on core ({},{}): "
                    "expected event={} got event={:#x}.{}",
                    k_Timeout.count(),
                    translated_core_sender.x,
                    translated_core_sender.y,
                    expectedOrderId,
                    header.command_fields.noc_read.event,
                    diag_str));
            }
        };

        ++read_event_counter;
    }

    void send_payload_flush_non_blocking_from_address(
        FabricLiteHeader& header, CoreCoord translated_core_sender, uint32_t channel_address) {
        if (!header.get_payload_size_excluding_header()) {
            return;
        }

        uint32_t addr = get_next_send_buffer_slot_address(channel_address);
        header.debug = 0xcafe0000;
        // Use NOC0 because UMD encodes TRANSLATED coordinates which are NOC0 coordinates.
        // NOC0 and NOC1 have mirrored coordinate systems on Blackhole, so using NOC1
        // with NOC0 coordinates sends writes to the wrong physical cores.
        // ERISC0 is held in reset on the remote chip, so NOC0 is safe for ERISC1 to use.
        header.noc_send_type.fields.noc_index = 0;

        tt_device->write_to_device(&header, translated_core_sender, addr, sizeof(FabricLiteHeader));

        // TODO: Membar shouldn't be need here because we are using TTDevice read/writes which
        // are using strict ordering so it should commit transactions in order they were issued.
        // chip->l1_membar({translated_core_sender});

        h2d.sender_host_write_index =
            lite_fabric::wrap_increment<SENDER_NUM_BUFFERS_ARRAY[0]>(h2d.sender_host_write_index);

        log_debug(LogUMD, "Flushing h2d sender_host_write_index to {}", h2d.sender_host_write_index);
        flush_h2d(translated_core_sender);
    }

    void send_payload_without_header_non_blocking_from_address(
        void* data, size_t size, CoreCoord translated_core_sender, uint32_t channel_address) {
        if (!size) {
            return;
        }
        if (size > CHANNEL_BUFFER_SIZE - sizeof(FabricLiteHeader)) {
            throw std::runtime_error("Payload size exceeds channel buffer size");
        }
        uint32_t addr = get_next_send_buffer_slot_address(channel_address) + sizeof(FabricLiteHeader);
        log_debug(LogUMD, "Send {}B payload only {:#x}", size, addr);
        tt_device->write_to_device(data, translated_core_sender, addr, size);
    }

    void flush_h2d(CoreCoord translated_core_sender) {
        tt_driver_atomics::mfence();

        tt_device->write_to_device(
            (void*)(reinterpret_cast<uintptr_t>(this) + offsetof(HostToLiteFabricInterface, h2d)),
            translated_core_sender,

            host_interface_on_device_addr + offsetof(HostToLiteFabricInterface, h2d),
            sizeof(HostToDevice));

        // TODO: Membar shouldn't be need here because we are using TTDevice read/writes which
        // are using strict ordering so it should commit transactions in order they were issued.
        // chip->l1_membar({translated_core_sender});
    }

    void write_one_page(void* mem_ptr, size_t size, CoreCoord sender_core, uint64_t dst_noc_addr) {
        FabricLiteHeader header;
        header.to_chip_unicast(num_hops);
        header.to_noc_unicast_write(lite_fabric::NocUnicastCommandHeader{dst_noc_addr}, size);

        header.unaligned_offset = dst_noc_addr & (l1_alignment_bytes - 1);

        log_debug(
            LogUMD,
            "write_one_page: sender=({},{}) dst_noc={:#x} size={} h2d.sender_idx={} d2h.sender_idx={}",
            sender_core.x,
            sender_core.y,
            dst_noc_addr,
            size,
            h2d.sender_host_write_index,
            d2h.fabric_sender_channel_index);

        wait_for_empty_write_slot(sender_core);

        send_payload_without_header_non_blocking_from_address(
            mem_ptr, size, sender_core, sender_channel_base + header.unaligned_offset);
        send_payload_flush_non_blocking_from_address(header, sender_core, sender_channel_base);
    }

    void write_noc_addr(void* mem_ptr, size_t size, CoreCoord sender_core, uint64_t dst_noc_addr) {
        size_t num_pages = size / get_max_payload_data_size_bytes();
        for (size_t i = 0; i < num_pages; i++) {
            write_one_page(
                reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(mem_ptr) + i * get_max_payload_data_size_bytes()),
                get_max_payload_data_size_bytes(),
                sender_core,
                dst_noc_addr + i * get_max_payload_data_size_bytes());
        }

        size_t remaining_bytes = size % get_max_payload_data_size_bytes();
        if (remaining_bytes > 0) {
            write_one_page(
                reinterpret_cast<void*>(
                    reinterpret_cast<uintptr_t>(mem_ptr) + num_pages * get_max_payload_data_size_bytes()),
                remaining_bytes,
                sender_core,
                dst_noc_addr + num_pages * get_max_payload_data_size_bytes());
        }
    }

    void read_one_page(void* mem_ptr, size_t size, CoreCoord receiver_core, uint64_t src_noc_addr) {
        FabricLiteHeader header;
        header.to_chip_unicast(num_hops);
        header.to_noc_read(lite_fabric::NocReadCommandHeader{src_noc_addr, read_event_counter}, size);
        header.unaligned_offset = 0;

        // Read responses arrive on ch1 receiver buffers.
        // recv_ch1.receiver_host_read_index is tracked locally and always
        // correct for the current interface.  Re-syncing from device d2h is
        // handled by set_remote_transfer_ethernet_cores when channels are
        // rebound.  Reading d2h here was harmful: the FW's d2h.receiver lags
        // behind recv_ch1 due to the flow control gate ((d2h+1)%N != h2d),
        // causing the self-healing to regress recv_ch1 and poll the wrong
        // buffer slot (stale 0xdeadbeef sentinel → 10s timeout).
        uint32_t receiver_header_address = get_next_ch1_receiver_buffer_slot_address();
        log_debug(
            LogUMD,
            "Board {:#x} {} Reading data from {} {:#x} unaligned {} src node {:#x}",
            tt_device->get_chip_info().board_id,
            tt_device->get_chip_info().asic_location,
            receiver_core.str(),
            receiver_header_address,
            header.unaligned_offset,
            src_noc_addr);
        uint32_t receiver_data_address = receiver_header_address + sizeof(FabricLiteHeader);

        // Send read command through ch0 sender.
        wait_for_empty_write_slot(receiver_core);
        send_payload_flush_non_blocking_from_address(header, receiver_core, sender_channel_base);

        // Wait for response on ch1 receiver.
        wait_for_read_event(receiver_core, receiver_header_address);

        uint8_t read_back_unaligned_offset = 0;
        tt_device->read_from_device(
            &read_back_unaligned_offset,
            receiver_core,
            receiver_header_address + offsetof(FabricLiteHeader, unaligned_offset),
            sizeof(uint8_t));

        tt_device->read_from_device(mem_ptr, receiver_core, receiver_data_address + read_back_unaligned_offset, size);

        // Clear the event field in the ch1 receiver buffer to prevent stale
        // events from confusing future reads by different
        // HostToLiteFabricInterfaces that share the same MMIO ETH core.
        uint32_t dead = 0xdeadbeef;
        tt_device->write_to_device(
            &dead, receiver_core,
            receiver_header_address + offsetof(FabricLiteHeader, command_fields) +
                offsetof(lite_fabric::NocReadCommandHeader, event),
            sizeof(dead));

        // Advance ch1 receiver index and flush to ch1 host interface on device.
        recv_ch1.receiver_host_read_index =
            lite_fabric::wrap_increment<RECEIVER_NUM_BUFFERS_ARRAY[1]>(recv_ch1.receiver_host_read_index);
        flush_recv_ch1_h2d(receiver_core);
    }

    void read_noc_addr(void* mem_ptr, size_t size, CoreCoord receiver_core, uint64_t src_noc_addr) {
        size_t num_pages = size / get_max_payload_data_size_bytes();
        for (size_t i = 0; i < num_pages; i++) {
            read_one_page(
                reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(mem_ptr) + i * get_max_payload_data_size_bytes()),
                get_max_payload_data_size_bytes(),
                receiver_core,
                src_noc_addr + i * get_max_payload_data_size_bytes());
        }

        size_t remaining_bytes = size % get_max_payload_data_size_bytes();
        if (remaining_bytes > 0) {
            read_one_page(
                reinterpret_cast<void*>(
                    reinterpret_cast<uintptr_t>(mem_ptr) + num_pages * get_max_payload_data_size_bytes()),
                remaining_bytes,
                receiver_core,
                src_noc_addr + num_pages * get_max_payload_data_size_bytes());
        }
    }
}

__attribute__((packed));

struct LiteFabricMemoryMap {
    uint32_t sender_flow_control_semaphore{};
    uint32_t padding0[3]{};
    uint32_t sender_connection_live_semaphore{};
    uint32_t padding1[3]{};
    uint32_t worker_semaphore{};
    uint32_t padding2[7]{};

    // Channel 0 sender buffers (outbound commands: writes + read commands)
    unsigned char sender_ch0_buffer[lite_fabric::SENDER_NUM_BUFFERS_ARRAY[0] * lite_fabric::CHANNEL_BUFFER_SIZE]{};
    // Channel 1 sender buffers (read responses going back to host)
    unsigned char sender_ch1_buffer[lite_fabric::SENDER_NUM_BUFFERS_ARRAY[1] * lite_fabric::CHANNEL_BUFFER_SIZE]{};

    unsigned char padding3[64]{};

    // Channel 0 receiver buffers (incoming commands on remote side)
    unsigned char receiver_ch0_buffer[lite_fabric::RECEIVER_NUM_BUFFERS_ARRAY[0] * lite_fabric::CHANNEL_BUFFER_SIZE]{};
    // Channel 1 receiver buffers (incoming read responses on MMIO side)
    unsigned char receiver_ch1_buffer[lite_fabric::RECEIVER_NUM_BUFFERS_ARRAY[1] * lite_fabric::CHANNEL_BUFFER_SIZE]{};

    // L1 address of the service_lite_fabric function.
    uint32_t service_lite_fabric_addr{};
    unsigned char padding4[12]{};

    lite_fabric::LiteFabricConfig config;
    lite_fabric::EDMChannelWorkerLocationInfo sender_ch0_location_info;
    lite_fabric::EDMChannelWorkerLocationInfo sender_ch1_location_info;

    // Channel 0 host interface (outbound commands)
    HostToLiteFabricInterface<lite_fabric::SENDER_NUM_BUFFERS_ARRAY[0], lite_fabric::CHANNEL_BUFFER_SIZE>
        host_interface;
    // Channel 1 host interface (read responses)
    HostToLiteFabricInterface<lite_fabric::SENDER_NUM_BUFFERS_ARRAY[1], lite_fabric::CHANNEL_BUFFER_SIZE>
        host_interface_ch1;

    static auto make_host_interface(TTDevice* tt_device) {
        lite_fabric::HostToLiteFabricInterface<SENDER_NUM_BUFFERS_ARRAY[0], CHANNEL_BUFFER_SIZE> host_interface;
        host_interface.host_interface_on_device_addr = lite_fabric::LiteFabricMemoryMap::get_host_interface_ch0_addr();
        host_interface.sender_channel_base = lite_fabric::LiteFabricMemoryMap::get_send_channel_ch0_addr();
        // Ch0 receiver is used on the remote side; MMIO host reads responses from ch1.
        host_interface.receiver_channel_base = lite_fabric::LiteFabricMemoryMap::get_receiver_channel_ch1_addr();
        // Track the ch1 host interface device address for receiver h2d flushing.
        host_interface.receiver_host_interface_on_device_addr =
            lite_fabric::LiteFabricMemoryMap::get_host_interface_ch1_addr();

        // TODO: these constants need to be moved to HAL once we have it.
        constexpr uint32_t eth_barrier_addr = 12;
        constexpr uint32_t tensix_barrier_addr = 12;
        constexpr uint32_t l1_alignment_bytes = GLOBAL_ALIGNMENT;
        host_interface.eth_barrier_addr = eth_barrier_addr;
        host_interface.tensix_barrier_addr = tensix_barrier_addr;
        host_interface.l1_alignment_bytes = l1_alignment_bytes;
        host_interface.config_on_device_addr =
            get_address() + offsetof(lite_fabric::LiteFabricMemoryMap, config);
        host_interface.tt_device = tt_device;

        host_interface.init();
        return host_interface;
    }

    static uint32_t get_address() {
        auto addr = LITE_FABRIC_CONFIG_START;
        return addr;
    }

    static uint32_t get_host_interface_ch0_addr() {
        return get_address() + offsetof(lite_fabric::LiteFabricMemoryMap, host_interface);
    }

    static uint32_t get_host_interface_ch1_addr() {
        return get_address() + offsetof(lite_fabric::LiteFabricMemoryMap, host_interface_ch1);
    }

    // Legacy alias
    static uint32_t get_host_interface_addr() {
        return get_host_interface_ch0_addr();
    }

    static uint32_t get_send_channel_ch0_addr() {
        return get_address() + offsetof(lite_fabric::LiteFabricMemoryMap, sender_ch0_buffer);
    }

    static uint32_t get_send_channel_ch1_addr() {
        return get_address() + offsetof(lite_fabric::LiteFabricMemoryMap, sender_ch1_buffer);
    }

    // Legacy alias
    static uint32_t get_send_channel_addr() {
        return get_send_channel_ch0_addr();
    }

    static uint32_t get_receiver_channel_ch0_addr() {
        return get_address() + offsetof(lite_fabric::LiteFabricMemoryMap, receiver_ch0_buffer);
    }

    static uint32_t get_receiver_channel_ch1_addr() {
        return get_address() + offsetof(lite_fabric::LiteFabricMemoryMap, receiver_ch1_buffer);
    }

    // Legacy alias
    static uint32_t get_receiver_channel_addr() {
        return get_receiver_channel_ch0_addr();
    }

    static uint32_t get_service_channel_func_addr() {
        return get_address() + offsetof(lite_fabric::LiteFabricMemoryMap, service_lite_fabric_addr);
    }
};

static_assert(offsetof(LiteFabricMemoryMap, sender_flow_control_semaphore) % 16 == 0);
static_assert(offsetof(LiteFabricMemoryMap, sender_connection_live_semaphore) % 16 == 0);
static_assert(offsetof(LiteFabricMemoryMap, worker_semaphore) % 16 == 0);
static_assert(offsetof(LiteFabricMemoryMap, sender_ch0_buffer) % GLOBAL_ALIGNMENT == 0);
static_assert(offsetof(LiteFabricMemoryMap, receiver_ch0_buffer) % GLOBAL_ALIGNMENT == 0);
static_assert(offsetof(LiteFabricMemoryMap, config) % 16 == 0);
static_assert(offsetof(LiteFabricMemoryMap, host_interface) % 16 == 0);

}  // namespace lite_fabric

}  // namespace tt::umd
