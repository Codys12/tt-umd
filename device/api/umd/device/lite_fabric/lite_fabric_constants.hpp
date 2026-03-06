// SPDX-FileCopyrightText: © 2025 Tenstorrent AI ULC
//
// SPDX-License-Identifier: Apache-2.0

// Substitute for 1d_fabric_constants.hpp.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "umd/device/lite_fabric/lite_fabric_header.hpp"

namespace tt::umd {

namespace lite_fabric {

// Dual channels: ch0 for outbound commands, ch1 for read responses.
constexpr uint32_t NUM_RECEIVER_CHANNELS = 2;
constexpr uint32_t NUM_SENDER_CHANNELS = 2;

// 4 buffer slots per channel for pipelining.
constexpr std::array<size_t, NUM_SENDER_CHANNELS> SENDER_NUM_BUFFERS_ARRAY = {4, 4};
constexpr std::array<size_t, NUM_RECEIVER_CHANNELS> RECEIVER_NUM_BUFFERS_ARRAY = {4, 4};

static_assert(NUM_SENDER_CHANNELS == 2);

// Alignment for read and write to work on all core types.
constexpr uint32_t GLOBAL_ALIGNMENT = 64;

// Additional 64B reserved for data alignment.
constexpr uint32_t ALIGNMENT_BUFFER_SIZE = GLOBAL_ALIGNMENT;

constexpr uint32_t CHANNEL_BUFFER_SIZE = 2048 + ALIGNMENT_BUFFER_SIZE + sizeof(lite_fabric::FabricLiteHeader);

}  // namespace lite_fabric
}  // namespace tt::umd
