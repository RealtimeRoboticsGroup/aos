#ifndef AOS_IPC_LIB_MEMORY_ESTIMATION_H_
#define AOS_IPC_LIB_MEMORY_ESTIMATION_H_

#include <functional>

#include "aos/configuration.h"

namespace aos::ipc_lib {
// Returns the total shared memory that will be used by the specified config on
// the specified node, in bytes.
size_t TotalSharedMemoryUsage(const aos::Configuration *config,
                              const aos::Node *node);
// Indicates the amount of memory that must be allocated to be able to buffer up
// all the data from the specified channel for buffer_duration.
// Currently does not account for any header-related overhead.
size_t LogMemoryBufferSizeForChannel(const aos::Channel *channel,
                                     std::chrono::nanoseconds buffer_duration);
// Similar to LogMemoryBufferSizeForChannel, but calculates the space required
// for the timestamps associated with the channel.
size_t LogMemoryBufferSizeForChannelTimestamps(
    const aos::Channel *channel, std::chrono::nanoseconds buffer_duration);
// Calls LogMemoryBufferSizeForChannel for all channels that will be logged on a
// given node, with the provided channel filter.
size_t TotalLogMemoryBufferSizeForChannels(
    const aos::Configuration *config, const aos::Node *node,
    std::function<bool(const aos::Channel *)> filter,
    std::chrono::nanoseconds buffer_duration);
}  // namespace aos::ipc_lib

#endif  // AOS_IPC_LIB_MEMORY_ESTIMATION_H_
