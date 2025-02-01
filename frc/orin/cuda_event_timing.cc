#include "frc/orin/cuda_event_timing.h"

#include <iostream>

#include "frc/orin/cuda.h"
#include "nvToolsExt.h"

Timing::Timing(const std::string &name) : m_name{name} {
  CHECK_CUDA(cudaEventCreate(&m_startEvent));
  CHECK_CUDA(cudaEventCreate(&m_endEvent));
}

Timing::~Timing() {
  CHECK_CUDA(cudaEventDestroy(m_startEvent));
  CHECK_CUDA(cudaEventDestroy(m_endEvent));
}

void Timing::start(cudaStream_t cudaStream) {
  if (m_startEventSeen) {
    if (m_endEventSeen) {
      endFrame();
    } else {
      std::cerr << "Error : duplicate start event " << m_name << " seen."
                << std::endl;
    }
  }
  m_cudaStream = cudaStream;
  CHECK_CUDA(cudaEventRecord(m_startEvent, cudaStream));
  nvtxRangePushA(m_name.c_str());

  m_startEventSeen = true;
}

void Timing::end(void) {
  if (m_endEventSeen) {
    std::cerr << "Error : duplicate end event " << m_name << " seen."
              << std::endl;
  }
  CHECK_CUDA(cudaEventRecord(m_endEvent, m_cudaStream));
  nvtxRangePop();

  m_endEventSeen = true;
}

void Timing::endFrame(void) {
  if (m_startEventSeen && m_endEventSeen) {
    // CHECK_CUDA(cudaEventSynchronize(m_startEvent)); Shouldn't need this one
    // - if end has been seen, the corresponding start event has also occurred
    CHECK_CUDA(cudaEventSynchronize(m_endEvent));
    m_startEventSeen = false;
    m_endEventSeen = false;
    if (++m_count > 0) {
      float elapsedTime;
      CHECK_CUDA(
          cudaEventElapsedTime(&elapsedTime, m_startEvent, m_endEvent));
      m_elapsedSeconds += elapsedTime;
    }
  }
}

std::ostream &operator<<(std::ostream &stream, const Timing &timing) {
  if (timing.m_count <= 0) {
    stream << timing.m_name << " : no events recorded";
  } else {
    stream << timing.m_name << " : " << timing.m_count
           << " events. Total time = " << timing.m_elapsedSeconds
           << " average mSec = " << (timing.m_elapsedSeconds / timing.m_count);
  }
  return stream;
}

Timings::Timings() = default;

Timings::~Timings() {
  endFrame();
  std::cout << *this << std::endl;
}

void Timings::start(const std::string &name, cudaStream_t cudaStream) {
  if (m_enabled) {
    const auto &[it, placed] = m_timings.try_emplace(name, name);
    if (placed) {
      m_keys_in_insert_order_.push_back(name);
    }
    it->second.start(cudaStream);
  }
}

void Timings::end(const std::string &name) {
  if (m_enabled) {
    auto it = m_timings.find(name);
    if (it == m_timings.end()) {
      std::cerr << " Error - end called before start for " << name << std::endl;
      return;
    }
    it->second.end();
  }
}

void Timings::endFrame(void) {
  if (m_enabled) {
    for (auto &[name, timing] : m_timings) {
      timing.endFrame();
    }
  }
}

void Timings::setEnabled(const bool enabled) {
  if (m_enabled && !enabled) {
    endFrame();
  }
  m_enabled = enabled;
}

std::ostream &operator<<(std::ostream &stream, const Timings &timings) {
  for (const auto &name : timings.m_keys_in_insert_order_) {
    stream << timings.m_timings.at(name) << std::endl;
  }
  return stream;
}

ScopedEventTiming::ScopedEventTiming(Timings &timings, const std::string &name,
                                     cudaStream_t cudaStream)
    : m_timings{timings}, m_name{name} {
  m_timings.start(m_name, cudaStream);
}

ScopedEventTiming::~ScopedEventTiming() { m_timings.end(m_name); }
