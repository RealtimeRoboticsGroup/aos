#include "aos/events/logging/buffer_encoder.h"

#include <algorithm>

#include "absl/log/check.h"

namespace aos::logger {

DummyEncoder::DummyEncoder(size_t /*max_message_size*/, size_t buffer_size) {
  // Round up to the nearest page size.
  input_buffer_.reserve(buffer_size);
  return_queue_.resize(1);
}

size_t DummyEncoder::space() const {
  return input_buffer_.capacity() - input_buffer_.size();
}

bool DummyEncoder::HasSpace(size_t request) const { return request <= space(); }

size_t DummyEncoder::Encode(Copier *copy, size_t start_byte,
                            std::chrono::nanoseconds * /*encode_duration*/) {
  const size_t input_buffer_initial_size = input_buffer_.size();

  size_t expected_write_size =
      std::min(input_buffer_.capacity() - input_buffer_initial_size,
               copy->size() - start_byte);
  input_buffer_.resize(input_buffer_initial_size + expected_write_size);
  const size_t written_size =
      copy->Copy(input_buffer_.data() + input_buffer_initial_size, start_byte,
                 expected_write_size + start_byte);

  total_bytes_ += written_size;

  return written_size;
}

void DummyEncoder::Clear(const int n) {
  CHECK_GE(n, 0);
  CHECK_LE(static_cast<size_t>(n), queue_size());
  if (n != 0) {
    input_buffer_.resize(0u);
  }
}

absl::Span<const absl::Span<const uint8_t>> DummyEncoder::queue() {
  if (input_buffer_.size() != 0) {
    return_queue_[0] =
        absl::Span<const uint8_t>(input_buffer_.data(), input_buffer_.size());
    return return_queue_;
  } else {
    return absl::Span<const absl::Span<const uint8_t>>();
  }
}

size_t DummyEncoder::queued_bytes() const { return input_buffer_.size(); }

}  // namespace aos::logger
