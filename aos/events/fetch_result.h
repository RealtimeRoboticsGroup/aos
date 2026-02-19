#ifndef AOS_EVENTS_FETCH_RESULT_H_
#define AOS_EVENTS_FETCH_RESULT_H_
namespace aos::internal {
// Corresponds to the possible outcomes when reading a message from an aOS
// channel.
// This is somewhat tied to the shared-memory queue implementation.
enum class FetchResult {
  // Message we read was too old and no longer is in the queue.
  TOO_OLD,
  // Success!
  GOOD,
  // The message is in the future and we haven't written it yet.
  NOTHING_NEW,
  // There is a message, but should_read_callback() returned false so we
  // didn't fetch it.
  FILTERED,
  // The message got overwritten while we were reading it.
  OVERWROTE,
};
}  // namespace aos::internal
#endif  // AOS_EVENTS_FETCH_RESULT_H_
