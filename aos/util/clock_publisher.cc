#include "aos/util/clock_publisher.h"

#include <algorithm>
#include <chrono>
#include <vector>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"

#include "aos/configuration_generated.h"
#include "aos/events/context.h"

namespace aos {
ClockPublisher::ClockPublisher(aos::SimulatedEventLoopFactory *factory,
                               aos::EventLoop *event_loop)
    : factory_(factory),
      timepoints_sender_(event_loop->MakeSender<ClockTimepoints>("/clocks")) {
  aos::TimerHandler *timer_handler =
      event_loop->AddTimer([this]() { SendTimepoints(); });
  event_loop->OnRun([timer_handler, event_loop]() {
    timer_handler->Schedule(event_loop->context().monotonic_event_time,
                            std::chrono::seconds(1));
  });
}

void ClockPublisher::SendTimepoints() {
  std::vector<flatbuffers::Offset<NodeTimepoint>> timepoints;
  auto builder = timepoints_sender_.MakeBuilder();
  for (const aos::Node *node : factory_->nodes()) {
    const NodeEventLoopFactory *node_factory =
        factory_->GetNodeEventLoopFactory(node);
    flatbuffers::Offset<flatbuffers::String> node_name =
        (node != nullptr)
            ? builder.fbb()->CreateString(node->name()->string_view())
            : flatbuffers::Offset<flatbuffers::String>(0);
    flatbuffers::Offset<flatbuffers::String> boot_uuid =
        node_factory->is_running()
            ? node_factory->boot_uuid().PackString(builder.fbb())
            : flatbuffers::Offset<flatbuffers::String>(0);
    NodeTimepoint::Builder timepoint_builder =
        builder.MakeBuilder<NodeTimepoint>();
    if (node != nullptr) {
      timepoint_builder.add_node(node_name);
    }
    if (node_factory->is_running()) {
      timepoint_builder.add_boot_count(node_factory->boot_count());
      timepoint_builder.add_boot_uuid(boot_uuid);
      timepoint_builder.add_monotonic_time(
          node_factory->monotonic_now().time_since_epoch().count());
      timepoint_builder.add_realtime_time(
          node_factory->realtime_now().time_since_epoch().count());
    }
    timepoints.push_back(timepoint_builder.Finish());
  }
  const flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<NodeTimepoint>>>
      timepoints_offset = builder.fbb()->CreateVector(timepoints);
  ClockTimepoints::Builder timepoints_builder =
      builder.MakeBuilder<ClockTimepoints>();
  timepoints_builder.add_distributed_clock(
      factory_->distributed_now().time_since_epoch().count());
  timepoints_builder.add_clocks(timepoints_offset);
  builder.CheckOk(builder.Send(timepoints_builder.Finish()));
}

}  // namespace aos
