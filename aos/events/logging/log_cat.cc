#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <vector>

#include "absl/flags/flag.h"
#include "absl/flags/usage.h"
#include "absl/strings/escaping.h"

#include "aos/aos_cli_utils.h"
#include "aos/configuration.h"
#include "aos/events/logging/log_reader.h"
#include "aos/events/logging/node_printer.h"
#include "aos/events/simulated_event_loop.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/sha256.h"

ABSL_FLAG(bool, raw, false,
          "If true, just print the data out unsorted and unparsed");
ABSL_FLAG(std::string, raw_header, "",
          "If set, the file to read the header from in raw mode");
ABSL_FLAG(bool, format_raw, true,
          "If true and --raw is specified, print out raw data, but use the "
          "schema to format the data.");
ABSL_FLAG(bool, print_parts_only, false,
          "If true, only print out the results of logfile sorting.");
ABSL_FLAG(bool, channels, false,
          "If true, print out all the configured channels for this log.");

using aos::monotonic_clock;
namespace chrono = std::chrono;

// Prints out raw log parts to stdout.
int PrintRaw(int argc, char **argv) {
  if (argc == 1) {
    CHECK(!absl::GetFlag(FLAGS_raw_header).empty());
    aos::logger::MessageReader raw_header_reader(
        absl::GetFlag(FLAGS_raw_header));
    std::cout << aos::FlatbufferToJson(
                     raw_header_reader.raw_log_file_header(),
                     {.multi_line = absl::GetFlag(FLAGS_pretty),
                      .max_vector_size = static_cast<size_t>(
                          absl::GetFlag(FLAGS_max_vector_size))})
              << std::endl;
    return 0;
  }
  if (argc != 2 && argc != 1) {
    LOG(FATAL) << "Expected 1 logfile as an argument.";
  }
  aos::logger::SpanReader reader(argv[1]);
  absl::Span<const uint8_t> raw_log_file_header_span = reader.ReadMessage();

  if (raw_log_file_header_span.empty()) {
    LOG(WARNING) << "Empty log file on " << reader.filename();
    return 0;
  }

  // Now, reproduce the log file header deduplication logic inline so we can
  // print out all the headers we find.
  aos::SizePrefixedFlatbufferVector<aos::logger::LogFileHeader> log_file_header(
      raw_log_file_header_span);
  if (!log_file_header.Verify()) {
    LOG(ERROR) << "Header corrupted on " << reader.filename();
    return 1;
  }
  while (true) {
    absl::Span<const uint8_t> maybe_header_data = reader.PeekMessage();
    if (maybe_header_data.empty()) {
      break;
    }

    aos::SizePrefixedFlatbufferSpan<aos::logger::LogFileHeader> maybe_header(
        maybe_header_data);
    if (maybe_header.Verify()) {
      std::cout << aos::FlatbufferToJson(
                       log_file_header,
                       {.multi_line = absl::GetFlag(FLAGS_pretty),
                        .max_vector_size = static_cast<size_t>(
                            absl::GetFlag(FLAGS_max_vector_size))})
                << std::endl;
      LOG(WARNING) << "Found duplicate LogFileHeader in " << reader.filename();
      log_file_header =
          aos::SizePrefixedFlatbufferVector<aos::logger::LogFileHeader>(
              maybe_header_data);

      reader.ConsumeMessage();
    } else {
      break;
    }
  }

  // And now use the final sha256 to match the raw_header.
  std::optional<aos::logger::MessageReader> raw_header_reader;
  const aos::logger::LogFileHeader *full_header = &log_file_header.message();
  if (!absl::GetFlag(FLAGS_raw_header).empty()) {
    raw_header_reader.emplace(absl::GetFlag(FLAGS_raw_header));
    std::cout << aos::FlatbufferToJson(
                     full_header, {.multi_line = absl::GetFlag(FLAGS_pretty),
                                   .max_vector_size = static_cast<size_t>(
                                       absl::GetFlag(FLAGS_max_vector_size))})
              << std::endl;
    CHECK_EQ(full_header->configuration_sha256()->string_view(),
             aos::Sha256(raw_header_reader->raw_log_file_header().span()));
    full_header = raw_header_reader->log_file_header();
  }

  if (!absl::GetFlag(FLAGS_print)) {
    return 0;
  }

  std::cout << aos::FlatbufferToJson(
                   full_header, {.multi_line = absl::GetFlag(FLAGS_pretty),
                                 .max_vector_size = static_cast<size_t>(
                                     absl::GetFlag(FLAGS_max_vector_size))})
            << std::endl;
  CHECK(full_header->has_configuration())
      << ": Missing configuration! You may want to provide the path to the "
         "logged configuration file using the --raw_header flag.";

  while (true) {
    const aos::SizePrefixedFlatbufferSpan<aos::logger::MessageHeader> message(
        reader.ReadMessage());
    if (message.span().empty()) {
      break;
    }
    CHECK(message.Verify());

    const auto *const channels = full_header->configuration()->channels();
    const size_t channel_index = message.message().channel_index();
    CHECK_LT(channel_index, channels->size());
    const aos::Channel *const channel = channels->Get(channel_index);

    CHECK(message.Verify()) << absl::BytesToHexString(
        std::string_view(reinterpret_cast<const char *>(message.span().data()),
                         message.span().size()));

    if (message.message().data() != nullptr) {
      CHECK(channel->has_schema());

      CHECK(flatbuffers::Verify(
          *channel->schema(), *channel->schema()->root_table(),
          message.message().data()->data(), message.message().data()->size()))
          << ": Corrupted flatbuffer on " << channel->name()->c_str() << " "
          << channel->type()->c_str();
    }

    if (absl::GetFlag(FLAGS_format_raw) &&
        message.message().data() != nullptr) {
      std::cout << aos::configuration::StrippedChannelToString(channel) << " "
                << aos::FlatbufferToJson(
                       message, {.multi_line = absl::GetFlag(FLAGS_pretty),
                                 .max_vector_size = 4})
                << ": "
                << aos::FlatbufferToJson(
                       channel->schema(), message.message().data()->data(),
                       {absl::GetFlag(FLAGS_pretty),
                        static_cast<size_t>(
                            absl::GetFlag(FLAGS_max_vector_size))})
                << std::endl;
    } else {
      std::cout << aos::configuration::StrippedChannelToString(channel) << " "
                << aos::FlatbufferToJson(
                       message, {absl::GetFlag(FLAGS_pretty),
                                 static_cast<size_t>(
                                     absl::GetFlag(FLAGS_max_vector_size))})
                << std::endl;
    }
  }
  return 0;
}

int main(int argc, char **argv) {
  absl::SetProgramUsageMessage(
      "Usage:\n"
      "  log_cat [args] logfile1 logfile2 ...\n"
      "\n"
      "This program provides a basic interface to dump data from a logfile to "
      "stdout. Given a logfile, channel name filter, and type filter, it will "
      "print all the messages in the logfile matching the filters. The message "
      "filters work by taking the values of --name and --type and printing any "
      "channel whose name contains --name as a substr and whose type contains "
      "--type as a substr. Not specifying --name or --type leaves them free. "
      "Calling this program without --name or --type specified prints out all "
      "the logged data.");
  aos::InitGoogle(&argc, &argv);

  if (absl::GetFlag(FLAGS_raw)) {
    return PrintRaw(argc, argv);
  }

  if (argc < 2) {
    LOG(FATAL) << "Expected at least 1 logfile as an argument.";
  }

  const std::vector<aos::logger::LogFile> logfiles =
      aos::logger::SortParts(aos::logger::FindLogs(argc, argv));

  for (auto &it : logfiles) {
    VLOG(1) << it;
    if (absl::GetFlag(FLAGS_print_parts_only)) {
      std::cout << it << std::endl;
    }
  }
  if (absl::GetFlag(FLAGS_print_parts_only)) {
    return 0;
  }

  aos::logger::LogReader reader(logfiles);

  if (absl::GetFlag(FLAGS_channels)) {
    const aos::Configuration *config = reader.configuration();
    for (const aos::Channel *channel : *config->channels()) {
      std::cout << channel->name()->c_str() << " " << channel->type()->c_str()
                << '\n';
    }
    return 0;
  }

  {
    std::function<bool(const aos::Channel *)> channel_should_be_printed =
        aos::logging::GetChannelShouldBePrintedTester();
    CHECK(std::ranges::any_of(*reader.configuration()->channels(),
                              channel_should_be_printed))
        << ": Could not find any channels";
  }

  aos::Printer printer = aos::logging::MakePrinter();

  std::vector<aos::logging::NodePrinter *> printers;
  printers.resize(aos::configuration::NodesCount(reader.configuration()),
                  nullptr);

  aos::SimulatedEventLoopFactory event_loop_factory(reader.configuration());

  reader.RegisterWithoutStarting(&event_loop_factory);

  for (const aos::Node *node :
       aos::configuration::GetNodes(event_loop_factory.configuration())) {
    size_t node_index = aos::configuration::GetNodeIndex(
        event_loop_factory.configuration(), node);
    // Spin up the printer, and hook up the SetStarted method so that it gets
    // notified when the log starts and stops.
    aos::NodeEventLoopFactory *node_factory =
        event_loop_factory.GetNodeEventLoopFactory(node);
    node_factory->OnStartup(
        [&event_loop_factory, node_factory, &printer, &printers, node_index]() {
          printers[node_index] =
              node_factory->AlwaysStart<aos::logging::NodePrinter>(
                  "printer", &event_loop_factory, &printer);
        });
    node_factory->OnShutdown(
        [&printers, node_index]() { printers[node_index] = nullptr; });

    reader.OnStart(node, [&printers, node_index, node_factory]() {
      CHECK(printers[node_index]);
      printers[node_index]->SetStarted(true, node_factory->monotonic_now(),
                                       node_factory->realtime_now());
    });
    reader.OnEnd(node, [&printers, node_index, node_factory]() {
      CHECK(printers[node_index]);
      printers[node_index]->SetStarted(false, node_factory->monotonic_now(),
                                       node_factory->realtime_now());
    });
  }

  event_loop_factory.Run();

  reader.Deregister();

  return 0;
}
