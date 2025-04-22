#include <poll.h>
#include <unistd.h>

#include <iostream>

#include "absl/flags/flag.h"
#include "absl/flags/usage.h"
#include "absl/log/log.h"

#include "aos/aos_cli_utils.h"
#include "aos/configuration.h"
#include "aos/init.h"
#include "aos/json_to_flatbuffer.h"

ABSL_FLAG(double, rate, -1,
          "Rate at which to send the message (-1 to send once).");
ABSL_FLAG(bool, bfbs, false, "If true, treat message as a binary flatbuffer.");

int main(int argc, char **argv) {
  absl::SetProgramUsageMessage(
      "Sends messages on arbitrary channels.\n"
      "Typical Usage: aos_send [--config path_to_config.json]"
      " channel_name message_type '{\"foo\": \"bar\"}'\n"
      "Example usage: aos_send /test aos.examples.Ping "
      "'{\"value\": 1}'\n"
      "Pipe usage: cat ping.json | aos_send /test/ aos.examples.Ping -");
  aos::InitGoogle(&argc, &argv);

  aos::CliUtilInfo cli_info;
  if (cli_info.Initialize(
          &argc, &argv,
          [&cli_info](const aos::Channel *channel) {
            return aos::configuration::ChannelIsSendableOnNode(
                channel, cli_info.event_loop->node());
          },
          "channel is sendable on node", false)) {
    return 0;
  }
  if (cli_info.found_channels.size() > 1) {
    LOG(FATAL) << "Matched multiple channels, but may only send on 1";
  }

  if (argc == 1) {
    LOG(FATAL) << "Must specify a message to send";
  }

  // Check if the user wants to use stdin (denoted by '-') or the argument
  // present in argv for the message to send. CliUtilInfo will ensure the
  // message data will be in argv[1]
  std::string_view message_to_send{argv[1]};
  std::string stdin_data;
  if (message_to_send == "-") {
    // Read in everything from stdin, blocks when there's no data on stdin
    stdin_data = std::string(std::istreambuf_iterator(std::cin), {});
    message_to_send = stdin_data;
  }

  const aos::Channel *const channel = cli_info.found_channels[0];
  const std::unique_ptr<aos::RawSender> sender =
      cli_info.event_loop->MakeRawSender(channel);
  if (absl::GetFlag(FLAGS_bfbs)) {
    if (absl::GetFlag(FLAGS_rate) < 0) {
      sender->CheckOk(
          sender->Send(message_to_send.data(), message_to_send.size()));
    } else {
      cli_info.event_loop
          ->AddTimer([&message_to_send, &sender]() {
            sender->CheckOk(
                sender->Send(message_to_send.data(), message_to_send.size()));
          })
          ->Schedule(cli_info.event_loop->monotonic_now(),
                     std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::duration<double>(
                             1.0 / absl::GetFlag(FLAGS_rate))));
      cli_info.event_loop->Run();
    }
  } else {
    flatbuffers::FlatBufferBuilder fbb(sender->fbb_allocator()->size(),
                                       sender->fbb_allocator());
    fbb.ForceDefaults(true);
    flatbuffers::Offset<flatbuffers::Table> msg_offset =
        aos::JsonToFlatbuffer(message_to_send, channel->schema(), &fbb);

    if (msg_offset.IsNull()) {
      return 1;
    }

    fbb.Finish(msg_offset);

    if (absl::GetFlag(FLAGS_rate) < 0) {
      sender->CheckOk(sender->Send(fbb.GetSize()));
    } else {
      cli_info.event_loop
          ->AddTimer([&fbb, &sender]() {
            sender->CheckOk(
                sender->Send(fbb.GetBufferPointer(), fbb.GetSize()));
          })
          ->Schedule(cli_info.event_loop->monotonic_now(),
                     std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::duration<double>(
                             1.0 / absl::GetFlag(FLAGS_rate))));
      cli_info.event_loop->Run();
    }
  }

  return 0;
}
