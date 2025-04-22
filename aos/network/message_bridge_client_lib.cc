#include "aos/network/message_bridge_client_lib.h"

#include <chrono>
#include <string_view>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/events/logging/log_reader.h"
#include "aos/events/shm_event_loop.h"
#include "aos/network/connect_generated.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_protocol.h"
#include "aos/network/remote_data_generated.h"
#include "aos/network/sctp_client.h"
#include "aos/network/sctp_config_generated.h"
#include "aos/network/sctp_config_request_generated.h"
#include "aos/network/sctp_error.h"
#include "aos/network/timestamp_generated.h"
#include "aos/unique_malloc_ptr.h"
#include "aos/util/file.h"

// The casts required to read datastructures from sockets trip - Wcast - align.
#ifdef __clang
#pragma clang diagnostic ignored "-Wcast-align"
#endif

ABSL_DECLARE_FLAG(bool, use_sctp_authentication);

// This application receives messages from another node and re-publishes them on
// this node.
//
// To simulate packet loss for testing, run:
//   tc qdisc add dev eth0 root netem loss random 10
// To restore it, run:
//   tc qdisc del dev eth0 root netem

namespace aos::message_bridge {
namespace {
namespace chrono = std::chrono;

// How often we should poll for the active SCTP authentication key.
constexpr chrono::seconds kRefreshAuthKeyPeriod{3};

std::vector<int> StreamToChannel(const Configuration *config,
                                 const Node *my_node, const Node *other_node) {
  std::vector<int> stream_to_channel;
  int channel_index = 0;
  for (const Channel *channel : *config->channels()) {
    if (configuration::ChannelIsSendableOnNode(channel, other_node)) {
      const Connection *connection =
          configuration::ConnectionToNode(channel, my_node);
      if (connection != nullptr) {
        VLOG(1) << "Channel " << channel->name()->string_view() << " "
                << channel->type()->string_view() << " mapped to stream "
                << stream_to_channel.size() + kControlStreams();
        stream_to_channel.emplace_back(channel_index);
      }
    }
    ++channel_index;
  }

  return stream_to_channel;
}

std::vector<bool> StreamReplyWithTimestamp(const Configuration *config,
                                           const Node *my_node,
                                           const Node *other_node) {
  std::vector<bool> stream_reply_with_timestamp;
  for (const Channel *channel : *config->channels()) {
    if (configuration::ChannelIsSendableOnNode(channel, other_node)) {
      const Connection *connection =
          configuration::ConnectionToNode(channel, my_node);
      if (connection != nullptr) {
        // We want to reply with a timestamp if the other node is logging the
        // timestamp (and it therefore needs the timestamp), or if we are
        // logging the message and it needs to know if we received it so it can
        // log (in the future) it through different mechanisms on failure.
        stream_reply_with_timestamp.emplace_back(
            configuration::ConnectionDeliveryTimeIsLoggedOnNode(connection,
                                                                other_node) ||
            configuration::ChannelMessageIsLoggedOnNode(channel, my_node));
      }
    }
  }

  return stream_reply_with_timestamp;
}

aos::FlatbufferDetachedBuffer<aos::logger::MessageHeader>
MakeMessageHeaderReply() {
  flatbuffers::FlatBufferBuilder fbb;
  fbb.ForceDefaults(true);
  logger::MessageHeader::Builder message_header_builder(fbb);
  message_header_builder.add_channel_index(0);
  message_header_builder.add_monotonic_sent_time(0);
  message_header_builder.add_realtime_sent_time(0);
  message_header_builder.add_queue_index(0);
  message_header_builder.add_monotonic_remote_time(0);
  message_header_builder.add_realtime_remote_time(0);
  message_header_builder.add_monotonic_remote_transmit_time(0);
  message_header_builder.add_remote_queue_index(0);
  fbb.Finish(message_header_builder.Finish());

  return fbb.Release();
}

}  // namespace

SctpClientConnection::SctpClientConnection(
    aos::ShmEventLoop *const event_loop, std::string_view remote_name,
    const Node *my_node, std::string_view local_host,
    std::vector<SctpClientChannelState> *channels, int client_index,
    MessageBridgeClientStatus *client_status, std::string_view config_sha256,
    SctpAuthMethod requested_authentication)
    : event_loop_(event_loop),
      connect_message_(MakeConnectMessage(event_loop->configuration(), my_node,
                                          remote_name, event_loop->boot_uuid(),
                                          config_sha256)),
      message_reception_reply_(MakeMessageHeaderReply()),
      remote_node_([&]() {
        const aos::Node *node =
            configuration::GetNode(event_loop->configuration(), remote_name);
        CHECK(node != nullptr);
        return node;
      }()),
      client_(remote_node_->hostname()->string_view(), remote_node_->port(),
              connect_message_.message().channels_to_transfer()->size() +
                  kControlStreams(),
              local_host, 0, requested_authentication),
      channels_(channels),
      stream_to_channel_(
          StreamToChannel(event_loop->configuration(), my_node, remote_node_)),
      stream_reply_with_timestamp_(StreamReplyWithTimestamp(
          event_loop->configuration(), my_node, remote_node_)),
      client_status_(client_status),
      client_index_(client_index),
      connection_(client_status_->GetClientConnection(client_index_)) {
  VLOG(1) << "Connect request for " << remote_node_->name()->string_view()
          << ": " << FlatbufferToJson(connect_message_);

  connect_timer_ = event_loop_->AddTimer([this]() { SendConnect(); });
  connect_timer_->set_name(std::string("connect_") +
                           remote_node_->name()->str());
  timestamp_retry_buffer_ =
      event_loop_->AddTimer([this]() { SendTimestamps(); });
  timestamp_retry_buffer_->set_name(std::string("timestamp_retry_") +
                                    remote_node_->name()->str());
  event_loop_->OnRun(
      [this]() { connect_timer_->Schedule(event_loop_->monotonic_now()); });

  size_t max_write_size =
      std::max(kHeaderSizeOverhead(), connect_message_.span().size());
  size_t max_read_size = 0u;

  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    CHECK(channel->has_source_node());

    if (configuration::ChannelIsSendableOnNode(channel, remote_node_) &&
        configuration::ChannelIsReadableOnNode(channel, event_loop_->node())) {
      VLOG(1) << "Receiving channel "
              << configuration::CleanedChannelToString(channel);
      max_read_size = std::max(
          static_cast<size_t>(channel->max_size() + kHeaderSizeOverhead()),
          max_read_size);
    }
  }

  // Buffer up the max size a bit so everything fits nicely.
  LOG(INFO) << "Max read message size for all servers is " << max_read_size;
  LOG(INFO) << "Max write message size for all servers is " << max_write_size;
  // RemoteMessage header appears to be between 100 and 204 bytes of overhead
  // from the vector of data.  No need to get super tight to that bound.
  client_.SetMaxReadSize(max_read_size);
  client_.SetMaxWriteSize(max_write_size);

  // 1 client talks to 1 server.  With interleaving support 1 turned on, we'll
  // at most see 1 partial message, and 1 incoming part, for a total of 2
  // messages in flight.
  client_.SetPoolSize(2u);

  event_loop_->epoll()->OnReadable(client_.fd(),
                                   [this]() { MessageReceived(); });
}

void SctpClientConnection::MessageReceived() {
  // Dispatch the message to the correct receiver.
  aos::unique_c_ptr<Message> message = client_.Read();
  if (!message) {
    return;
  }

  switch (message->message_type) {
    case Message::kNotification: {
      const union sctp_notification *snp =
          (const union sctp_notification *)message->data();

      switch (snp->sn_header.sn_type) {
        case SCTP_ASSOC_CHANGE: {
          const struct sctp_assoc_change *sac = &snp->sn_assoc_change;
          switch (sac->sac_state) {
            case SCTP_RESTART:
              NodeDisconnected();
              [[fallthrough]];
            case SCTP_COMM_UP:
              NodeConnected(sac->sac_assoc_id);

              VLOG(1) << "Received up from " << message->PeerAddress()
                      << " on assoc " << sac->sac_assoc_id << " state "
                      << sac->sac_state;
              break;
            case SCTP_COMM_LOST:
            case SCTP_SHUTDOWN_COMP:
            case SCTP_CANT_STR_ASSOC: {
              NodeDisconnected();
              VLOG(1) << "Disconnect from " << message->PeerAddress() << " on "
                      << sac->sac_assoc_id << " state " << sac->sac_state
                      << " error " << sac->sac_error << ": "
                      << sctp::GetErrorString(
                             sctp::ToSctpError(sac->sac_error));
            } break;
            default:
              LOG(FATAL) << "Never seen state " << sac->sac_state << " before.";
              break;
          }
        } break;
      }

      if (VLOG_IS_ON(1)) {
        PrintNotification(message.get());
      }
    } break;
    case Message::kMessage:
      HandleData(message.get());
      break;
    case Message::kOverflow:
      NodeDisconnected();
      break;
  }
  client_.FreeMessage(std::move(message));
}

void SctpClientConnection::SendConnect() {
  ScheduleConnectTimeout();

  // If we're already connected, assume something went wrong and abort
  // the connection.
  if (client_status_->GetClientConnection(client_index_)->state() ==
      aos::message_bridge::State::CONNECTED) {
    client_.Abort();
    return;
  }
  // Try to send the connect message.  If that fails, retry.
  if (client_.Send(kConnectStream(),
                   std::string_view(reinterpret_cast<const char *>(
                                        connect_message_.span().data()),
                                    connect_message_.span().size()),
                   0)) {
    VLOG(1) << "Sending connect to " << remote_node_->hostname()->string_view()
            << " port " << remote_node_->port() << " succeeded.";
  } else {
    VLOG(1) << "Connect to " << remote_node_->hostname()->string_view()
            << " port " << remote_node_->port() << " failed.";
  }
}

void SctpClientConnection::NodeConnected(sctp_assoc_t assoc_id) {
  ScheduleConnectTimeout();

  // We want to tell the kernel to schedule the packets on this new stream with
  // the priority scheduler.  This only needs to be done once per stream.
  client_.SetPriorityScheduler(assoc_id);
  client_.SetAssociationId(assoc_id);

  client_status_->Connect(client_index_);
}

void SctpClientConnection::NodeDisconnected() {
  client_.SetAssociationId(0);

  connect_timer_->Schedule(
      event_loop_->monotonic_now() + chrono::milliseconds(100),
      chrono::milliseconds(100));

  // Don't try resending timestamps since there is nobody to send them to.  Let
  // Connect handle that instead.  HandleData will retry when they reply.
  timestamp_retry_buffer_->Disable();

  client_status_->Disconnect(client_index_);
  client_status_->SampleReset(client_index_);
}

void SctpClientConnection::HandleData(const Message *message) {
  ScheduleConnectTimeout();

  const RemoteData *remote_data =
      flatbuffers::GetSizePrefixedRoot<RemoteData>(message->data());

  VLOG(2) << "Got a message of size " << message->size;
  CHECK_EQ(message->size, flatbuffers::GetPrefixedSize(message->data()) +
                              sizeof(flatbuffers::uoffset_t));
  {
    flatbuffers::Verifier verifier(message->data(), message->size);
    CHECK(remote_data->Verify(verifier));
  }

  const int stream = message->header.rcvinfo.rcv_sid - kControlStreams();
  SctpClientChannelState *channel_state =
      &((*channels_)[stream_to_channel_[stream]]);

  if (remote_data->queue_index() == channel_state->last_queue_index &&
      monotonic_clock::time_point(
          chrono::nanoseconds(remote_data->monotonic_sent_time())) ==
          channel_state->last_timestamp) {
    VLOG(1) << "Duplicate message from " << message->PeerAddress();
    connection_->mutate_duplicate_packets(connection_->duplicate_packets() + 1);
    // Duplicate message, ignore.
  } else {
    connection_->mutate_received_packets(connection_->received_packets() + 1);
    connection_->mutate_partial_deliveries(connection_->partial_deliveries() +
                                           message->partial_deliveries);

    channel_state->last_queue_index = remote_data->queue_index();
    channel_state->last_timestamp = monotonic_clock::time_point(
        chrono::nanoseconds(remote_data->monotonic_sent_time()));

    // Publish the message.
    UUID remote_boot_uuid = UUID::FromVector(remote_data->boot_uuid());
    RawSender *sender = channel_state->sender.get();
    sender->CheckOk(sender->Send(
        remote_data->data()->data(), remote_data->data()->size(),
        monotonic_clock::time_point(
            chrono::nanoseconds(remote_data->monotonic_sent_time())),
        realtime_clock::time_point(
            chrono::nanoseconds(remote_data->realtime_sent_time())),
        monotonic_clock::time_point(
            chrono::nanoseconds(remote_data->monotonic_remote_transmit_time())),
        remote_data->queue_index(), remote_boot_uuid));
    VLOG(2) << "Sent "
            << configuration::StrippedChannelToString(
                   channel_state->sender->channel())
            << " -> {\"channel_index\": " << remote_data->channel_index()
            << ", \"monotonic_sent_time\": "
            << remote_data->monotonic_sent_time()
            << ", \"monotonic_remote_transmit_time\": "
            << remote_data->monotonic_remote_transmit_time()
            << ", \"realtime_sent_time\": " << remote_data->realtime_sent_time()
            << ", \"queue_index\": " << remote_data->queue_index()
            << ", \"monotonic_remote_time\": " << sender->monotonic_sent_time()
            << ", \"realtime_remote_time\": " << sender->realtime_sent_time()
            << ", \"remote_queue_index\": " << sender->sent_queue_index()
            << "}";

    client_status_->SampleFilter(
        client_index_,
        monotonic_clock::time_point(
            chrono::nanoseconds(remote_data->monotonic_remote_transmit_time())),
        sender->monotonic_sent_time(), remote_boot_uuid);

    if (stream_reply_with_timestamp_[stream]) {
      SavedTimestamp timestamp{
          .channel_index = remote_data->channel_index(),
          .monotonic_sent_time = remote_data->monotonic_sent_time(),
          .realtime_sent_time = remote_data->realtime_sent_time(),
          .queue_index = remote_data->queue_index(),
          .monotonic_remote_time =
              sender->monotonic_sent_time().time_since_epoch().count(),
          .monotonic_remote_transmit_time =
              remote_data->monotonic_remote_transmit_time(),
          .realtime_remote_time =
              sender->realtime_sent_time().time_since_epoch().count(),
          .remote_queue_index = sender->sent_queue_index(),
      };

      // Sort out if we should try to queue the timestamp or just send it
      // directly.  We don't want to get too far ahead.
      //
      // In this case, the other side rebooted.  There is no sense telling it
      // about previous timestamps.  Wipe them and reset which boot we are.
      if (remote_boot_uuid != timestamps_uuid_) {
        timestamp_buffer_.Reset();
        timestamps_uuid_ = remote_boot_uuid;
      }

      // Then, we only can send if there are no pending timestamps.
      bool push_timestamp = false;
      if (timestamp_buffer_.empty()) {
        if (!SendTimestamp(timestamp)) {
          // Whops, we failed to send, queue and try again.
          push_timestamp = true;
        }
      } else {
        push_timestamp = true;
      }

      if (push_timestamp) {
        // Trigger the timer if we are the first timestamp added, or if the
        // timer was disabled because the far side disconnected.
        if (timestamp_buffer_.empty() ||
            timestamp_retry_buffer_->IsDisabled()) {
          timestamp_retry_buffer_->Schedule(event_loop_->monotonic_now() +
                                            chrono::milliseconds(100));
        }
        VLOG(1) << this << " Queued timestamp " << timestamp.channel_index
                << " " << timestamp.queue_index;
        timestamp_buffer_.Push(timestamp);
      }
    }
  }

  VLOG(2) << "Received data of length " << message->size << " from "
          << message->PeerAddress();

  if (VLOG_IS_ON(2)) {
    client_.LogSctpStatus(message->header.rcvinfo.rcv_assoc_id);
  }

  VLOG(3) << "\tSNDRCV (stream=" << message->header.rcvinfo.rcv_sid
          << " ssn=" << message->header.rcvinfo.rcv_ssn
          << " tsn=" << message->header.rcvinfo.rcv_tsn << " flags=0x"
          << std::hex << message->header.rcvinfo.rcv_flags << std::dec
          << " ppid=" << message->header.rcvinfo.rcv_ppid
          << " cumtsn=" << message->header.rcvinfo.rcv_cumtsn << ")";
}

bool SctpClientConnection::SendTimestamp(SavedTimestamp timestamp) {
  // Now fill out the message received reply.  This uses a MessageHeader
  // container so it can be directly logged.
  message_reception_reply_.mutable_message()->mutate_channel_index(
      timestamp.channel_index);
  message_reception_reply_.mutable_message()->mutate_monotonic_sent_time(
      timestamp.monotonic_sent_time);
  message_reception_reply_.mutable_message()
      ->mutate_monotonic_remote_transmit_time(
          timestamp.monotonic_remote_transmit_time);
  message_reception_reply_.mutable_message()->mutate_realtime_sent_time(
      timestamp.realtime_sent_time);
  message_reception_reply_.mutable_message()->mutate_queue_index(
      timestamp.queue_index);

  // And capture the relevant data needed to generate the forwarding
  // MessageHeader.
  message_reception_reply_.mutable_message()->mutate_monotonic_remote_time(
      timestamp.monotonic_remote_time);
  message_reception_reply_.mutable_message()->mutate_realtime_remote_time(
      timestamp.realtime_remote_time);
  message_reception_reply_.mutable_message()->mutate_remote_queue_index(
      timestamp.remote_queue_index);

  // Unique ID is channel_index and monotonic clock.
  VLOG(1) << this << " Sent timestamp for channel " << timestamp.channel_index
          << ", queue index " << timestamp.queue_index;
  if (!client_.Send(
          kTimestampStream(),
          std::string_view(reinterpret_cast<const char *>(
                               message_reception_reply_.span().data()),
                           message_reception_reply_.span().size()),
          0)) {
    // Count the failure.
    connection_->mutate_timestamp_send_failures(
        connection_->timestamp_send_failures() + 1);
    return false;
  }
  return true;
}

void SctpClientConnection::SendTimestamps() {
  // This is only called from the timer, and the timer is only enabled when
  // there is something in the buffer.  Explode if that assumption is false.
  CHECK(!timestamp_buffer_.empty());
  do {
    if (!SendTimestamp(timestamp_buffer_[0])) {
      timestamp_retry_buffer_->Schedule(event_loop_->monotonic_now() +
                                        chrono::milliseconds(100));
      return;
    } else {
      VLOG(1) << this << " Resent timestamp "
              << timestamp_buffer_[0].channel_index << " "
              << timestamp_buffer_[0].queue_index;
      timestamp_buffer_.Shift();
    }
  } while (!timestamp_buffer_.empty());
}

MessageBridgeClient::MessageBridgeClient(
    aos::ShmEventLoop *event_loop, std::string config_sha256,
    SctpAuthMethod requested_authentication)
    : event_loop_(event_loop),
      client_status_(event_loop_),
      config_sha256_(std::move(config_sha256)),
      refresh_key_timer_(event_loop->AddTimer([this]() { RequestAuthKey(); })),
      sctp_config_request_(
          event_loop_->TryMakeSender<SctpConfigRequest>("/aos")) {
  std::string_view node_name = event_loop->node()->name()->string_view();

  // Set up the SCTP configuration watcher and timer.
  if (requested_authentication == SctpAuthMethod::kAuth && HasSctpAuth()) {
    CHECK(sctp_config_request_.valid())
        << ": Must have SctpConfig channel configured to use SCTP "
           "authentication.";
    event_loop->MakeWatcher("/aos", [this](const SctpConfig &config) {
      if (config.has_key()) {
        for (auto &conn : connections_) {
          conn->SetAuthKey(*config.key());
        }
      }
    });

    // We poll in case the SCTP authentication key has changed.
    refresh_key_timer_->set_name("refresh_key");
    event_loop_->OnRun([this]() {
      refresh_key_timer_->Schedule(event_loop_->monotonic_now(),
                                   kRefreshAuthKeyPeriod);
    });
  }

  // Find all the channels which are supposed to be delivered to us.
  channels_.resize(event_loop_->configuration()->channels()->size());
  int channel_index = 0;
  for (const Channel *channel : *event_loop_->configuration()->channels()) {
    if (channel->has_destination_nodes()) {
      for (const Connection *connection : *channel->destination_nodes()) {
        if (connection->name()->string_view() == node_name) {
          // Give the config a chance to remap us.  This helps with testing on a
          // single node.
          const Channel *mapped_channel = configuration::GetChannel(
              event_loop_->configuration(), channel->name()->string_view(),
              channel->type()->string_view(), event_loop_->name(),
              event_loop_->node());

          channels_[channel_index].sender =
              event_loop_->MakeRawSender(mapped_channel);

          std::unique_ptr<aos::RawFetcher> raw_fetcher =
              event_loop_->MakeRawFetcher(mapped_channel);
          raw_fetcher->Fetch();

          if (raw_fetcher->context().data != nullptr) {
            VLOG(1) << "Found data on "
                    << configuration::CleanedChannelToString(channel)
                    << ", won't resend it.";
            channels_[channel_index].last_queue_index =
                raw_fetcher->context().remote_queue_index;
            channels_[channel_index].last_timestamp =
                raw_fetcher->context().monotonic_remote_time;
          }

          break;
        }
      }
    }
    ++channel_index;
  }

  // Now, for each source node, build a connection.
  for (const std::string_view source_node : configuration::SourceNodeNames(
           event_loop->configuration(), event_loop->node())) {
    // Open an unspecified connection (:: in ipv6 terminology)
    connections_.emplace_back(new SctpClientConnection(
        event_loop, source_node, event_loop->node(), "", &channels_,
        client_status_.FindClientIndex(source_node), &client_status_,
        config_sha256_, requested_authentication));
  }
}

void MessageBridgeClient::RequestAuthKey() {
  CHECK(sctp_config_request_.valid());
  auto sender = sctp_config_request_.MakeBuilder();
  auto builder = sender.MakeBuilder<SctpConfigRequest>();
  builder.add_request_key(true);
  sender.CheckOk(sender.Send(builder.Finish()));
}

}  // namespace aos::message_bridge
