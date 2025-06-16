#include "aos/events/simulated_event_loop.h"

#include <chrono>
#include <functional>
#include <string_view>

#include "absl/flags/flag.h"
#include "absl/flags/reflection.h"
#include "gmock/gmock-matchers.h"
#include "gtest/gtest.h"

#include "aos/events/event_loop_param_test.h"
#include "aos/events/function_scheduler.h"
#include "aos/events/logging/logger_generated.h"
#include "aos/events/message_counter.h"
#include "aos/events/ping_lib.h"
#include "aos/events/pong_lib.h"
#include "aos/events/test_message_generated.h"
#include "aos/network/message_bridge_client_generated.h"
#include "aos/network/message_bridge_server_generated.h"
#include "aos/network/remote_message_generated.h"
#include "aos/network/testing_time_converter.h"
#include "aos/network/timestamp_generated.h"
#include "aos/scoped/scoped_fd.h"
#include "aos/testing/path.h"
#include "aos/testing/tmpdir.h"
#include "aos/util/file.h"

ABSL_DECLARE_FLAG(bool, use_simulated_clocks_for_logs);
ABSL_DECLARE_FLAG(std::string, vmodule);
ABSL_DECLARE_FLAG(bool, die_on_malloc);

namespace aos::testing {
namespace {

using aos::testing::ArtifactPath;

using logger::BootTimestamp;
using message_bridge::RemoteMessage;
namespace chrono = ::std::chrono;

}  // namespace

class SimulatedEventLoopTestFactory : public EventLoopTestFactory {
 public:
  ::std::unique_ptr<EventLoop> Make(std::string_view name) override {
    MaybeMake();
    return event_loop_factory_->MakeEventLoop(name, my_node());
  }
  ::std::unique_ptr<EventLoop> MakePrimary(std::string_view name) override {
    MaybeMake();
    return event_loop_factory_->MakeEventLoop(name, my_node());
  }

  Status Run() override { return event_loop_factory_->NonFatalRun(); }

  std::unique_ptr<ExitHandle> MakeExitHandle() override {
    MaybeMake();
    return event_loop_factory_->MakeExitHandle();
  }

  void Exit() override { event_loop_factory_->Exit(); }

  // TODO(austin): Implement this.  It's used currently for a phased loop test.
  // I'm not sure how much that matters.
  void SleepFor(::std::chrono::nanoseconds /*duration*/) override {}

  void set_send_delay(std::chrono::nanoseconds send_delay) {
    MaybeMake();
    event_loop_factory_->set_send_delay(send_delay);
  }

 private:
  void MaybeMake() {
    if (!event_loop_factory_) {
      if (configuration()->has_nodes()) {
        event_loop_factory_ =
            std::make_unique<SimulatedEventLoopFactory>(configuration());
      } else {
        event_loop_factory_ =
            std::make_unique<SimulatedEventLoopFactory>(configuration());
      }
    }
  }
  std::unique_ptr<SimulatedEventLoopFactory> event_loop_factory_;
};

auto CommonParameters() {
  return ::testing::Combine(
      ::testing::Values([]() { return new SimulatedEventLoopTestFactory(); }),
      ::testing::Values(ReadMethod::COPY, ReadMethod::PIN),
      ::testing::Values(DoTimingReports::kYes, DoTimingReports::kNo));
}

INSTANTIATE_TEST_SUITE_P(SimulatedEventLoopCommonTest, AbstractEventLoopTest,
                         CommonParameters());

INSTANTIATE_TEST_SUITE_P(SimulatedEventLoopCommonDeathTest,
                         AbstractEventLoopDeathTest, CommonParameters());

// Parameters to run all the tests with.
struct Param {
  // The config file to use.
  std::string config;
  // If true, the RemoteMessage channel should be shared between all the remote
  // channels.  If false, there will be 1 RemoteMessage channel per remote
  // channel.
  bool shared;
};

class RemoteMessageSimulatedEventLoopTest
    : public ::testing::TestWithParam<struct Param> {
 public:
  RemoteMessageSimulatedEventLoopTest()
      : config(aos::configuration::ReadConfig(
            ArtifactPath(absl::StrCat("aos/events/", GetParam().config)))) {
    LOG(INFO) << "Config " << GetParam().config;
  }

  bool shared() const { return GetParam().shared; }

  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
  MakePi2OnPi1MessageCounters(aos::EventLoop *event_loop) {
    std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>> counters;
    if (shared()) {
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop, "/aos/remote_timestamps/pi2"));
    } else {
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop,
          "/aos/remote_timestamps/pi2/pi1/aos/aos-message_bridge-Timestamp"));
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop, "/aos/remote_timestamps/pi2/test/aos-examples-Ping"));
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop, "/aos/remote_timestamps/pi2/reliable/aos-examples-Ping"));
    }
    return counters;
  }

  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
  MakePi1OnPi2MessageCounters(aos::EventLoop *event_loop) {
    std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>> counters;
    if (shared()) {
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop, "/aos/remote_timestamps/pi1"));
    } else {
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop, "/aos/remote_timestamps/pi1/test/aos-examples-Pong"));
      counters.emplace_back(std::make_unique<MessageCounter<RemoteMessage>>(
          event_loop,
          "/aos/remote_timestamps/pi1/pi2/aos/aos-message_bridge-Timestamp"));
    }
    return counters;
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config;
};

// Test that sending a message after running gets properly notified.
TEST(SimulatedEventLoopTest, SendAfterRunFor) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());

  ::std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping");
  aos::Sender<TestMessage> test_message_sender =
      ping_event_loop->MakeSender<TestMessage>("/test");
  ASSERT_EQ(SendTestMessage(test_message_sender), RawSender::Error::kOk);

  std::unique_ptr<EventLoop> pong1_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong");
  MessageCounter<TestMessage> test_message_counter1(pong1_event_loop.get(),
                                                    "/test");

  EXPECT_FALSE(ping_event_loop->is_running());

  // Watchers start when you start running, so there should be nothing counted.
  simulated_event_loop_factory.RunFor(chrono::seconds(1));
  EXPECT_EQ(test_message_counter1.count(), 0u);

  std::unique_ptr<EventLoop> pong2_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong");
  MessageCounter<TestMessage> test_message_counter2(pong2_event_loop.get(),
                                                    "/test");

  // Pauses in the middle don't count though, so this should be counted.
  // But, the fresh watcher shouldn't pick it up yet.
  ASSERT_EQ(SendTestMessage(test_message_sender), RawSender::Error::kOk);

  EXPECT_EQ(test_message_counter1.count(), 0u);
  EXPECT_EQ(test_message_counter2.count(), 0u);
  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_EQ(test_message_counter1.count(), 1u);
  EXPECT_EQ(test_message_counter2.count(), 0u);
}

// Test that OnRun callbacks get deleted if the event loop gets deleted.
TEST(SimulatedEventLoopTest, DestructEventLoopBeforeOnRun) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());

  {
    ::std::unique_ptr<EventLoop> test_event_loop =
        simulated_event_loop_factory.MakeEventLoop("test");
    test_event_loop->OnRun([]() { LOG(FATAL) << "Don't run this"; });
  }

  simulated_event_loop_factory.RunFor(chrono::seconds(1));
}

// Tests that the order event loops are created is the order that the OnRun
// callbacks are run.
TEST(SimulatedEventLoopTest, OnRunOrderFollowsConstructionOrder) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());

  int count = 0;

  std::unique_ptr<EventLoop> test1_event_loop =
      simulated_event_loop_factory.MakeEventLoop("test1");
  std::unique_ptr<EventLoop> test2_event_loop =
      simulated_event_loop_factory.MakeEventLoop("test2");
  test2_event_loop->OnRun([&count]() {
    EXPECT_EQ(count, 1u);
    ++count;
  });
  test1_event_loop->OnRun([&count]() {
    EXPECT_EQ(count, 0u);
    ++count;
  });

  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_EQ(count, 2u);
}

// Test that we can't register OnRun callbacks after starting.
TEST(SimulatedEventLoopDeathTest, OnRunAfterRunning) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());

  std::unique_ptr<EventLoop> test_event_loop =
      simulated_event_loop_factory.MakeEventLoop("test");
  test_event_loop->OnRun([]() {});

  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_DEATH(test_event_loop->OnRun([]() {}), "OnRun");
}

// Test that if we configure an event loop to be able to send too fast that we
// do allow it to do so.
TEST(SimulatedEventLoopTest, AllowSendTooFast) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());

  // Create two event loops: One will be allowed to send too fast, one won't. We
  // will then test to ensure that the one that is allowed to send too fast can
  // indeed send too fast, but that it then makes it so that the second event
  // loop can no longer send anything because *it* is still limited.
  ::std::unique_ptr<EventLoop> too_fast_event_loop =
      simulated_event_loop_factory.GetNodeEventLoopFactory(nullptr)
          ->MakeEventLoop("too_fast_sender",
                          {NodeEventLoopFactory::CheckSentTooFast::kNo,
                           NodeEventLoopFactory::ExclusiveSenders::kNo,
                           {}});
  aos::Sender<TestMessage> too_fast_message_sender =
      too_fast_event_loop->MakeSender<TestMessage>("/test");

  ::std::unique_ptr<EventLoop> limited_event_loop =
      simulated_event_loop_factory.MakeEventLoop("limited_sender");
  aos::Sender<TestMessage> limited_message_sender =
      limited_event_loop->MakeSender<TestMessage>("/test");

  const int queue_size = TestChannelQueueSize(too_fast_event_loop.get());
  for (int ii = 0; ii < queue_size; ++ii) {
    ASSERT_EQ(SendTestMessage(too_fast_message_sender), RawSender::Error::kOk);
  }
  // And now we should start being in the sending-too-fast phase.
  for (int ii = 0; ii < queue_size; ++ii) {
    ASSERT_EQ(SendTestMessage(too_fast_message_sender), RawSender::Error::kOk);
    ASSERT_EQ(SendTestMessage(limited_message_sender),
              RawSender::Error::kMessagesSentTooFast);
  }
}

// Test that if we setup an exclusive sender that it is indeed exclusive.
TEST(SimulatedEventLoopDeathTest, ExclusiveSenders) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());

  ::std::unique_ptr<EventLoop> exclusive_event_loop =
      simulated_event_loop_factory.GetNodeEventLoopFactory(nullptr)
          ->MakeEventLoop(
              "too_fast_sender",
              {NodeEventLoopFactory::CheckSentTooFast::kYes,
               NodeEventLoopFactory::ExclusiveSenders::kYes,
               {{configuration::GetChannel(factory.configuration(), "/test1",
                                           "aos.TestMessage", "", nullptr),
                 NodeEventLoopFactory::ExclusiveSenders::kNo}}});
  exclusive_event_loop->SkipAosLog();
  exclusive_event_loop->SkipTimingReport();
  ::std::unique_ptr<EventLoop> normal_event_loop =
      simulated_event_loop_factory.MakeEventLoop("limited_sender");
  // Set things up to have the exclusive sender be destroyed so we can test
  // recovery.
  {
    aos::Sender<TestMessage> exclusive_sender =
        exclusive_event_loop->MakeSender<TestMessage>("/test");

    EXPECT_DEATH(normal_event_loop->MakeSender<TestMessage>("/test"),
                 "TestMessage");
  }
  // This one should succeed now that the exclusive channel is removed.
  aos::Sender<TestMessage> normal_sender =
      normal_event_loop->MakeSender<TestMessage>("/test");
  EXPECT_DEATH(exclusive_event_loop->MakeSender<TestMessage>("/test"),
               "TestMessage");

  // And check an explicitly exempted channel:
  aos::Sender<TestMessage> non_exclusive_sender =
      exclusive_event_loop->MakeSender<TestMessage>("/test1");
  aos::Sender<TestMessage> non_exclusive_sender_regular_event_loop =
      normal_event_loop->MakeSender<TestMessage>("/test1");
}

void TestSentTooFastCheckEdgeCase(
    const std::function<RawSender::Error(int, int)> expected_err,
    const bool send_twice_at_end) {
  SimulatedEventLoopTestFactory factory;

  auto event_loop = factory.MakePrimary("primary");

  auto sender = event_loop->MakeSender<TestMessage>("/test");

  const int queue_size = TestChannelQueueSize(event_loop.get());
  int msgs_sent = 0;
  event_loop->AddPhasedLoop(
      [&](int) {
        EXPECT_EQ(SendTestMessage(sender), expected_err(msgs_sent, queue_size));
        msgs_sent++;

        // If send_twice_at_end, send the last two messages (message
        // queue_size and queue_size + 1) in the same iteration, meaning that
        // we would be sending very slightly too fast. Otherwise, we will send
        // message queue_size + 1 in the next iteration and we will continue
        // to be sending exactly at the channel frequency.
        if (send_twice_at_end && (msgs_sent == queue_size)) {
          EXPECT_EQ(SendTestMessage(sender),
                    expected_err(msgs_sent, queue_size));
          msgs_sent++;
        }

        if (msgs_sent > queue_size) {
          factory.Exit();
        }
      },
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(
              1.0 / TestChannelFrequency(event_loop.get()))));

  factory.Run();
}

// Tests that RawSender::Error::kMessagesSentTooFast is not returned
// when messages are sent at the exact frequency of the channel.
TEST(SimulatedEventLoopTest, SendingAtExactlyChannelFrequency) {
  TestSentTooFastCheckEdgeCase([](int, int) { return RawSender::Error::kOk; },
                               false);
}

// Tests that RawSender::Error::kMessagesSentTooFast is returned
// when sending exactly one more message than allowed in a channel storage
// duration.
TEST(SimulatedEventLoopTest, SendingSlightlyTooFast) {
  TestSentTooFastCheckEdgeCase(
      [](const int msgs_sent, const int queue_size) {
        return (msgs_sent == queue_size ? RawSender::Error::kMessagesSentTooFast
                                        : RawSender::Error::kOk);
      },
      true);
}

// Test that creating an event loop while running dies.
TEST(SimulatedEventLoopDeathTest, MakeEventLoopWhileRunning) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());

  ::std::unique_ptr<EventLoop> event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping");

  auto timer = event_loop->AddTimer([&]() {
    EXPECT_DEATH(
        {
          ::std::unique_ptr<EventLoop> event_loop2 =
              simulated_event_loop_factory.MakeEventLoop("ping");
        },
        "event loop while running");
    simulated_event_loop_factory.Exit();
  });

  event_loop->OnRun([&event_loop, &timer] {
    timer->Schedule(event_loop->monotonic_now() + chrono::milliseconds(50));
  });

  simulated_event_loop_factory.Run();
}

// Test that creating a watcher after running dies.
TEST(SimulatedEventLoopDeathTest, MakeWatcherAfterRunning) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());

  ::std::unique_ptr<EventLoop> event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping");

  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_DEATH(
      { MessageCounter<TestMessage> counter(event_loop.get(), "/test"); },
      "Can't add a watcher after running");

  ::std::unique_ptr<EventLoop> event_loop2 =
      simulated_event_loop_factory.MakeEventLoop("ping");

  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_DEATH(
      { MessageCounter<TestMessage> counter(event_loop2.get(), "/test"); },
      "Can't add a watcher after running");
}

// Test that running for a time period with no handlers causes time to progress
// correctly.
TEST(SimulatedEventLoopTest, RunForNoHandlers) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());
  ::std::unique_ptr<EventLoop> event_loop =
      simulated_event_loop_factory.MakeEventLoop("loop");

  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_EQ(::aos::monotonic_clock::epoch() + chrono::seconds(1),
            event_loop->monotonic_now());
}

// Test that running for a time with a periodic handler causes time to end
// correctly.
TEST(SimulatedEventLoopTest, RunForTimerHandler) {
  SimulatedEventLoopTestFactory factory;

  SimulatedEventLoopFactory simulated_event_loop_factory(
      factory.configuration());
  ::std::unique_ptr<EventLoop> event_loop =
      simulated_event_loop_factory.MakeEventLoop("loop");

  int counter = 0;
  auto timer = event_loop->AddTimer([&counter]() { ++counter; });
  event_loop->OnRun([&event_loop, &timer] {
    timer->Schedule(event_loop->monotonic_now() + chrono::milliseconds(50),
                    chrono::milliseconds(100));
  });

  simulated_event_loop_factory.RunFor(chrono::seconds(1));

  EXPECT_EQ(::aos::monotonic_clock::epoch() + chrono::seconds(1),
            event_loop->monotonic_now());
  EXPECT_EQ(counter, 10);
}

// Tests that watchers have latency in simulation.
TEST(SimulatedEventLoopTest, WatcherTimingReport) {
  absl::FlagSaver flag_saver;
  SimulatedEventLoopTestFactory factory;
  factory.set_send_delay(std::chrono::microseconds(50));

  absl::SetFlag(&FLAGS_timing_report_ms, 1000);
  auto loop1 = factory.MakePrimary("primary");
  loop1->MakeWatcher("/test", [](const TestMessage &) {});

  auto loop2 = factory.Make("sender_loop");

  auto loop3 = factory.Make("report_fetcher");

  Fetcher<timing::Report> report_fetcher =
      loop3->MakeFetcher<timing::Report>("/aos");
  EXPECT_FALSE(report_fetcher.Fetch());

  auto sender = loop2->MakeSender<TestMessage>("/test");

  // Send 10 messages in the middle of a timing report period so we get
  // something interesting back.
  auto test_timer = loop2->AddTimer([&sender]() {
    for (int i = 0; i < 10; ++i) {
      aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
      TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
      builder.add_value(200 + i);
      msg.CheckOk(msg.Send(builder.Finish()));
    }
  });

  // Quit after 1 timing report, mid way through the next cycle.
  {
    auto end_timer = loop1->AddTimer([&factory]() { factory.Exit(); });
    end_timer->Schedule(loop1->monotonic_now() + chrono::milliseconds(2500));
    end_timer->set_name("end");
  }

  loop1->OnRun([&test_timer, &loop1]() {
    test_timer->Schedule(loop1->monotonic_now() + chrono::milliseconds(1500));
  });

  factory.Run();

  // And, since we are here, check that the timing report makes sense.
  // Start by looking for our event loop's timing.
  FlatbufferDetachedBuffer<timing::Report> primary_report =
      FlatbufferDetachedBuffer<timing::Report>::Empty();
  while (report_fetcher.FetchNext()) {
    LOG(INFO) << "Report " << FlatbufferToJson(report_fetcher.get());
    if (report_fetcher->name()->string_view() == "primary") {
      primary_report = CopyFlatBuffer(report_fetcher.get());
    }
  }

  // Check the watcher report.
  VLOG(1) << FlatbufferToJson(primary_report, {.multi_line = true});

  EXPECT_EQ(primary_report.message().name()->string_view(), "primary");

  // Just the timing report timer.
  ASSERT_NE(primary_report.message().timers(), nullptr);
  EXPECT_EQ(primary_report.message().timers()->size(), 2);

  // No phased loops
  ASSERT_EQ(primary_report.message().phased_loops(), nullptr);

  // And now confirm that the watcher received all 10 messages, and has latency.
  ASSERT_NE(primary_report.message().watchers(), nullptr);
  ASSERT_EQ(primary_report.message().watchers()->size(), 1);
  EXPECT_EQ(primary_report.message().watchers()->Get(0)->count(), 10);
  EXPECT_NEAR(
      primary_report.message().watchers()->Get(0)->wakeup_latency()->average(),
      0.00005, 1e-9);
  EXPECT_NEAR(
      primary_report.message().watchers()->Get(0)->wakeup_latency()->min(),
      0.00005, 1e-9);
  EXPECT_NEAR(
      primary_report.message().watchers()->Get(0)->wakeup_latency()->max(),
      0.00005, 1e-9);
  EXPECT_EQ(primary_report.message()
                .watchers()
                ->Get(0)
                ->wakeup_latency()
                ->standard_deviation(),
            0.0);

  EXPECT_EQ(
      primary_report.message().watchers()->Get(0)->handler_time()->average(),
      0.0);
  EXPECT_EQ(primary_report.message().watchers()->Get(0)->handler_time()->min(),
            0.0);
  EXPECT_EQ(primary_report.message().watchers()->Get(0)->handler_time()->max(),
            0.0);
  EXPECT_EQ(primary_report.message()
                .watchers()
                ->Get(0)
                ->handler_time()
                ->standard_deviation(),
            0.0);
}

size_t CountAll(
    const std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
        &counters) {
  size_t count = 0u;
  for (const std::unique_ptr<MessageCounter<RemoteMessage>> &counter :
       counters) {
    count += counter->count();
  }
  return count;
}

// Tests that ping and pong work when on 2 different nodes, and the message
// gateway messages are sent out as expected.
TEST_P(RemoteMessageSimulatedEventLoopTest, MultinodePingPong) {
  const Node *pi1 = configuration::GetNode(&config.message(), "pi1");
  const Node *pi2 = configuration::GetNode(&config.message(), "pi2");
  const Node *pi3 = configuration::GetNode(&config.message(), "pi3");

  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());

  std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping", pi1);
  Ping ping(ping_event_loop.get());

  std::unique_ptr<EventLoop> pong_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong", pi2);
  Pong pong(pong_event_loop.get());

  std::unique_ptr<EventLoop> pi2_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi2_pong_counter", pi2);
  MessageCounter<examples::Pong> pi2_pong_counter(
      pi2_pong_counter_event_loop.get(), "/test");
  aos::Fetcher<message_bridge::Timestamp> pi1_on_pi2_timestamp_fetcher =
      pi2_pong_counter_event_loop->MakeFetcher<message_bridge::Timestamp>(
          "/pi1/aos");
  aos::Fetcher<examples::Ping> ping_on_pi2_fetcher =
      pi2_pong_counter_event_loop->MakeFetcher<examples::Ping>("/test");

  std::unique_ptr<EventLoop> pi3_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi3_pong_counter", pi3);

  std::unique_ptr<EventLoop> pi1_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi1_pong_counter", pi1);
  MessageCounter<examples::Pong> pi1_pong_counter(
      pi1_pong_counter_event_loop.get(), "/test");
  aos::Fetcher<examples::Ping> ping_on_pi1_fetcher =
      pi1_pong_counter_event_loop->MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<message_bridge::Timestamp> pi1_on_pi1_timestamp_fetcher =
      pi1_pong_counter_event_loop->MakeFetcher<message_bridge::Timestamp>(
          "/aos");

  // Count timestamps.
  MessageCounter<message_bridge::Timestamp> pi1_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi1_on_pi2_timestamp_counter(
      pi2_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi1_on_pi3_timestamp_counter(
      pi3_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi2_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi2/aos");
  MessageCounter<message_bridge::Timestamp> pi2_on_pi2_timestamp_counter(
      pi2_pong_counter_event_loop.get(), "/pi2/aos");
  MessageCounter<message_bridge::Timestamp> pi3_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi3/aos");
  MessageCounter<message_bridge::Timestamp> pi3_on_pi3_timestamp_counter(
      pi3_pong_counter_event_loop.get(), "/pi3/aos");

  // Count remote timestamps
  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
      remote_timestamps_pi2_on_pi1 =
          MakePi2OnPi1MessageCounters(pi1_pong_counter_event_loop.get());
  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
      remote_timestamps_pi1_on_pi2 =
          MakePi1OnPi2MessageCounters(pi2_pong_counter_event_loop.get());

  // Wait to let timestamp estimation start up before looking for the results.
  simulated_event_loop_factory.RunFor(chrono::milliseconds(500));

  std::unique_ptr<EventLoop> pi1_statistics_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi1_statistics_counter", pi1);
  std::unique_ptr<EventLoop> pi2_statistics_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi2_statistics_counter", pi2);
  std::unique_ptr<EventLoop> pi3_statistics_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi3_statistics_counter", pi3);

  int pi1_server_statistics_count = 0;
  pi1_statistics_counter_event_loop->MakeWatcher(
      "/pi1/aos", [&pi1_server_statistics_count](
                      const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi1 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 2u);
        for (const message_bridge::ServerConnection *connection :
             *stats.connections()) {
          EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
          EXPECT_EQ(connection->connection_count(), 1u);
          EXPECT_EQ(connection->connected_since_time(), 0);
          EXPECT_TRUE(connection->has_boot_uuid());
          if (connection->node()->name()->string_view() == "pi2") {
            EXPECT_GT(connection->sent_packets(), 50);
          } else if (connection->node()->name()->string_view() == "pi3") {
            EXPECT_GE(connection->sent_packets(), 5);
          } else {
            LOG(FATAL) << "Unknown connection";
          }

          EXPECT_TRUE(connection->has_monotonic_offset());
          EXPECT_EQ(connection->monotonic_offset(), 0);

          EXPECT_TRUE(connection->has_channels());
          int accumulated_sent_count = 0;
          int accumulated_dropped_count = 0;
          for (const message_bridge::ServerChannelStatistics *channel :
               *connection->channels()) {
            accumulated_sent_count += channel->sent_packets();
            accumulated_dropped_count += channel->dropped_packets();
          }
          EXPECT_EQ(connection->sent_packets(), accumulated_sent_count);
          EXPECT_EQ(connection->dropped_packets(), accumulated_dropped_count);
        }
        ++pi1_server_statistics_count;
      });

  int pi2_server_statistics_count = 0;
  pi2_statistics_counter_event_loop->MakeWatcher(
      "/pi2/aos", [&pi2_server_statistics_count](
                      const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi2 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ServerConnection *connection =
            stats.connections()->Get(0);
        EXPECT_TRUE(connection->has_boot_uuid());
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_GT(connection->sent_packets(), 50);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 0);
        EXPECT_EQ(connection->connection_count(), 1u);
        EXPECT_EQ(connection->connected_since_time(), 0);

        EXPECT_TRUE(connection->has_channels());
        int accumulated_sent_count = 0;
        int accumulated_dropped_count = 0;
        for (const message_bridge::ServerChannelStatistics *channel :
             *connection->channels()) {
          accumulated_sent_count += channel->sent_packets();
          accumulated_dropped_count += channel->dropped_packets();
        }
        EXPECT_EQ(connection->sent_packets(), accumulated_sent_count);
        EXPECT_EQ(connection->dropped_packets(), accumulated_dropped_count);

        ++pi2_server_statistics_count;
      });

  int pi3_server_statistics_count = 0;
  pi3_statistics_counter_event_loop->MakeWatcher(
      "/pi3/aos", [&pi3_server_statistics_count](
                      const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi3 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ServerConnection *connection =
            stats.connections()->Get(0);
        EXPECT_TRUE(connection->has_boot_uuid());
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_GE(connection->sent_packets(), 5);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 0);
        EXPECT_EQ(connection->connection_count(), 1u);
        EXPECT_EQ(connection->connected_since_time(), 0);

        EXPECT_TRUE(connection->has_channels());
        int accumulated_sent_count = 0;
        int accumulated_dropped_count = 0;
        for (const message_bridge::ServerChannelStatistics *channel :
             *connection->channels()) {
          accumulated_sent_count += channel->sent_packets();
          accumulated_dropped_count += channel->dropped_packets();
        }
        EXPECT_EQ(connection->sent_packets(), accumulated_sent_count);
        EXPECT_EQ(connection->dropped_packets(), accumulated_dropped_count);

        ++pi3_server_statistics_count;
      });

  int pi1_client_statistics_count = 0;
  pi1_statistics_counter_event_loop->MakeWatcher(
      "/pi1/aos", [&pi1_client_statistics_count](
                      const message_bridge::ClientStatistics &stats) {
        VLOG(1) << "pi1 ClientStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 2u);

        for (const message_bridge::ClientConnection *connection :
             *stats.connections()) {
          EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
          if (connection->node()->name()->string_view() == "pi2") {
            EXPECT_GT(connection->received_packets(), 50);
          } else if (connection->node()->name()->string_view() == "pi3") {
            EXPECT_GE(connection->received_packets(), 5);
          } else {
            LOG(FATAL) << "Unknown connection";
          }

          EXPECT_EQ(connection->partial_deliveries(), 0);
          EXPECT_TRUE(connection->has_monotonic_offset());
          EXPECT_EQ(connection->monotonic_offset(), 100000);
          EXPECT_EQ(connection->connection_count(), 1u);
          EXPECT_EQ(connection->connected_since_time(), 0);
        }
        ++pi1_client_statistics_count;
      });

  int pi2_client_statistics_count = 0;
  pi2_statistics_counter_event_loop->MakeWatcher(
      "/pi2/aos", [&pi2_client_statistics_count](
                      const message_bridge::ClientStatistics &stats) {
        VLOG(1) << "pi2 ClientStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ClientConnection *connection =
            stats.connections()->Get(0);
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_GT(connection->received_packets(), 50);
        EXPECT_EQ(connection->partial_deliveries(), 0);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 100000);
        EXPECT_EQ(connection->connection_count(), 1u);
        EXPECT_EQ(connection->connected_since_time(), 0);
        ++pi2_client_statistics_count;
      });

  int pi3_client_statistics_count = 0;
  pi3_statistics_counter_event_loop->MakeWatcher(
      "/pi3/aos", [&pi3_client_statistics_count](
                      const message_bridge::ClientStatistics &stats) {
        VLOG(1) << "pi3 ClientStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ClientConnection *connection =
            stats.connections()->Get(0);
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_GE(connection->received_packets(), 5);
        EXPECT_EQ(connection->partial_deliveries(), 0);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 100000);
        EXPECT_EQ(connection->connection_count(), 1u);
        EXPECT_EQ(connection->connected_since_time(), 0);
        ++pi3_client_statistics_count;
      });

  // Find the channel index for both the /pi1/aos Timestamp channel and Ping
  // channel.
  const size_t pi1_timestamp_channel =
      configuration::ChannelIndex(pi1_pong_counter_event_loop->configuration(),
                                  pi1_on_pi2_timestamp_fetcher.channel());
  const size_t ping_timestamp_channel =
      configuration::ChannelIndex(pi1_pong_counter_event_loop->configuration(),
                                  ping_on_pi2_fetcher.channel());

  for (const Channel *channel :
       *pi1_pong_counter_event_loop->configuration()->channels()) {
    VLOG(1) << "Channel "
            << configuration::ChannelIndex(
                   pi1_pong_counter_event_loop->configuration(), channel)
            << " " << configuration::CleanedChannelToString(channel);
  }

  std::unique_ptr<EventLoop> pi1_remote_timestamp =
      simulated_event_loop_factory.MakeEventLoop("pi1_remote_timestamp", pi1);

  for (std::pair<int, std::string> channel :
       shared()
           ? std::vector<std::pair<
                 int, std::string>>{{-1, "/pi1/aos/remote_timestamps/pi2"}}
           : std::vector<std::pair<int, std::string>>{
                 {pi1_timestamp_channel,
                  "/pi1/aos/remote_timestamps/pi2/pi1/aos/"
                  "aos-message_bridge-Timestamp"},
                 {ping_timestamp_channel,
                  "/pi1/aos/remote_timestamps/pi2/test/aos-examples-Ping"}}) {
    // For each remote timestamp we get back, confirm that it is either a ping
    // message, or a timestamp we sent out.  Also confirm that the timestamps
    // are correct.
    pi1_remote_timestamp->MakeWatcher(
        channel.second,
        [pi1_timestamp_channel, ping_timestamp_channel, &ping_on_pi2_fetcher,
         &ping_on_pi1_fetcher, &pi1_on_pi2_timestamp_fetcher,
         &pi1_on_pi1_timestamp_fetcher, &simulated_event_loop_factory, pi2,
         channel_index = channel.first,
         channel_name = channel.second](const RemoteMessage &header) {
          VLOG(1) << channel_name << " aos::message_bridge::RemoteMessage -> "
                  << aos::FlatbufferToJson(&header);
          EXPECT_TRUE(header.has_boot_uuid());
          EXPECT_EQ(UUID::FromVector(header.boot_uuid()),
                    simulated_event_loop_factory.GetNodeEventLoopFactory(pi2)
                        ->boot_uuid());

          const aos::monotonic_clock::time_point header_monotonic_sent_time(
              chrono::nanoseconds(header.monotonic_sent_time()));
          const aos::realtime_clock::time_point header_realtime_sent_time(
              chrono::nanoseconds(header.realtime_sent_time()));
          const aos::monotonic_clock::time_point header_monotonic_remote_time(
              chrono::nanoseconds(header.monotonic_remote_time()));
          const aos::monotonic_clock::time_point
              header_monotonic_remote_transmit_time(
                  chrono::nanoseconds(header.monotonic_remote_transmit_time()));
          const aos::realtime_clock::time_point header_realtime_remote_time(
              chrono::nanoseconds(header.realtime_remote_time()));

          if (channel_index != -1) {
            ASSERT_EQ(channel_index, header.channel_index());
          }

          const Context *pi1_context = nullptr;
          const Context *pi2_context = nullptr;

          if (header.channel_index() == pi1_timestamp_channel) {
            // Find the forwarded message.
            while (pi1_on_pi2_timestamp_fetcher.context().monotonic_event_time <
                   header_monotonic_sent_time) {
              ASSERT_TRUE(pi1_on_pi2_timestamp_fetcher.FetchNext());
            }

            // And the source message.
            while (pi1_on_pi1_timestamp_fetcher.context().monotonic_event_time <
                   header_monotonic_remote_time) {
              ASSERT_TRUE(pi1_on_pi1_timestamp_fetcher.FetchNext());
            }

            pi1_context = &pi1_on_pi1_timestamp_fetcher.context();
            pi2_context = &pi1_on_pi2_timestamp_fetcher.context();

            EXPECT_EQ(header_monotonic_remote_transmit_time,
                      pi2_context->monotonic_remote_time);
          } else if (header.channel_index() == ping_timestamp_channel) {
            // Find the forwarded message.
            while (ping_on_pi2_fetcher.context().monotonic_event_time <
                   header_monotonic_sent_time) {
              ASSERT_TRUE(ping_on_pi2_fetcher.FetchNext());
            }

            // And the source message.
            while (ping_on_pi1_fetcher.context().monotonic_event_time <
                   header_monotonic_remote_time) {
              ASSERT_TRUE(ping_on_pi1_fetcher.FetchNext());
            }

            pi1_context = &ping_on_pi1_fetcher.context();
            pi2_context = &ping_on_pi2_fetcher.context();

            EXPECT_EQ(header_monotonic_remote_transmit_time,
                      pi2_context->monotonic_event_time -
                          simulated_event_loop_factory.network_delay());
          } else {
            LOG(FATAL) << "Unknown channel";
          }

          // Confirm the forwarded message has matching timestamps to the
          // timestamps we got back.
          EXPECT_EQ(pi2_context->queue_index, header.queue_index());
          EXPECT_EQ(pi2_context->remote_queue_index,
                    header.remote_queue_index());
          EXPECT_EQ(pi2_context->monotonic_event_time,
                    header_monotonic_sent_time);
          EXPECT_EQ(pi2_context->realtime_event_time,
                    header_realtime_sent_time);
          EXPECT_EQ(pi2_context->realtime_remote_time,
                    header_realtime_remote_time);
          EXPECT_EQ(pi2_context->monotonic_remote_time,
                    header_monotonic_remote_time);
          EXPECT_EQ(pi2_context->monotonic_remote_transmit_time,
                    header_monotonic_remote_transmit_time);

          // Confirm the forwarded message also matches the source message.
          EXPECT_EQ(pi1_context->queue_index, header.remote_queue_index());
          EXPECT_EQ(pi1_context->monotonic_event_time,
                    header_monotonic_remote_time);
          EXPECT_EQ(pi1_context->realtime_event_time,
                    header_realtime_remote_time);
        });
  }

  simulated_event_loop_factory.RunFor(chrono::seconds(10) -
                                      chrono::milliseconds(500) +
                                      chrono::milliseconds(5));

  EXPECT_EQ(pi1_pong_counter.count(), 1001);
  EXPECT_EQ(pi2_pong_counter.count(), 1001);

  EXPECT_EQ(pi1_on_pi1_timestamp_counter.count(), 100);
  EXPECT_EQ(pi1_on_pi2_timestamp_counter.count(), 100);
  EXPECT_EQ(pi1_on_pi3_timestamp_counter.count(), 100);
  EXPECT_EQ(pi2_on_pi1_timestamp_counter.count(), 100);
  EXPECT_EQ(pi2_on_pi2_timestamp_counter.count(), 100);
  EXPECT_EQ(pi3_on_pi1_timestamp_counter.count(), 100);
  EXPECT_EQ(pi3_on_pi3_timestamp_counter.count(), 100);

  EXPECT_EQ(pi1_server_statistics_count, 10);
  EXPECT_EQ(pi2_server_statistics_count, 10);
  EXPECT_EQ(pi3_server_statistics_count, 10);

  EXPECT_EQ(pi1_client_statistics_count, 95);
  EXPECT_EQ(pi2_client_statistics_count, 95);
  EXPECT_EQ(pi3_client_statistics_count, 95);

  // Also confirm that remote timestamps are being forwarded correctly.
  EXPECT_EQ(CountAll(remote_timestamps_pi2_on_pi1), 1101);
  EXPECT_EQ(CountAll(remote_timestamps_pi1_on_pi2), 1101);
}

// Tests that an offset between nodes can be recovered and shows up in
// ServerStatistics correctly.
TEST(SimulatedEventLoopTest, MultinodePingPongWithOffset) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(ArtifactPath(
          "aos/events/multinode_pingpong_test_combined_config.json"));
  const Node *pi1 = configuration::GetNode(&config.message(), "pi1");
  const size_t pi1_index = configuration::GetNodeIndex(&config.message(), pi1);
  ASSERT_EQ(pi1_index, 0u);
  const Node *pi2 = configuration::GetNode(&config.message(), "pi2");
  const size_t pi2_index = configuration::GetNodeIndex(&config.message(), pi2);
  ASSERT_EQ(pi2_index, 1u);
  const Node *pi3 = configuration::GetNode(&config.message(), "pi3");
  const size_t pi3_index = configuration::GetNodeIndex(&config.message(), pi3);
  ASSERT_EQ(pi3_index, 2u);

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());
  simulated_event_loop_factory.SetTimeConverter(&time);

  constexpr chrono::milliseconds kOffset{1501};
  time.AddNextTimestamp(
      distributed_clock::epoch(),
      {BootTimestamp::epoch(), BootTimestamp::epoch() + kOffset,
       BootTimestamp::epoch()});

  std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping", pi1);
  Ping ping(ping_event_loop.get());

  std::unique_ptr<EventLoop> pong_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong", pi2);
  Pong pong(pong_event_loop.get());

  // Wait to let timestamp estimation start up before looking for the results.
  simulated_event_loop_factory.RunFor(chrono::milliseconds(500));

  std::unique_ptr<EventLoop> pi1_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi1_pong_counter", pi1);

  std::unique_ptr<EventLoop> pi2_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi2_pong_counter", pi2);

  std::unique_ptr<EventLoop> pi3_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi3_pong_counter", pi3);

  // Confirm the offsets are being recovered correctly.
  int pi1_server_statistics_count = 0;
  pi1_pong_counter_event_loop->MakeWatcher(
      "/pi1/aos", [&pi1_server_statistics_count,
                   kOffset](const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi1 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 2u);
        for (const message_bridge::ServerConnection *connection :
             *stats.connections()) {
          EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
          EXPECT_TRUE(connection->has_boot_uuid());
          if (connection->node()->name()->string_view() == "pi2") {
            EXPECT_EQ(connection->monotonic_offset(),
                      chrono::nanoseconds(kOffset).count());
          } else if (connection->node()->name()->string_view() == "pi3") {
            EXPECT_EQ(connection->monotonic_offset(), 0);
          } else {
            LOG(FATAL) << "Unknown connection";
          }

          EXPECT_TRUE(connection->has_monotonic_offset());
        }
        ++pi1_server_statistics_count;
      });

  int pi2_server_statistics_count = 0;
  pi2_pong_counter_event_loop->MakeWatcher(
      "/pi2/aos", [&pi2_server_statistics_count,
                   kOffset](const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi2 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ServerConnection *connection =
            stats.connections()->Get(0);
        EXPECT_TRUE(connection->has_boot_uuid());
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(),
                  -chrono::nanoseconds(kOffset).count());
        ++pi2_server_statistics_count;
      });

  int pi3_server_statistics_count = 0;
  pi3_pong_counter_event_loop->MakeWatcher(
      "/pi3/aos", [&pi3_server_statistics_count](
                      const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi3 ServerStatistics " << FlatbufferToJson(&stats);
        EXPECT_EQ(stats.connections()->size(), 1u);

        const message_bridge::ServerConnection *connection =
            stats.connections()->Get(0);
        EXPECT_TRUE(connection->has_boot_uuid());
        EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
        EXPECT_TRUE(connection->has_monotonic_offset());
        EXPECT_EQ(connection->monotonic_offset(), 0);
        ++pi3_server_statistics_count;
      });

  simulated_event_loop_factory.RunFor(chrono::seconds(10) -
                                      chrono::milliseconds(500) +
                                      chrono::milliseconds(5));

  EXPECT_EQ(pi1_server_statistics_count, 10);
  EXPECT_EQ(pi2_server_statistics_count, 10);
  EXPECT_EQ(pi3_server_statistics_count, 10);
}

// Test that disabling statistics actually disables them.
TEST_P(RemoteMessageSimulatedEventLoopTest, MultinodeWithoutStatistics) {
  const Node *pi1 = configuration::GetNode(&config.message(), "pi1");
  const Node *pi2 = configuration::GetNode(&config.message(), "pi2");
  const Node *pi3 = configuration::GetNode(&config.message(), "pi3");

  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());
  simulated_event_loop_factory.DisableStatistics();

  std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping", pi1);
  Ping ping(ping_event_loop.get());

  std::unique_ptr<EventLoop> pong_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong", pi2);
  Pong pong(pong_event_loop.get());

  std::unique_ptr<EventLoop> pi2_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi2_pong_counter", pi2);

  MessageCounter<examples::Pong> pi2_pong_counter(
      pi2_pong_counter_event_loop.get(), "/test");

  std::unique_ptr<EventLoop> pi3_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi3_pong_counter", pi3);

  std::unique_ptr<EventLoop> pi1_pong_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi1_pong_counter", pi1);

  MessageCounter<examples::Pong> pi1_pong_counter(
      pi1_pong_counter_event_loop.get(), "/test");

  // Count timestamps.
  MessageCounter<message_bridge::Timestamp> pi1_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi1_on_pi2_timestamp_counter(
      pi2_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi1_on_pi3_timestamp_counter(
      pi3_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi2_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi2/aos");
  MessageCounter<message_bridge::Timestamp> pi2_on_pi2_timestamp_counter(
      pi2_pong_counter_event_loop.get(), "/pi2/aos");
  MessageCounter<message_bridge::Timestamp> pi3_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi3/aos");
  MessageCounter<message_bridge::Timestamp> pi3_on_pi3_timestamp_counter(
      pi3_pong_counter_event_loop.get(), "/pi3/aos");

  // Count remote timestamps
  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
      remote_timestamps_pi2_on_pi1 =
          MakePi2OnPi1MessageCounters(pi1_pong_counter_event_loop.get());
  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
      remote_timestamps_pi1_on_pi2 =
          MakePi1OnPi2MessageCounters(pi2_pong_counter_event_loop.get());

  MessageCounter<message_bridge::ServerStatistics>
      pi1_server_statistics_counter(pi1_pong_counter_event_loop.get(),
                                    "/pi1/aos");
  MessageCounter<message_bridge::ServerStatistics>
      pi2_server_statistics_counter(pi2_pong_counter_event_loop.get(),
                                    "/pi2/aos");
  MessageCounter<message_bridge::ServerStatistics>
      pi3_server_statistics_counter(pi3_pong_counter_event_loop.get(),
                                    "/pi3/aos");

  MessageCounter<message_bridge::ClientStatistics>
      pi1_client_statistics_counter(pi1_pong_counter_event_loop.get(),
                                    "/pi1/aos");
  MessageCounter<message_bridge::ClientStatistics>
      pi2_client_statistics_counter(pi2_pong_counter_event_loop.get(),
                                    "/pi2/aos");
  MessageCounter<message_bridge::ClientStatistics>
      pi3_client_statistics_counter(pi3_pong_counter_event_loop.get(),
                                    "/pi3/aos");

  simulated_event_loop_factory.RunFor(chrono::seconds(10) +
                                      chrono::milliseconds(5));

  EXPECT_EQ(pi1_pong_counter.count(), 1001u);
  EXPECT_EQ(pi2_pong_counter.count(), 1001u);

  EXPECT_EQ(pi1_on_pi1_timestamp_counter.count(), 0u);
  EXPECT_EQ(pi1_on_pi2_timestamp_counter.count(), 0u);
  EXPECT_EQ(pi1_on_pi3_timestamp_counter.count(), 0u);
  EXPECT_EQ(pi2_on_pi1_timestamp_counter.count(), 0u);
  EXPECT_EQ(pi2_on_pi2_timestamp_counter.count(), 0u);
  EXPECT_EQ(pi3_on_pi1_timestamp_counter.count(), 0u);
  EXPECT_EQ(pi3_on_pi3_timestamp_counter.count(), 0u);

  EXPECT_EQ(pi1_server_statistics_counter.count(), 0u);
  EXPECT_EQ(pi2_server_statistics_counter.count(), 0u);
  EXPECT_EQ(pi3_server_statistics_counter.count(), 0u);

  EXPECT_EQ(pi1_client_statistics_counter.count(), 0u);
  EXPECT_EQ(pi2_client_statistics_counter.count(), 0u);
  EXPECT_EQ(pi3_client_statistics_counter.count(), 0u);

  // Also confirm that remote timestamps are being forwarded correctly.
  EXPECT_EQ(CountAll(remote_timestamps_pi2_on_pi1), 1001);
  EXPECT_EQ(CountAll(remote_timestamps_pi1_on_pi2), 1001);
}

bool AllConnected(const message_bridge::ServerStatistics *server_statistics) {
  for (const message_bridge::ServerConnection *connection :
       *server_statistics->connections()) {
    if (connection->state() != message_bridge::State::CONNECTED) {
      return false;
    }
  }
  return true;
}

bool AllConnectedBut(const message_bridge::ServerStatistics *server_statistics,
                     std::string_view target) {
  for (const message_bridge::ServerConnection *connection :
       *server_statistics->connections()) {
    if (connection->node()->name()->string_view() == target) {
      if (connection->state() == message_bridge::State::CONNECTED) {
        return false;
      }
    } else {
      if (connection->state() != message_bridge::State::CONNECTED) {
        return false;
      }
    }
  }
  return true;
}

bool AllConnected(const message_bridge::ClientStatistics *client_statistics) {
  for (const message_bridge::ClientConnection *connection :
       *client_statistics->connections()) {
    if (connection->state() != message_bridge::State::CONNECTED) {
      return false;
    }
    EXPECT_TRUE(connection->has_boot_uuid());
    EXPECT_TRUE(connection->has_connected_since_time());
    EXPECT_TRUE(connection->has_connection_count());
  }
  return true;
}

bool AllConnectedBut(const message_bridge::ClientStatistics *client_statistics,
                     std::string_view target) {
  for (const message_bridge::ClientConnection *connection :
       *client_statistics->connections()) {
    if (connection->node()->name()->string_view() == target) {
      if (connection->state() == message_bridge::State::CONNECTED) {
        return false;
      }
      EXPECT_FALSE(connection->has_boot_uuid());
      EXPECT_FALSE(connection->has_connected_since_time());
    } else {
      if (connection->state() != message_bridge::State::CONNECTED) {
        return false;
      }
      EXPECT_TRUE(connection->has_boot_uuid());
      EXPECT_TRUE(connection->has_connected_since_time());
      EXPECT_TRUE(connection->has_connection_count());
    }
  }
  return true;
}

int ConnectedCount(const message_bridge::ClientStatistics *client_statistics,
                   std::string_view target) {
  for (const message_bridge::ClientConnection *connection :
       *client_statistics->connections()) {
    if (connection->node()->name()->string_view() == target) {
      return connection->connection_count();
    }
  }
  return 0;
}

int ConnectedCount(const message_bridge::ServerStatistics *server_statistics,
                   std::string_view target) {
  for (const message_bridge::ServerConnection *connection :
       *server_statistics->connections()) {
    if (connection->node()->name()->string_view() == target) {
      return connection->connection_count();
    }
  }
  return 0;
}

// Test that disconnecting nodes actually disconnects them.
TEST_P(RemoteMessageSimulatedEventLoopTest, MultinodeDisconnect) {
  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());

  NodeEventLoopFactory *pi1 =
      simulated_event_loop_factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 =
      simulated_event_loop_factory.GetNodeEventLoopFactory("pi2");
  NodeEventLoopFactory *pi3 =
      simulated_event_loop_factory.GetNodeEventLoopFactory("pi3");

  std::unique_ptr<EventLoop> ping_event_loop = pi1->MakeEventLoop("ping");
  Ping ping(ping_event_loop.get());

  std::unique_ptr<EventLoop> pong_event_loop = pi2->MakeEventLoop("pong");
  Pong pong(pong_event_loop.get());

  std::unique_ptr<EventLoop> pi2_pong_counter_event_loop =
      pi2->MakeEventLoop("pi2_pong_counter");

  MessageCounter<examples::Pong> pi2_pong_counter(
      pi2_pong_counter_event_loop.get(), "/test");

  std::unique_ptr<EventLoop> pi3_pong_counter_event_loop =
      pi3->MakeEventLoop("pi3_pong_counter");

  std::unique_ptr<EventLoop> pi1_pong_counter_event_loop =
      pi1->MakeEventLoop("pi1_pong_counter");

  MessageCounter<examples::Pong> pi1_pong_counter(
      pi1_pong_counter_event_loop.get(), "/test");

  // Count timestamps.
  MessageCounter<message_bridge::Timestamp> pi1_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi1_on_pi2_timestamp_counter(
      pi2_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi1_on_pi3_timestamp_counter(
      pi3_pong_counter_event_loop.get(), "/pi1/aos");
  MessageCounter<message_bridge::Timestamp> pi2_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi2/aos");
  MessageCounter<message_bridge::Timestamp> pi2_on_pi2_timestamp_counter(
      pi2_pong_counter_event_loop.get(), "/pi2/aos");
  MessageCounter<message_bridge::Timestamp> pi3_on_pi1_timestamp_counter(
      pi1_pong_counter_event_loop.get(), "/pi3/aos");
  MessageCounter<message_bridge::Timestamp> pi3_on_pi3_timestamp_counter(
      pi3_pong_counter_event_loop.get(), "/pi3/aos");

  // Count remote timestamps
  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
      remote_timestamps_pi2_on_pi1 =
          MakePi2OnPi1MessageCounters(pi1_pong_counter_event_loop.get());
  std::vector<std::unique_ptr<MessageCounter<RemoteMessage>>>
      remote_timestamps_pi1_on_pi2 =
          MakePi1OnPi2MessageCounters(pi2_pong_counter_event_loop.get());

  MessageCounter<message_bridge::ServerStatistics>
      *pi1_server_statistics_counter;
  pi1->OnStartup([pi1, &pi1_server_statistics_counter]() {
    pi1_server_statistics_counter =
        pi1->AlwaysStart<MessageCounter<message_bridge::ServerStatistics>>(
            "pi1_server_statistics_counter", "/pi1/aos");
  });

  aos::Fetcher<message_bridge::ServerStatistics> pi1_server_statistics_fetcher =
      pi1_pong_counter_event_loop
          ->MakeFetcher<message_bridge::ServerStatistics>("/pi1/aos");
  aos::Fetcher<message_bridge::ClientStatistics> pi1_client_statistics_fetcher =
      pi1_pong_counter_event_loop
          ->MakeFetcher<message_bridge::ClientStatistics>("/pi1/aos");

  MessageCounter<message_bridge::ServerStatistics>
      *pi2_server_statistics_counter;
  pi2->OnStartup([pi2, &pi2_server_statistics_counter]() {
    pi2_server_statistics_counter =
        pi2->AlwaysStart<MessageCounter<message_bridge::ServerStatistics>>(
            "pi2_server_statistics_counter", "/pi2/aos");
  });
  aos::Fetcher<message_bridge::ServerStatistics> pi2_server_statistics_fetcher =
      pi2_pong_counter_event_loop
          ->MakeFetcher<message_bridge::ServerStatistics>("/pi2/aos");
  aos::Fetcher<message_bridge::ClientStatistics> pi2_client_statistics_fetcher =
      pi2_pong_counter_event_loop
          ->MakeFetcher<message_bridge::ClientStatistics>("/pi2/aos");

  MessageCounter<message_bridge::ServerStatistics>
      *pi3_server_statistics_counter;
  pi3->OnStartup([pi3, &pi3_server_statistics_counter]() {
    pi3_server_statistics_counter =
        pi3->AlwaysStart<MessageCounter<message_bridge::ServerStatistics>>(
            "pi3_server_statistics_counter", "/pi3/aos");
  });
  aos::Fetcher<message_bridge::ServerStatistics> pi3_server_statistics_fetcher =
      pi3_pong_counter_event_loop
          ->MakeFetcher<message_bridge::ServerStatistics>("/pi3/aos");
  aos::Fetcher<message_bridge::ClientStatistics> pi3_client_statistics_fetcher =
      pi3_pong_counter_event_loop
          ->MakeFetcher<message_bridge::ClientStatistics>("/pi3/aos");

  MessageCounter<message_bridge::ClientStatistics>
      pi1_client_statistics_counter(pi1_pong_counter_event_loop.get(),
                                    "/pi1/aos");
  MessageCounter<message_bridge::ClientStatistics>
      pi2_client_statistics_counter(pi2_pong_counter_event_loop.get(),
                                    "/pi2/aos");
  MessageCounter<message_bridge::ClientStatistics>
      pi3_client_statistics_counter(pi3_pong_counter_event_loop.get(),
                                    "/pi3/aos");

  std::vector<std::unique_ptr<aos::EventLoop>> statistics_watcher_loops;
  statistics_watcher_loops.emplace_back(pi1->MakeEventLoop("test"));
  statistics_watcher_loops.emplace_back(pi2->MakeEventLoop("test"));
  statistics_watcher_loops.emplace_back(pi3->MakeEventLoop("test"));
  // The currenct contract is that, if all nodes boot simultaneously in
  // simulation, that they should all act as if they area already connected,
  // without ever observing the transition from disconnected to connected (note
  // that on a real system the ServerStatistics message will get resent for each
  // and every new connection, even if the new connections happen
  // "simultaneously"--in simulation, we are essentially acting as if we are
  // starting execution in an already running system, rather than observing the
  // boot process).
  for (auto &event_loop : statistics_watcher_loops) {
    event_loop->MakeWatcher(
        "/aos", [](const message_bridge::ServerStatistics &msg) {
          for (const message_bridge::ServerConnection *connection :
               *msg.connections()) {
            EXPECT_EQ(message_bridge::State::CONNECTED, connection->state())
                << connection->node()->name()->string_view();
          }
        });
  }

  simulated_event_loop_factory.RunFor(chrono::seconds(2) +
                                      chrono::milliseconds(5));

  statistics_watcher_loops.clear();

  EXPECT_EQ(pi1_pong_counter.count(), 201u);
  EXPECT_EQ(pi2_pong_counter.count(), 201u);

  EXPECT_EQ(pi1_on_pi1_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi1_on_pi2_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi1_on_pi3_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi2_on_pi1_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi2_on_pi2_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi3_on_pi1_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi3_on_pi3_timestamp_counter.count(), 20u);

  EXPECT_EQ(pi1_server_statistics_counter->count(), 2u);
  EXPECT_EQ(pi2_server_statistics_counter->count(), 2u);
  EXPECT_EQ(pi3_server_statistics_counter->count(), 2u);

  EXPECT_EQ(pi1_client_statistics_counter.count(), 20u);
  EXPECT_EQ(pi2_client_statistics_counter.count(), 20u);
  EXPECT_EQ(pi3_client_statistics_counter.count(), 20u);

  // Also confirm that remote timestamps are being forwarded correctly.
  EXPECT_EQ(CountAll(remote_timestamps_pi2_on_pi1), 221);
  EXPECT_EQ(CountAll(remote_timestamps_pi1_on_pi2), 221);

  EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi1_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi1_server_statistics_fetcher.get());
  EXPECT_TRUE(pi1_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi1_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi1_client_statistics_fetcher.get());
  EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi2_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi2_server_statistics_fetcher.get());
  EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi2_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi2_client_statistics_fetcher.get());
  EXPECT_TRUE(pi3_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi3_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi3_server_statistics_fetcher.get());
  EXPECT_TRUE(pi3_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi3_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi3_client_statistics_fetcher.get());

  pi1->Disconnect(pi3->node());

  simulated_event_loop_factory.RunFor(chrono::seconds(2));

  EXPECT_EQ(pi1_pong_counter.count(), 401u);
  EXPECT_EQ(pi2_pong_counter.count(), 401u);

  EXPECT_EQ(pi1_on_pi1_timestamp_counter.count(), 40u);
  EXPECT_EQ(pi1_on_pi2_timestamp_counter.count(), 40u);
  EXPECT_EQ(pi1_on_pi3_timestamp_counter.count(), 20u);
  EXPECT_EQ(pi2_on_pi1_timestamp_counter.count(), 40u);
  EXPECT_EQ(pi2_on_pi2_timestamp_counter.count(), 40u);
  EXPECT_EQ(pi3_on_pi1_timestamp_counter.count(), 40u);
  EXPECT_EQ(pi3_on_pi3_timestamp_counter.count(), 40u);

  EXPECT_EQ(pi1_server_statistics_counter->count(), 4u);
  EXPECT_EQ(pi2_server_statistics_counter->count(), 4u);
  EXPECT_EQ(pi3_server_statistics_counter->count(), 4u);

  EXPECT_EQ(pi1_client_statistics_counter.count(), 40u);
  EXPECT_EQ(pi2_client_statistics_counter.count(), 40u);
  EXPECT_EQ(pi3_client_statistics_counter.count(), 40u);

  // Also confirm that remote timestamps are being forwarded correctly.
  EXPECT_EQ(CountAll(remote_timestamps_pi2_on_pi1), 441);
  EXPECT_EQ(CountAll(remote_timestamps_pi1_on_pi2), 441);

  EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnectedBut(pi1_server_statistics_fetcher.get(), "pi3"))
      << " : " << aos::FlatbufferToJson(pi1_server_statistics_fetcher.get());
  EXPECT_TRUE(pi1_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi1_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi1_client_statistics_fetcher.get());
  EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi2_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi2_server_statistics_fetcher.get());
  EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi2_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi2_client_statistics_fetcher.get());
  EXPECT_TRUE(pi3_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnected(pi3_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi3_server_statistics_fetcher.get());
  EXPECT_TRUE(pi3_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(AllConnectedBut(pi3_client_statistics_fetcher.get(), "pi1"))
      << " : " << aos::FlatbufferToJson(pi3_client_statistics_fetcher.get());

  pi1->Connect(pi3->node());

  simulated_event_loop_factory.RunFor(chrono::seconds(2));

  EXPECT_TRUE(pi1_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(pi1_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(pi2_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(pi2_client_statistics_fetcher.Fetch());
  EXPECT_TRUE(pi3_server_statistics_fetcher.Fetch());
  EXPECT_TRUE(pi3_client_statistics_fetcher.Fetch());

  EXPECT_EQ(ConnectedCount(pi1_server_statistics_fetcher.get(), "pi3"), 2u)
      << " : " << aos::FlatbufferToJson(pi1_server_statistics_fetcher.get());
  EXPECT_EQ(ConnectedCount(pi1_server_statistics_fetcher.get(), "pi2"), 1u)
      << " : " << aos::FlatbufferToJson(pi1_server_statistics_fetcher.get());
  EXPECT_EQ(ConnectedCount(pi1_client_statistics_fetcher.get(), "pi3"), 1u)
      << " : " << aos::FlatbufferToJson(pi1_client_statistics_fetcher.get());
  EXPECT_EQ(ConnectedCount(pi1_client_statistics_fetcher.get(), "pi2"), 1u)
      << " : " << aos::FlatbufferToJson(pi1_client_statistics_fetcher.get());

  EXPECT_EQ(ConnectedCount(pi2_server_statistics_fetcher.get(), "pi1"), 1u)
      << " : " << aos::FlatbufferToJson(pi2_server_statistics_fetcher.get());
  EXPECT_EQ(ConnectedCount(pi2_client_statistics_fetcher.get(), "pi1"), 1u)
      << " : " << aos::FlatbufferToJson(pi2_client_statistics_fetcher.get());

  EXPECT_EQ(ConnectedCount(pi3_server_statistics_fetcher.get(), "pi1"), 1u)
      << " : " << aos::FlatbufferToJson(pi3_server_statistics_fetcher.get());
  EXPECT_EQ(ConnectedCount(pi3_client_statistics_fetcher.get(), "pi1"), 2u)
      << " : " << aos::FlatbufferToJson(pi3_client_statistics_fetcher.get());

  EXPECT_EQ(pi1_pong_counter.count(), 601u);
  EXPECT_EQ(pi2_pong_counter.count(), 601u);

  EXPECT_EQ(pi1_on_pi1_timestamp_counter.count(), 60u);
  EXPECT_EQ(pi1_on_pi2_timestamp_counter.count(), 60u);
  EXPECT_EQ(pi1_on_pi3_timestamp_counter.count(), 40u);
  EXPECT_EQ(pi2_on_pi1_timestamp_counter.count(), 60u);
  EXPECT_EQ(pi2_on_pi2_timestamp_counter.count(), 60u);
  EXPECT_EQ(pi3_on_pi1_timestamp_counter.count(), 60u);
  EXPECT_EQ(pi3_on_pi3_timestamp_counter.count(), 60u);

  EXPECT_EQ(pi1_server_statistics_counter->count(), 6u);
  EXPECT_EQ(pi2_server_statistics_counter->count(), 6u);
  EXPECT_EQ(pi3_server_statistics_counter->count(), 6u);

  EXPECT_EQ(pi1_client_statistics_counter.count(), 60u);
  EXPECT_EQ(pi2_client_statistics_counter.count(), 60u);
  EXPECT_EQ(pi3_client_statistics_counter.count(), 60u);

  // Also confirm that remote timestamps are being forwarded correctly.
  EXPECT_EQ(CountAll(remote_timestamps_pi2_on_pi1), 661);
  EXPECT_EQ(CountAll(remote_timestamps_pi1_on_pi2), 661);

  EXPECT_TRUE(AllConnected(pi1_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi1_server_statistics_fetcher.get());
  EXPECT_TRUE(AllConnected(pi1_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi1_client_statistics_fetcher.get());
  EXPECT_TRUE(AllConnected(pi2_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi2_server_statistics_fetcher.get());
  EXPECT_TRUE(AllConnected(pi2_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi2_client_statistics_fetcher.get());
  EXPECT_TRUE(AllConnected(pi3_server_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi3_server_statistics_fetcher.get());
  EXPECT_TRUE(AllConnected(pi3_client_statistics_fetcher.get()))
      << " : " << aos::FlatbufferToJson(pi3_client_statistics_fetcher.get());
}

// Tests that the time offset having a slope doesn't break the world.
// SimulatedMessageBridge has enough self consistency CHECK statements to
// confirm, and we can can also check a message in each direction to make sure
// it gets delivered as expected.
TEST(SimulatedEventLoopTest, MultinodePingPongWithOffsetAndSlope) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(ArtifactPath(
          "aos/events/multinode_pingpong_test_combined_config.json"));
  const Node *pi1 = configuration::GetNode(&config.message(), "pi1");
  const size_t pi1_index = configuration::GetNodeIndex(&config.message(), pi1);
  ASSERT_EQ(pi1_index, 0u);
  const Node *pi2 = configuration::GetNode(&config.message(), "pi2");
  const size_t pi2_index = configuration::GetNodeIndex(&config.message(), pi2);
  ASSERT_EQ(pi2_index, 1u);
  const Node *pi3 = configuration::GetNode(&config.message(), "pi3");
  const size_t pi3_index = configuration::GetNodeIndex(&config.message(), pi3);
  ASSERT_EQ(pi3_index, 2u);

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());
  simulated_event_loop_factory.SetTimeConverter(&time);

  constexpr chrono::milliseconds kOffset{150100};
  time.AddNextTimestamp(
      distributed_clock::epoch(),
      {BootTimestamp::epoch(), BootTimestamp::epoch() + kOffset,
       BootTimestamp::epoch()});
  time.AddNextTimestamp(distributed_clock::epoch() + chrono::seconds(10),
                        {BootTimestamp::epoch() + chrono::milliseconds(9999),
                         BootTimestamp::epoch() + kOffset + chrono::seconds(10),
                         BootTimestamp::epoch() + chrono::milliseconds(9999)});

  std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping", pi1);
  Ping ping(ping_event_loop.get());

  std::unique_ptr<EventLoop> pong_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong", pi2);
  Pong pong(pong_event_loop.get());

  std::unique_ptr<EventLoop> pi1_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi1_counter", pi1);
  std::unique_ptr<EventLoop> pi2_counter_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pi2_counter", pi2);

  aos::Fetcher<examples::Ping> ping_on_pi1_fetcher =
      pi1_counter_event_loop->MakeFetcher<examples::Ping>("/test");
  aos::Fetcher<examples::Ping> ping_on_pi2_fetcher =
      pi2_counter_event_loop->MakeFetcher<examples::Ping>("/test");

  aos::Fetcher<examples::Pong> pong_on_pi2_fetcher =
      pi2_counter_event_loop->MakeFetcher<examples::Pong>("/test");
  aos::Fetcher<examples::Pong> pong_on_pi1_fetcher =
      pi1_counter_event_loop->MakeFetcher<examples::Pong>("/test");

  // End after a pong message comes back.  This will leave the latest messages
  // on all channels so we can look at timestamps easily and check they make
  // sense.
  std::unique_ptr<EventLoop> pi1_pong_ender =
      simulated_event_loop_factory.MakeEventLoop("pi2_counter", pi1);
  int count = 0;
  pi1_pong_ender->MakeWatcher(
      "/test", [&simulated_event_loop_factory, &count](const examples::Pong &) {
        if (++count == 100) {
          simulated_event_loop_factory.Exit();
        }
      });

  // Run enough that messages should be delivered.
  simulated_event_loop_factory.Run();

  // Grab the latest messages.
  EXPECT_TRUE(ping_on_pi1_fetcher.Fetch());
  EXPECT_TRUE(ping_on_pi2_fetcher.Fetch());
  EXPECT_TRUE(pong_on_pi1_fetcher.Fetch());
  EXPECT_TRUE(pong_on_pi2_fetcher.Fetch());

  // Compute their time on the global distributed clock so we can compute
  // distance betwen them.
  const distributed_clock::time_point pi1_ping_time = CheckExpected(
      simulated_event_loop_factory.GetNodeEventLoopFactory(pi1)
          ->ToDistributedClock(
              ping_on_pi1_fetcher.context().monotonic_event_time));
  const distributed_clock::time_point pi2_ping_time = CheckExpected(
      simulated_event_loop_factory.GetNodeEventLoopFactory(pi2)
          ->ToDistributedClock(
              ping_on_pi2_fetcher.context().monotonic_event_time));
  const distributed_clock::time_point pi1_pong_time = CheckExpected(
      simulated_event_loop_factory.GetNodeEventLoopFactory(pi1)
          ->ToDistributedClock(
              pong_on_pi1_fetcher.context().monotonic_event_time));
  const distributed_clock::time_point pi2_pong_time = CheckExpected(
      simulated_event_loop_factory.GetNodeEventLoopFactory(pi2)
          ->ToDistributedClock(
              pong_on_pi2_fetcher.context().monotonic_event_time));

  // And confirm the delivery delay is just about exactly 150 uS for both
  // directions like expected.  There will be a couple ns of rounding errors in
  // the conversion functions that aren't worth accounting for right now.  This
  // will either be really close, or really far.
  EXPECT_GE(pi2_ping_time, chrono::microseconds(150) - chrono::nanoseconds(10) +
                               pi1_ping_time);
  EXPECT_LE(pi2_ping_time, chrono::microseconds(150) + chrono::nanoseconds(10) +
                               pi1_ping_time);

  EXPECT_GE(pi1_pong_time, chrono::microseconds(150) - chrono::nanoseconds(10) +
                               pi2_pong_time);
  EXPECT_LE(pi1_pong_time, chrono::microseconds(150) + chrono::nanoseconds(10) +
                               pi2_pong_time);
}

void SendPing(aos::Sender<examples::Ping> *sender, int value) {
  aos::Sender<examples::Ping>::Builder builder = sender->MakeBuilder();
  examples::Ping::Builder ping_builder = builder.MakeBuilder<examples::Ping>();
  ping_builder.add_value(value);
  builder.CheckOk(builder.Send(ping_builder.Finish()));
}

// Tests that reliable (and unreliable) ping messages get forwarded as expected.
TEST_P(RemoteMessageSimulatedEventLoopTest, MultinodeStartupTesting) {
  const Node *pi1 = configuration::GetNode(&config.message(), "pi1");
  const Node *pi2 = configuration::GetNode(&config.message(), "pi2");

  SimulatedEventLoopFactory simulated_event_loop_factory(&config.message());

  std::unique_ptr<EventLoop> ping_event_loop =
      simulated_event_loop_factory.MakeEventLoop("ping", pi1);
  aos::Sender<examples::Ping> pi1_reliable_sender =
      ping_event_loop->MakeSender<examples::Ping>("/reliable");
  aos::Sender<examples::Ping> pi1_unreliable_sender =
      ping_event_loop->MakeSender<examples::Ping>("/unreliable");
  SendPing(&pi1_reliable_sender, 1);
  SendPing(&pi1_unreliable_sender, 1);

  std::unique_ptr<EventLoop> pi2_pong_event_loop =
      simulated_event_loop_factory.MakeEventLoop("pong", pi2);
  aos::Sender<examples::Ping> pi2_reliable_sender =
      pi2_pong_event_loop->MakeSender<examples::Ping>("/reliable2");
  SendPing(&pi2_reliable_sender, 1);
  MessageCounter<examples::Ping> pi2_reliable_counter(pi2_pong_event_loop.get(),
                                                      "/reliable");
  MessageCounter<examples::Ping> pi1_reliable_counter(ping_event_loop.get(),
                                                      "/reliable2");
  MessageCounter<examples::Ping> pi2_unreliable_counter(
      pi2_pong_event_loop.get(), "/unreliable");
  aos::Fetcher<examples::Ping> reliable_on_pi2_fetcher =
      pi2_pong_event_loop->MakeFetcher<examples::Ping>("/reliable");
  aos::Fetcher<examples::Ping> unreliable_on_pi2_fetcher =
      pi2_pong_event_loop->MakeFetcher<examples::Ping>("/unreliable");

  const size_t reliable_channel_index = configuration::ChannelIndex(
      pi2_pong_event_loop->configuration(), reliable_on_pi2_fetcher.channel());

  std::unique_ptr<EventLoop> pi1_remote_timestamp =
      simulated_event_loop_factory.MakeEventLoop("pi1_remote_timestamp", pi1);

  const chrono::nanoseconds network_delay =
      simulated_event_loop_factory.network_delay();

  int reliable_timestamp_count = 0;
  pi1_remote_timestamp->MakeWatcher(
      shared() ? "/pi1/aos/remote_timestamps/pi2"
               : "/pi1/aos/remote_timestamps/pi2/reliable/aos-examples-Ping",
      [reliable_channel_index, &reliable_timestamp_count,
       &simulated_event_loop_factory, pi2, network_delay, &pi2_pong_event_loop,
       &pi1_remote_timestamp](const RemoteMessage &header) {
        EXPECT_TRUE(header.has_boot_uuid());
        EXPECT_EQ(UUID::FromVector(header.boot_uuid()),
                  simulated_event_loop_factory.GetNodeEventLoopFactory(pi2)
                      ->boot_uuid());
        VLOG(1) << aos::FlatbufferToJson(&header);
        if (header.channel_index() == reliable_channel_index) {
          ++reliable_timestamp_count;
        }

        const aos::monotonic_clock::time_point header_monotonic_sent_time(
            chrono::nanoseconds(header.monotonic_sent_time()));

        EXPECT_EQ(pi1_remote_timestamp->context().monotonic_event_time,
                  header_monotonic_sent_time + network_delay +
                      (pi1_remote_timestamp->monotonic_now() -
                       pi2_pong_event_loop->monotonic_now()));
      });

  // Wait to let timestamp estimation start up before looking for the results.
  simulated_event_loop_factory.RunFor(chrono::milliseconds(500));

  EXPECT_EQ(pi2_reliable_counter.count(), 1u);
  // This one isn't reliable, but was sent before the start.  It should *not* be
  // delivered.
  EXPECT_EQ(pi2_unreliable_counter.count(), 0u);
  // Confirm we got a timestamp logged for the message that was forwarded.
  EXPECT_EQ(reliable_timestamp_count, 1u);

  SendPing(&pi1_reliable_sender, 2);
  SendPing(&pi1_unreliable_sender, 2);
  simulated_event_loop_factory.RunFor(chrono::milliseconds(500));
  EXPECT_EQ(pi2_reliable_counter.count(), 2u);
  EXPECT_EQ(pi1_reliable_counter.count(), 1u);
  EXPECT_EQ(pi2_unreliable_counter.count(), 1u);

  EXPECT_EQ(reliable_timestamp_count, 2u);
}

// Tests that rebooting a node changes the ServerStatistics message and the
// RemoteTimestamp message.
TEST_P(RemoteMessageSimulatedEventLoopTest, BootUUIDTest) {
  const UUID pi1_boot0 = UUID::Random();
  const UUID pi2_boot0 = UUID::Random();
  const UUID pi2_boot1 = UUID::Random();
  const UUID pi3_boot0 = UUID::Random();
  UUID expected_boot_uuid = pi2_boot0;
  int boot_number = 0;
  monotonic_clock::time_point expected_connection_time;

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);

  const size_t pi1_index =
      configuration::GetNodeIndex(&config.message(), "pi1");
  const size_t pi2_index =
      configuration::GetNodeIndex(&config.message(), "pi2");
  const size_t pi3_index =
      configuration::GetNodeIndex(&config.message(), "pi3");

  {
    time.AddNextTimestamp(distributed_clock::epoch(),
                          {BootTimestamp::epoch(), BootTimestamp::epoch(),
                           BootTimestamp::epoch()});

    const chrono::nanoseconds dt = chrono::milliseconds(2001);

    time.AddNextTimestamp(
        distributed_clock::epoch() + dt,
        {BootTimestamp::epoch() + dt,
         BootTimestamp{.boot = 1, .time = monotonic_clock::epoch()},
         BootTimestamp::epoch() + dt});

    time.set_boot_uuid(pi1_index, 0, pi1_boot0);
    time.set_boot_uuid(pi2_index, 0, pi2_boot0);
    time.set_boot_uuid(pi2_index, 1, pi2_boot1);
    time.set_boot_uuid(pi3_index, 0, pi3_boot0);
  }

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  pi1->OnStartup([pi1]() { pi1->AlwaysStart<Ping>("ping"); });
  pi2->OnStartup([pi2]() { pi2->AlwaysStart<Pong>("pong"); });

  std::unique_ptr<EventLoop> pi1_remote_timestamp =
      pi1->MakeEventLoop("pi1_remote_timestamp");

  int timestamp_count = 0;
  pi1_remote_timestamp->MakeWatcher(
      "/pi2/aos", [&expected_boot_uuid,
                   &pi1_remote_timestamp](const message_bridge::Timestamp &) {
        EXPECT_EQ(pi1_remote_timestamp->context().source_boot_uuid,
                  expected_boot_uuid);
      });
  pi1_remote_timestamp->MakeWatcher(
      "/test",
      [&expected_boot_uuid, &pi1_remote_timestamp](const examples::Pong &) {
        EXPECT_EQ(pi1_remote_timestamp->context().source_boot_uuid,
                  expected_boot_uuid);
      });
  pi1_remote_timestamp->MakeWatcher(
      shared() ? "/pi1/aos/remote_timestamps/pi2"
               : "/pi1/aos/remote_timestamps/pi2/test/aos-examples-Ping",
      [&timestamp_count, &expected_boot_uuid](const RemoteMessage &header) {
        EXPECT_TRUE(header.has_boot_uuid());
        EXPECT_EQ(UUID::FromVector(header.boot_uuid()), expected_boot_uuid);
        VLOG(1) << aos::FlatbufferToJson(&header);
        ++timestamp_count;
      });

  int pi1_server_statistics_count = 0;
  bool first_pi1_server_statistics = true;
  expected_connection_time = pi1->monotonic_now();
  pi1_remote_timestamp->MakeWatcher(
      "/pi1/aos",
      [&pi1_server_statistics_count, &expected_boot_uuid,
       &expected_connection_time, &first_pi1_server_statistics,
       &boot_number](const message_bridge::ServerStatistics &stats) {
        VLOG(1) << "pi1 ServerStatistics " << FlatbufferToJson(&stats);
        for (const message_bridge::ServerConnection *connection :
             *stats.connections()) {
          if (connection->state() == message_bridge::State::CONNECTED) {
            ASSERT_TRUE(connection->has_boot_uuid());
          }
          if (!first_pi1_server_statistics) {
            EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
          }
          if (connection->node()->name()->string_view() == "pi2") {
            EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
            ASSERT_TRUE(connection->has_boot_uuid());
            EXPECT_EQ(expected_boot_uuid,
                      UUID::FromString(connection->boot_uuid()))
                << " : Got " << aos::FlatbufferToJson(&stats);
            EXPECT_EQ(monotonic_clock::time_point(chrono::nanoseconds(
                          connection->connected_since_time())),
                      expected_connection_time);
            EXPECT_EQ(boot_number + 1, connection->connection_count());
            ++pi1_server_statistics_count;
          }
        }
        first_pi1_server_statistics = false;
      });

  int pi1_client_statistics_count = 0;
  pi1_remote_timestamp->MakeWatcher(
      "/pi1/aos", [&pi1_client_statistics_count, &expected_boot_uuid,
                   &expected_connection_time, &boot_number](
                      const message_bridge::ClientStatistics &stats) {
        VLOG(1) << "pi1 ClientStatistics " << FlatbufferToJson(&stats);
        for (const message_bridge::ClientConnection *connection :
             *stats.connections()) {
          EXPECT_EQ(connection->state(), message_bridge::State::CONNECTED);
          if (connection->node()->name()->string_view() == "pi2") {
            ++pi1_client_statistics_count;
            EXPECT_EQ(expected_boot_uuid,
                      UUID::FromString(connection->boot_uuid()))
                << " : Got " << aos::FlatbufferToJson(&stats);
            EXPECT_EQ(monotonic_clock::time_point(chrono::nanoseconds(
                          connection->connected_since_time())),
                      expected_connection_time);
            EXPECT_EQ(boot_number + 1, connection->connection_count());
          } else {
            EXPECT_EQ(connection->connected_since_time(), 0);
            EXPECT_EQ(1, connection->connection_count());
          }
        }
      });

  // Confirm that reboot changes the UUID.
  pi2->OnShutdown([&expected_boot_uuid, &boot_number, &expected_connection_time,
                   pi1, pi2, pi2_boot1]() {
    expected_boot_uuid = pi2_boot1;
    ++boot_number;
    LOG(INFO) << "OnShutdown triggered for pi2";
    pi2->OnStartup(
        [&expected_boot_uuid, &expected_connection_time, pi1, pi2]() {
          EXPECT_EQ(expected_boot_uuid, pi2->boot_uuid());
          expected_connection_time = pi1->monotonic_now();
        });
  });

  // Let a couple of ServerStatistics messages show up before rebooting.
  factory.RunFor(chrono::milliseconds(2002));

  EXPECT_GT(timestamp_count, 100);
  EXPECT_GE(pi1_server_statistics_count, 1u);

  timestamp_count = 0;
  pi1_server_statistics_count = 0;

  factory.RunFor(chrono::milliseconds(2000));
  EXPECT_GT(timestamp_count, 100);
  EXPECT_GE(pi1_server_statistics_count, 1u);
}

INSTANTIATE_TEST_SUITE_P(
    All, RemoteMessageSimulatedEventLoopTest,
    ::testing::Values(
        Param{"multinode_pingpong_test_combined_config.json", true},
        Param{"multinode_pingpong_test_split_config.json", false}));

// Tests that Startup and Shutdown do reasonable things.
TEST(SimulatedEventLoopTest, MultinodePingPongStartup) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  size_t pi1_shutdown_counter = 0;
  size_t pi2_shutdown_counter = 0;
  MessageCounter<examples::Pong> *pi1_pong_counter = nullptr;
  MessageCounter<examples::Ping> *pi2_ping_counter = nullptr;

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);
  time.AddNextTimestamp(
      distributed_clock::epoch(),
      {BootTimestamp::epoch(), BootTimestamp::epoch(), BootTimestamp::epoch()});

  const chrono::nanoseconds dt = chrono::seconds(10) + chrono::milliseconds(6);

  time.AddNextTimestamp(
      distributed_clock::epoch() + dt,
      {BootTimestamp{.boot = 1, .time = monotonic_clock::epoch()},
       BootTimestamp{.boot = 1, .time = monotonic_clock::epoch()},
       BootTimestamp::epoch() + dt});

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  // Configure startup to start Ping and Pong, and count.
  size_t pi1_startup_counter = 0;
  size_t pi2_startup_counter = 0;
  pi1->OnStartup([pi1]() {
    LOG(INFO) << "Made ping";
    pi1->AlwaysStart<Ping>("ping");
  });
  pi1->OnStartup([&pi1_startup_counter]() { ++pi1_startup_counter; });
  pi2->OnStartup([pi2]() {
    LOG(INFO) << "Made pong";
    pi2->AlwaysStart<Pong>("pong");
  });
  pi2->OnStartup([&pi2_startup_counter]() { ++pi2_startup_counter; });

  // Shutdown just counts.
  pi1->OnShutdown([&pi1_shutdown_counter]() { ++pi1_shutdown_counter; });
  pi2->OnShutdown([&pi2_shutdown_counter]() { ++pi2_shutdown_counter; });

  // Automatically make counters on startup.
  pi1->OnStartup([&pi1_pong_counter, pi1]() {
    pi1_pong_counter = pi1->AlwaysStart<MessageCounter<examples::Pong>>(
        "pi1_pong_counter", "/test");
  });
  pi1->OnShutdown([&pi1_pong_counter]() { pi1_pong_counter = nullptr; });
  pi2->OnStartup([&pi2_ping_counter, pi2]() {
    pi2_ping_counter = pi2->AlwaysStart<MessageCounter<examples::Ping>>(
        "pi2_ping_counter", "/test");
  });
  pi2->OnShutdown([&pi2_ping_counter]() { pi2_ping_counter = nullptr; });

  EXPECT_EQ(pi2_ping_counter, nullptr);
  EXPECT_EQ(pi1_pong_counter, nullptr);

  EXPECT_EQ(pi1_startup_counter, 0u);
  EXPECT_EQ(pi2_startup_counter, 0u);
  EXPECT_EQ(pi1_shutdown_counter, 0u);
  EXPECT_EQ(pi2_shutdown_counter, 0u);

  factory.RunFor(chrono::seconds(10) + chrono::milliseconds(5));
  EXPECT_EQ(pi1_startup_counter, 1u);
  EXPECT_EQ(pi2_startup_counter, 1u);
  EXPECT_EQ(pi1_shutdown_counter, 0u);
  EXPECT_EQ(pi2_shutdown_counter, 0u);
  EXPECT_EQ(pi2_ping_counter->count(), 1001);
  EXPECT_EQ(pi1_pong_counter->count(), 1001);

  LOG(INFO) << pi1->monotonic_now();
  LOG(INFO) << pi2->monotonic_now();

  factory.RunFor(chrono::seconds(5) + chrono::milliseconds(5));

  EXPECT_EQ(pi1_startup_counter, 2u);
  EXPECT_EQ(pi2_startup_counter, 2u);
  EXPECT_EQ(pi1_shutdown_counter, 1u);
  EXPECT_EQ(pi2_shutdown_counter, 1u);
  EXPECT_EQ(pi2_ping_counter->count(), 501);
  EXPECT_EQ(pi1_pong_counter->count(), 501);
}

// Tests that OnStartup handlers can be added after running and get called, and
// can't be called when running.
TEST(SimulatedEventLoopDeathTest, OnStartupWhileRunning) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  // Test that we can add startup handlers as long as we aren't running, and
  // they get run when Run gets called again.
  // Test that adding a startup handler when running fails.
  //
  // Test shutdown handlers get called on destruction.
  SimulatedEventLoopFactory factory(&config.message());

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");

  int startup_count0 = 0;
  int startup_count1 = 0;

  pi1->OnStartup([&]() { ++startup_count0; });
  EXPECT_EQ(startup_count0, 0);
  EXPECT_EQ(startup_count1, 0);

  factory.RunFor(chrono::nanoseconds(1));
  EXPECT_EQ(startup_count0, 1);
  EXPECT_EQ(startup_count1, 0);

  pi1->OnStartup([&]() { ++startup_count1; });
  EXPECT_EQ(startup_count0, 1);
  EXPECT_EQ(startup_count1, 0);

  factory.RunFor(chrono::nanoseconds(1));
  EXPECT_EQ(startup_count0, 1);
  EXPECT_EQ(startup_count1, 1);

  std::unique_ptr<EventLoop> loop = pi1->MakeEventLoop("foo");
  loop->OnRun([&]() { pi1->OnStartup([]() {}); });

  EXPECT_DEATH(
      { factory.RunFor(chrono::nanoseconds(1)); },
      "Can only register OnStartup handlers when not running.");
}

// Tests that OnStartup handlers can be added after running and get called, and
// all the handlers get called on reboot.  Shutdown handlers are tested the same
// way.
TEST(SimulatedEventLoopTest, OnStartupShutdownAllRestarts) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  int startup_count0 = 0;
  int shutdown_count0 = 0;
  int startup_count1 = 0;
  int shutdown_count1 = 0;

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);
  time.StartEqual();

  const chrono::nanoseconds dt = chrono::seconds(10);
  time.RebootAt(0, distributed_clock::epoch() + dt);
  time.RebootAt(0, distributed_clock::epoch() + 2 * dt);

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");

  pi1->OnStartup([&]() { ++startup_count0; });
  pi1->OnShutdown([&]() { ++shutdown_count0; });
  EXPECT_EQ(startup_count0, 0);
  EXPECT_EQ(startup_count1, 0);
  EXPECT_EQ(shutdown_count0, 0);
  EXPECT_EQ(shutdown_count1, 0);

  factory.RunFor(chrono::nanoseconds(1));
  EXPECT_EQ(startup_count0, 1);
  EXPECT_EQ(startup_count1, 0);
  EXPECT_EQ(shutdown_count0, 0);
  EXPECT_EQ(shutdown_count1, 0);

  pi1->OnStartup([&]() { ++startup_count1; });
  EXPECT_EQ(startup_count0, 1);
  EXPECT_EQ(startup_count1, 0);
  EXPECT_EQ(shutdown_count0, 0);
  EXPECT_EQ(shutdown_count1, 0);

  factory.RunFor(chrono::nanoseconds(1));
  EXPECT_EQ(startup_count0, 1);
  EXPECT_EQ(startup_count1, 1);
  EXPECT_EQ(shutdown_count0, 0);
  EXPECT_EQ(shutdown_count1, 0);

  factory.RunFor(chrono::seconds(15));

  EXPECT_EQ(startup_count0, 2);
  EXPECT_EQ(startup_count1, 2);
  EXPECT_EQ(shutdown_count0, 1);
  EXPECT_EQ(shutdown_count1, 0);

  pi1->OnShutdown([&]() { ++shutdown_count1; });
  factory.RunFor(chrono::seconds(10));

  EXPECT_EQ(startup_count0, 3);
  EXPECT_EQ(startup_count1, 3);
  EXPECT_EQ(shutdown_count0, 2);
  EXPECT_EQ(shutdown_count1, 1);
}

// Tests that event loops which outlive shutdown crash.
TEST(SimulatedEventLoopDeathTest, EventLoopOutlivesReboot) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);
  time.StartEqual();

  const chrono::nanoseconds dt = chrono::seconds(10);
  time.RebootAt(0, distributed_clock::epoch() + dt);

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");

  std::unique_ptr<EventLoop> loop = pi1->MakeEventLoop("foo");

  EXPECT_DEATH({ factory.RunFor(dt * 2); }, "Event loop");
}

// Test that an ExitHandle outliving its factory is caught.
TEST(SimulatedEventLoopDeathTest, ExitHandleOutlivesFactory) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));
  auto factory = std::make_unique<SimulatedEventLoopFactory>(&config.message());
  NodeEventLoopFactory *pi1 = factory->GetNodeEventLoopFactory("pi1");
  std::unique_ptr<EventLoop> loop = pi1->MakeEventLoop("foo");
  auto exit_handle = factory->MakeExitHandle();
  EXPECT_DEATH(factory.reset(),
               "All ExitHandles must be destroyed before the factory");
}

// Test that AllowApplicationCreationDuring can't happen in OnRun callbacks.
TEST(SimulatedEventLoopDeathTest, AllowApplicationCreationDuringInOnRun) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));
  auto factory = std::make_unique<SimulatedEventLoopFactory>(&config.message());
  NodeEventLoopFactory *pi1 = factory->GetNodeEventLoopFactory("pi1");
  std::unique_ptr<EventLoop> loop = pi1->MakeEventLoop("foo");
  loop->OnRun([&]() { factory->AllowApplicationCreationDuring([]() {}); });
  EXPECT_DEATH(factory->RunFor(chrono::seconds(1)), "OnRun");
}

// Tests that boot UUIDs match for const and non-const
// SimulatedEventLoopFactory. Since the implementations of these two are
// different, it's important to validate despite appearing like an awkward test.
TEST(SimulatedEventLoopTest, BootUUIDsWorkForConstNodeEventLoopFactory) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  SimulatedEventLoopFactory factory(&config.message());
  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");

  EXPECT_NE(pi1->boot_uuid(), UUID::Zero());
  EXPECT_EQ(pi1->boot_uuid(),
            static_cast<const NodeEventLoopFactory *>(pi1)->boot_uuid());
}

// Tests that messages don't survive a reboot of a node.
TEST(SimulatedEventLoopTest, ChannelClearedOnReboot) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);
  time.StartEqual();

  const chrono::nanoseconds dt = chrono::seconds(10);
  time.RebootAt(0, distributed_clock::epoch() + dt);

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");

  const UUID boot_uuid = pi1->boot_uuid();
  EXPECT_NE(boot_uuid, UUID::Zero());

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi1->MakeEventLoop("ping");
    aos::Sender<examples::Ping> test_message_sender =
        ping_event_loop->MakeSender<examples::Ping>("/reliable");
    SendPing(&test_message_sender, 1);
  }

  factory.RunFor(chrono::seconds(5));

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi1->MakeEventLoop("ping");
    aos::Fetcher<examples::Ping> fetcher =
        ping_event_loop->MakeFetcher<examples::Ping>("/reliable");
    EXPECT_TRUE(fetcher.Fetch());
  }

  factory.RunFor(chrono::seconds(10));

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi1->MakeEventLoop("ping");
    aos::Fetcher<examples::Ping> fetcher =
        ping_event_loop->MakeFetcher<examples::Ping>("/reliable");
    EXPECT_FALSE(fetcher.Fetch());
  }
  EXPECT_NE(boot_uuid, pi1->boot_uuid());
}

// Tests that reliable messages get resent on reboot.
TEST(SimulatedEventLoopTest, ReliableMessageResentOnReboot) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);
  time.StartEqual();

  const chrono::nanoseconds dt = chrono::seconds(1);
  time.RebootAt(1, distributed_clock::epoch() + dt);

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  const UUID pi1_boot_uuid = pi1->boot_uuid();
  const UUID pi2_boot_uuid = pi2->boot_uuid();
  EXPECT_NE(pi1_boot_uuid, UUID::Zero());
  EXPECT_NE(pi2_boot_uuid, UUID::Zero());

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi1->MakeEventLoop("ping");
    aos::Sender<examples::Ping> test_message_sender =
        ping_event_loop->MakeSender<examples::Ping>("/reliable");
    SendPing(&test_message_sender, 1);
  }

  factory.RunFor(chrono::milliseconds(500));

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi2->MakeEventLoop("pong");
    aos::Fetcher<examples::Ping> fetcher =
        ping_event_loop->MakeFetcher<examples::Ping>("/reliable");
    ASSERT_TRUE(fetcher.Fetch());
    EXPECT_EQ(fetcher.context().monotonic_remote_time,
              monotonic_clock::epoch());
    // Message bridge picks up the Ping message immediately on reboot.
    EXPECT_EQ(fetcher.context().monotonic_remote_transmit_time,
              monotonic_clock::epoch());
    EXPECT_EQ(fetcher.context().monotonic_event_time,
              monotonic_clock::epoch() + factory.network_delay());
    ASSERT_FALSE(fetcher.Fetch());
  }

  factory.RunFor(chrono::seconds(1));

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi2->MakeEventLoop("pong");
    aos::Fetcher<examples::Ping> fetcher =
        ping_event_loop->MakeFetcher<examples::Ping>("/reliable");
    ASSERT_TRUE(fetcher.Fetch());
    EXPECT_EQ(fetcher.context().monotonic_remote_time,
              monotonic_clock::epoch());
    // Message bridge picks up the Ping message immediately on reboot.
    EXPECT_EQ(fetcher.context().monotonic_remote_transmit_time,
              monotonic_clock::epoch() + chrono::seconds(1));
    EXPECT_EQ(fetcher.context().monotonic_event_time,
              monotonic_clock::epoch() + factory.network_delay());
    ASSERT_FALSE(fetcher.Fetch());
  }
  EXPECT_NE(pi2_boot_uuid, pi2->boot_uuid());
}

TEST(SimulatedEventLoopTest, ReliableMessageSentOnStaggeredBoot) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  time.AddNextTimestamp(
      distributed_clock::epoch(),
      {BootTimestamp{0, monotonic_clock::epoch()},
       BootTimestamp{0, monotonic_clock::epoch() - chrono::seconds(1)},
       BootTimestamp{0, monotonic_clock::epoch()}});
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  const UUID pi1_boot_uuid = pi1->boot_uuid();
  const UUID pi2_boot_uuid = pi2->boot_uuid();
  EXPECT_NE(pi1_boot_uuid, UUID::Zero());
  EXPECT_NE(pi2_boot_uuid, UUID::Zero());

  {
    ::std::unique_ptr<EventLoop> pi1_event_loop = pi1->MakeEventLoop("ping");
    aos::Sender<examples::Ping> pi1_sender =
        pi1_event_loop->MakeSender<examples::Ping>("/reliable");
    SendPing(&pi1_sender, 1);
  }
  ::std::unique_ptr<EventLoop> pi2_event_loop = pi2->MakeEventLoop("ping");
  aos::Sender<examples::Ping> pi2_sender =
      pi2_event_loop->MakeSender<examples::Ping>("/reliable2");
  SendPing(&pi2_sender, 1);
  // Verify that we staggered the OnRun callback correctly.
  pi2_event_loop->OnRun([pi1, pi2]() {
    EXPECT_EQ(pi1->monotonic_now(),
              monotonic_clock::epoch() + std::chrono::seconds(1));
    EXPECT_EQ(pi2->monotonic_now(), monotonic_clock::epoch());
  });

  factory.RunFor(chrono::seconds(2));

  {
    ::std::unique_ptr<EventLoop> pi2_event_loop = pi2->MakeEventLoop("pong");
    aos::Fetcher<examples::Ping> fetcher =
        pi2_event_loop->MakeFetcher<examples::Ping>("/reliable");
    ASSERT_TRUE(fetcher.Fetch());
    EXPECT_EQ(fetcher.context().monotonic_event_time,
              monotonic_clock::epoch() + factory.network_delay());
    EXPECT_EQ(fetcher.context().monotonic_remote_time,
              monotonic_clock::epoch());
    EXPECT_EQ(fetcher.context().monotonic_remote_transmit_time,
              monotonic_clock::epoch() + chrono::seconds(1));
  }
  {
    ::std::unique_ptr<EventLoop> pi1_event_loop = pi1->MakeEventLoop("pong");
    aos::Fetcher<examples::Ping> fetcher =
        pi1_event_loop->MakeFetcher<examples::Ping>("/reliable2");
    ASSERT_TRUE(fetcher.Fetch());
    EXPECT_EQ(fetcher.context().monotonic_event_time,
              monotonic_clock::epoch() + std::chrono::seconds(1) +
                  factory.network_delay());
    EXPECT_EQ(fetcher.context().monotonic_remote_time,
              monotonic_clock::epoch() - std::chrono::seconds(1));
    EXPECT_EQ(fetcher.context().monotonic_remote_transmit_time,
              monotonic_clock::epoch());
  }
}

class SimulatedEventLoopDisconnectTest : public ::testing::Test {
 public:
  SimulatedEventLoopDisconnectTest()
      : config(aos::configuration::ReadConfig(ArtifactPath(
            "aos/events/multinode_pingpong_test_split_config.json"))),
        time(configuration::NodesCount(&config.message())),
        factory(&config.message()) {
    factory.SetTimeConverter(&time);
  }

  void VerifyChannels(std::set<const aos::Channel *> statistics_channels,
                      const monotonic_clock::time_point allowable_message_time,
                      std::set<const aos::Node *> empty_nodes) {
    NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
    NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");
    std::unique_ptr<aos::EventLoop> pi1_event_loop =
        pi1->MakeEventLoop("fetcher");
    std::unique_ptr<aos::EventLoop> pi2_event_loop =
        pi2->MakeEventLoop("fetcher");
    for (const aos::Channel *channel : *factory.configuration()->channels()) {
      if (configuration::ChannelIsReadableOnNode(channel,
                                                 pi1_event_loop->node())) {
        std::unique_ptr<aos::RawFetcher> fetcher =
            pi1_event_loop->MakeRawFetcher(channel);
        if (statistics_channels.find(channel) == statistics_channels.end() ||
            empty_nodes.find(pi1_event_loop->node()) != empty_nodes.end()) {
          EXPECT_FALSE(fetcher->Fetch() &&
                       fetcher->context().monotonic_event_time >
                           allowable_message_time)
              << ": Found recent message on channel "
              << configuration::CleanedChannelToString(channel) << " and time "
              << fetcher->context().monotonic_event_time << " > "
              << allowable_message_time << " on pi1";
        } else {
          EXPECT_TRUE(fetcher->Fetch() &&
                      fetcher->context().monotonic_event_time >=
                          allowable_message_time)
              << ": Didn't find recent message on channel "
              << configuration::CleanedChannelToString(channel) << " on pi1";
        }
      }
      if (configuration::ChannelIsReadableOnNode(channel,
                                                 pi2_event_loop->node())) {
        std::unique_ptr<aos::RawFetcher> fetcher =
            pi2_event_loop->MakeRawFetcher(channel);
        if (statistics_channels.find(channel) == statistics_channels.end() ||
            empty_nodes.find(pi2_event_loop->node()) != empty_nodes.end()) {
          EXPECT_FALSE(fetcher->Fetch() &&
                       fetcher->context().monotonic_event_time >
                           allowable_message_time)
              << ": Found message on channel "
              << configuration::CleanedChannelToString(channel) << " and time "
              << fetcher->context().monotonic_event_time << " > "
              << allowable_message_time << " on pi2";
        } else {
          EXPECT_TRUE(fetcher->Fetch() &&
                      fetcher->context().monotonic_event_time >=
                          allowable_message_time)
              << ": Didn't find message on channel "
              << configuration::CleanedChannelToString(channel) << " on pi2";
        }
      }
    }
  }

  aos::FlatbufferDetachedBuffer<aos::Configuration> config;

  message_bridge::TestingTimeConverter time;
  SimulatedEventLoopFactory factory;
};

// Tests that if we have message bridge client/server disabled, and timing
// reports disabled, no messages are sent.  Also tests that we can disconnect a
// node and disable statistics on it and it actually fully disconnects.
TEST_F(SimulatedEventLoopDisconnectTest, NoMessagesWhenDisabled) {
  time.StartEqual();
  factory.SkipTimingReport();
  factory.DisableStatistics();

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  std::unique_ptr<aos::EventLoop> pi1_event_loop =
      pi1->MakeEventLoop("fetcher");
  std::unique_ptr<aos::EventLoop> pi2_event_loop =
      pi2->MakeEventLoop("fetcher");

  factory.RunFor(chrono::milliseconds(100000));

  // Confirm no messages are sent if we've configured them all off.
  VerifyChannels({}, monotonic_clock::min_time, {});

  // Now, confirm that all the message_bridge channels come back when we
  // re-enable.
  factory.EnableStatistics();

  factory.RunFor(chrono::milliseconds(10050));

  // Build up the list of all the messages we expect when we come back.
  {
    std::set<const aos::Channel *> statistics_channels;
    for (const std::pair<std::string_view, const Node *> &pi :
         std::vector<std::pair<std::string_view, const Node *>>{
             {"/pi1/aos", pi1->node()},
             {"/pi2/aos", pi1->node()},
             {"/pi3/aos", pi1->node()}}) {
      statistics_channels.insert(configuration::GetChannel(
          factory.configuration(), pi.first, "aos.message_bridge.Timestamp", "",
          pi.second));
      statistics_channels.insert(configuration::GetChannel(
          factory.configuration(), pi.first,
          "aos.message_bridge.ServerStatistics", "", pi.second));
      statistics_channels.insert(configuration::GetChannel(
          factory.configuration(), pi.first,
          "aos.message_bridge.ClientStatistics", "", pi.second));
    }

    statistics_channels.insert(configuration::GetChannel(
        factory.configuration(),
        "/pi1/aos/remote_timestamps/pi2/pi1/aos/aos-message_bridge-Timestamp",
        "aos.message_bridge.RemoteMessage", "", pi1->node()));
    statistics_channels.insert(configuration::GetChannel(
        factory.configuration(),
        "/pi2/aos/remote_timestamps/pi1/pi2/aos/aos-message_bridge-Timestamp",
        "aos.message_bridge.RemoteMessage", "", pi2->node()));
    VerifyChannels(statistics_channels, monotonic_clock::min_time, {});
  }

  // Now test that we can disable the messages for a single node
  pi2->DisableStatistics();
  const aos::monotonic_clock::time_point statistics_disable_time =
      pi2->monotonic_now();
  factory.RunFor(chrono::milliseconds(10000));

  // We should see a much smaller set of messages, but should still see messages
  // forwarded, mainly the timestamp message.
  {
    std::set<const aos::Channel *> statistics_channels;
    for (const std::pair<std::string_view, const Node *> &pi :
         std::vector<std::pair<std::string_view, const Node *>>{
             {"/pi1/aos", pi1->node()}, {"/pi3/aos", pi1->node()}}) {
      statistics_channels.insert(configuration::GetChannel(
          factory.configuration(), pi.first, "aos.message_bridge.Timestamp", "",
          pi.second));
      statistics_channels.insert(configuration::GetChannel(
          factory.configuration(), pi.first,
          "aos.message_bridge.ServerStatistics", "", pi.second));
      statistics_channels.insert(configuration::GetChannel(
          factory.configuration(), pi.first,
          "aos.message_bridge.ClientStatistics", "", pi.second));
    }

    statistics_channels.insert(configuration::GetChannel(
        factory.configuration(),
        "/pi1/aos/remote_timestamps/pi2/pi1/aos/aos-message_bridge-Timestamp",
        "aos.message_bridge.RemoteMessage", "", pi1->node()));
    VerifyChannels(statistics_channels, statistics_disable_time, {});
  }

  // Now, fully disconnect the node.  This will completely quiet down pi2.
  pi1->Disconnect(pi2->node());
  pi2->Disconnect(pi1->node());

  const aos::monotonic_clock::time_point disconnect_disable_time =
      pi2->monotonic_now();
  factory.RunFor(chrono::milliseconds(10000));

  {
    std::set<const aos::Channel *> statistics_channels;
    for (const std::pair<std::string_view, const Node *> &pi :
         std::vector<std::pair<std::string_view, const Node *>>{
             {"/pi1/aos", pi1->node()}, {"/pi3/aos", pi1->node()}}) {
      statistics_channels.insert(configuration::GetChannel(
          factory.configuration(), pi.first, "aos.message_bridge.Timestamp", "",
          pi.second));
      statistics_channels.insert(configuration::GetChannel(
          factory.configuration(), pi.first,
          "aos.message_bridge.ServerStatistics", "", pi.second));
      statistics_channels.insert(configuration::GetChannel(
          factory.configuration(), pi.first,
          "aos.message_bridge.ClientStatistics", "", pi.second));
    }

    VerifyChannels(statistics_channels, disconnect_disable_time, {pi2->node()});
  }
}

// Struct to capture the expected time a message should be received (and it's
// value).  This is from the perspective of the node receiving the message.
struct ExpectedTimestamps {
  // The time that the message was published on the sending node's monotonic
  // clock.
  monotonic_clock::time_point remote_time;
  // The time that the message was virtually transmitted over the virtual
  // network on the sending node's monotonic clock.
  monotonic_clock::time_point remote_transmit_time;
  // The time that the message was received on the receiving node's clock.
  monotonic_clock::time_point event_time;
  // The value inside the message.
  int value;
};

// Tests that rapidly sent messages get timestamped correctly.
TEST(SimulatedEventLoopTest, TransmitTimestamps) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(
          ArtifactPath("aos/events/multinode_pingpong_test_split_config.json"));

  message_bridge::TestingTimeConverter time(
      configuration::NodesCount(&config.message()));
  SimulatedEventLoopFactory factory(&config.message());
  factory.SetTimeConverter(&time);
  time.StartEqual();

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  ::std::unique_ptr<EventLoop> ping_event_loop = pi2->MakeEventLoop("pong");
  aos::Fetcher<examples::Ping> fetcher =
      ping_event_loop->MakeFetcher<examples::Ping>("/reliable");
  EXPECT_FALSE(fetcher.Fetch());

  {
    ::std::unique_ptr<EventLoop> ping_event_loop = pi1->MakeEventLoop("ping");
    FunctionScheduler run_at(ping_event_loop.get());
    aos::Sender<examples::Ping> test_message_sender =
        ping_event_loop->MakeSender<examples::Ping>("/reliable");
    aos::monotonic_clock::time_point now = ping_event_loop->monotonic_now();
    for (const std::chrono::nanoseconds dt :
         {chrono::microseconds(5000), chrono::microseconds(1),
          chrono::microseconds(2), chrono::microseconds(70),
          chrono::microseconds(63), chrono::microseconds(140)}) {
      now += dt;
      run_at.ScheduleAt([&]() { SendPing(&test_message_sender, 1); }, now);
    }

    now += chrono::milliseconds(10);

    factory.RunFor(now - ping_event_loop->monotonic_now());
  }

  const monotonic_clock::time_point e = monotonic_clock::epoch();
  const chrono::nanoseconds send_delay = factory.send_delay();
  const chrono::nanoseconds network_delay = factory.network_delay();

  const std::vector<ExpectedTimestamps> expected_values = {
      // First message shows up after wakeup + network delay as expected.
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(5000),
          .remote_transmit_time = e + chrono::microseconds(5000) + send_delay,
          .event_time =
              e + chrono::microseconds(5000) + send_delay + network_delay,
          .value = 1,
      },
      // Next message is close enough that it gets picked up at the same wakeup.
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(5001),
          .remote_transmit_time = e + chrono::microseconds(5000) + send_delay,
          .event_time =
              e + chrono::microseconds(5000) + send_delay + network_delay,
          .value = 1,
      },
      // Same for the third.
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(5003),
          .remote_transmit_time = e + chrono::microseconds(5000) + send_delay,
          .event_time =
              e + chrono::microseconds(5000) + send_delay + network_delay,
          .value = 1,
      },
      // Fourth waits long enough to do the right thing.
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(5073),
          .remote_transmit_time = e + chrono::microseconds(5073) + send_delay,
          .event_time =
              e + chrono::microseconds(5073) + send_delay + network_delay,
          .value = 1,
      },
      // Fifth waits long enough to do the right thing as well (but kicks off
      // while the fourth is in flight over the network).
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(5136),
          .remote_transmit_time = e + chrono::microseconds(5136) + send_delay,
          .event_time =
              e + chrono::microseconds(5136) + send_delay + network_delay,
          .value = 1,
      },
      // Sixth waits long enough to do the right thing as well (but kicks off
      // while the fifth is in flight over the network and has almost landed).
      // The timer wakeup for the Timestamp message coming back will find the
      // sixth message a little bit early.
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(5276),
          .remote_transmit_time = e + chrono::microseconds(5273) + send_delay,
          .event_time =
              e + chrono::microseconds(5273) + send_delay + network_delay,
          .value = 1,
      },
  };

  for (const ExpectedTimestamps value : expected_values) {
    ASSERT_TRUE(fetcher.FetchNext());
    EXPECT_EQ(fetcher.context().monotonic_remote_time, value.remote_time);
    EXPECT_EQ(fetcher.context().monotonic_remote_transmit_time,
              value.remote_transmit_time);
    EXPECT_EQ(fetcher.context().monotonic_event_time, value.event_time);
    EXPECT_EQ(fetcher->value(), value.value);
  }

  ASSERT_FALSE(fetcher.FetchNext());
}

// Tests that a reliable message gets forwarded if it was sent originally when
// nodes were disconnected.
TEST_F(SimulatedEventLoopDisconnectTest, ReliableMessageSendsOnConnect) {
  time.StartEqual();
  factory.SkipTimingReport();
  factory.DisableStatistics();

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  // Fully disconnect the nodes.
  pi1->Disconnect(pi2->node());
  pi2->Disconnect(pi1->node());

  std::unique_ptr<aos::EventLoop> pi2_event_loop =
      pi2->MakeEventLoop("fetcher");
  aos::Fetcher<examples::Ping> pi2_reliable_fetcher =
      pi2_event_loop->MakeFetcher<examples::Ping>("/reliable");

  factory.RunFor(chrono::milliseconds(100));

  {
    std::unique_ptr<aos::EventLoop> pi1_event_loop =
        pi1->MakeEventLoop("sender");
    aos::Sender<examples::Ping> pi1_reliable_sender =
        pi1_event_loop->MakeSender<examples::Ping>("/reliable");
    FunctionScheduler run_at(pi1_event_loop.get());
    aos::monotonic_clock::time_point now = pi1_event_loop->monotonic_now();
    for (int i = 0; i < 100; ++i) {
      run_at.ScheduleAt([&, i = i]() { SendPing(&pi1_reliable_sender, i); },
                        now);
      now += chrono::milliseconds(100);
    }
    now += chrono::milliseconds(50);

    factory.RunFor(now - pi1_event_loop->monotonic_now());
  }

  ASSERT_FALSE(pi2_reliable_fetcher.Fetch());

  pi1->Connect(pi2->node());
  pi2->Connect(pi1->node());

  factory.RunFor(chrono::milliseconds(1));

  ASSERT_TRUE(pi2_reliable_fetcher.Fetch());
  ASSERT_EQ(pi2_reliable_fetcher.context().monotonic_remote_time,
            monotonic_clock::epoch() + chrono::milliseconds(10000));
  ASSERT_EQ(pi2_reliable_fetcher.context().monotonic_remote_transmit_time,
            monotonic_clock::epoch() + chrono::milliseconds(10150));
  ASSERT_EQ(pi2_reliable_fetcher.context().monotonic_event_time,
            monotonic_clock::epoch() + chrono::milliseconds(10150) +
                factory.network_delay());
  ASSERT_EQ(pi2_reliable_fetcher->value(), 99);

  // TODO(austin): Verify that the dropped packet count increases.

  ASSERT_FALSE(pi2_reliable_fetcher.Fetch());
}

// Tests that if we disconnect while a message is in various states of being
// queued, it gets either dropped or sent as expected.
TEST_F(SimulatedEventLoopDisconnectTest, MessageInFlightDuringDisconnect) {
  time.StartEqual();
  factory.SkipTimingReport();
  factory.DisableStatistics();

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  std::unique_ptr<aos::EventLoop> pi1_event_loop = pi1->MakeEventLoop("sender");

  std::unique_ptr<aos::EventLoop> pi2_event_loop =
      pi2->MakeEventLoop("fetcher");
  aos::Fetcher<examples::Ping> fetcher =
      pi2_event_loop->MakeFetcher<examples::Ping>("/unreliable");

  ASSERT_FALSE(fetcher.Fetch());

  aos::monotonic_clock::time_point now = pi1_event_loop->monotonic_now();
  {
    FunctionScheduler run_at(pi1_event_loop.get());
    aos::Sender<examples::Ping> pi1_sender =
        pi1_event_loop->MakeSender<examples::Ping>("/unreliable");

    int i = 0;
    for (const std::chrono::nanoseconds dt :
         {chrono::microseconds(5000), chrono::microseconds(1),
          chrono::microseconds(2), chrono::microseconds(70),
          chrono::microseconds(63), chrono::microseconds(140),
          chrono::microseconds(160)}) {
      run_at.ScheduleAt(
          [&]() {
            pi1->Connect(pi2->node());
            pi2->Connect(pi1->node());
          },
          now);

      now += chrono::milliseconds(100);

      run_at.ScheduleAt([&, i = i]() { SendPing(&pi1_sender, i); }, now);

      now += dt;

      run_at.ScheduleAt(
          [&]() {
            // Fully disconnect the nodes.
            pi1->Disconnect(pi2->node());
            pi2->Disconnect(pi1->node());
          },
          now);

      now += chrono::milliseconds(100) - dt;
      ++i;
    }

    factory.RunFor(now - pi1_event_loop->monotonic_now());
  }

  const monotonic_clock::time_point e = monotonic_clock::epoch();
  const chrono::nanoseconds send_delay = factory.send_delay();
  const chrono::nanoseconds network_delay = factory.network_delay();

  const std::vector<ExpectedTimestamps> expected_values = {
      ExpectedTimestamps{
          .remote_time = e + chrono::milliseconds(100),
          .remote_transmit_time = e + chrono::milliseconds(100) + send_delay,
          .event_time =
              e + chrono::milliseconds(100) + send_delay + network_delay,
          .value = 0,
      },
      ExpectedTimestamps{
          .remote_time = e + chrono::milliseconds(1300),
          .remote_transmit_time = e + chrono::milliseconds(1300) + send_delay,
          .event_time =
              e + chrono::milliseconds(1300) + send_delay + network_delay,
          .value = 6,
      },
  };

  for (const ExpectedTimestamps value : expected_values) {
    ASSERT_TRUE(fetcher.FetchNext());
    EXPECT_EQ(fetcher.context().monotonic_remote_time, value.remote_time);
    EXPECT_EQ(fetcher.context().monotonic_remote_transmit_time,
              value.remote_transmit_time);
    EXPECT_EQ(fetcher.context().monotonic_event_time, value.event_time);
    EXPECT_EQ(fetcher->value(), value.value);
  }

  // TODO(austin): Verify that the dropped packet count increases.

  ASSERT_FALSE(fetcher.Fetch());
}

class PingLogger {
 public:
  PingLogger(aos::EventLoop *event_loop, std::string_view channel,
             std::vector<std::pair<aos::Context, int>> *msgs)
      : event_loop_(event_loop),
        fetcher_(event_loop_->MakeFetcher<examples::Ping>(channel)),
        msgs_(msgs) {
    event_loop_->OnRun([this]() { CHECK(!fetcher_.Fetch()); });
  }

  ~PingLogger() {
    while (fetcher_.FetchNext()) {
      msgs_->emplace_back(fetcher_.context(), fetcher_->value());
    }
  }

 private:
  aos::EventLoop *event_loop_;
  aos::Fetcher<examples::Ping> fetcher_;
  std::vector<std::pair<aos::Context, int>> *msgs_;
};

// Tests that rebooting while a message is in flight works as expected.
TEST_F(SimulatedEventLoopDisconnectTest, MessageInFlightDuringReboot) {
  time.StartEqual();
  for (int i = 0; i < 8; ++i) {
    time.RebootAt(1, distributed_clock::epoch() + chrono::seconds(10 * i));
  }

  factory.SkipTimingReport();
  factory.DisableStatistics();

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  std::unique_ptr<aos::EventLoop> pi1_event_loop = pi1->MakeEventLoop("sender");

  aos::monotonic_clock::time_point now = pi1_event_loop->monotonic_now();
  FunctionScheduler run_at(pi1_event_loop.get());
  aos::Sender<examples::Ping> pi1_sender =
      pi1_event_loop->MakeSender<examples::Ping>("/unreliable");

  int i = 0;
  for (const std::chrono::nanoseconds dt :
       {chrono::microseconds(5000), chrono::microseconds(1),
        chrono::microseconds(2), chrono::microseconds(70),
        chrono::microseconds(63), chrono::microseconds(140),
        chrono::microseconds(160)}) {
    run_at.ScheduleAt([&, i = i]() { SendPing(&pi1_sender, i); },
                      now + chrono::seconds(10) - dt);

    now += chrono::seconds(10);
    ++i;
  }

  std::vector<std::pair<aos::Context, int>> msgs;

  pi2->OnStartup([pi2, &msgs]() {
    pi2->AlwaysStart<PingLogger>("ping_logger", "/unreliable", &msgs);
  });

  factory.RunFor(now - pi1_event_loop->monotonic_now() + chrono::seconds(10));

  const monotonic_clock::time_point e = monotonic_clock::epoch();
  const chrono::nanoseconds send_delay = factory.send_delay();
  const chrono::nanoseconds network_delay = factory.network_delay();

  const std::vector<ExpectedTimestamps> expected_values = {
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(9995000),
          .remote_transmit_time =
              e + chrono::microseconds(9995000) + send_delay,
          .event_time =
              e + chrono::microseconds(9995000) + send_delay + network_delay,
          .value = 0,
      },
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(19999999),
          .remote_transmit_time =
              e + chrono::microseconds(19999999) + send_delay,
          .event_time =
              e + chrono::microseconds(-1) + send_delay + network_delay,
          .value = 1,
      },
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(29999998),
          .remote_transmit_time =
              e + chrono::microseconds(29999998) + send_delay,
          .event_time =
              e + chrono::microseconds(-2) + send_delay + network_delay,
          .value = 2,
      },
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(69999840),
          .remote_transmit_time =
              e + chrono::microseconds(69999840) + send_delay,
          .event_time =
              e + chrono::microseconds(9999840) + send_delay + network_delay,
          .value = 6,
      },
  };

  ASSERT_EQ(msgs.size(), expected_values.size());

  for (size_t i = 0; i < msgs.size(); ++i) {
    EXPECT_EQ(msgs[i].first.monotonic_remote_time,
              expected_values[i].remote_time);
    EXPECT_EQ(msgs[i].first.monotonic_remote_transmit_time,
              expected_values[i].remote_transmit_time);
    EXPECT_EQ(msgs[i].first.monotonic_event_time,
              expected_values[i].event_time);
    EXPECT_EQ(msgs[i].second, expected_values[i].value);
  }

  // TODO(austin): Verify that the dropped packet count increases.
}

// Tests that rebooting while a message is in flight works as expected.
TEST_F(SimulatedEventLoopDisconnectTest, ReliableMessageInFlightDuringReboot) {
  time.StartEqual();
  for (int i = 0; i < 8; ++i) {
    time.RebootAt(1, distributed_clock::epoch() + chrono::seconds(10 * i));
  }

  factory.SkipTimingReport();
  factory.DisableStatistics();

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  std::unique_ptr<aos::EventLoop> pi1_event_loop = pi1->MakeEventLoop("sender");

  aos::monotonic_clock::time_point now = pi1_event_loop->monotonic_now();
  FunctionScheduler run_at(pi1_event_loop.get());
  aos::Sender<examples::Ping> pi1_sender =
      pi1_event_loop->MakeSender<examples::Ping>("/reliable");

  int i = 0;
  for (const std::chrono::nanoseconds dt :
       {chrono::microseconds(5000), chrono::microseconds(1),
        chrono::microseconds(2), chrono::microseconds(70),
        chrono::microseconds(63), chrono::microseconds(140),
        chrono::microseconds(160)}) {
    run_at.ScheduleAt([&, i = i]() { SendPing(&pi1_sender, i); },
                      now + chrono::seconds(10) - dt);

    now += chrono::seconds(10);
    ++i;
  }

  std::vector<std::pair<aos::Context, int>> msgs;

  PingLogger *logger;
  pi2->OnStartup([pi2, &msgs, &logger]() {
    logger = pi2->AlwaysStart<PingLogger>("ping_logger", "/reliable", &msgs);
  });

  factory.RunFor(now - pi1_event_loop->monotonic_now() + chrono::seconds(10));

  // Stop the logger to flush the last boot of data.
  pi2->Stop(logger);

  const monotonic_clock::time_point e = monotonic_clock::epoch();
  const chrono::nanoseconds send_delay = factory.send_delay();
  const chrono::nanoseconds network_delay = factory.network_delay();

  // Verified using --vmodule=simulated_event_loop=1 and looking at the actual
  // event times to confirm what should have been forwarded when.
  const std::vector<ExpectedTimestamps> expected_values = {
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(9995000),
          .remote_transmit_time =
              e + chrono::microseconds(9995000) + send_delay,
          .event_time =
              e + chrono::microseconds(9995000) + send_delay + network_delay,
          .value = 0,
      },
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(9995000),
          .remote_transmit_time = e + chrono::microseconds(10000000),
          .event_time = e + network_delay,
          .value = 0,
      },
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(19999999),
          .remote_transmit_time = e + chrono::microseconds(20000000),
          .event_time = e + network_delay,
          .value = 1,
      },
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(29999998),
          .remote_transmit_time = e + chrono::microseconds(30000000),
          .event_time = e + network_delay,
          .value = 2,
      },
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(39999930),
          .remote_transmit_time = e + chrono::microseconds(40000000),
          .event_time = e + network_delay,
          .value = 3,
      },
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(49999937),
          .remote_transmit_time = e + chrono::microseconds(50000000),
          .event_time = e + network_delay,
          .value = 4,
      },
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(59999860),
          .remote_transmit_time = e + chrono::microseconds(60000000),
          .event_time = e + network_delay,
          .value = 5,
      },
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(69999840),
          .remote_transmit_time = e + chrono::microseconds(69999890),
          .event_time = e + chrono::microseconds(9999890) + network_delay,
          .value = 6,
      },
      ExpectedTimestamps{
          .remote_time = e + chrono::microseconds(69999840),
          .remote_transmit_time = e + chrono::microseconds(70000000),
          .event_time = e + network_delay,
          .value = 6,
      },
  };

  ASSERT_EQ(msgs.size(), expected_values.size());

  for (size_t i = 0; i < msgs.size(); ++i) {
    EXPECT_EQ(msgs[i].first.monotonic_remote_time,
              expected_values[i].remote_time);
    EXPECT_EQ(msgs[i].first.monotonic_remote_transmit_time,
              expected_values[i].remote_transmit_time);
    EXPECT_EQ(msgs[i].first.monotonic_event_time,
              expected_values[i].event_time);
    EXPECT_EQ(msgs[i].second, expected_values[i].value);
  }

  // TODO(austin): Verify that the dropped packet count increases.
}

namespace {

// Creates a copy of a file descriptor and then closes the original.
class BackupFD {
 public:
  BackupFD(int fd) : original_fd_(fd), copied_fd_(dup(fd)) {
    PCHECK(copied_fd_ != -1) << ": Failed to dup(" << fd << ")";
    PCHECK(close(original_fd_) != -1);
  }
  ~BackupFD() {
    // Close the possibly-duplicated file descriptor that someone else created.
    // Ignore return codes here since the caller may have closed this already.
    close(original_fd_);

    // Now put the original file descriptor back to where it was.
    PCHECK(dup2(copied_fd_, original_fd_) != -1);
    PCHECK(close(copied_fd_) != -1);
  }

 private:
  const int original_fd_;
  const int copied_fd_;
};

// Logs all data written to the specified file descriptor in the log file.
class OutputCaptor {
 public:
  OutputCaptor(int fd, std::filesystem::path output)
      // Create a backup of the original file descriptor. This will close it.
      : backup_fd_(fd),
        // Create an output file for the logged data.
        file_writer_(output.string(), S_IRWXU) {
    // Point all data written to the original file descriptor to the log file
    // instead.
    PCHECK(dup2(file_writer_.fd(), fd) != -1);
  }

 private:
  BackupFD backup_fd_;
  util::FileWriter file_writer_;
};

}  // namespace

// Validates that setting --use_simulated_clocks_for_logs causes LOG statements
// to use the simulated clocks and print the node name.
TEST(SimulatedEventLoopTest, SimulatedLogSink) {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(ArtifactPath(
          "aos/events/multinode_pingpong_test_combined_config.json"));

  SimulatedEventLoopFactory factory(&config.message());

  NodeEventLoopFactory *pi1 = factory.GetNodeEventLoopFactory("pi1");
  NodeEventLoopFactory *pi2 = factory.GetNodeEventLoopFactory("pi2");

  // Add some realtime offset to make the test a bit more interesting.
  pi1->SetRealtimeOffset(monotonic_clock::epoch(),
                         realtime_clock::epoch() + std::chrono::days(5));
  pi2->SetRealtimeOffset(monotonic_clock::epoch(),
                         realtime_clock::epoch() + std::chrono::days(118) +
                             std::chrono::hours(4) + std::chrono::minutes(30));

  std::unique_ptr<EventLoop> pi1_event_loop = pi1->MakeEventLoop("ping");
  std::unique_ptr<EventLoop> pi2_event_loop = pi2->MakeEventLoop("pong");

  Ping ping(pi1_event_loop.get());
  Pong pong(pi2_event_loop.get());

  const auto output_path = std::filesystem::path(testing::TestTmpDir()) /
                           "simulated_log_sink_output.txt";
  {
    absl::FlagSaver flag_saver;
    // Set the flag to enable the use of simulated clocks for LOG statements.
    absl::SetFlag(&FLAGS_use_simulated_clocks_for_logs, true);
    // Make sure that we actually hit some LOG statements.
    absl::SetFlag(&FLAGS_vmodule, "ping_lib=2,pong_lib=2");
    absl::SetFlag(&FLAGS_die_on_malloc, false);

    // Save the output to a file.
    OutputCaptor output_captor(STDERR_FILENO, output_path);

    // Run for a short time here. We just need to validate that a few LOG
    // statements work as expected. We don't need anything huge here.
    factory.RunFor(std::chrono::milliseconds(100));
  }

  const std::string output = util::ReadFileToStringOrDie(output_path.string());
  EXPECT_THAT(output, ::testing::ContainsRegex(R"(
I0106 00:00:00\.000000 pi1 .*? ping_lib\.cc:.*?] Sending ping
I0429 04:30:00\.000200 pi2 .*? pong_lib\.cc:.*?] Sending pong
I0106 00:00:00\.000400 pi1 .*? ping_lib\.cc:.*?] Elapsed time 400000 ns \{ "value": 1, "initial_send_time": 0 \}
I0106 00:00:00\.010000 pi1 .*? ping_lib\.cc:.*?] Sending ping
I0429 04:30:00\.010200 pi2 .*? pong_lib\.cc:.*?] Sending pong
I0106 00:00:00\.010400 pi1 .*? ping_lib\.cc:.*?] Elapsed time 400000 ns \{ "value": 2, "initial_send_time": 10000000 \}
I0106 00:00:00\.020000 pi1 .*? ping_lib\.cc:.*?] Sending ping
I0429 04:30:00\.020200 pi2 .*? pong_lib\.cc:.*?] Sending pong
I0106 00:00:00\.020400 pi1 .*? ping_lib\.cc:.*?] Elapsed time 400000 ns \{ "value": 3, "initial_send_time": 20000000 \}
I0106 00:00:00\.030000 pi1 .*? ping_lib\.cc:.*?] Sending ping
I0429 04:30:00\.030200 pi2 .*? pong_lib\.cc:.*?] Sending pong
I0106 00:00:00\.030400 pi1 .*? ping_lib\.cc:.*?] Elapsed time 400000 ns \{ "value": 4, "initial_send_time": 30000000 \}
I0106 00:00:00\.040000 pi1 .*? ping_lib\.cc:.*?] Sending ping
I0429 04:30:00\.040200 pi2 .*? pong_lib\.cc:.*?] Sending pong
I0106 00:00:00\.040400 pi1 .*? ping_lib\.cc:.*?] Elapsed time 400000 ns \{ "value": 5, "initial_send_time": 40000000 \}
I0106 00:00:00\.050000 pi1 .*? ping_lib\.cc:.*?] Sending ping
I0429 04:30:00\.050200 pi2 .*? pong_lib\.cc:.*?] Sending pong
I0106 00:00:00\.050400 pi1 .*? ping_lib\.cc:.*?] Elapsed time 400000 ns \{ "value": 6, "initial_send_time": 50000000 \}
I0106 00:00:00\.060000 pi1 .*? ping_lib\.cc:.*?] Sending ping
I0429 04:30:00\.060200 pi2 .*? pong_lib\.cc:.*?] Sending pong
I0106 00:00:00\.060400 pi1 .*? ping_lib\.cc:.*?] Elapsed time 400000 ns \{ "value": 7, "initial_send_time": 60000000 \}
I0106 00:00:00\.070000 pi1 .*? ping_lib\.cc:.*?] Sending ping
I0429 04:30:00\.070200 pi2 .*? pong_lib\.cc:.*?] Sending pong
I0106 00:00:00\.070400 pi1 .*? ping_lib\.cc:.*?] Elapsed time 400000 ns \{ "value": 8, "initial_send_time": 70000000 \}
I0106 00:00:00\.080000 pi1 .*? ping_lib\.cc:.*?] Sending ping
I0429 04:30:00\.080200 pi2 .*? pong_lib\.cc:.*?] Sending pong
I0106 00:00:00\.080400 pi1 .*? ping_lib\.cc:.*?] Elapsed time 400000 ns \{ "value": 9, "initial_send_time": 80000000 \}
I0106 00:00:00\.090000 pi1 .*? ping_lib\.cc:.*?] Sending ping
I0429 04:30:00\.090200 pi2 .*? pong_lib\.cc:.*?] Sending pong
I0106 00:00:00\.090400 pi1 .*? ping_lib\.cc:.*?] Elapsed time 400000 ns \{ "value": 10, "initial_send_time": 90000000 \}
I0106 00:00:00\.100000 pi1 .*? ping_lib\.cc:.*?] Sending ping
)"));
}

}  // namespace aos::testing
