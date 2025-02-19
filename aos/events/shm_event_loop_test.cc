#include "aos/events/shm_event_loop.h"

#include <string_view>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/events/event_loop_param_test.h"
#include "aos/events/test_message_generated.h"
#include "aos/network/team_number.h"
#include "aos/realtime.h"

namespace aos::testing {
namespace {
namespace chrono = ::std::chrono;

class ShmEventLoopTestFactory : public EventLoopTestFactory {
 public:
  ShmEventLoopTestFactory() {
    // Clean up anything left there before.
    unlink(
        (absl::GetFlag(FLAGS_shm_base) + "/test/aos.TestMessage.v7").c_str());
    unlink(
        (absl::GetFlag(FLAGS_shm_base) + "/test1/aos.TestMessage.v7").c_str());
    unlink(
        (absl::GetFlag(FLAGS_shm_base) + "/test2/aos.TestMessage.v7").c_str());
    unlink(
        (absl::GetFlag(FLAGS_shm_base) + "/test2/aos.TestMessage.v7").c_str());
    unlink(
        (absl::GetFlag(FLAGS_shm_base) + "/aos/aos.timing.Report.v7").c_str());
    unlink((absl::GetFlag(FLAGS_shm_base) + "/aos/aos.logging.LogMessageFbs.v7")
               .c_str());
  }

  ~ShmEventLoopTestFactory() { absl::SetFlag(&FLAGS_override_hostname, ""); }

  ::std::unique_ptr<EventLoop> Make(std::string_view name) override {
    if (configuration()->has_nodes()) {
      absl::SetFlag(&FLAGS_override_hostname,
                    std::string(my_node()->hostname()->string_view()));
    }
    ::std::unique_ptr<ShmEventLoop> loop(new ShmEventLoop(configuration()));
    loop->set_name(name);
    return loop;
  }

  ::std::unique_ptr<EventLoop> MakePrimary(std::string_view name) override {
    if (configuration()->has_nodes()) {
      absl::SetFlag(&FLAGS_override_hostname,
                    std::string(my_node()->hostname()->string_view()));
    }
    ::std::unique_ptr<ShmEventLoop> loop =
        ::std::unique_ptr<ShmEventLoop>(new ShmEventLoop(configuration()));
    primary_event_loop_ = loop.get();
    loop->set_name(name);
    return loop;
  }

  Result<void> Run() override {
    CHECK(primary_event_loop_ != nullptr);
    return primary_event_loop_->Run();
  }

  std::unique_ptr<ExitHandle> MakeExitHandle() override {
    CHECK(primary_event_loop_ != nullptr);
    return primary_event_loop_->MakeExitHandle();
  }

  void Exit() override {
    CHECK(primary_event_loop_ != nullptr);
    primary_event_loop_->Exit();
  }

  void SleepFor(::std::chrono::nanoseconds duration) override {
    ::std::this_thread::sleep_for(duration);
  }

 private:
  ::aos::ShmEventLoop *primary_event_loop_ = nullptr;
};

auto CommonParameters() {
  return ::testing::Combine(
      ::testing::Values([]() { return new ShmEventLoopTestFactory(); }),
      ::testing::Values(ReadMethod::COPY, ReadMethod::PIN),
      ::testing::Values(DoTimingReports::kYes, DoTimingReports::kNo));
}

INSTANTIATE_TEST_SUITE_P(ShmEventLoopCommonTest, AbstractEventLoopTest,
                         CommonParameters());

INSTANTIATE_TEST_SUITE_P(ShmEventLoopCommonDeathTest,
                         AbstractEventLoopDeathTest, CommonParameters());

}  // namespace

bool IsRealtime() {
  int scheduler;
  PCHECK((scheduler = sched_getscheduler(0)) != -1);

  {
    // If we are RT, logging the scheduler will crash us.  Mark that we just
    // don't care.
    aos::ScopedNotRealtime nrt;
    LOG(INFO) << "scheduler is " << scheduler;
  }

  const bool result = scheduler == SCHED_FIFO || scheduler == SCHED_RR;
  // Confirm that the scheduler matches AOS' interpretation of if we are
  // realtime or not.
  if (result) {
    aos::CheckRealtime();
  } else {
    aos::CheckNotRealtime();
  }
  return result;
}

class ShmEventLoopTest : public ::testing::TestWithParam<ReadMethod> {
 public:
  ShmEventLoopTest() {
    if (GetParam() == ReadMethod::PIN) {
      factory_.PinReads();
    }
  }

  ShmEventLoopTestFactory *factory() { return &factory_; }

  // Helper functions for testing when a fetcher cannot fetch the next message
  // because it was overwritten
  void TestNextMessageNotAvailable(const bool skip_timing_report) {
    auto loop1 = factory()->MakePrimary("loop1");
    if (skip_timing_report) {
      loop1->SkipTimingReport();
    }
    auto fetcher = loop1->MakeFetcher<TestMessage>("/test");
    auto loop2 = factory()->Make("loop2");
    auto sender = loop2->MakeSender<TestMessage>("/test");
    bool ran = false;
    loop1->AddPhasedLoop(
        [&sender](int) {
          auto builder = sender.MakeBuilder();
          TestMessage::Builder test_builder(*builder.fbb());
          test_builder.add_value(0);
          builder.CheckOk(builder.Send(test_builder.Finish()));
        },
        std::chrono::milliseconds(2));
    loop1
        ->AddTimer([this, &fetcher, &ran]() {
          EXPECT_DEATH(fetcher.FetchNext(),
                       "The next message is no longer "
                       "available.*\"/test\".*\"aos\\.TestMessage\"");
          factory()->Exit();
          ran = true;
        })
        ->Schedule(loop1->monotonic_now() + std::chrono::seconds(4));
    factory()->Run();
    EXPECT_TRUE(ran);
  }
  void TestNextMessageNotAvailableNoRun(const bool skip_timing_report) {
    auto loop1 = factory()->MakePrimary("loop1");
    if (skip_timing_report) {
      loop1->SkipTimingReport();
    }
    auto fetcher = loop1->MakeFetcher<TestMessage>("/test");
    auto loop2 = factory()->Make("loop2");
    auto sender = loop2->MakeSender<TestMessage>("/test");
    time::PhasedLoop phased_loop(std::chrono::milliseconds(2),
                                 loop2->monotonic_now());
    for (int i = 0; i < 2000; ++i) {
      auto builder = sender.MakeBuilder();
      TestMessage::Builder test_builder(*builder.fbb());
      test_builder.add_value(0);
      builder.CheckOk(builder.Send(test_builder.Finish()));
      phased_loop.SleepUntilNext();
    }
    EXPECT_DEATH(fetcher.FetchNext(),
                 "The next message is no longer "
                 "available.*\"/test\".*\"aos\\.TestMessage\"");
  }

 private:
  ShmEventLoopTestFactory factory_;
};

using ShmEventLoopDeathTest = ShmEventLoopTest;

// Tests that we don't leave the calling thread realtime when calling Send
// before Run.
TEST_P(ShmEventLoopTest, SendBeforeRun) {
  auto loop = factory()->MakePrimary("primary");
  loop->SetRuntimeRealtimePriority(1);

  auto loop2 = factory()->Make("loop2");
  loop2->SetRuntimeRealtimePriority(2);
  loop2->MakeWatcher("/test", [](const TestMessage &) {});
  // Need the other one running for its watcher to record in SHM that it wants
  // wakers to boost their priority, so leave it running in a thread for this
  // test.
  std::thread loop2_thread(
      [&loop2]() { static_cast<ShmEventLoop *>(loop2.get())->Run(); });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto sender = loop->MakeSender<TestMessage>("/test");
  EXPECT_FALSE(IsRealtime());
  {
    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    msg.CheckOk(msg.Send(builder.Finish()));
  }
  EXPECT_FALSE(IsRealtime());

  static_cast<ShmEventLoop *>(loop2.get())->Exit();
  loop2_thread.join();
}

// Tests that every handler type is realtime and runs.  There are threads
// involved and it's easy to miss one.
TEST_P(ShmEventLoopTest, AllHandlersAreRealtime) {
  auto loop = factory()->MakePrimary("primary");
  auto loop2 = factory()->Make("loop2");

  loop->SetRuntimeRealtimePriority(1);

  auto sender = loop2->MakeSender<TestMessage>("/test");

  bool did_onrun = false;
  bool did_timer = false;
  bool did_watcher = false;

  auto timer = loop->AddTimer([this, &did_timer]() {
    EXPECT_TRUE(IsRealtime());
    did_timer = true;
    factory()->Exit();
  });

  loop->MakeWatcher("/test", [&did_watcher](const TestMessage &) {
    EXPECT_TRUE(IsRealtime());
    did_watcher = true;
  });

  loop->OnRun([&loop, &did_onrun, &sender, timer]() {
    EXPECT_TRUE(IsRealtime());
    did_onrun = true;
    timer->Schedule(loop->monotonic_now() + chrono::milliseconds(100));

    aos::Sender<TestMessage>::Builder msg = sender.MakeBuilder();
    TestMessage::Builder builder = msg.MakeBuilder<TestMessage>();
    builder.add_value(200);
    msg.CheckOk(msg.Send(builder.Finish()));
  });

  factory()->Run();

  EXPECT_TRUE(did_onrun);
  EXPECT_TRUE(did_timer);
  EXPECT_TRUE(did_watcher);
}

// Tests that missing a deadline inside the function still results in PhasedLoop
// running at the right offset.
TEST_P(ShmEventLoopTest, DelayedPhasedLoop) {
  auto loop1 = factory()->MakePrimary("primary");

  ::std::vector<::aos::monotonic_clock::time_point> times;

  constexpr chrono::milliseconds kOffset = chrono::milliseconds(400);

  loop1->AddPhasedLoop(
      [this, &times, &loop1, &kOffset](int count) {
        const ::aos::monotonic_clock::time_point monotonic_now =
            loop1->monotonic_now();

        // Compute our offset.
        const ::aos::monotonic_clock::duration remainder =
            monotonic_now.time_since_epoch() -
            chrono::duration_cast<chrono::seconds>(
                monotonic_now.time_since_epoch());

        // Make sure we we are called near where we should be even when we
        // delay.
        constexpr chrono::milliseconds kEpsilon(200);
        EXPECT_LT(remainder, kOffset + kEpsilon);
        EXPECT_GT(remainder, kOffset - kEpsilon);

        // Confirm that we see the missed count when we sleep.
        if (times.size() == 0) {
          CHECK_EQ(count, 1);
        } else {
          CHECK_EQ(count, 3);
        }

        times.push_back(loop1->monotonic_now());
        if (times.size() == 2) {
          factory()->Exit();
        }

        // Now, add a large delay.  This should push us up to 3 cycles.
        ::std::this_thread::sleep_for(chrono::milliseconds(2500));
      },
      chrono::seconds(1), kOffset);

  factory()->Run();

  EXPECT_EQ(times.size(), 2u);
}

// Tests that the ShmEventLoop::Exit() method causes the ShmEventLoop to return
// with a successful status.
TEST_P(ShmEventLoopTest, SuccessfulExitTest) {
  auto loop1 = factory()->MakePrimary("primary");
  auto exit_handle = factory()->MakeExitHandle();

  loop1->OnRun([this, &exit_handle]() {
    factory()->Exit();
    // The second Exit() call should get ignored.
    exit_handle->Exit(aos::Error::MakeUnexpectedError("Hello, World!"));
  });

  EXPECT_TRUE(factory()->Run().has_value());
}

// Test GetWatcherSharedMemory in a few basic scenarios.
TEST_P(ShmEventLoopDeathTest, GetWatcherSharedMemory) {
  auto generic_loop1 = factory()->MakePrimary("primary");
  ShmEventLoop *const loop1 = static_cast<ShmEventLoop *>(generic_loop1.get());
  const auto channel = configuration::GetChannel(
      loop1->configuration(), "/test", TestMessage::GetFullyQualifiedName(),
      loop1->name(), loop1->node());

  // First verify it handles an invalid channel reasonably.
  EXPECT_DEATH(loop1->GetWatcherSharedMemory(channel),
               "No watcher found for channel");

  // Then, actually create a watcher, and verify it returns something sane.
  absl::Span<const char> shared_memory;
  bool ran = false;
  loop1->MakeWatcher("/test", [this, &shared_memory,
                               &ran](const TestMessage &message) {
    EXPECT_FALSE(ran);
    ran = true;
    // If we're using pinning, then we can verify that the message is actually
    // in the specified region.
    if (GetParam() == ReadMethod::PIN) {
      EXPECT_GE(reinterpret_cast<const char *>(&message),
                shared_memory.begin());
      EXPECT_LT(reinterpret_cast<const char *>(&message), shared_memory.end());
    }
    factory()->Exit();
  });
  loop1->SetWatcherUseWritableMemory(channel, true);
  shared_memory = loop1->GetWatcherSharedMemory(channel);
  EXPECT_FALSE(shared_memory.empty());

  auto loop2 = factory()->Make("sender");
  auto sender = loop2->MakeSender<TestMessage>("/test");
  generic_loop1->OnRun([&sender]() {
    auto builder = sender.MakeBuilder();
    TestMessage::Builder test_builder(*builder.fbb());
    test_builder.add_value(1);
    builder.CheckOk(builder.Send(test_builder.Finish()));
  });
  factory()->Run();
  EXPECT_TRUE(ran);
}

TEST_P(ShmEventLoopTest, GetSenderSharedMemory) {
  auto generic_loop1 = factory()->MakePrimary("primary");
  ShmEventLoop *const loop1 = static_cast<ShmEventLoop *>(generic_loop1.get());

  // Check that GetSenderSharedMemory returns non-null/non-empty memory span.
  auto sender = loop1->MakeSender<TestMessage>("/test");
  const absl::Span<char> shared_memory = loop1->GetSenderSharedMemory(&sender);
  EXPECT_FALSE(shared_memory.empty());

  auto builder = sender.MakeBuilder();
  uint8_t *buffer;
  builder.fbb()->CreateUninitializedVector(5, &buffer);
  EXPECT_GE(reinterpret_cast<char *>(buffer), shared_memory.begin());
  EXPECT_LT(reinterpret_cast<char *>(buffer), shared_memory.end());
}

TEST_P(ShmEventLoopTest, GetFetcherPrivateMemory) {
  auto generic_loop1 = factory()->MakePrimary("primary");
  ShmEventLoop *const loop1 = static_cast<ShmEventLoop *>(generic_loop1.get());

  // Check that GetFetcherPrivateMemory returns non-null/non-empty memory span.
  auto fetcher = loop1->MakeFetcher<TestMessage>("/test");
  const auto private_memory = loop1->GetFetcherPrivateMemory(&fetcher);
  EXPECT_FALSE(private_memory.empty());

  auto loop2 = factory()->Make("sender");
  auto sender = loop2->MakeSender<TestMessage>("/test");
  {
    auto builder = sender.MakeBuilder();
    const std::vector<uint8_t> data{1, 2, 3};
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
        builder.fbb()->CreateVector(data);

    TestMessage::Builder test_builder(*builder.fbb());
    test_builder.add_value(1);
    test_builder.add_data(data_offset);

    builder.CheckOk(builder.Send(test_builder.Finish()));
  }

  ASSERT_TRUE(fetcher.Fetch());
  EXPECT_GE(fetcher.context().data, private_memory.begin());
  EXPECT_LT(fetcher.context().data, private_memory.end());

  if (GetParam() == ReadMethod::PIN) {
    // For pinned messages only, we can get access to the full underlying
    // memory. For copied messages, we only get a sub-portion of the memory so
    // this test won't work. Validate that we can also access the underlying
    // memory in a writable manner.
    ASSERT_TRUE(fetcher->has_data());
    EXPECT_THAT(*fetcher->data(), ::testing::ElementsAre(1, 2, 3));

    // Get read-write access to the fetcher's underlying memory. It should not
    // be overlapping with the private memory.
    const auto shared_memory = loop1->GetFetcherSharedMemory(&fetcher);
    EXPECT_TRUE(shared_memory.end() <= private_memory.data() ||
                shared_memory.data() >= private_memory.end());
    const ptrdiff_t offset =
        fetcher->data()->data() -
        reinterpret_cast<const uint8_t *>(private_memory.data());
    // Do some sanity checks that we got the right data.
    EXPECT_EQ(shared_memory[offset + 0], 1);
    EXPECT_EQ(shared_memory[offset + 1], 2);
    EXPECT_EQ(shared_memory[offset + 2], 3);

    // Change the fetcher's result by poking directly at the shared memory.
    shared_memory[offset] = 5;
    EXPECT_THAT(*fetcher->data(), ::testing::ElementsAre(5, 2, 3));
  }
}

// Validates that we can make fetchers point at writable memory.
TEST_P(ShmEventLoopTest, SetFetcherUseWritableMemory) {
  auto generic_loop1 = factory()->MakePrimary("primary");
  ShmEventLoop *const loop1 = static_cast<ShmEventLoop *>(generic_loop1.get());

  // Check that GetFetcherSharedMemory returns non-null/non-empty memory span.
  auto fetcher = loop1->MakeFetcher<TestMessage>("/test");
  const auto shared_memory = loop1->GetFetcherSharedMemory(&fetcher);
  EXPECT_FALSE(shared_memory.empty());

  loop1->SetFetcherUseWritableMemory(&fetcher, true);

  // Send out a simple message with some bytes stuffed in.
  auto loop2 = factory()->Make("sender");
  auto sender = loop2->MakeSender<TestMessage>("/test");
  {
    auto builder = sender.MakeBuilder();
    const std::vector<uint8_t> data{1, 2, 3};
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
        builder.fbb()->CreateVector(data);
    TestMessage::Builder test_builder(*builder.fbb());
    test_builder.add_data(data_offset);
    builder.CheckOk(builder.Send(test_builder.Finish()));
  }

  ASSERT_TRUE(fetcher.Fetch());

  if (GetParam() == ReadMethod::PIN) {
    // For pinned messages only, we can get access to the full underlying
    // memory. For copied messages, we only get a sub-portion of the memory so
    // this test won't work. Validate that we can also access the underlying
    // memory in a writable manner.
    ASSERT_TRUE(fetcher->has_data());
    EXPECT_THAT(*fetcher->data(), ::testing::ElementsAre(1, 2, 3));

    // Get read-write access to the fetcher's underlying memory. Since we called
    // SetFetcherUseWritableMemory(true), we can modify the memory directly.
    uint8_t *data = const_cast<uint8_t *>(fetcher->data()->data());

    // Validate that the message is in the writable section.
    EXPECT_GE(data, reinterpret_cast<uint8_t *>(shared_memory.begin()));
    EXPECT_LT(data, reinterpret_cast<uint8_t *>(shared_memory.end()));

    // Change the fetcher's result by poking directly at the shared memory.
    *data = 5;
    EXPECT_THAT(*fetcher->data(), ::testing::ElementsAre(5, 2, 3));
  }
}

// Tests that corrupting the bytes around the data buffer results in a crash.
TEST_P(ShmEventLoopDeathTest, OutOfBoundsWrite) {
  auto loop1 = factory()->Make("loop1");
  std::unique_ptr<aos::RawSender> sender =
      loop1->MakeRawSender(configuration::GetChannel(
          loop1->configuration(), "/test", "aos.TestMessage", "", nullptr));
  for (size_t i = 0; i < kChannelDataRedzone; ++i) {
    SCOPED_TRACE(std::to_string(i));
    EXPECT_DEATH(
        {
          // Can't use `data()[-1 -i]` here because that's undefined behaviour.
          // We do manual pointer arithmetic to work around that.
          ++(*(static_cast<char *>(sender->data()) - 1 - i));
          sender->CheckOk(sender->Send(0));
        },
        "Somebody wrote outside the buffer of their message");
    EXPECT_DEATH(
        {
          ++static_cast<char *>(sender->data())[sender->size() + i];
          sender->CheckOk(sender->Send(0));
        },
        "Somebody wrote outside the buffer of their message");
  }
}

// Tests that the next message not being available prints a helpful error in the
// normal case.
TEST_P(ShmEventLoopDeathTest, NextMessageNotAvailable) {
  TestNextMessageNotAvailable(false);
}

// Tests that the next message not being available prints a helpful error with
// timing reports disabled.
TEST_P(ShmEventLoopDeathTest, NextMessageNotAvailableNoTimingReports) {
  TestNextMessageNotAvailable(true);
}

// Tests that the next message not being available prints a helpful error even
// when Run is never called.
TEST_P(ShmEventLoopDeathTest, NextMessageNotAvailableNoRun) {
  TestNextMessageNotAvailableNoRun(false);
}

// Tests that the next message not being available prints a helpful error even
// when Run is never called without timing reports.
TEST_P(ShmEventLoopDeathTest, NextMessageNotAvailableNoRunNoTimingReports) {
  TestNextMessageNotAvailableNoRun(true);
}

// Test that an ExitHandle outliving its EventLoop is caught.
TEST_P(ShmEventLoopDeathTest, ExitHandleOutlivesEventLoop) {
  auto loop1 = factory()->MakePrimary("loop1");
  auto exit_handle = static_cast<ShmEventLoop *>(loop1.get())->MakeExitHandle();
  EXPECT_DEATH(loop1.reset(),
               "All ExitHandles must be destroyed before the ShmEventLoop");
}

// TODO(austin): Test that missing a deadline with a timer recovers as expected.

INSTANTIATE_TEST_SUITE_P(ShmEventLoopCopyTest, ShmEventLoopTest,
                         ::testing::Values(ReadMethod::COPY));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopPinTest, ShmEventLoopTest,
                         ::testing::Values(ReadMethod::PIN));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopCopyDeathTest, ShmEventLoopDeathTest,
                         ::testing::Values(ReadMethod::COPY));
INSTANTIATE_TEST_SUITE_P(ShmEventLoopPinDeathTest, ShmEventLoopDeathTest,
                         ::testing::Values(ReadMethod::PIN));

}  // namespace aos::testing
