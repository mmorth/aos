#include "aos/configuration.h"

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/strip.h"
#include "flatbuffers/reflection.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "aos/configuration_static.h"
#include "aos/events/ping_generated.h"
#include "aos/json_to_flatbuffer.h"
#include "aos/testing/flatbuffer_eq.h"
#include "aos/testing/path.h"
#include "aos/testing/test_logging.h"
#include "aos/util/file.h"

namespace aos::configuration::testing {

using aos::testing::ArtifactPath;
namespace chrono = std::chrono;

class ConfigurationTest : public ::testing::Test {
 public:
  ConfigurationTest() { ::aos::testing::EnableTestLogging(); }
};

typedef ConfigurationTest ConfigurationDeathTest;

// *the* expected location for all working tests.
aos::FlatbufferDetachedBuffer<Channel> ExpectedLocation() {
  return JsonToFlatbuffer<Channel>(
      "{ \"name\": \"/foo\", \"type\": \".aos.bar\", \"max_size\": 5 }");
}

// And for multinode setups
aos::FlatbufferDetachedBuffer<Channel> ExpectedMultinodeLocation() {
  return JsonToFlatbuffer<Channel>(
      "{ \"name\": \"/foo\", \"type\": \".aos.bar\", \"max_size\": 5, "
      "\"source_node\": \"pi1\" }");
}

// Tests that we can read and merge a configuration.
TEST_F(ConfigurationTest, ConfigMerge) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));
  LOG(INFO) << "Read: " << FlatbufferToJson(config, {.multi_line = true});

  EXPECT_EQ(absl::StripSuffix(util::ReadFileToStringOrDie(
                                  ArtifactPath("aos/testdata/expected.json")),
                              "\n"),
            FlatbufferToJson(config, {.multi_line = true}));
}

// Tests that we can get back a ChannelIndex.
TEST_F(ConfigurationTest, ChannelIndex) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));

  EXPECT_EQ(
      ChannelIndex(&config.message(), config.message().channels()->Get(1u)),
      1u);
}

// Tests that we can read and merge a multinode configuration.
TEST_F(ConfigurationTest, ConfigMergeMultinode) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1_multinode.json"));
  LOG(INFO) << "Read: " << FlatbufferToJson(config, {.multi_line = true});

  EXPECT_EQ(std::string(absl::StripSuffix(
                util::ReadFileToStringOrDie(
                    ArtifactPath("aos/testdata/expected_multinode.json")),
                "\n")),
            FlatbufferToJson(config, {.multi_line = true}));
}

// Tests that we sort the entries in a config so we can look entries up.
TEST_F(ConfigurationTest, UnsortedConfig) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/backwards.json"));

  LOG(INFO) << "Read: " << FlatbufferToJson(config, {.multi_line = true});

  EXPECT_EQ(FlatbufferToJson(GetChannel(config, "/frc/robot_state",
                                        "frc.RobotState", "app1", nullptr)),
            "{ \"name\": \"/frc/robot_state\", \"type\": \"frc.RobotState\", "
            "\"max_size\": 5 }");
}

// Tests that we die when a file is imported twice.
TEST_F(ConfigurationDeathTest, DuplicateFile) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/config1_bad.json"));
      },
      "aos/testdata/config1_bad.json");
}

// Tests that we die when we give an invalid path.
TEST_F(ConfigurationDeathTest, NonexistentFile) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig("nonexistent/config.json");
      },
      "above error");
}

// Tests that we return std::nullopt when we give an invalid path.
TEST_F(ConfigurationTest, NonexistentFileOptional) {
  std::optional<FlatbufferDetachedBuffer<Configuration>> config =
      MaybeReadConfig("nonexistent/config.json");
  EXPECT_FALSE(config.has_value());
}

// Tests that we reject invalid channel names.  This means any channels with //
// in their name, a trailing /, or regex characters.
TEST_F(ConfigurationDeathTest, InvalidChannelName) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/invalid_channel_name1.json"));
      },
      "Channel names can't end with '/'");
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/invalid_channel_name2.json"));
      },
      "Invalid channel name");
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/invalid_channel_name3.json"));
        LOG(FATAL) << "Foo";
      },
      "Invalid channel name");
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/invalid_channel_name4.json"));
        LOG(FATAL) << "Foo";
      },
      "Channel names must start with '/'");
}

// Tests that we can modify a config with a json snippet.
TEST_F(ConfigurationTest, MergeWithConfig) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));
  LOG(INFO) << "Read: " << FlatbufferToJson(config, {.multi_line = true});

  FlatbufferDetachedBuffer<Configuration> updated_config =
      MergeWithConfig(&config.message(),
                      R"channel({
  "channels": [
    {
      "name": "/foo",
      "type": ".aos.bar",
      "max_size": 100
    }
  ]
})channel");

  EXPECT_EQ(absl::StripSuffix(util::ReadFileToStringOrDie(ArtifactPath(
                                  "aos/testdata/expected_merge_with.json")),
                              "\n"),
            FlatbufferToJson(updated_config, {.multi_line = true}));
}

// Tests that we can modify a config with a static flatbuffer.
TEST_F(ConfigurationTest, MergeWithConfigFromStatic) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));
  LOG(INFO) << "Read: " << FlatbufferToJson(config, {.multi_line = true});

  fbs::Builder<ConfigurationStatic> config_addition_builder;
  ConfigurationStatic *config_addition = config_addition_builder.get();
  {
    fbs::Vector<aos::ChannelStatic, 0, false> *channels_addition =
        config_addition->add_channels();
    ASSERT_TRUE(channels_addition != nullptr);
    ASSERT_TRUE(channels_addition->reserve(1));
    ChannelStatic *channel_override = channels_addition->emplace_back();
    ASSERT_TRUE(channel_override != nullptr);

    fbs::String<0> *name = channel_override->add_name();
    ASSERT_TRUE(name != nullptr);
    ASSERT_TRUE(name->reserve(10));
    name->SetString("/foo");

    fbs::String<0> *type = channel_override->add_type();
    ASSERT_TRUE(type != nullptr);
    ASSERT_TRUE(type->reserve(10));
    type->SetString(".aos.bar");

    channel_override->set_max_size(100);
  }

  FlatbufferDetachedBuffer<Configuration> updated_config =
      MergeWithConfig(&config.message(), config_addition->AsFlatbuffer());

  EXPECT_EQ(absl::StripSuffix(util::ReadFileToStringOrDie(ArtifactPath(
                                  "aos/testdata/expected_merge_with.json")),
                              "\n"),
            FlatbufferToJson(updated_config, {.multi_line = true}));
}

// Tests that we can properly strip the schemas from the channels.
TEST_F(ConfigurationTest, StripConfiguration) {
  FlatbufferDetachedBuffer<Configuration> original_config =
      ReadConfig(ArtifactPath("aos/events/pingpong_config.json"));
  ASSERT_TRUE(original_config.message().has_channels());
  for (const Channel *channel : *original_config.message().channels()) {
    EXPECT_TRUE(channel->has_schema());
  }

  FlatbufferDetachedBuffer<Configuration> stripped_config =
      StripConfiguration(&original_config.message());
  ASSERT_TRUE(stripped_config.message().has_channels());
  for (const Channel *channel : *stripped_config.message().channels()) {
    EXPECT_FALSE(channel->has_schema());
  }
}

// Tests that we can lookup a location, complete with maps, from a merged
// config.
TEST_F(ConfigurationTest, GetChannel) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));

  // Test a basic lookup first.
  EXPECT_THAT(GetChannel(config, "/foo", ".aos.bar", "app1", nullptr),
              aos::testing::FlatbufferEq(ExpectedLocation()));

  // Test that an invalid name results in nullptr back.
  EXPECT_EQ(GetChannel(config, "/invalid_name", ".aos.bar", "app1", nullptr),
            nullptr);

  // Tests that a root map/rename works. And that they get processed from the
  // bottom up.
  EXPECT_THAT(GetChannel(config, "/batman", ".aos.bar", "app1", nullptr),
              aos::testing::FlatbufferEq(ExpectedLocation()));

  // And then test that an application specific map/rename works.
  EXPECT_THAT(GetChannel(config, "/bar", ".aos.bar", "app1", nullptr),
              aos::testing::FlatbufferEq(ExpectedLocation()));
  EXPECT_THAT(GetChannel(config, "/baz", ".aos.bar", "app2", nullptr),
              aos::testing::FlatbufferEq(ExpectedLocation()));

  // And then test that an invalid application name gets properly ignored.
  EXPECT_THAT(GetChannel(config, "/foo", ".aos.bar", "app3", nullptr),
              aos::testing::FlatbufferEq(ExpectedLocation()));
}

// Tests that we can do reverse-lookups of channel names.
TEST_F(ConfigurationTest, GetChannelAliases) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));

  // Test a basic lookup first.
  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo", ".aos.bar", "app1", nullptr),
      ::testing::UnorderedElementsAre("/foo", "/batman", "/bar"));
  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/bar", ".aos.bar", "app1", nullptr),
      ::testing::UnorderedElementsAre("/batman", "/bar"));
  EXPECT_THAT(GetChannelAliases(&config.message(), "/batman", ".aos.bar",
                                "app1", nullptr),
              ::testing::UnorderedElementsAre("/batman"));
  // /bar (deliberately) does not get included because of the ordering in the
  // map.
  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo", ".aos.bar", "", nullptr),
      ::testing::UnorderedElementsAre("/foo", "/batman"));
  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo", ".aos.bar", "app2", nullptr),
      ::testing::UnorderedElementsAre("/foo", "/batman", "/baz"));
}

// Tests that we can lookup a location with node specific maps.
TEST_F(ConfigurationTest, GetChannelMultinode) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  const Node *pi1 = GetNode(&config.message(), "pi1");
  const Node *pi2 = GetNode(&config.message(), "pi2");

  // Test a basic lookup first.
  EXPECT_THAT(GetChannel(config, "/foo", ".aos.bar", "app1", pi1),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));
  EXPECT_THAT(GetChannel(config, "/foo", ".aos.bar", "app1", pi2),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));

  // Tests that a root map/rename works with a node specific map.
  EXPECT_THAT(GetChannel(config, "/batman", ".aos.bar", "app1", pi1),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));

  // Tests that node specific maps get ignored
  EXPECT_EQ(GetChannel(config, "/batman", ".aos.bar", "", nullptr), nullptr);

  // Tests that a root map/rename fails with a node specific map for the wrong
  // node.
  EXPECT_EQ(GetChannel(config, "/batman", ".aos.bar", "app1", pi2), nullptr);

  // And then test that an application specific map/rename works.
  EXPECT_THAT(GetChannel(config, "/batman2", ".aos.bar", "app1", pi1),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));
  EXPECT_THAT(GetChannel(config, "/batman3", ".aos.bar", "app1", pi1),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));

  // And then that it fails when the node changes.
  EXPECT_EQ(GetChannel(config, "/batman3", ".aos.bar", "app1", pi2), nullptr);
}

// Tests that reverse channel lookup on a multi-node config (including with
// wildcards) works.
TEST_F(ConfigurationTest, GetChannelAliasesMultinode) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));

  const Node *pi1 = GetNode(&config.message(), "pi1");
  const Node *pi2 = GetNode(&config.message(), "pi2");

  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo", ".aos.bar", "app1", pi1),
      ::testing::UnorderedElementsAre("/foo", "/batman", "/batman2", "/batman3",
                                      "/magic/string"));

  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo", ".aos.bar", "", pi1),
      ::testing::UnorderedElementsAre("/foo", "/batman", "/magic/string"));

  EXPECT_TRUE(GetChannelAliases(&config.message(), "/foo", ".aos.baz", "", pi1)
                  .empty());

  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo/testing", ".aos.bar", "", pi1),
      ::testing::UnorderedElementsAre("/foo/testing", "/magic/string/testing"));

  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/foo/testing", ".aos.bar", "app1",
                        pi2),
      ::testing::UnorderedElementsAre("/foo/testing", "/magic/string/testing"));

  // The second map in the config (/aos -> /aos/second) always takes precedence
  // over the first one (/aos -> /aos/first), so this shouldn't have "/aos" as
  // an alias.
  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/aos/first", ".aos.test", "", pi1),
      ::testing::UnorderedElementsAre("/aos/first"));

  EXPECT_THAT(
      GetChannelAliases(&config.message(), "/aos/second", ".aos.test", "", pi1),
      ::testing::UnorderedElementsAre("/aos/second", "/aos"));
}

// Tests that we can lookup a location with type specific maps.
TEST_F(ConfigurationTest, GetChannelTypedMultinode) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  const Node *pi1 = GetNode(&config.message(), "pi1");

  // Test a basic lookup first.
  EXPECT_THAT(GetChannel(config, "/batman", ".aos.bar", "app1", pi1),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));

  // Now confirm that a second message on the same name doesn't get remapped.
  const char *kExpectedBazMultinodeLocation =
      "{ \"name\": \"/batman\", \"type\": \".aos.baz\", \"max_size\": 5, "
      "\"source_node\": \"pi1\" }";
  EXPECT_EQ(
      FlatbufferToJson(GetChannel(config, "/batman", ".aos.baz", "app1", pi1)),
      kExpectedBazMultinodeLocation);
}

// Tests that we can lookup a location with a glob
TEST_F(ConfigurationTest, GetChannelGlob) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  const Node *pi1 = GetNode(&config.message(), "pi1");

  // Confirm that a glob with nothing after it matches.
  EXPECT_THAT(GetChannel(config, "/magic/string", ".aos.bar", "app7", pi1),
              aos::testing::FlatbufferEq(ExpectedMultinodeLocation()));

  // Now confirm that glob with something following it matches and renames
  // correctly.
  const char *kExpectedSubfolderMultinodeLocation =
      "{ \"name\": \"/foo/testing\", \"type\": \".aos.bar\", \"max_size\": "
      "5, \"source_node\": \"pi1\" }";
  EXPECT_EQ(FlatbufferToJson(GetChannel(config, "/magic/string/testing",
                                        ".aos.bar", "app7", pi1)),
            kExpectedSubfolderMultinodeLocation);
}

// Tests that we reject a configuration which has a nodes list, but has channels
// withoout source_node filled out.
TEST_F(ConfigurationDeathTest, InvalidSourceNode) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/invalid_nodes.json"));
      },
      "source_node");

  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/invalid_source_node.json"));
      },
      "source_node");

  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config = ReadConfig(
            ArtifactPath("aos/testdata/invalid_destination_node.json"));
      },
      "destination_nodes");

  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config =
            ReadConfig(ArtifactPath("aos/testdata/self_forward.json"));
      },
      "forwarding data to itself");
}

// Tests that our node writeable helpers work as intended.
TEST_F(ConfigurationTest, ChannelIsSendableOnNode) {
  FlatbufferDetachedBuffer<Channel> good_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "foo"
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> bad_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar"
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> node(JsonToFlatbuffer(
      R"node({
  "name": "foo"
})node",
      Node::MiniReflectTypeTable()));

  EXPECT_TRUE(
      ChannelIsSendableOnNode(&good_channel.message(), &node.message()));
  EXPECT_FALSE(
      ChannelIsSendableOnNode(&bad_channel.message(), &node.message()));
}

// Tests that our node readable and writeable helpers work as intended.
TEST_F(ConfigurationTest, ChannelIsReadableOnNode) {
  FlatbufferDetachedBuffer<Channel> good_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "destination_nodes": [
    {
      "name": "baz"
    },
    {
      "name": "foo"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> bad_channel1(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar"
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> bad_channel2(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> node(JsonToFlatbuffer(
      R"node({
  "name": "foo"
})node",
      Node::MiniReflectTypeTable()));

  EXPECT_TRUE(
      ChannelIsReadableOnNode(&good_channel.message(), &node.message()));
  EXPECT_FALSE(
      ChannelIsReadableOnNode(&bad_channel1.message(), &node.message()));
  EXPECT_FALSE(
      ChannelIsReadableOnNode(&bad_channel2.message(), &node.message()));
}

// Tests that our channel is forwarded helpers work as intended.
TEST_F(ConfigurationTest, ChannelIsForwardedFromNode) {
  FlatbufferDetachedBuffer<Channel> forwarded_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "destination_nodes": [
    {
      "name": "baz"
    },
    {
      "name": "foo"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> single_node_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping"
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> zero_length_vector_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "destination_nodes": [
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> node(JsonToFlatbuffer(
      R"node({
  "name": "bar"
})node",
      Node::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> readable_node(JsonToFlatbuffer(
      R"node({
  "name": "foo"
})node",
      Node::MiniReflectTypeTable()));

  EXPECT_TRUE(ChannelIsForwardedFromNode(&forwarded_channel.message(),
                                         &node.message()));
  EXPECT_FALSE(ChannelIsForwardedFromNode(&forwarded_channel.message(),
                                          &readable_node.message()));
  EXPECT_FALSE(
      ChannelIsForwardedFromNode(&single_node_channel.message(), nullptr));
  EXPECT_FALSE(ChannelIsForwardedFromNode(&zero_length_vector_channel.message(),
                                          &node.message()));
}

// Tests that our node message is logged helpers work as intended.
TEST_F(ConfigurationTest, ChannelMessageIsLoggedOnNode) {
  FlatbufferDetachedBuffer<Channel> logged_on_self_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> not_logged_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "NOT_LOGGED",
  "destination_nodes": [
    {
      "name": "baz",
      "timestamp_logger": "LOCAL_LOGGER"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_remote_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "REMOTE_LOGGER",
  "logger_nodes": ["baz"],
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_separate_logger_node_channel(
      JsonToFlatbuffer(
          R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "REMOTE_LOGGER",
  "logger_nodes": ["foo"],
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
          Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_both_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "LOCAL_AND_REMOTE_LOGGER",
  "logger_nodes": ["baz"],
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> foo_node(JsonToFlatbuffer(
      R"node({
  "name": "foo"
})node",
      Node::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> bar_node(JsonToFlatbuffer(
      R"node({
  "name": "bar"
})node",
      Node::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> baz_node(JsonToFlatbuffer(
      R"node({
  "name": "baz"
})node",
      Node::MiniReflectTypeTable()));

  // Local logger.
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&logged_on_self_channel.message(),
                                            &foo_node.message()));
  EXPECT_TRUE(ChannelMessageIsLoggedOnNode(&logged_on_self_channel.message(),
                                           &bar_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&logged_on_self_channel.message(),
                                            &baz_node.message()));
  EXPECT_TRUE(
      ChannelMessageIsLoggedOnNode(&logged_on_self_channel.message(), nullptr));

  // No logger.
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&not_logged_channel.message(),
                                            &foo_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&not_logged_channel.message(),
                                            &bar_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&not_logged_channel.message(),
                                            &baz_node.message()));
  EXPECT_FALSE(
      ChannelMessageIsLoggedOnNode(&not_logged_channel.message(), nullptr));

  // Remote logger.
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&logged_on_remote_channel.message(),
                                            &foo_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&logged_on_remote_channel.message(),
                                            &bar_node.message()));
  EXPECT_TRUE(ChannelMessageIsLoggedOnNode(&logged_on_remote_channel.message(),
                                           &baz_node.message()));

  // Separate logger.
  EXPECT_TRUE(ChannelMessageIsLoggedOnNode(
      &logged_on_separate_logger_node_channel.message(), &foo_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(
      &logged_on_separate_logger_node_channel.message(), &bar_node.message()));
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(
      &logged_on_separate_logger_node_channel.message(), &baz_node.message()));

  // Logged in multiple places.
  EXPECT_FALSE(ChannelMessageIsLoggedOnNode(&logged_on_both_channel.message(),
                                            &foo_node.message()));
  EXPECT_TRUE(ChannelMessageIsLoggedOnNode(&logged_on_both_channel.message(),
                                           &bar_node.message()));
  EXPECT_TRUE(ChannelMessageIsLoggedOnNode(&logged_on_both_channel.message(),
                                           &baz_node.message()));
}

// Tests that our node message is logged helpers work as intended.
TEST_F(ConfigurationDeathTest, ChannelMessageIsLoggedOnNode) {
  FlatbufferDetachedBuffer<Channel> logged_on_both_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "LOCAL_AND_REMOTE_LOGGER",
  "logger_nodes": ["baz"],
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_separate_logger_node_channel(
      JsonToFlatbuffer(
          R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "REMOTE_LOGGER",
  "logger_nodes": ["foo"],
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
          Channel::MiniReflectTypeTable()));

  EXPECT_DEATH(
      {
        ChannelMessageIsLoggedOnNode(&logged_on_both_channel.message(),
                                     nullptr);
      },
      "Unsupported logging configuration in a single node world");
  EXPECT_DEATH(
      {
        ChannelMessageIsLoggedOnNode(
            &logged_on_separate_logger_node_channel.message(), nullptr);
      },
      "Unsupported logging configuration in a single node world");
}

// Tests that our forwarding timestamps are logged helpers work as intended.
TEST_F(ConfigurationTest, ConnectionDeliveryTimeIsLoggedOnNode) {
  FlatbufferDetachedBuffer<Channel> logged_on_self_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "REMOTE_LOGGER",
  "logger_nodes": ["baz"],
  "destination_nodes": [
    {
      "name": "baz"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> not_logged_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "NOT_LOGGED",
  "destination_nodes": [
    {
      "name": "baz",
      "timestamp_logger": "NOT_LOGGED"
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_remote_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "destination_nodes": [
    {
      "name": "baz",
      "timestamp_logger": "REMOTE_LOGGER",
      "timestamp_logger_nodes": ["bar"]
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_separate_logger_node_channel(
      JsonToFlatbuffer(
          R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "logger": "REMOTE_LOGGER",
  "logger_nodes": ["foo"],
  "destination_nodes": [
    {
      "name": "baz",
      "timestamp_logger": "REMOTE_LOGGER",
      "timestamp_logger_nodes": ["foo"]
    }
  ]
})channel",
          Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Channel> logged_on_both_channel(JsonToFlatbuffer(
      R"channel({
  "name": "/test",
  "type": "aos.examples.Ping",
  "source_node": "bar",
  "destination_nodes": [
    {
      "name": "baz",
      "timestamp_logger": "LOCAL_AND_REMOTE_LOGGER",
      "timestamp_logger_nodes": ["bar"]
    }
  ]
})channel",
      Channel::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> foo_node(JsonToFlatbuffer(
      R"node({
  "name": "foo"
})node",
      Node::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> bar_node(JsonToFlatbuffer(
      R"node({
  "name": "bar"
})node",
      Node::MiniReflectTypeTable()));

  FlatbufferDetachedBuffer<Node> baz_node(JsonToFlatbuffer(
      R"node({
  "name": "baz"
})node",
      Node::MiniReflectTypeTable()));

  // Local logger.
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_self_channel.message(), &baz_node.message(),
      &foo_node.message()));
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_self_channel.message(), &baz_node.message(),
      &bar_node.message()));
  EXPECT_TRUE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_self_channel.message(), &baz_node.message(),
      &baz_node.message()));

  // No logger means.
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &not_logged_channel.message(), &baz_node.message(), &foo_node.message()));
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &not_logged_channel.message(), &baz_node.message(), &bar_node.message()));
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &not_logged_channel.message(), &baz_node.message(), &baz_node.message()));

  // Remote logger.
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_remote_channel.message(), &baz_node.message(),
      &foo_node.message()));
  EXPECT_TRUE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_remote_channel.message(), &baz_node.message(),
      &bar_node.message()));
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_remote_channel.message(), &baz_node.message(),
      &baz_node.message()));

  // Separate logger.
  EXPECT_TRUE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_separate_logger_node_channel.message(), &baz_node.message(),
      &foo_node.message()));
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_separate_logger_node_channel.message(), &baz_node.message(),
      &bar_node.message()));
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_separate_logger_node_channel.message(), &baz_node.message(),
      &baz_node.message()));

  // Logged on both the node and a remote node.
  EXPECT_FALSE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_both_channel.message(), &baz_node.message(),
      &foo_node.message()));
  EXPECT_TRUE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_both_channel.message(), &baz_node.message(),
      &bar_node.message()));
  EXPECT_TRUE(ConnectionDeliveryTimeIsLoggedOnNode(
      &logged_on_both_channel.message(), &baz_node.message(),
      &baz_node.message()));
}

// Tests that we can deduce source nodes from a multinode config.
TEST_F(ConfigurationTest, SourceNodeNames) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1_multinode.json"));

  // This is a bit simplistic in that it doesn't test deduplication, but it does
  // exercise a lot of the logic.
  EXPECT_THAT(
      SourceNodeNames(&config.message(), config.message().nodes()->Get(0)),
      ::testing::ElementsAreArray({"pi2"}));
  EXPECT_THAT(
      SourceNodeNames(&config.message(), config.message().nodes()->Get(1)),
      ::testing::ElementsAreArray({"pi1"}));
}

// Tests that we can deduce destination nodes from a multinode config.
TEST_F(ConfigurationTest, DestinationNodeNames) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1_multinode.json"));

  // This is a bit simplistic in that it doesn't test deduplication, but it does
  // exercise a lot of the logic.
  EXPECT_THAT(
      DestinationNodeNames(&config.message(), config.message().nodes()->Get(0)),
      ::testing::ElementsAreArray({"pi2"}));
  EXPECT_THAT(
      DestinationNodeNames(&config.message(), config.message().nodes()->Get(1)),
      ::testing::ElementsAreArray({"pi1"}));
}

// Tests that we can pull out all the nodes.
TEST_F(ConfigurationTest, GetNodes) {
  {
    FlatbufferDetachedBuffer<Configuration> config =
        ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
    const Node *pi1 = GetNode(&config.message(), "pi1");
    const Node *pi2 = GetNode(&config.message(), "pi2");

    EXPECT_THAT(GetNodes(&config.message()), ::testing::ElementsAre(pi1, pi2));
  }

  {
    FlatbufferDetachedBuffer<Configuration> config =
        ReadConfig(ArtifactPath("aos/testdata/config1.json"));
    EXPECT_THAT(GetNodes(&config.message()), ::testing::ElementsAre(nullptr));
  }
}

// Tests that we can pull out all the nodes with a tag.
TEST_F(ConfigurationTest, GetNodesWithTag) {
  {
    FlatbufferDetachedBuffer<Configuration> config =
        ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
    const Node *pi1 = GetNode(&config.message(), "pi1");
    const Node *pi2 = GetNode(&config.message(), "pi2");

    EXPECT_THAT(GetNodesWithTag(&config.message(), "a"),
                ::testing::ElementsAre(pi1));
    EXPECT_THAT(GetNodesWithTag(&config.message(), "b"),
                ::testing::ElementsAre(pi2));
    EXPECT_THAT(GetNodesWithTag(&config.message(), "c"),
                ::testing::ElementsAre(pi1, pi2));
  }

  {
    FlatbufferDetachedBuffer<Configuration> config =
        ReadConfig(ArtifactPath("aos/testdata/config1.json"));
    EXPECT_THAT(GetNodesWithTag(&config.message(), "arglfish"),
                ::testing::ElementsAre(nullptr));
  }
}

// Tests that we can check if a node has a tag.
TEST_F(ConfigurationTest, NodeHasTag) {
  {
    FlatbufferDetachedBuffer<Configuration> config =
        ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
    const Node *pi1 = GetNode(&config.message(), "pi1");
    const Node *pi2 = GetNode(&config.message(), "pi2");

    EXPECT_TRUE(NodeHasTag(pi1, "a"));
    EXPECT_FALSE(NodeHasTag(pi2, "a"));
    EXPECT_FALSE(NodeHasTag(pi1, "b"));
    EXPECT_TRUE(NodeHasTag(pi2, "b"));
    EXPECT_TRUE(NodeHasTag(pi1, "c"));
    EXPECT_TRUE(NodeHasTag(pi2, "c"));
    EXPECT_FALSE(NodeHasTag(pi1, "nope"));
    EXPECT_FALSE(NodeHasTag(pi2, "nope"));
  }

  EXPECT_TRUE(NodeHasTag(nullptr, "arglfish"));
}

// Tests that we can extract a node index from a config.
TEST_F(ConfigurationTest, GetNodeIndex) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  FlatbufferDetachedBuffer<Configuration> config2 =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  const Node *pi1 = GetNode(&config.message(), "pi1");
  const Node *pi2 = GetNode(&config.message(), "pi2");

  // Try the normal case.
  EXPECT_EQ(GetNodeIndex(&config.message(), pi1), 0);
  EXPECT_EQ(GetNodeIndex(&config.message(), pi2), 1);

  // Now try if we have node pointers from a different message.
  EXPECT_EQ(GetNodeIndex(&config2.message(), pi1), 0);
  EXPECT_EQ(GetNodeIndex(&config2.message(), pi2), 1);

  // And now try string names.
  EXPECT_EQ(GetNodeIndex(&config2.message(), pi1->name()->string_view()), 0);
  EXPECT_EQ(GetNodeIndex(&config2.message(), pi2->name()->string_view()), 1);
}

// Tests that GetNodeOrDie handles both single and multi-node worlds and returns
// valid nodes.
TEST_F(ConfigurationDeathTest, GetNodeOrDie) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  FlatbufferDetachedBuffer<Configuration> config2 =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  {
    // Simple case, nullptr -> nullptr
    FlatbufferDetachedBuffer<Configuration> single_node_config =
        ReadConfig(ArtifactPath("aos/testdata/config1.json"));
    EXPECT_EQ(nullptr, GetNodeOrDie(&single_node_config.message(), nullptr));
    EXPECT_EQ(nullptr, GetNodeOrDie(&single_node_config.message(), ""));

    // Confirm that we die when a node is passed in.
    EXPECT_DEATH(
        {
          GetNodeOrDie(&single_node_config.message(),
                       config.message().nodes()->Get(0));
        },
        "Provided a node name of 'pi1' in a single node world.");
    EXPECT_DEATH(
        {
          GetNodeOrDie(&single_node_config.message(),
                       config.message().nodes()->Get(0)->name()->string_view());
        },
        "Provided a node name of 'pi1' in a single node world.");
  }

  const Node *pi1 = GetNode(&config.message(), "pi1");
  // Now try a lookup using a node from a different instance of the config.
  EXPECT_EQ(pi1,
            GetNodeOrDie(&config.message(), config2.message().nodes()->Get(0)));
  EXPECT_EQ(pi1, GetNodeOrDie(
                     &config.message(),
                     config2.message().nodes()->Get(0)->name()->string_view()));
}

TEST_F(ConfigurationTest, GetNodeFromHostname) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  {
    const Node *pi1 = GetNodeFromHostname(&config.message(), "raspberrypi");
    ASSERT_TRUE(pi1 != nullptr);
    EXPECT_EQ("pi1", pi1->name()->string_view());
  }
  {
    const Node *pi2 = GetNodeFromHostname(&config.message(), "raspberrypi2");
    ASSERT_TRUE(pi2 != nullptr);
    EXPECT_EQ("pi2", pi2->name()->string_view());
  }
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "raspberrypi3"));
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "localhost"));
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "3"));
}

TEST_F(ConfigurationTest, GetNodeFromHostnames) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode_hostnames.json"));
  {
    const Node *pi1 = GetNodeFromHostname(&config.message(), "raspberrypi");
    ASSERT_TRUE(pi1 != nullptr);
    EXPECT_EQ("pi1", pi1->name()->string_view());
  }
  {
    const Node *pi2 = GetNodeFromHostname(&config.message(), "raspberrypi2");
    ASSERT_TRUE(pi2 != nullptr);
    EXPECT_EQ("pi2", pi2->name()->string_view());
  }
  {
    const Node *pi2 = GetNodeFromHostname(&config.message(), "raspberrypi3");
    ASSERT_TRUE(pi2 != nullptr);
    EXPECT_EQ("pi2", pi2->name()->string_view());
  }
  {
    const Node *pi2 = GetNodeFromHostname(&config.message(), "other");
    ASSERT_TRUE(pi2 != nullptr);
    EXPECT_EQ("pi2", pi2->name()->string_view());
  }
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "raspberrypi4"));
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "localhost"));
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "3"));
}

TEST_F(ConfigurationTest, GetNodeFromRegexHostname) {
  FlatbufferDetachedBuffer<Configuration> config = ReadConfig(
      ArtifactPath("aos/testdata/good_multinode_regex_hostname.json"));
  {
    const Node *pi1 = GetNodeFromHostname(&config.message(), "device-123-1");
    ASSERT_TRUE(pi1 != nullptr);
    EXPECT_EQ("pi1", pi1->name()->string_view());
  }
  {
    const Node *pi2 = GetNodeFromHostname(&config.message(), "device-456-2");
    ASSERT_TRUE(pi2 != nullptr);
    EXPECT_EQ("pi2", pi2->name()->string_view());
  }
  {
    const Node *pi2 = GetNodeFromHostname(&config.message(), "device-789-2");
    ASSERT_TRUE(pi2 != nullptr);
    EXPECT_EQ("pi2", pi2->name()->string_view());
  }
  {
    const Node *pi2 = GetNodeFromHostname(&config.message(), "device--2");
    ASSERT_TRUE(pi2 != nullptr);
    EXPECT_EQ("pi2", pi2->name()->string_view());
  }
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "device"));
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "device-abc-1"));
  EXPECT_EQ(nullptr, GetNodeFromHostname(&config.message(), "3"));
}

// Tests that SourceNodeIndex reasonably handles a multi-node log file.
TEST_F(ConfigurationTest, SourceNodeIndex) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
  std::vector<size_t> result = SourceNodeIndex(&config.message());

  EXPECT_THAT(result, ::testing::ElementsAreArray({0, 0, 0, 1, 0, 0}));
}

// Tests that SourceNode reasonably handles both single and multi-node configs.
TEST_F(ConfigurationTest, SourceNode) {
  {
    FlatbufferDetachedBuffer<Configuration> config_single_node =
        ReadConfig(ArtifactPath("aos/testdata/config1.json"));
    const Node *result =
        SourceNode(&config_single_node.message(),
                   config_single_node.message().channels()->Get(0));
    EXPECT_EQ(result, nullptr);
  }

  {
    FlatbufferDetachedBuffer<Configuration> config_multi_node =
        ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));
    size_t pi1_channels = 0;
    size_t pi2_channels = 0;
    for (const aos::Channel *channel :
         *config_multi_node.message().channels()) {
      const Node *result = SourceNode(&config_multi_node.message(), channel);
      if (channel->source_node()->string_view() == "pi1") {
        ++pi1_channels;
        EXPECT_EQ(result, config_multi_node.message().nodes()->Get(0));
      } else {
        ++pi2_channels;
        EXPECT_EQ(result, config_multi_node.message().nodes()->Get(1));
      }
    }
    EXPECT_GT(pi1_channels, 0u);
    EXPECT_GT(pi2_channels, 0u);
  }
}

// Tests that we reject invalid logging configurations.
TEST_F(ConfigurationDeathTest, InvalidLoggerConfig) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config = ReadConfig(
            ArtifactPath("aos/testdata/invalid_logging_configuration.json"));
      },
      "Logging timestamps without data");
}

// Tests that we reject duplicate timestamp destination node configurations.
TEST_F(ConfigurationDeathTest, DuplicateTimestampDestinationNodes) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config = ReadConfig(
            ArtifactPath("aos/testdata/duplicate_destination_nodes.json"));
      },
      "Found duplicate timestamp_logger_nodes in");
}

// Tests that we reject duplicate logger node configurations for a channel's
// data.
TEST_F(ConfigurationDeathTest, DuplicateLoggerNodes) {
  EXPECT_DEATH(
      {
        FlatbufferDetachedBuffer<Configuration> config = ReadConfig(
            ArtifactPath("aos/testdata/duplicate_logger_nodes.json"));
      },
      "Found duplicate logger_nodes in");
}

// Tests that we properly compute the queue size for the provided duration.
TEST_F(ConfigurationTest, QueueSize) {
  EXPECT_EQ(QueueSize(100, chrono::seconds(2)), 200);
  EXPECT_EQ(QueueSize(200, chrono::seconds(2)), 400);
  EXPECT_EQ(QueueSize(100, chrono::seconds(6)), 600);
  EXPECT_EQ(QueueSize(100, chrono::milliseconds(10)), 1);
  EXPECT_EQ(QueueSize(100, chrono::milliseconds(10) - chrono::nanoseconds(1)),
            1);
  EXPECT_EQ(QueueSize(100, chrono::milliseconds(10) - chrono::nanoseconds(2)),
            1);
}

// Tests that we compute scratch buffer size correctly too.
TEST_F(ConfigurationTest, QueueScratchBufferSize) {
  const aos::FlatbufferDetachedBuffer<Channel> channel =
      JsonToFlatbuffer<Channel>(
          "{ \"name\": \"/foo\", \"type\": \".aos.bar\", \"num_readers\": 5, "
          "\"num_senders\": 10 }");
  EXPECT_EQ(QueueScratchBufferSize(&channel.message()), 15);
}

// Tests that GetSchema returns schema of specified type
TEST_F(ConfigurationTest, GetSchema) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/events/pingpong_config.json"));
  FlatbufferVector<reflection::Schema> expected_schema =
      FileToFlatbuffer<reflection::Schema>(
          ArtifactPath("aos/events/ping.bfbs"));
  EXPECT_EQ(FlatbufferToJson(GetSchema(&config.message(), "aos.examples.Ping")),
            FlatbufferToJson(expected_schema));
  EXPECT_EQ(GetSchema(&config.message(), "invalid_name"), nullptr);
}

// Tests that GetSchema template returns schema of specified type
TEST_F(ConfigurationTest, GetSchemaTemplate) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/events/pingpong_config.json"));
  FlatbufferVector<reflection::Schema> expected_schema =
      FileToFlatbuffer<reflection::Schema>(
          ArtifactPath("aos/events/ping.bfbs"));
  EXPECT_EQ(FlatbufferToJson(GetSchema<aos::examples::Ping>(&config.message())),
            FlatbufferToJson(expected_schema));
}

// Tests that GetSchemaDetachedBuffer returns detached buffer of specified type
TEST_F(ConfigurationTest, GetSchemaDetachedBuffer) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/events/pingpong_config.json"));
  FlatbufferVector<reflection::Schema> expected_schema =
      FileToFlatbuffer<reflection::Schema>(
          ArtifactPath("aos/events/ping.bfbs"));
  EXPECT_EQ(FlatbufferToJson(
                GetSchemaDetachedBuffer(&config.message(), "aos.examples.Ping")
                    .value()),
            FlatbufferToJson(expected_schema));
  EXPECT_EQ(GetSchemaDetachedBuffer(&config.message(), "invalid_name"),
            std::nullopt);
}

// Tests that we can use a utility to add individual channels to a single-node
// config.
TEST_F(ConfigurationTest, AddChannelToConfigSingleNode) {
  FlatbufferDetachedBuffer<Configuration> base_config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));

  FlatbufferVector<reflection::Schema> schema =
      FileToFlatbuffer<reflection::Schema>(
          ArtifactPath("aos/events/ping.bfbs"));

  FlatbufferDetachedBuffer<Configuration> new_config =
      AddChannelToConfiguration(&base_config.message(), "/new", schema);

  ASSERT_EQ(new_config.message().channels()->size(),
            base_config.message().channels()->size() + 1);

  const Channel *channel =
      GetChannel(new_config, "/new", "aos.examples.Ping", "", nullptr);
  ASSERT_TRUE(channel != nullptr);
  ASSERT_TRUE(channel->has_schema());
  // Check that we don't populate channel settings that we don't override the
  // defaults of.
  ASSERT_FALSE(channel->has_frequency());
}

// Tests that we can use a utility to add individual channels to a multi-node
// config.
TEST_F(ConfigurationTest, AddChannelToConfigMultiNode) {
  FlatbufferDetachedBuffer<Configuration> base_config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));

  FlatbufferVector<reflection::Schema> schema =
      FileToFlatbuffer<reflection::Schema>(
          ArtifactPath("aos/events/ping.bfbs"));

  aos::ChannelT channel_overrides;
  channel_overrides.frequency = 649;
  FlatbufferDetachedBuffer<Configuration> new_config =
      AddChannelToConfiguration(&base_config.message(), "/new", schema,
                                GetNode(&base_config.message(), "pi1"),
                                channel_overrides);

  ASSERT_EQ(new_config.message().channels()->size(),
            base_config.message().channels()->size() + 1);

  const Channel *channel =
      GetChannel(new_config, "/new", "aos.examples.Ping", "", nullptr);
  ASSERT_TRUE(channel != nullptr);
  ASSERT_TRUE(channel->has_schema());
  ASSERT_TRUE(channel->has_source_node());
  ASSERT_EQ("pi1", channel->source_node()->string_view());
  ASSERT_EQ(649, channel->frequency());
}

class GetApplicationsContainingSubstringTest : public ConfigurationTest {};

// Tests that GetApplicationsContainingSubstring handles no applications in the
// config, and returns an empty list.
TEST_F(GetApplicationsContainingSubstringTest, NoApps) {
  {
    const FlatbufferDetachedBuffer<Configuration> config =
        JsonToFlatbuffer<Configuration>(R"config({
  "nodes": [
    {
      "name": "node1"
    },
    {
      "name": "node2"
    }
  ]
})config");
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "node1", "app");
    EXPECT_TRUE(result.empty());
  }
  {
    const FlatbufferDetachedBuffer<Configuration> config =
        JsonToFlatbuffer<Configuration>(R"config({
  "applications": [],
  "nodes": [
    {
      "name": "node1"
    },
    {
      "name": "node2"
    }
  ]
})config");
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "node1", "app");
    EXPECT_TRUE(result.empty());
  }
}

// Tests that GetApplicationsContainingSubstring returns the correct
// applications for a query on a single node config.
TEST_F(GetApplicationsContainingSubstringTest, SingleNode) {
  const FlatbufferDetachedBuffer<Configuration> config =
      JsonToFlatbuffer<Configuration>(R"config({
  "applications": [
    {
      "name": "foo"
    },
    {
      "name": "bar"
    },
    {
      "name": "baz",
      "autostart": false
    },
    {
      "name": "sparse"
    }
  ]
})config");
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "", "fo");
    EXPECT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0]->name()->string_view(), "foo");
  }
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "", "ba",
                                           Autostart::kYes);
    EXPECT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0]->name()->string_view(), "bar");
  }
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "", "ar");
    EXPECT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0]->name()->string_view(), "bar");
    EXPECT_EQ(result[1]->name()->string_view(), "sparse");
  }
}

// Tests that GetApplicationsContainingSubstring returns the correct
// applications for a query on a multi node config.
TEST_F(GetApplicationsContainingSubstringTest, MultiNode) {
  const FlatbufferDetachedBuffer<Configuration> config =
      JsonToFlatbuffer<Configuration>(R"config({
  "applications": [
    {
      "name": "foo1",
      "nodes": [
        "node1",
        "node2"
      ]
    },
    {
      "name": "foo2",
      "nodes": [
        "node2",
        "node3"
      ]
    },
    {
      "name": "bar",
      "nodes": [
        "node1",
        "node2"
      ]
    },
    {
      "name": "baz",
      "nodes": [
        "node2"
      ],
      "autostart": false
    },
    {
      "name": "sparse",
      "nodes": [
        "node2",
        "node3"
      ]
    }
  ],
  "nodes": [
    {
      "name": "node1"
    },
    {
      "name": "node2"
    },
    {
      "name": "node3"
    }
  ]
})config");
  // If node_name is empty, we should get back all apps that contain the
  // substring.
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "", "foo");
    EXPECT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0]->name()->string_view(), "foo1");
    EXPECT_EQ(result[1]->name()->string_view(), "foo2");
  }
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "", "ba");
    EXPECT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0]->name()->string_view(), "bar");
    EXPECT_EQ(result[1]->name()->string_view(), "baz");
  }
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "", "ar");
    EXPECT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0]->name()->string_view(), "bar");
    EXPECT_EQ(result[1]->name()->string_view(), "sparse");
  }
  // If node_name has a value, we should get apps filtered by substring and
  // node.
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "node1", "foo");
    EXPECT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0]->name()->string_view(), "foo1");
  }
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "node2", "foo");
    EXPECT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0]->name()->string_view(), "foo1");
    EXPECT_EQ(result[1]->name()->string_view(), "foo2");
  }
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "node1", "ba");
    EXPECT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0]->name()->string_view(), "bar");
  }
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "node2", "ba");
    EXPECT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0]->name()->string_view(), "bar");
    EXPECT_EQ(result[1]->name()->string_view(), "baz");
  }
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "node3", "ar");
    EXPECT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0]->name()->string_view(), "sparse");
  }
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "node2", "ar");
    EXPECT_EQ(result.size(), 2u);
    EXPECT_EQ(result[0]->name()->string_view(), "bar");
    EXPECT_EQ(result[1]->name()->string_view(), "sparse");
  }
  // If autostart is kYes, we should get apps filtered by substring and
  // autostart (and node, if specified).
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "", "ba",
                                           Autostart::kYes);
    EXPECT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0]->name()->string_view(), "bar");
  }
  {
    const std::vector<const Application *> result =
        GetApplicationsContainingSubstring(&config.message(), "node2", "ba",
                                           Autostart::kYes);
    EXPECT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0]->name()->string_view(), "bar");
  }
}

class GetApplicationsContainingSubstringDeathTest
    : public GetApplicationsContainingSubstringTest {};

// Tests that GetApplicationsContainingSubstring fails if the provided substring
// is empty.
TEST_F(GetApplicationsContainingSubstringDeathTest, EmptySubstring) {
  const FlatbufferDetachedBuffer<Configuration> config =
      JsonToFlatbuffer<Configuration>(R"config({
  "applications": [
    {
      "name": "foo1",
      "nodes": [
        "node1",
        "node2"
      ]
    },
    {
      "name": "foo2",
      "nodes": [
        "node2",
        "node3"
      ]
    },
    {
      "name": "bar",
      "nodes": [
        "node1",
        "node2"
      ]
    },
    {
      "name": "sparse",
      "nodes": [
        "node2",
        "node3"
      ]
    }
  ],
  "nodes": [
    {
      "name": "node1"
    },
    {
      "name": "node2"
    },
    {
      "name": "node3"
    }
  ]
})config");
  EXPECT_DEATH(
      GetApplicationsContainingSubstring(&config.message(), "node2", ""),
      "substring cannot be empty");
}

// Create a new configuration with the specified channel removed.
// Initially there must be exactly one channel in the base_config that matches
// the criteria. Check to make sure the new configuration has one less channel,
// and that channel is the specified channel.
void TestGetPartialConfiguration(const Configuration &base_config,
                                 std::string_view test_channel_name,
                                 std::string_view test_channel_type) {
  const Channel *channel_from_base_config = GetChannel(
      &base_config, test_channel_name, test_channel_type, "", nullptr);
  ASSERT_TRUE(channel_from_base_config != nullptr);

  const FlatbufferDetachedBuffer<Configuration> new_config =
      configuration::GetPartialConfiguration(
          base_config,
          // should_include_channel function
          [test_channel_name, test_channel_type](const Channel &channel) {
            if (channel.name()->string_view() == test_channel_name &&
                channel.type()->string_view() == test_channel_type) {
              LOG(INFO) << "Omitting channel from save_log, channel: "
                        << channel.name()->string_view() << ", "
                        << channel.type()->string_view();
              return false;
            }
            return true;
          });

  EXPECT_EQ(new_config.message().channels()->size(),
            base_config.channels()->size() - 1);

  channel_from_base_config = GetChannel(&base_config, test_channel_name,
                                        test_channel_type, "", nullptr);
  EXPECT_TRUE(channel_from_base_config != nullptr);

  const Channel *channel_from_new_config =
      GetChannel(new_config, test_channel_name, test_channel_type, "", nullptr);
  EXPECT_TRUE(channel_from_new_config == nullptr);
}

// Tests that we can use a utility to remove individual channels from a
// single-node config.
TEST_F(ConfigurationTest, RemoveChannelsFromConfigSingleNode) {
  FlatbufferDetachedBuffer<Configuration> base_config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));

  constexpr std::string_view test_channel_name = "/foo2";
  constexpr std::string_view test_channel_type = ".aos.bar";

  TestGetPartialConfiguration(base_config.message(), test_channel_name,
                              test_channel_type);
}

// Tests that we can use a utility to remove individual channels from a
// multi-node config.
TEST_F(ConfigurationTest, RemoveChannelsFromConfigMultiNode) {
  FlatbufferDetachedBuffer<Configuration> base_config =
      ReadConfig(ArtifactPath("aos/testdata/good_multinode.json"));

  constexpr std::string_view test_channel_name = "/batman";
  constexpr std::string_view test_channel_type = ".aos.baz";

  TestGetPartialConfiguration(base_config.message(), test_channel_name,
                              test_channel_type);
}

// Tests that schema validation fails when we fail to provide schemas for every
// channel.
TEST_F(ConfigurationDeathTest, ValidateAllSchemasAvailable) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));
  EXPECT_DEATH(MergeConfiguration(config, {}),
               "Failed to find schema.*.aos.bar");
}

// Test fixture for testing IsNodeFromConfiguration.
// Initializes multiple configurations which share the same node names.
// Use IsNodeFromConfiguration to check if a node is in a configuration.
class IsNodeFromConfigurationFixtureTest : public ConfigurationTest {
 protected:
  // Use unique_ptr for deferred initialization
  std::unique_ptr<FlatbufferDetachedBuffer<Configuration>> config1;
  std::unique_ptr<FlatbufferDetachedBuffer<Configuration>> config2;
  const Node *node1_config1;
  const Node *node2_config1;
  const Node *node1_config2;
  const Node *node2_config2;

  IsNodeFromConfigurationFixtureTest() {
    // Initialize configurations here
    config1 = std::make_unique<FlatbufferDetachedBuffer<Configuration>>(
        JsonToFlatbuffer(R"config({
          "nodes": [
            {"name": "node1"},
            {"name": "node2"}
          ]
        })config",
                         Configuration::MiniReflectTypeTable()));

    config2 = std::make_unique<FlatbufferDetachedBuffer<Configuration>>(
        JsonToFlatbuffer(R"config({
          "nodes": [
            {"name": "node1"},
            {"name": "node2"}
          ]
        })config",
                         Configuration::MiniReflectTypeTable()));

    // Initialize nodes after configuration initialization
    node1_config1 = config1->message().nodes()->Get(0);
    node2_config1 = config1->message().nodes()->Get(1);
    node1_config2 = config2->message().nodes()->Get(0);
    node2_config2 = config2->message().nodes()->Get(1);
  }

  void SetUp() override {
    ConfigurationTest::SetUp();
    // Any additional setup can go here.
  }
};

// Test case when node exists in the configuration.
TEST_F(IsNodeFromConfigurationFixtureTest, NodeExistsInConfiguration) {
  EXPECT_TRUE(IsNodeFromConfiguration(&config1->message(), node1_config1));
  EXPECT_TRUE(IsNodeFromConfiguration(&config1->message(), node2_config1));
}

// Test case when node does not exist in the configuration.
TEST_F(IsNodeFromConfigurationFixtureTest, NodeDoesNotExistsInConfiguration) {
  EXPECT_FALSE(IsNodeFromConfiguration(&config1->message(), node1_config2));
  EXPECT_FALSE(IsNodeFromConfiguration(&config1->message(), node2_config2));
}

// Test case for nodes with same names but from different configurations.
TEST_F(IsNodeFromConfigurationFixtureTest, SameNameDifferentConfiguration) {
  EXPECT_FALSE(IsNodeFromConfiguration(&config1->message(), node1_config2));
  EXPECT_FALSE(IsNodeFromConfiguration(&config1->message(), node2_config2));
  EXPECT_FALSE(IsNodeFromConfiguration(&config2->message(), node1_config1));
  EXPECT_FALSE(IsNodeFromConfiguration(&config2->message(), node2_config1));
}

// Test case for null pointers.
TEST_F(IsNodeFromConfigurationFixtureTest, NullPointers) {
  EXPECT_FALSE(IsNodeFromConfiguration(nullptr, nullptr));
  EXPECT_FALSE(IsNodeFromConfiguration(&config1->message(), nullptr));
  EXPECT_FALSE(IsNodeFromConfiguration(nullptr, node1_config1));
}

// Tests that SourceNode reasonably handles both single and multi-node configs.
TEST(IsNodeFromConfigurationTest, SingleNode) {
  FlatbufferDetachedBuffer<Configuration> config_single_node =
      ReadConfig(ArtifactPath("aos/testdata/config1.json"));
  EXPECT_TRUE(IsNodeFromConfiguration(&config_single_node.message(), nullptr));
}

// Tests that we can use a utility to remove individual channels from a
// multi-node config.
TEST_F(ConfigurationTest, MultinodeMerge) {
  FlatbufferDetachedBuffer<Configuration> config =
      ReadConfig(ArtifactPath("aos/testdata/multinode_merge.json"));

  EXPECT_EQ(
      absl::StripSuffix(util::ReadFileToStringOrDie(ArtifactPath(
                            "aos/testdata/multinode_merge_expected.json")),
                        "\n"),
      FlatbufferToJson(config, {.multi_line = true}));
}

}  // namespace aos::configuration::testing
