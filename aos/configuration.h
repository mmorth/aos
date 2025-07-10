#ifndef AOS_CONFIGURATION_H_
#define AOS_CONFIGURATION_H_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <chrono>
#include <cstdint>
#include <string_view>

#include "aos/configuration_generated.h"  // IWYU pragma: export
#include "aos/flatbuffers.h"

namespace aos {

// Holds global configuration data. All of the functions are safe to call
// from wherever.
namespace configuration {

// Reads a json configuration.  This includes all imports and merges.  Note:
// duplicate imports or invalid paths will result in a FATAL.
FlatbufferDetachedBuffer<Configuration> ReadConfig(
    const std::string_view path,
    const std::vector<std::string_view> &extra_import_paths = {});

// Reads a json configuration.  This includes all imports and merges. Returns
// std::nullopt on duplicate imports or invalid paths.
std::optional<FlatbufferDetachedBuffer<Configuration>> MaybeReadConfig(
    const std::string_view path,
    const std::vector<std::string_view> &extra_import_paths = {});

// Sorts and merges entries in a config.
FlatbufferDetachedBuffer<Configuration> MergeConfiguration(
    const Flatbuffer<Configuration> &config);

// Adds schema definitions to a sorted and merged config from the provided
// schema list.
FlatbufferDetachedBuffer<Configuration> MergeConfiguration(
    const Flatbuffer<Configuration> &config,
    const std::vector<aos::FlatbufferVector<reflection::Schema>> &schemas);

// Merges a configuration json snippet into the provided configuration and
// returns the modified config.
FlatbufferDetachedBuffer<Configuration> MergeWithConfig(
    const Configuration *config, std::string_view json);
FlatbufferDetachedBuffer<Configuration> MergeWithConfig(
    const Configuration *config, const Flatbuffer<Configuration> &addition);
FlatbufferDetachedBuffer<Configuration> MergeWithConfig(
    const Configuration *config, const Configuration &addition);

// Adds the list of schemas to the provided aos_config.json.  This should mostly
// be used for testing and in conjunction with MergeWithConfig.
FlatbufferDetachedBuffer<aos::Configuration> AddSchema(
    std::string_view json,
    const std::vector<FlatbufferVector<reflection::Schema>> &schemas);

// Removes the schemas from the provided configuration. This is useful for
// shrinking the size of the message significantly without affecting most use
// cases.
FlatbufferDetachedBuffer<Configuration> StripConfiguration(
    const Configuration *config);

// Returns the resolved Channel for a name, type, and application name. Returns
// nullptr if none is found.
//
// If the application name is empty, it is ignored.  Maps are processed in
// reverse order, and application specific first.
//
// The config should already be fully merged and sorted (as produced by
// MergeConfiguration() or any of the associated functions).
const Channel *GetChannel(const Configuration *config,
                          const std::string_view name,
                          const std::string_view type,
                          const std::string_view application_name,
                          const Node *node, bool quiet = false);
inline const Channel *GetChannel(const Flatbuffer<Configuration> &config,
                                 const std::string_view name,
                                 const std::string_view type,
                                 const std::string_view application_name,
                                 const Node *node) {
  return GetChannel(&config.message(), name, type, application_name, node);
}
template <typename T>
inline const Channel *GetChannel(const Configuration *config,
                                 const std::string_view name,
                                 const std::string_view application_name,
                                 const Node *node) {
  return GetChannel(config, name, T::GetFullyQualifiedName(), application_name,
                    node);
}
// Convenience wrapper for getting a channel from a specified config if you
// already have the name/type in a Channel object--this is useful if you Channel
// object you have does not point to memory within config.
inline const Channel *GetChannel(const Configuration *config,
                                 const Channel *channel,
                                 const std::string_view application_name,
                                 const Node *node) {
  return GetChannel(config, channel->name()->string_view(),
                    channel->type()->string_view(), application_name, node);
}

// Returns a list of all the channel names that can be used to refer to the
// specified channel on the given node/application. This allows a reverse-lookup
// of any renames that happen.
//
// Performs a forward lookup first, and if the provided combination of
// parameters doesn't resolve to a channel, returns an empty list.
std::set<std::string> GetChannelAliases(const Configuration *config,
                                        std::string_view name,
                                        std::string_view type,
                                        const std::string_view application_name,
                                        const Node *node);
inline std::set<std::string> GetChannelAliases(
    const Configuration *config, const Channel *channel,
    const std::string_view application_name, const Node *node) {
  return GetChannelAliases(config, channel->name()->string_view(),
                           channel->type()->string_view(), application_name,
                           node);
}

// Returns the channel index (or dies) of channel in the provided config.
size_t ChannelIndex(const Configuration *config, const Channel *channel);

// Returns the channel with the *exact* corresponding name and type. Does *not*
// perform any channel mappings.
const Channel *GetFullySpecifiedChannel(const Configuration *config,
                                        const std::string_view name,
                                        const std::string_view type);

// Returns the Node out of the config with the matching name, or nullptr if it
// can't be found.
const Node *GetNode(const Configuration *config, std::string_view name);
const Node *GetNode(const Configuration *config, const Node *node);
// Returns the node with the provided index.  This works in both a single and
// multi-node world.
const Node *GetNode(const Configuration *config, size_t node_index);
// Returns a matching node, or nullptr if the provided node is nullptr and we
// are in a single node world.
const Node *GetNodeOrDie(const Configuration *config, const Node *node);
const Node *GetNodeOrDie(const Configuration *config, std::string_view name);
// Returns the Node out of the configuration which matches our hostname.
// CHECKs if it can't be found.
const Node *GetMyNode(const Configuration *config);
const Node *GetNodeFromHostname(const Configuration *config,
                                std::string_view name);
// Determine if this node exists in this configuration. This function checks to
// make sure this pointer exists in the configuration. This function is useful
// for determing if the node is named the same but from a different
// configuration.
bool IsNodeFromConfiguration(const Configuration *config, const Node *node);

// Returns a printable name for the node.  (singlenode) if we are on a single
// node system, and the name otherwise.
std::string_view NodeName(const Configuration *config, size_t node_index);

// Returns a vector of the nodes in the config.  (nullptr is considered the node
// in a single node world.)
std::vector<const Node *> GetNodes(const Configuration *config);

// Returns a vector of the nodes in the config with the provided tag.  If this
// is a single-node world, we assume that all tags match.
std::vector<const Node *> GetNodesWithTag(const Configuration *config,
                                          std::string_view tag);

// Returns whether the given node has the provided tag. If this is a single-node
// world, we assume that all tags match.
bool NodeHasTag(const Node *node, std::string_view tag);

// Returns the node index for a node.  Note: will be faster if node is a pointer
// to a node in config, but is not required.
int GetNodeIndex(const Configuration *config, const Node *node);
int GetNodeIndex(const Configuration *config, std::string_view name);
// Returns the number of nodes.
size_t NodesCount(const Configuration *config);

// Returns true if we are running in a multinode configuration.
bool MultiNode(const Configuration *config);

// Returns true if the provided channel is sendable on the provided node.
bool ChannelIsSendableOnNode(const Channel *channel, const Node *node);
// Returns true if the provided channel is able to be watched or fetched on the
// provided node.
bool ChannelIsReadableOnNode(const Channel *channel, const Node *node);

// Returns true if the provided channel is sent on the provided node and gets
// forwarded to any other nodes.
bool ChannelIsForwardedFromNode(const Channel *channel, const Node *node);

// Returns true if the message is supposed to be logged on this node.
bool ChannelMessageIsLoggedOnNode(const Channel *channel, const Node *node);
bool ChannelMessageIsLoggedOnNode(const Channel *channel,
                                  std::string_view node_name);

// Returns the number of connections.
size_t ConnectionCount(const Channel *channel);

const Connection *ConnectionToNode(const Channel *channel, const Node *node);
// Returns true if the delivery timestamps are supposed to be logged on this
// node.
bool ConnectionDeliveryTimeIsLoggedOnNode(const Channel *channel,
                                          const Node *node,
                                          const Node *logger_node);
bool ConnectionDeliveryTimeIsLoggedOnNode(const Connection *connection,
                                          const Node *node);

// Prints a channel to json, but without the schema.
std::string CleanedChannelToString(const Channel *channel);
// Prints out a channel to json, but only with the name and type.
std::string StrippedChannelToString(const Channel *channel);

// Returns the node names that this node should be forwarding to.
std::vector<std::string_view> DestinationNodeNames(const Configuration *config,
                                                   const Node *my_node);

// Returns the node names that this node should be receiving messages from.
std::vector<std::string_view> SourceNodeNames(const Configuration *config,
                                              const Node *my_node);

// Returns the source node index for each channel in a config.
std::vector<size_t> SourceNodeIndex(const Configuration *config);

// Returns the source node for a channel.
const Node *SourceNode(const Configuration *config, const Channel *channel);

// Returns the all nodes that are logging timestamps on our node.
std::vector<const Node *> TimestampNodes(const Configuration *config,
                                         const Node *my_node);

// Returns the application for the provided node and name if it exists, or
// nullptr if it does not exist on this node.
const Application *GetApplication(const Configuration *config,
                                  const Node *my_node,
                                  std::string_view application_name);

enum class Autostart { kNo = 0, kYes };
// Returns all applications whose name contains the provided substring. If
// node_name is non-empty, then only the applications that run on that node are
// returned. If autostart is kYes, then only the applications are configured to
// autostart are returned.
std::vector<const Application *> GetApplicationsContainingSubstring(
    const Configuration *config, std::string_view node_name,
    std::string_view substring, Autostart autostart = Autostart::kNo);

// Returns true if the provided application should start on the provided node.
bool ApplicationShouldStart(const Configuration *config, const Node *my_node,
                            const Application *application);

// Returns the number of messages in the queue.
size_t QueueSize(const Configuration *config, const Channel *channel);
size_t QueueSize(size_t frequency,
                 std::chrono::nanoseconds channel_storage_duration);

// Returns the number of scratch buffers in the queue.
int QueueScratchBufferSize(const Channel *channel);

// Searches through configurations for schemas that include a certain type.
const reflection::Schema *GetSchema(const Configuration *config,
                                    std::string_view schema_type);

// GetSchema template
template <typename T>
const reflection::Schema *GetSchema(const Configuration *config) {
  return GetSchema(config, T::GetFullyQualifiedName());
}

// Copy schema reflection into detached flatbuffer
std::optional<FlatbufferDetachedBuffer<reflection::Schema>>
GetSchemaDetachedBuffer(const Configuration *config,
                        std::string_view schema_type);

// Returns the storage duration for a channel.
std::chrono::nanoseconds ChannelStorageDuration(const Configuration *config,
                                                const Channel *channel);

// Adds the specified channel to the config and returns the new, merged, config.
// The channel name is derived from the specified name, the type and schema from
// the provided schema, the source node from the specified node, and all other
// fields (e.g., frequency) will be derived from the overrides parameter.
aos::FlatbufferDetachedBuffer<Configuration> AddChannelToConfiguration(
    const Configuration *config, std::string_view name,
    aos::FlatbufferVector<reflection::Schema> schema,
    const aos::Node *source_node = nullptr, ChannelT overrides = {});

// Build a new configuration that only contains the channels we want to
// include. This is useful for excluding obsolete or deprecated channels, so
// they don't appear in the configuration when reading the log.
FlatbufferDetachedBuffer<Configuration> GetPartialConfiguration(
    const Configuration &configuration,
    std::function<bool(const Channel &)> should_include_channel);
}  // namespace configuration

// Compare and equality operators for Channel.  Note: these only check the name
// and type for equality.
bool operator<(const FlatbufferDetachedBuffer<Channel> &lhs,
               const FlatbufferDetachedBuffer<Channel> &rhs);
bool operator==(const FlatbufferDetachedBuffer<Channel> &lhs,
                const FlatbufferDetachedBuffer<Channel> &rhs);

// Enum to support different message handling strategies when falling behind
enum class FallBehindStrategy {
  CRASH=0,
  USE_LATEST=1,
  USE_OLDEST=2
};

}  // namespace aos

#endif  // AOS_CONFIGURATION_H_
