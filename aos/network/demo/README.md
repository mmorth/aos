# AOS Network Demo

This directory contains a set of binaries to do some network stress testing. It
allows you to test how certain parameters affect the network performance. This
package contains a "run_ping" and a "run_pong" bundle which are intended to be
run on two different machines on the same network. The binaries let you exercise
the AOS SCTP network stack.

Some knobs you can tweak:

- Message priorities (via the AOS config)
- Message sizes
- Message frequency
- Buffer sizes (e.g. via `message_bridge_server`'s `--force_wmem_max` flag)

## Architectural overview

There are 2 nodes in this setup. The "ping node" and the "pong node". The ping
node sends data to the pong node in two ways:

- Send an `aos.testing.Ping` message at 1 Hz with a monotonically increasing
  counter, and
- Send an `aos.testing.VariableSizeMessage` message with a monotonically
  increasing counter.

The `VariableSizeMessage` is sent at a user-specified frequency and size. The
user can also specify the SCTP stream priorities of both messages.

The goal here is to observe the effect that different sizes, frequencies, and
priorities of the `VariableSizeMessage` have on the `Ping` message.

## Building the bundles

You can build the bundles directly like so:

```console
$ bazel build -c opt @aos//aos/network/demo:run_ping
$ bazel build -c opt @aos//aos/network/demo:run_pong
```

You then need to copy the binaries (and the .runfiles directories) to the
corresponding nodes and run them from there.

## Changing parameters

There are a few parameters that you can change to influence the demo.

These are parameters that you can specify as `bazel` command line flags. You can
also temporarily put them in your `~/.bazelrc` file to avoid clutter on the
command line.

### `--@aos//aos/network/demo:ping_node_name`

This must be the hostname of the ping node. The pong node must be able to reach
the ping node under this name.

### `--@aos//aos/network/demo:pong_node_name`

This must be the hostname of the pong node. The ping node must be able to reach
the pong node under this name.

### `--@aos//aos/network/demo:ping_priority`

The SCTP stream priority of the stream that transfers the `Ping` message from
the ping node to the pong node.

### `--@aos//aos/network/demo:variable_size_message_priority`

The SCTP stream priority of the stream that transfers the `VariableSizeMessage`
message from the ping node to the pong node.

### `--@aos//aos/network/demo:variable_sleep_us`

The number of microseconds to wait between sending one or more
`VariableSizeMessage` messages.

### `--@aos//aos/network/demo:variable_message_size`

The number of bytes to stuff into the `VariableSizeMessage` message every time
it is sent.

### `--@aos//aos/network/demo:variable_num_messages`

The number of `VariableSizeMessage` messages that are sent every
`variable_sleep_us` microseconds in a single burst.

### Other options

You can also change command line parameters of the programs used here by
changing them in the `aos_demo_config.json.jinja` file. Some arguments of
interest may be:

- `--vmodule`
- `--nodie_on_malloc`
- `--force_wmem_max` (on `message_bridge_server`)
