#ifndef AOS_NETWORK_SCTP_LIB_H_
#define AOS_NETWORK_SCTP_LIB_H_

#include <arpa/inet.h>
#include <linux/sctp.h>
#include <linux/version.h>

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/types/span.h"

#include "aos/unique_malloc_ptr.h"

#define HAS_SCTP_AUTH LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)

namespace aos::message_bridge {

constexpr bool HasSctpAuth() { return HAS_SCTP_AUTH; }

// Check if ipv6 is enabled.
// If we don't try IPv6, and omit AI_ADDRCONFIG when resolving addresses, the
// library will happily resolve nodes to IPv6 IPs that can't be used. If we add
// AI_ADDRCONFIG, the unit tests no longer work because they only have loopback
// addresses available.
bool Ipv6Enabled();

// Resolves a socket and returns the address.  This can be either an ipv4 or
// ipv6 address.
struct sockaddr_storage ResolveSocket(std::string_view host, int port,
                                      bool use_ipv6);

// Returns a formatted version of the address.
std::string Address(const struct sockaddr_storage &sockaddr);
// Returns a formatted version of the address family.
std::string_view Family(const struct sockaddr_storage &sockaddr);

// Message received.
// This message is malloced bigger than needed and the extra space after it is
// the data.
struct Message {
  // Struct to let us force data to be well aligned.
  struct OveralignedChar {
    uint8_t data alignas(32);
  };

  // Headers.
  struct {
    struct sctp_rcvinfo rcvinfo;
  } header;

  // Address of the sender.
  struct sockaddr_storage sin;

  // Data type. Is it a block of data, or is it a struct sctp_notification?
  enum MessageType {
    // Block of data?
    kMessage,
    // struct sctp_notification?
    kNotification,
    // Client sent too large a message and was disconnected.
    kOverflow,
  } message_type;

  size_t size = 0u;
  uint8_t *mutable_data() {
    return reinterpret_cast<uint8_t *>(&actual_data[0].data);
  }
  const uint8_t *data() const {
    return reinterpret_cast<const uint8_t *>(&actual_data[0].data);
  }

  uint32_t partial_deliveries = 0;

  // Whenever a Message object that is using the internal message pool in the
  // SctpReadWrite class, but is currently owned by the user, it wil have this
  // flag set (must_be_returned_to_pool will get set back to false when it is
  // actually in the pool).
  // Note: We can't just do the actual returning-to-the-pool in the
  // Message::~Message() destructor because at that point the memory is already
  // committed to being free'd; if we separated out the allocation of just the
  // actual_data[] member from the allocation of the struct as a whole we could
  // perhaps do this differently.
  bool must_be_returned_to_pool = false;

  ~Message() {
    CHECK(!must_be_returned_to_pool)
        << ": Did you attempt to destroy a Message without "
           "using FreeMessage()?";
  }

  // Returns a human readable peer IP address.
  std::string PeerAddress() const;

  // Prints out the RcvInfo structure.
  void LogRcvInfo() const;

  // The start of the data.
  OveralignedChar actual_data[];
};

void PrintNotification(const Message *msg);

std::string GetHostname();

// Gets and logs the contents of the sctp_status message.
void LogSctpStatus(int fd, sctp_assoc_t assoc_id);

// Authentication method used for the SCTP socket.
enum class SctpAuthMethod {
  // Use unauthenticated sockets.
  kNoAuth,
  // Use RFC4895 authentication for SCTP.
  kAuth,
};

// Manages reading and writing SCTP messages.
class SctpReadWrite {
 public:
  // When `requested_authentication` is kAuth, it will use SCTP authentication
  // if it's provided by the kernel. Note that this will ignore the value of
  // `requested_authentication` if the kernel is too old and will fall back to
  // an unauthenticated channel.
  SctpReadWrite(
      SctpAuthMethod requested_authentication = SctpAuthMethod::kNoAuth)
      : sctp_authentication_(HasSctpAuth() ? requested_authentication ==
                                                 SctpAuthMethod::kAuth
                                           : false) {
    LOG_IF(WARNING,
           requested_authentication == SctpAuthMethod::kAuth && !HasSctpAuth())
        << "SCTP authentication requested but not provided by the kernel... "
           "You may need a newer kernel";
  }
  ~SctpReadWrite() { CloseSocket(); }

  // Opens a new socket.
  void OpenSocket(const struct sockaddr_storage &sockaddr_local);

  // Sends a message to the kernel.
  // Returns true for success. Will not send a partial message on failure.
  bool SendMessage(int stream, std::string_view data, int time_to_live,
                   std::optional<struct sockaddr_storage> sockaddr_remote,
                   sctp_assoc_t snd_assoc_id);

  // Reads from the kernel until a complete message is received or it blocks.
  // Returns nullptr if the kernel blocks before returning a complete message.
  aos::unique_c_ptr<Message> ReadMessage();

  // Send an abort message for the given association.
  bool Abort(sctp_assoc_t snd_assoc_id);

  int fd() const { return fd_; }

  void SetMaxReadSize(size_t max_size) {
    CHECK(partial_messages_.empty())
        << ": May not update size with queued fragments because we do not "
           "track individual message sizes";
    max_read_size_ = max_size;
    if (fd_ != -1) {
      DoSetMaxSize();
    }
  }

  void SetMaxWriteSize(size_t max_size) {
    CHECK(partial_messages_.empty())
        << ": May not update size with queued fragments because we do not "
           "track individual message sizes";
    max_write_size_ = max_size;
    if (fd_ != -1) {
      DoSetMaxSize();
    }
  }

  // Returns a message returned from ReadMessage back to the pool.
  void FreeMessage(aos::unique_c_ptr<Message> &&message);

  // Allocates messages for the pool.  SetMaxSize must be set first.
  void SetPoolSize(size_t pool_size);

  // Set the active authentication key to `auth_key`.
  void SetAuthKey(absl::Span<const uint8_t> auth_key);

 private:
  aos::unique_c_ptr<Message> AcquireMessage();

  void CloseSocket();
  void DoSetMaxSize();

  // Examines a notification message for ones we handle here.
  // Returns true if the notification was handled by this class.
  bool ProcessNotification(const Message *message);

  int fd_ = -1;

  // We use this as a unique identifier that just increments for each message.
  uint32_t send_ppid_ = 0;

  size_t max_read_size_ = 1000;
  size_t max_write_size_ = 1000;

  // TODO(james): It is not clear to me that we really need to be doing this
  // aos::unique_c_ptr thing here; the memory in Message does not need to be
  // contiguous (it's just that the internal data buffer needs to have a certain
  // alignment); it may make sense to move the unique_c_ptr down to inside of
  // the Message object instead.
  std::vector<aos::unique_c_ptr<Message>> partial_messages_;

  bool use_pool_ = false;
  std::vector<aos::unique_c_ptr<Message>> free_messages_;

  // Use SCTP authentication (RFC4895).
  bool sctp_authentication_;
  std::vector<uint8_t> current_key_;
};

// Returns the max network buffer available for reading for a socket.
size_t ReadRMemMax();
// Returns the max network buffer available for writing for a socket.
size_t ReadWMemMax();

}  // namespace aos::message_bridge

#endif  // AOS_NETWORK_SCTP_LIB_H_
