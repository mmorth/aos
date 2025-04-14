#ifndef AOS_NETWORK_SCTP_ERROR_H_
#define AOS_NETWORK_SCTP_ERROR_H_

#include <inttypes.h>

#include <optional>
#include <string_view>

namespace aos::sctp {

// These error codes are copied from the include/linux/sctp.h file of the
// linux-tegra-5.10 repo. Specifically from this revision:
// https://github.deere.com/ISG-Automation-and-Autonomy-BSP/linux-tegra-5.10/blob/9d5487eceec364596ec7e48f9393e1273ae73a13/include/linux/sctp.h
//
// Note that the version here is in host byte order, not network byte order.
enum class SctpError : uint16_t {
  SCTP_ERROR_NO_ERROR = 0x00,
  SCTP_ERROR_INV_STRM = 0x01,
  SCTP_ERROR_MISS_PARAM = 0x02,
  SCTP_ERROR_STALE_COOKIE = 0x03,
  SCTP_ERROR_NO_RESOURCE = 0x04,
  SCTP_ERROR_DNS_FAILED = 0x05,
  SCTP_ERROR_UNKNOWN_CHUNK = 0x06,
  SCTP_ERROR_INV_PARAM = 0x07,
  SCTP_ERROR_UNKNOWN_PARAM = 0x08,
  SCTP_ERROR_NO_DATA = 0x09,
  SCTP_ERROR_COOKIE_IN_SHUTDOWN = 0x0a,
  SCTP_ERROR_RESTART = 0x0b,
  SCTP_ERROR_USER_ABORT = 0x0c,
  SCTP_ERROR_PROTO_VIOLATION = 0x0d,
  SCTP_ERROR_DEL_LAST_IP = 0x00A0,
  SCTP_ERROR_RSRC_LOW = 0x00A1,
  SCTP_ERROR_DEL_SRC_IP = 0x00A2,
  SCTP_ERROR_ASCONF_ACK = 0x00A3,
  SCTP_ERROR_REQ_REFUSED = 0x00A4,
  SCTP_ERROR_UNSUP_HMAC = 0x0105,
};

// Converts a raw big-endian (i.e. network order) error code into a strongly
// typed version.
//
// If this error code is unknown, this will return nullopt.
std::optional<SctpError> ToSctpError(uint16_t error);

// Converts an error code into a user readable string. If nullopt is specified,
// then this function will return the string "(unknown)".
std::string_view GetErrorString(std::optional<SctpError> error);

}  // namespace aos::sctp

#endif  // AOS_NETWORK_SCTP_ERROR_H_
