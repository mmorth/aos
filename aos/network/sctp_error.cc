#include "aos/network/sctp_error.h"

#include <arpa/inet.h>

#include <array>

namespace aos::sctp {

namespace {

struct ErrorInformation {
  SctpError error;
  const char *explanation;
};

// These error codes are copied from the include/linux/sctp.h file of the
// linux-tegra-5.10 repo. Specifically from this revision:
// https://github.deere.com/ISG-Automation-and-Autonomy-BSP/linux-tegra-5.10/blob/9d5487eceec364596ec7e48f9393e1273ae73a13/include/linux/sctp.h
constexpr auto kErrors = std::to_array<ErrorInformation>({
    {SctpError::SCTP_ERROR_NO_ERROR, "No error"},
    {SctpError::SCTP_ERROR_INV_STRM, "Invalid Stream Identifier"},
    {SctpError::SCTP_ERROR_MISS_PARAM, "Missing Mandatory Parameter"},
    {SctpError::SCTP_ERROR_STALE_COOKIE, "Stale Cookie Error"},
    {SctpError::SCTP_ERROR_NO_RESOURCE, "Out of Resource"},
    {SctpError::SCTP_ERROR_DNS_FAILED, "Unresolvable Address"},
    {SctpError::SCTP_ERROR_UNKNOWN_CHUNK, "Unrecognized Chunk Type"},
    {SctpError::SCTP_ERROR_INV_PARAM, "Invalid Mandatory Parameter"},
    {SctpError::SCTP_ERROR_UNKNOWN_PARAM, "Unrecognized Parameters"},
    {SctpError::SCTP_ERROR_NO_DATA, "No User Data"},
    {SctpError::SCTP_ERROR_COOKIE_IN_SHUTDOWN,
     "Cookie Received While Shutting Down"},
    {SctpError::SCTP_ERROR_RESTART,
     "Restart of an association with new addresses"},
    {SctpError::SCTP_ERROR_USER_ABORT, "User Initiated Abort"},
    {SctpError::SCTP_ERROR_PROTO_VIOLATION, "Protocol Violation"},
    {SctpError::SCTP_ERROR_DEL_LAST_IP,
     "Request to Delete Last Remaining IP Address."},
    {SctpError::SCTP_ERROR_RSRC_LOW,
     "Operation Refused Due to Resource Shortage."},
    {SctpError::SCTP_ERROR_DEL_SRC_IP, "Request to Delete Source IP Address."},
    {SctpError::SCTP_ERROR_ASCONF_ACK,
     "Association Aborted due to illegal ASCONF-ACK"},
    {SctpError::SCTP_ERROR_REQ_REFUSED, "Request refused - no authorization."},
    {SctpError::SCTP_ERROR_UNSUP_HMAC, "Unsupported HMAC Identifier"},
});

}  // namespace

std::optional<SctpError> ToSctpError(uint16_t error) {
  // Convert from network byte order.
  const uint16_t host_error = ntohs(error);

  // Perform the lookup.
  for (const ErrorInformation &info : kErrors) {
    if (static_cast<uint16_t>(info.error) == host_error) {
      return info.error;
    }
  }
  return std::nullopt;
}

std::string_view GetErrorString(std::optional<SctpError> error) {
  if (error.has_value()) {
    for (const ErrorInformation &info : kErrors) {
      if (info.error == *error) {
        return info.explanation;
      }
    }
  }
  return "(unknown)";
}

}  // namespace aos::sctp
