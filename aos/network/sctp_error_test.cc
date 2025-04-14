#include "aos/network/sctp_error.h"

#include <arpa/inet.h>

#include <array>

#include "gtest/gtest.h"

namespace aos::sctp::testing {

// Validates that the errors can be converted to/from uint16_t consistently.
TEST(SctpErrorTest, RoundTrip) {
  for (const auto &[error, raw_error] :
       std::to_array<std::pair<SctpError, uint16_t>>({
           {SctpError::SCTP_ERROR_NO_ERROR, 0x0000},
           {SctpError::SCTP_ERROR_INV_STRM, 0x0100},
           {SctpError::SCTP_ERROR_MISS_PARAM, 0x0200},
           {SctpError::SCTP_ERROR_STALE_COOKIE, 0x0300},
           {SctpError::SCTP_ERROR_NO_RESOURCE, 0x0400},
           {SctpError::SCTP_ERROR_DNS_FAILED, 0x0500},
           {SctpError::SCTP_ERROR_UNKNOWN_CHUNK, 0x0600},
           {SctpError::SCTP_ERROR_INV_PARAM, 0x0700},
           {SctpError::SCTP_ERROR_UNKNOWN_PARAM, 0x0800},
           {SctpError::SCTP_ERROR_NO_DATA, 0x0900},
           {SctpError::SCTP_ERROR_COOKIE_IN_SHUTDOWN, 0x0a00},
           {SctpError::SCTP_ERROR_RESTART, 0x0b00},
           {SctpError::SCTP_ERROR_USER_ABORT, 0x0c00},
           {SctpError::SCTP_ERROR_PROTO_VIOLATION, 0x0d00},
           {SctpError::SCTP_ERROR_DEL_LAST_IP, 0xa000},
           {SctpError::SCTP_ERROR_RSRC_LOW, 0xa100},
           {SctpError::SCTP_ERROR_DEL_SRC_IP, 0xa200},
           {SctpError::SCTP_ERROR_ASCONF_ACK, 0xa300},
           {SctpError::SCTP_ERROR_REQ_REFUSED, 0xa400},
           {SctpError::SCTP_ERROR_UNSUP_HMAC, 0x0501},
       })) {
    EXPECT_EQ(static_cast<uint16_t>(error), ntohs(raw_error));
    const std::optional<SctpError> maybe_error = ToSctpError(raw_error);
    ASSERT_TRUE(maybe_error.has_value());
    EXPECT_EQ(*maybe_error, error);
  }
}

// Validates that bogus error codes result in a "(unknown)" error string.
TEST(SctpErrorTest, Unknown) {
  EXPECT_EQ("(unknown)", GetErrorString(std::nullopt));
  EXPECT_EQ("(unknown)", GetErrorString(static_cast<SctpError>(0xffff)));
}

// Validates a couple of successful GetErrorString conversions.
TEST(SctpErrorTest, Success) {
  EXPECT_EQ(GetErrorString(SctpError::SCTP_ERROR_INV_STRM),
            "Invalid Stream Identifier");
  EXPECT_EQ(GetErrorString(SctpError::SCTP_ERROR_USER_ABORT),
            "User Initiated Abort");
}

}  // namespace aos::sctp::testing
