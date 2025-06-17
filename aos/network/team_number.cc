#include "aos/network/team_number.h"

#include <netinet/in.h>
#include <unistd.h>

#include <cinttypes>
#include <cstdlib>

#include "absl/flags/flag.h"
#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "absl/strings/numbers.h"

ABSL_FLAG(std::string, override_hostname, "",
          "If set, this forces the hostname of this node to be the provided "
          "hostname.");

namespace aos::network {
namespace team_number_internal {

std::optional<uint16_t> ParseRoborioTeamNumber(
    const std::string_view hostname) {
  for (size_t i = 0; i < hostname.size(); i++) {
    if (hostname[i] == '-') {
      const std::string_view num_as_s =
          hostname[hostname.size() - 1] == 'C'
              ? hostname.substr(i + 1, hostname.size() - 5 - i)
              : hostname.substr(i + 1);

      int num;
      if (!absl::SimpleAtoi(num_as_s, &num)) {
        return std::nullopt;
      }
      if (hostname.substr(0, i) == "roboRIO" &&
          std::to_string(num) == num_as_s) {
        return num;
      }
      return std::nullopt;
    }
  }
  return std::nullopt;
}

std::optional<uint16_t> ParsePiOrOrinTeamNumber(
    const std::string_view hostname) {
  if ((hostname.substr(0, 3) != "pi-") && (hostname.substr(0, 5) != "orin-")) {
    return std::nullopt;
  }
  size_t first_separator = hostname.find('-');
  if (first_separator == hostname.npos ||
      first_separator >= hostname.size() - 2) {
    return std::nullopt;
  }
  ++first_separator;
  const size_t second_separator = hostname.find('-', first_separator);
  if (second_separator == hostname.npos) {
    return std::nullopt;
  }
  const std::string_view number_string =
      hostname.substr(first_separator, second_separator - first_separator);
  int number;
  if (!absl::SimpleAtoi(number_string, &number)) {
    return std::nullopt;
  }
  return number;
}

}  // namespace team_number_internal

namespace {

uint16_t override_team;

uint16_t DoGetTeamNumber() {
  if (override_team != 0) {
    return override_team;
  }

  const char *override_number = getenv("AOS_TEAM_NUMBER");
  if (override_number != nullptr) {
    uint32_t result;
    if (!absl::SimpleAtoi(override_number, &result)) {
      ABSL_LOG(FATAL) << "Error parsing AOS_TEAM_NUMBER: " << override_number;
    }
    ABSL_LOG(WARNING)
        << "Team number overriden by AOS_TEAM_NUMBER environment variable to "
        << result;
    return result;
  }
  const auto hostname = GetHostname();
  {
    const auto result = team_number_internal::ParseRoborioTeamNumber(hostname);
    if (result) {
      ABSL_LOG(INFO) << "roboRIO hostname team number is: " << *result;
      return *result;
    }
  }
  {
    const auto result = team_number_internal::ParsePiOrOrinTeamNumber(hostname);
    if (result) {
      ABSL_LOG(INFO) << "Pi/Orin hostname team number is: " << *result;
      return *result;
    }
  }
  ABSL_LOG(FATAL) << "Failed to parse a team number from hostname: "
                  << hostname;
}

}  // namespace

::std::string GetHostname() {
  if (absl::GetFlag(FLAGS_override_hostname).empty()) {
    char buf[256];
    buf[sizeof(buf) - 1] = '\0';
    ABSL_PCHECK(gethostname(buf, sizeof(buf) - 1) == 0);
    return buf;
  } else {
    return absl::GetFlag(FLAGS_override_hostname);
  }
}

uint16_t GetTeamNumber() {
  const static uint16_t result = DoGetTeamNumber();
  return result;
}

void OverrideTeamNumber(uint16_t team) { override_team = team; }

std::optional<std::string_view> ParsePiOrOrin(const std::string_view hostname) {
  if (hostname.substr(0, 3) == "pi-") {
    return std::string_view("pi");
  } else if (hostname.substr(0, 5) == "orin-") {
    return std::string_view("orin");
  } else if (hostname.substr(0, 4) == "imu-") {
    return std::string_view("orin");
  } else
    return std::nullopt;
}

std::optional<uint16_t> ParsePiOrOrinNumber(const std::string_view hostname) {
  if ((hostname.substr(0, 3) != "pi-") && (hostname.substr(0, 5) != "orin-")) {
    return std::nullopt;
  }
  size_t first_separator = hostname.find('-');
  if (first_separator == hostname.npos ||
      first_separator >= hostname.size() - 2) {
    return std::nullopt;
  }
  ++first_separator;
  const size_t second_separator = hostname.find('-', first_separator);
  if (second_separator == hostname.npos) {
    return std::nullopt;
  }
  const std::string_view number_string = hostname.substr(
      second_separator + 1, hostname.size() - second_separator - 1);
  if (number_string.size() == 0) {
    return std::nullopt;
  }

  int number;
  if (!absl::SimpleAtoi(number_string, &number)) {
    return std::nullopt;
  }
  return number;
}

}  // namespace aos::network
