#include "aos/logging/log_namer.h"

#include <dirent.h>
#include <mntent.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ostream>
#include <string>

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"

#include "aos/configuration.h"
#include "aos/time/time.h"

#if defined(__clang)
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#elif defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
#endif

ABSL_FLAG(std::string, logging_folder,
#ifdef AOS_ARCHITECTURE_arm_frc
          "",
#else
          "./logs",
#endif
          "The folder to log to.  If empty, search for the /media/sd*1/ "
          "folder and place logs there.");

namespace aos::logging {
namespace {
void AllocateLogName(char **filename, const char *directory,
                     const char *basename) {
  int fileindex = 0;
  DIR *const d = opendir(directory);
  if (d == nullptr) {
    PLOG(FATAL) << "could not open directory" << directory;
  }
  int index = 0;
  while (true) {
    errno = 0;
    struct dirent *const dir = readdir(d);
    if (dir == nullptr) {
      if (errno == 0) {
        break;
      } else {
        PLOG(FATAL) << "readdir(" << d << ") failed";
      }
    } else {
      char previous_date[512];
      // Look for previous index and date
      const std::string format_string = std::string(basename) + "-%d_%s";
      if (sscanf(dir->d_name, format_string.c_str(), &index, &previous_date) ==
          2) {
        if (index >= fileindex) {
          fileindex = index + 1;
        }
      }
    }
  }
  closedir(d);

  char previous[512];
  ::std::string path = ::std::string(directory) + "/" + basename + "-current";
  ssize_t len = ::readlink(path.c_str(), previous, sizeof(previous));
  if (len != -1) {
    previous[len] = '\0';
  } else {
    previous[0] = '\0';
    LOG(INFO) << "Could not find " << path;
  }
  // Remove subsecond accuracy (after the ".").  We don't need it, and it makes
  // the string very long
  std::string time_short = aos::ToString(aos::realtime_clock::now());
  time_short = time_short.substr(0, time_short.find("."));

  if (asprintf(filename, "%s-%03d_%s", basename, fileindex,
               time_short.c_str()) == -1) {
    PLOG(FATAL) << "couldn't create final name";
  }
  // Fix basename formatting.
  LOG(INFO) << "Created log file (" << directory << "/" << *filename
            << "). Previous file was (" << directory << "/" << previous << ").";
}

bool FoundThumbDrive(const char *path) {
  FILE *mnt_fp = setmntent("/etc/mtab", "r");
  if (mnt_fp == nullptr) {
    LOG(FATAL) << "Could not open /etc/mtab";
  }

  bool found = false;
  struct mntent mntbuf;
  char buf[256];
  while (!found) {
    struct mntent *mount_list = getmntent_r(mnt_fp, &mntbuf, buf, sizeof(buf));
    if (mount_list == nullptr) {
      break;
    }
    if (strcmp(mount_list->mnt_dir, path) == 0) {
      found = true;
    }
  }
  endmntent(mnt_fp);
  return found;
}

bool FindDevice(char *device, size_t device_size) {
  char test_device[10];
  for (char i = 'a'; i < 'z'; ++i) {
    snprintf(test_device, sizeof(test_device), "/dev/sd%c", i);
    VLOG(1) << "Trying to access" << test_device;
    if (access(test_device, F_OK) != -1) {
      snprintf(device, device_size, "sd%c", i);
      return true;
    }
  }
  return false;
}

}  // namespace

std::optional<std::string> MaybeGetLogName(const char *basename) {
  if (absl::GetFlag(FLAGS_logging_folder).empty()) {
    char folder[128];
    {
      char dev_name[8];
      if (!FindDevice(dev_name, sizeof(dev_name))) {
        LOG(INFO) << "Waiting for a device";
        return std::nullopt;
      }
      snprintf(folder, sizeof(folder), "/media/%s1", dev_name);
      if (!FoundThumbDrive(folder)) {
        LOG(INFO) << "Waiting for" << folder;
        return std::nullopt;
      }
      snprintf(folder, sizeof(folder), "/media/%s1/", dev_name);
    }

    if (access(folder, F_OK) == -1) {
      LOG(FATAL) << "folder '" << folder
                 << "' does not exist. please create it.";
    }

    absl::SetFlag(&FLAGS_logging_folder, folder);
  }
  const std::string folder = absl::GetFlag(FLAGS_logging_folder);
  if (access(folder.c_str(), R_OK | W_OK) == -1) {
    LOG(FATAL) << "folder '" << folder << "' does not exist. please create it.";
  }
  LOG(INFO) << "logging to folder '" << folder << "'";

  char *tmp;
  AllocateLogName(&tmp, folder.c_str(), basename);

  std::string log_base_name = folder + "/" + std::string(tmp);
  std::string log_roborio_name = std::string(tmp) + "/";
  free(tmp);

  UpdateCurrentSymlink(folder, basename, log_roborio_name);
  return log_base_name;
}

std::string GetLogName(const char *basename) {
  std::optional<std::string> log_base_name;

  while (true) {
    log_base_name = MaybeGetLogName(basename);

    if (log_base_name.has_value()) {
      break;
    }

    sleep(5);
  }

  return log_base_name.value();
}

void UpdateCurrentSymlink(std::string_view folder, std::string_view basename,
                          std::string_view target) {
  // Convert string_views to std::string to ensure null-termination
  std::string folder_str(folder);
  std::string basename_str(basename);
  std::string target_str(target);

  char *tmp2;
  if (asprintf(&tmp2, "%s/%s-current", folder_str.c_str(),
               basename_str.c_str()) == -1) {
    PLOG(WARNING) << "couldn't create current symlink name";
    return;
  }

  if (unlink(tmp2) == -1 && (errno != EROFS && errno != ENOENT)) {
    LOG(WARNING) << "unlink('" << tmp2 << "') failed";
  }

  if (symlink(target_str.c_str(), tmp2) == -1) {
    PLOG(WARNING) << "symlink('" << target_str << "', '" << tmp2 << "') failed";
  } else {
    VLOG(1) << "Updated symlink " << tmp2 << " -> " << target_str;
  }

  free(tmp2);
}

}  // namespace aos::logging
