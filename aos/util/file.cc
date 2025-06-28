#include "aos/util/file.h"

#include <errno.h>
#include <fcntl.h>
#include <fts.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <iterator>
#include <optional>
#include <ostream>
#include <string_view>

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "flatbuffers/util.h"

#include "aos/sanitizers.h"
#include "aos/scoped/scoped_fd.h"

#if defined(AOS_SANITIZE_MEMORY)
#include <sanitizer/msan_interface.h>
#endif

namespace aos::util {

std::string ReadFileToStringOrDie(const std::string_view filename) {
  std::optional<std::string> r = MaybeReadFileToString(filename);
  ABSL_PCHECK(r.has_value()) << "Failed to read " << filename << " to string";
  return r.value();
}

std::optional<std::string> MaybeReadFileToString(
    const std::string_view filename) {
  std::string r;
  ScopedFD fd(open(::std::string(filename).c_str(), O_RDONLY));
  if (fd.get() == -1) {
    ABSL_PLOG(ERROR) << "Failed to open " << filename;
    return std::nullopt;
  }
  while (true) {
    char buffer[1024];
    const ssize_t result = read(fd.get(), buffer, sizeof(buffer));
    if (result < 0) {
      ABSL_PLOG(ERROR) << "Failed to read from " << filename;
      return std::nullopt;
    }
    if (result == 0) {
      break;
    }
    r.append(buffer, result);
  }
  return r;
}

std::vector<uint8_t> ReadFileToVecOrDie(const std::string_view filename) {
  std::vector<uint8_t> r;
  ScopedFD fd(open(::std::string(filename).c_str(), O_RDONLY));
  ABSL_PCHECK(fd.get() != -1) << ": opening " << filename;
  while (true) {
    uint8_t buffer[1024];
    const ssize_t result = read(fd.get(), buffer, sizeof(buffer));
    ABSL_PCHECK(result >= 0) << ": reading from " << filename;
    if (result == 0) {
      break;
    }
    std::copy(buffer, buffer + result, std::back_inserter(r));
  }
  return r;
}

void WriteStringToFileOrDie(const std::string_view filename,
                            const std::string_view contents,
                            mode_t permissions) {
  FileWriter writer(filename, permissions);
  writer.WriteBytesOrDie(
      {reinterpret_cast<const uint8_t *>(contents.data()), contents.size()});
}

void SyncDirectory(const std::filesystem::path &path) {
  const int dir_fd = open(path.c_str(), O_DIRECTORY);
  ABSL_PCHECK(dir_fd != -1) << "Failed to open directory " << path;
  ABSL_PCHECK(fsync(dir_fd) != -1) << "Failed to fsync directory " << path;
  ABSL_PCHECK(close(dir_fd) != -1) << "Failed to close directory " << path;
}

bool MkdirPIfSpace(std::string_view path, mode_t mode, bool sync) {
  auto last_slash_pos = path.find_last_of("/");

  std::string folder(last_slash_pos == std::string_view::npos
                         ? std::string_view("")
                         : path.substr(0, last_slash_pos));
  if (folder.empty()) {
    return true;
  }
  if (!MkdirPIfSpace(folder, mode, sync)) {
    return false;
  }
  const int result = mkdir(folder.c_str(), mode);
  if (result == -1 && errno == EEXIST) {
    ABSL_VLOG(2) << folder << " already exists";
    return true;
  } else if (result == -1 && errno == ENOSPC) {
    ABSL_VLOG(2) << "Out of space";
    return false;
  } else {
    ABSL_VLOG(1) << "Created " << folder;
  }
  ABSL_PCHECK(result == 0) << ": Error creating " << folder;
  if (sync) {
    // Sync the newly created directory.
    SyncDirectory(std::filesystem::path(folder));

    // Also sync the parent directory to ensure the directory entry is written
    // to disk.
    auto parent_slash_pos = folder.find_last_of("/");
    if (parent_slash_pos != std::string::npos) {
      std::string parent_dir = folder.substr(0, parent_slash_pos);
      if (!parent_dir.empty()) {
        SyncDirectory(std::filesystem::path(parent_dir));
      }
    }
  }
  return true;
}

bool PathExists(std::string_view path) {
  struct stat buffer;
  return stat(path.data(), &buffer) == 0;
}

void UnlinkRecursive(std::string_view path) {
  FTS *ftsp = NULL;
  FTSENT *curr;

  // Cast needed (in C) because fts_open() takes a "char * const *", instead
  // of a "const char * const *", which is only allowed in C++. fts_open()
  // does not modify the argument.
  std::string p(path);
  char *files[] = {const_cast<char *>(p.c_str()), NULL};

  // FTS_NOCHDIR  - Avoid changing cwd, which could cause unexpected behavior
  //                in multithreaded programs
  // FTS_PHYSICAL - Don't follow symlinks. Prevents deletion of files outside
  //                of the specified directory
  // FTS_XDEV     - Don't cross filesystem boundaries
  ftsp = fts_open(files, FTS_NOCHDIR | FTS_PHYSICAL | FTS_XDEV, NULL);
  if (!ftsp) {
    return;
  }

  while ((curr = fts_read(ftsp))) {
#if AOS_SANITIZE_MEMORY
    // fts_read doesn't have proper msan interceptors.  Unpoison it ourselves.
    if (curr) {
      __msan_unpoison(curr, sizeof(*curr));
      __msan_unpoison_string(curr->fts_accpath);
      __msan_unpoison_string(curr->fts_path);
      __msan_unpoison_string(curr->fts_name);
    }
#endif
    switch (curr->fts_info) {
      case FTS_NS:
      case FTS_DNR:
      case FTS_ERR:
        ABSL_LOG(WARNING) << "Can't read " << curr->fts_accpath;
        break;

      case FTS_DC:
      case FTS_DOT:
      case FTS_NSOK:
        // Not reached unless FTS_LOGICAL, FTS_SEEDOT, or FTS_NOSTAT were
        // passed to fts_open()
        break;

      case FTS_D:
        // Do nothing. Need depth-first search, so directories are deleted
        // in FTS_DP
        break;

      case FTS_DP:
      case FTS_F:
      case FTS_SL:
      case FTS_SLNONE:
      case FTS_DEFAULT:
        ABSL_VLOG(1) << "Removing " << curr->fts_path;
        if (remove(curr->fts_accpath) < 0) {
          ABSL_LOG(WARNING)
              << curr->fts_path
              << ": Failed to remove: " << strerror(curr->fts_errno);
        }
        break;
    }
  }

  if (ftsp) {
    fts_close(ftsp);
  }
}

std::shared_ptr<absl::Span<uint8_t>> MMapFile(const std::string &path,
                                              FileOptions options) {
  int fd =
      open(path.c_str(), options == FileOptions::kReadable ? O_RDONLY : O_RDWR);
  ABSL_PCHECK(fd != -1) << "Unable to open file " << path;
  struct stat sb;
  ABSL_PCHECK(fstat(fd, &sb) != -1) << ": Unable to get file size of " << path;
  uint8_t *start = reinterpret_cast<uint8_t *>(mmap(
      NULL, sb.st_size,
      options == FileOptions::kReadable ? PROT_READ : (PROT_READ | PROT_WRITE),
      MAP_SHARED, fd, 0));
  ABSL_CHECK(start != MAP_FAILED)
      << ": Unable to open mapping to file " << path;
  std::shared_ptr<absl::Span<uint8_t>> span =
      std::shared_ptr<absl::Span<uint8_t>>(
          new absl::Span<uint8_t>(start, sb.st_size),
          [](absl::Span<uint8_t> *span) {
            ABSL_PCHECK(msync(span->data(), span->size(), MS_SYNC) == 0)
                << ": Failed to flush data before unmapping.";
            ABSL_PCHECK(munmap(span->data(), span->size()) != -1);
            delete span;
          });
  close(fd);
  return span;
}

FileReader::FileReader(std::string_view filename,
                       FileReaderErrorType error_type)
    : file_(open(::std::string(filename).c_str(), O_RDONLY)) {
  if (!is_open()) {
    ABSL_PLOG_IF(FATAL, error_type == FileReaderErrorType::kFatal)
        << ": opening " << filename;
    ABSL_PLOG(ERROR) << "opening " << filename;
  }
}

std::optional<absl::Span<char>> FileReader::ReadContents(
    absl::Span<char> buffer) {
  ABSL_PCHECK(0 == lseek(file_.get(), 0, SEEK_SET));
  const ssize_t result = read(file_.get(), buffer.data(), buffer.size());
  if (result < 0) {
    // Read timeout for an i2c request returns this.
    if (errno == EREMOTEIO) {
      return std::nullopt;
    }
  }

  ABSL_PCHECK(result >= 0);
  return absl::Span<char>{buffer.data(), static_cast<size_t>(result)};
}

FileWriter::FileWriter(std::string_view filename, mode_t permissions)
    : file_(open(::std::string(filename).c_str(), O_WRONLY | O_CREAT | O_TRUNC,
                 permissions)) {
  ABSL_PCHECK(file_.get() != -1) << ": opening " << filename;
}

// absl::SimpleAtoi doesn't interpret a leading 0x as hex, which we need here.
// Instead, we use the flatbufers API, which unfortunately relies on NUL
// termination.
std::optional<int32_t> FileReader::ReadInt32() {
  // Maximum characters for a 32-bit integer, +1 for the NUL.
  // Hex is the same size with the leading 0x.
  std::array<char, 11> buffer;
  int32_t result;
  const std::optional<absl::Span<char>> string_span =
      ReadContents(absl::Span<char>(buffer.data(), buffer.size())
                       .subspan(0, buffer.size() - 1));
  if (!string_span.has_value()) {
    return std::nullopt;
  }

  // Verify we found the newline.
  ABSL_CHECK_EQ(buffer[string_span->size() - 1], '\n');
  // Truncate the newline.
  buffer[string_span->size() - 1] = '\0';
  ABSL_CHECK(flatbuffers::StringToNumber(buffer.data(), &result))
      << ": Error parsing string to integer: "
      << std::string_view(string_span->data(), string_span->size());

  return result;
}

FileWriter::WriteResult FileWriter::WriteBytes(
    absl::Span<const uint8_t> bytes) {
  size_t size_written = 0;
  while (size_written != bytes.size()) {
    const ssize_t result = write(file_.get(), bytes.data() + size_written,
                                 bytes.size() - size_written);
    if (result < 0) {
      return {size_written, static_cast<int>(result)};
    }
    // Not really supposed to happen unless writing zero bytes without an error.
    // See, e.g.,
    // https://stackoverflow.com/questions/2176443/is-a-return-value-of-0-from-write2-in-c-an-error
    if (result == 0) {
      return {size_written, static_cast<int>(result)};
    }

    size_written += result;
  }
  return {size_written, static_cast<int>(size_written)};
}

FileWriter::WriteResult FileWriter::WriteBytes(std::string_view bytes) {
  return WriteBytes(absl::Span<const uint8_t>{
      reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()});
}

void FileWriter::WriteBytesOrDie(std::string_view bytes) {
  WriteBytesOrDie(absl::Span<const uint8_t>{
      reinterpret_cast<const uint8_t *>(bytes.data()), bytes.size()});
}

void FileWriter::WriteBytesOrDie(absl::Span<const uint8_t> bytes) {
  ABSL_PCHECK(bytes.size() == WriteBytes(bytes).bytes_written)
      << ": Failed to write " << bytes.size() << " bytes.";
}

}  // namespace aos::util
