#ifndef AOS_UTIL_FILE_H_
#define AOS_UTIL_FILE_H_

#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <array>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/types/span.h"

#include "aos/scoped/scoped_fd.h"

namespace aos::util {

// Returns the complete contents of filename. LOG(FATAL)s if any errors are
// encountered.
std::string ReadFileToStringOrDie(const std::string_view filename);

// Returns the complete contents of filename. Returns nullopt, but never dies
// if any errors are encountered.
std::optional<std::string> MaybeReadFileToString(
    const std::string_view filename);

// Returns the complete contents of filename as a byte vector. LOG(FATAL)s if
// any errors are encountered.
std::vector<uint8_t> ReadFileToVecOrDie(const std::string_view filename);

// Creates filename if it doesn't exist and sets the contents to contents.
void WriteStringToFileOrDie(const std::string_view filename,
                            const std::string_view contents,
                            mode_t permissions = S_IRWXU);

// Creates all parent directories in the given path.
// For example, if path is "/a/b/c/file.txt", it creates directories "/a",
// "/a/b", and "/a/b/c". If path is "/a/b/c/" (with a trailing slash), it
// creates directories "/a", "/a/b", and "/a/b/c". When sync is true, each
// created directory and its parent directory are synced to disk. Returns true
// if it succeeds or false if the filesystem is full.
bool MkdirPIfSpace(std::string_view path, mode_t mode, bool sync = false);

// Synchronizes the provided directory to disk.
void SyncDirectory(const std::filesystem::path &path);

inline void MkdirP(std::string_view path, mode_t mode) {
  CHECK(MkdirPIfSpace(path, mode));
}

bool PathExists(std::string_view path);

// Recursively removes everything in the provided path.  Ignores any errors it
// runs across.
void UnlinkRecursive(std::string_view path);

enum class FileOptions { kReadable, kWriteable };

enum class FileReaderErrorType { kFatal, kNonFatal };

// Maps file from disk into memory
std::shared_ptr<absl::Span<uint8_t>> MMapFile(
    const std::string &path, FileOptions options = FileOptions::kReadable);

// Wrapper to handle reading the contents of a file into a buffer. Meant for
// situations where the malloc'ing of ReadFileToStringOrDie is inappropriate,
// but where you still want to read a file.
class FileReader {
 public:
  // Opens the file for reading. Crashes if file does not open when flag is set
  // to fatal (default). Logs error if file does not open and flag is non-fatal.
  FileReader(std::string_view filename,
             FileReaderErrorType error_type = FileReaderErrorType::kFatal);
  // Reads the entire contents of the file into the internal buffer and returns
  // a string_view of it.  Returns nullopt if we failed to read the contents
  // (currently only on EREMOTEIO). Note: The result may not be null-terminated.
  std::optional<absl::Span<char>> ReadContents(absl::Span<char> buffer);
  // Returns the value of the file as a string, for a fixed-length file.
  // Returns nullopt if the result is smaller than kSize. Ignores any
  // bytes beyond kSize.
  template <int kSize>
  std::optional<std::array<char, kSize>> ReadString() {
    std::array<char, kSize> result;
    const std::optional<absl::Span<char>> used_span =
        ReadContents(absl::Span<char>(result.data(), result.size()));
    if (!used_span.has_value()) {
      return std::nullopt;
    }

    if (used_span->size() == kSize) {
      return result;
    } else {
      return std::nullopt;
    }
  }
  // Returns the value of the file as an integer. Crashes if it doesn't fit in a
  // 32-bit integer. The value may start with 0x for a hex value, otherwise it
  // must be base 10.  Returns nullopt if we failed to read the file.
  std::optional<int32_t> ReadInt32();

  // Checks if file could be opened.
  bool is_open() const { return file_.get() != -1; }

 private:
  aos::ScopedFD file_;
};

// Simple interface to allow opening a file for writing and then writing it
// without any malloc's.
class FileWriter {
 public:
  // The result of an individual call to WriteBytes().
  // Because WriteBytes() may repeatedly call write() when partial writes occur,
  // it is possible for a non-zero number of bytes to have been written while
  // still having an error because a late call to write() failed.
  struct WriteResult {
    // Total number of bytes successfully written to the file.
    size_t bytes_written;
    // If the write was successful (return_code > 0), equal to bytes_written.
    // Otherwise, equal to the return value of the final call to write.
    int return_code;
  };

  FileWriter(std::string_view filename, mode_t permissions = S_IRWXU);

  WriteResult WriteBytes(absl::Span<const uint8_t> bytes);
  WriteResult WriteBytes(std::string_view bytes);
  void WriteBytesOrDie(absl::Span<const uint8_t> bytes);
  void WriteBytesOrDie(std::string_view bytes);
  int fd() const { return file_.get(); }

 private:
  aos::ScopedFD file_;
};

}  // namespace aos::util

#endif  // AOS_UTIL_FILE_H_
