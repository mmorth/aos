#ifndef AOS_JSON_TO_FLATBUFFER_H_
#define AOS_JSON_TO_FLATBUFFER_H_

#include <cstddef>
#include <fstream>
#include <string>
#include <string_view>

#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection.h"

#include "aos/fast_string_builder.h"
#include "aos/flatbuffer_utils.h"
#include "aos/flatbuffers.h"
#include "aos/flatbuffers/builder.h"
#include "aos/util/file.h"

namespace aos {

// Parses the flatbuffer into the buffer, or returns an empty buffer.
flatbuffers::DetachedBuffer JsonToFlatbuffer(std::string_view data,
                                             FlatbufferType type);

// Parses the flatbuffer into the builder, and returns the offset.
flatbuffers::Offset<flatbuffers::Table> JsonToFlatbuffer(
    std::string_view data, FlatbufferType type,
    flatbuffers::FlatBufferBuilder *fbb);

// Typed versions of the above methods.
template <typename T>
inline flatbuffers::DetachedBuffer JsonToFlatbuffer(
    const std::string_view data) {
  return JsonToFlatbuffer(data, FlatbufferType(T::MiniReflectTypeTable()));
}
template <typename T>
inline flatbuffers::Offset<T> JsonToFlatbuffer(
    const std::string_view data, flatbuffers::FlatBufferBuilder *fbb) {
  return flatbuffers::Offset<T>(
      JsonToFlatbuffer(data, FlatbufferType(T::MiniReflectTypeTable()), fbb).o);
}

template <typename T>
inline fbs::Builder<T> JsonToStaticFlatbuffer(const std::string_view data) {
  const aos::FlatbufferDetachedBuffer<typename T::Flatbuffer> fbs =
      JsonToFlatbuffer<typename T::Flatbuffer>(data);
  fbs::Builder<T> builder(std::make_unique<aos::fbs::AlignedVectorAllocator>());
  CHECK(builder.get()->FromFlatbuffer(&fbs.message()));
  return builder;
}

struct JsonOptions {
  // controls if the Json is written out on multiple lines or one.
  bool multi_line = false;
  // the contents of vectors longer than max_vector_size will be skipped.
  size_t max_vector_size = SIZE_MAX;
  // more extensive version of multi_line that prints every single field on its
  // own line.
  bool max_multi_line = false;
  // Will integers be printed in hexadecimal form instead of decimal.
  bool use_hex = false;
  // If set, will output standards-compliant JSON. This does not override
  // max_vector_size, so non-standards-compliant JSON may be output if you use a
  // non-SIZE_MAX value for max_vector_size.
  // When using standards-compliant JSON, the following substitutions will be
  // made:
  // * Non-finite floating-point values will be wrapped with quotes (i.e.,
  //   `"nan"`will be output instead of a bare `nan`).
  // * Strings containing non-UTF-8 characters will be treated as uint8 vectors
  //   (i.e., instead of outputting `"\xFF"` for an invalid UTF-8 string we'd
  //   output `[255]`).
  bool use_standard_json = false;
  // Optional precision for floating-point numbers. When not specified, the
  // default precision is used. This is fractional precision, i.e. the number of
  // digits after the decimal point. Trailing zeros after the first trailing
  // zero to the right of the decimal point will be stripped.
  std::optional<int> float_precision = std::nullopt;
};

// Converts a flatbuffer into a Json string.
// The methods below are generally more useful than TableFlatbufferToJson.
::std::string TableFlatbufferToJson(const flatbuffers::Table *t,
                                    const flatbuffers::TypeTable *typetable,
                                    JsonOptions json_options = {});

// Converts a Flatbuffer<T> holding a flatbuffer to JSON.
template <typename T>
inline ::std::string FlatbufferToJson(const Flatbuffer<T> &flatbuffer,
                                      JsonOptions json_options = {}) {
  return TableFlatbufferToJson(
      reinterpret_cast<const flatbuffers::Table *>(&flatbuffer.message()),
      Flatbuffer<T>::MiniReflectTypeTable(), json_options);
}

template <typename T, typename Enable = T::Flatbuffer>
inline ::std::string FlatbufferToJson(const fbs::Builder<T> &flatbuffer,
                                      JsonOptions json_options = {}) {
  return FlatbufferToJson(flatbuffer.AsFlatbufferSpan(), json_options);
}

// Converts a flatbuffer::Table to JSON.
template <typename T>
typename std::enable_if<
    std::is_base_of<flatbuffers::Table, T>::value,
    std::string>::type inline FlatbufferToJson(const T *flatbuffer,
                                               JsonOptions json_options = {}) {
  return TableFlatbufferToJson(
      reinterpret_cast<const flatbuffers::Table *>(flatbuffer),
      Flatbuffer<T>::MiniReflectTypeTable(), json_options);
}

std::string FlatbufferToJson(const reflection::Schema *schema,
                             const uint8_t *data,
                             JsonOptions json_options = {});

void FlatbufferToJson(FastStringBuilder *builder,
                      const reflection::Schema *schema, const uint8_t *data,
                      JsonOptions json_options = {});

// Generic overload version of FlatbufferToJson for any
// flatbuffers::NativeTable, i.e. the types that have the 'T' suffix in the
// name.
template <typename T, typename = std::enable_if_t<
                          std::is_base_of<flatbuffers::NativeTable, T>::value>>
std::string FlatbufferToJson(const T &native_table,
                             JsonOptions json_options = {}) {
  // Serialize the flatbuffers::NativeTable object so we can use the same
  // function.
  flatbuffers::FlatBufferBuilder builder;
  using TableType = typename T::TableType;
  flatbuffers::Offset<TableType> offset =
      TableType::Pack(builder, &native_table);
  builder.Finish(offset);
  const TableType *message =
      flatbuffers::GetRoot<TableType>(builder.GetBufferPointer());

  // Convert the FlatBuffer to JSON with the given options.
  return aos::FlatbufferToJson(message, json_options);
}

// Writes a Flatbuffer to a file, or dies.
template <typename T>
inline void WriteFlatbufferToJson(std::string_view filename, const T *msg) {
  std::ofstream json_file(std::string(filename), std::ios::out);
  CHECK(json_file) << ": Couldn't open " << filename;
  json_file << FlatbufferToJson(msg);
  json_file.close();
}

template <typename T>
inline void WriteFlatbufferToJson(std::string_view filename,
                                  const Flatbuffer<T> &msg) {
  WriteFlatbufferToJson(filename, &msg.message());
}

// Writes a NonSizePrefixedFlatbuffer to a binary file, or dies.
template <typename T>
inline void WriteFlatbufferToFile(std::string_view filename,
                                  const NonSizePrefixedFlatbuffer<T> &msg) {
  std::ofstream file(std::string(filename),
                     std::ios::out | std::ofstream::binary);
  CHECK(file) << ": Couldn't open " << filename;
  std::copy(msg.span().begin(), msg.span().end(),
            std::ostreambuf_iterator<char>(file));
}

// Parses a file as JSON and returns the corresponding Flatbuffer. Dies if
// reading the file fails, returns an empty buffer if the contents are invalid.
template <typename T>
inline FlatbufferDetachedBuffer<T> JsonFileToFlatbuffer(
    const std::string_view path) {
  return FlatbufferDetachedBuffer<T>(
      JsonToFlatbuffer<T>(util::ReadFileToStringOrDie(path)));
}

// Parses a file as a binary flatbuffer or dies.
template <typename T>
inline FlatbufferVector<T> FileToFlatbuffer(const std::string_view path) {
  const std::string data_string = util::ReadFileToStringOrDie(path);
  ResizeableBuffer data;
  data.resize(data_string.size());
  memcpy(data.data(), data_string.data(), data_string.size());
  return FlatbufferVector<T>(std::move(data));
}

}  // namespace aos

#endif  // AOS_JSON_TO_FLATBUFFER_H_
