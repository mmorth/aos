#ifndef AOS_JSON_TOKENIZER_H_
#define AOS_JSON_TOKENIZER_H_

#include <string>
#include <string_view>
#include <vector>

#include "absl/strings/numbers.h"
#include "flatbuffers/util.h"

namespace aos {

// This class implements the state machine at json.org
//
// The only modification is that it:
// * Supports C++ comments /**/ in all whitespace.
// * Supports parsing nan/inf (without quotes) for floating point numbers.
// * Supports a \x escape sequence in strings for non-unicode bytes.
class Tokenizer {
 public:
  Tokenizer(const std::string_view data) : data_(data) {}

  enum class TokenType {
    kEnd,
    kError,
    kStartObject,
    kEndObject,
    kStartArray,
    kEndArray,
    kField,
    kNumberValue,
    kStringValue,
    kTrueValue,
    kFalseValue,
    kNullValue,
  };

  // Returns the next token.
  TokenType Next();

  // Returns the last field_name and field_value.  These are only valid when
  // Next returns them.
  const ::std::string &field_name() const { return field_name_; }
  const ::std::string &field_value() const { return field_value_; }

  // Parses the current field value as a long long.  Returns false if it failed
  // to parse.
  bool FieldAsInt(absl::int128 *value);
  // Parses the current field value as a double.  Returns false if it failed
  // to parse.
  bool FieldAsDouble(double *value);

  // Returns true if we are at the end of the input.
  bool AtEnd() { return data_.size() == 0; }

  const std::string_view data_left() const { return data_; }

 private:
  // Consumes a single character.
  void ConsumeChar() { data_ = data_.substr(1); }

  // Returns the current character.
  char Char() const { return data_[0]; }

  // Consumes a string out of data_.  Populates s with the string.  Returns true
  // if a valid string was found, and false otherwise.
  // data_ is updated only on success.
  bool ConsumeString(::std::string *s);
  // Consumes a number out of data_.  Populates s with the string containing the
  // number.  Returns true if a valid number was found, and false otherwise.
  // data_ is updated only on success.
  bool ConsumeNumber(::std::string *s);
  // Consumes a fixed token out of data_. Returns true if the string was found,
  // and false otherwise.
  // data_ is updated only on success.
  bool Consume(const char *token);
  // Consumes whitespace out of data_. Returns true if the string was found,
  // and false otherwise.
  // data_ is unconditionally updated.
  void ConsumeWhitespace();
  // Consumes a unicode out of data_.  Populates s with the unicode.  Returns
  // true if a valid unicode was found, and false otherwise. data_ is updated
  // only on success.
  bool ConsumeUnicode(::std::string *s);
  // Consumes a \x artbirary-byte-value. This escape sequence is used by the
  // flatbuffer JSON serialization by default to represent non-unicode sequences
  // (e.g., "\xFF" for a string that is just a single byte of all-ones).
  bool ConsumeStringHexByte(::std::string *s);

  // State for the parsing state machine.
  enum class State {
    kExpectField,
    kExpectObjectStart,
    kExpectObjectEnd,
    kExpectArrayEnd,
    kExpectValue,
    kExpectEnd,
  };

  State state_ = State::kExpectObjectStart;

  // Data pointer.
  std::string_view data_;
  // Current line number used for printing debug.
  int linenumber_ = 0;

  // Surrogate pairs i.e. high surrogates (\ud000 - \ud8ff) combined
  // with low surrogates (\udc00 - \udfff) cannot be interpreted when
  // they do not appear as a part of the pair.
  int unicode_high_surrogate_ = -1;

  // Stack used to track which object type we were in when we recursed.
  enum class ObjectType {
    kObject,
    kArray,
  };
  ::std::vector<ObjectType> object_type_;

  // Last field name.
  ::std::string field_name_;
  // Last field value.
  ::std::string field_value_;
};

}  // namespace aos

#endif  // AOS_JSON_TOKENIZER_H_
