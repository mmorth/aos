#include <cmath>
#include <iostream>
#include <sstream>

#include "absl/log/absl_check.h"

#include "aos/json_to_flatbuffer.h"
#include "aos/util/string_formatting.h"

namespace aos {

namespace {

using reflection::BaseType;

void IntToString(int64_t val, reflection::BaseType type, FastStringBuilder *out,
                 bool use_hex) {
  // For 1-byte types in hex mode, we need to cast to 2 bytes to get the desired
  // output and not unprintable characters.
  if (use_hex) {
    if (BaseType::UByte == type) {
      out->AppendInt(static_cast<uint16_t>(val), true);
      return;
    }
    if (BaseType::Byte == type) {
      out->AppendInt(static_cast<int16_t>(val), true);
      return;
    }
  }

  switch (type) {
    case BaseType::Bool:
      out->AppendBool(static_cast<bool>(val));
      break;
    case BaseType::UByte:
      out->AppendInt(static_cast<uint8_t>(val), use_hex);
      break;
    case BaseType::Byte:
      out->AppendInt(static_cast<int8_t>(val), use_hex);
      break;
    case BaseType::Short:
      out->AppendInt(static_cast<int16_t>(val), use_hex);
      break;
    case BaseType::UShort:
      out->AppendInt(static_cast<uint16_t>(val), use_hex);
      break;
    case BaseType::Int:
      out->AppendInt(static_cast<int32_t>(val), use_hex);
      break;
    case BaseType::UInt:
      out->AppendInt(static_cast<uint32_t>(val), use_hex);
      break;
    case BaseType::Long:
      out->AppendInt(static_cast<int64_t>(val), use_hex);
      break;
    case BaseType::ULong:
      out->AppendInt(static_cast<uint64_t>(val), use_hex);
      break;
    default:
      out->Append("null");
  }
}

namespace {
// Writes the provided float value to the buffer; if it is a NaN, renders it as
// just "nan" (i.e., avoids things like "nan(ind)").
template <typename T>
void AppendMaybeNanToString(const T val,
                            const std::optional<int> float_precision,
                            FastStringBuilder *out) {
  if (std::isnan(val)) {
    out->Append(std::signbit(val) ? "-nan" : "nan");
    return;
  }
  if (float_precision.has_value()) {
    out->Append(util::FormatFloat(val, float_precision.value()));
  } else {
    out->Append(val);
  }
}
}  // namespace

void FloatToString(double val, reflection::BaseType type,
                   FastStringBuilder *out, JsonOptions json_options) {
  // If standards-compliant JSON has been requested, we will encode
  // NaN's/infinities as strings.
  if (json_options.use_standard_json && !std::isfinite(val)) {
    out->Append("\"");
    AppendMaybeNanToString<float>(val, json_options.float_precision, out);
    out->Append("\"");
    return;
  }

  switch (type) {
    case BaseType::Float:
      AppendMaybeNanToString<float>(val, json_options.float_precision, out);
      break;
    case BaseType::Double:
      AppendMaybeNanToString<double>(val, json_options.float_precision, out);
      break;
    default:
      // This should be unreachable under normal circumstances.
      out->Append("null");
      break;
  }
}

template <typename ObjT>
void ObjectToString(
    const reflection::Object *obj,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Object>> *objects,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Enum>> *enums,
    const ObjT *object, FastStringBuilder *out, JsonOptions json_options,
    int tree_depth = 0);

// Get enum value name
std::string_view EnumToString(
    int64_t enum_value,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::EnumVal>>
        *values) {
  auto search = values->LookupByKey(enum_value);
  return search != nullptr ? search->name()->string_view() : std::string_view();
}

// Convert integer to string, checking if it is an enum.
void IntOrEnumToString(
    int64_t val, const reflection::Type *type,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Enum>> *enums,
    FastStringBuilder *out, bool use_hex = false) {
  // Check if integer is an enum and print string, otherwise fallback to
  // printing as int.
  if (type->index() > -1 && type->index() < (int32_t)enums->size()) {
    const reflection::Enum *enum_props = enums->Get(type->index());
    if (!enum_props->is_union()) {
      const std::string_view value_string =
          EnumToString(val, enum_props->values());

      if (value_string.data() != nullptr) {
        out->AppendChar('"');
        out->Append(value_string);
        out->AppendChar('"');
      } else {
        out->AppendInt(val);
      }
    }
  } else {
    if (type->base_type() == BaseType::Vector ||
        type->base_type() == BaseType::Array) {
      IntToString(val, type->element(), out, use_hex);
    } else {
      IntToString(val, type->base_type(), out, use_hex);
    }
  }
}

// Adds a newline and indents
// Every increment in tree depth is two spaces
void AddWrapping(FastStringBuilder *out, int tree_depth) {
  out->Append("\n");
  for (int i = 0; i < tree_depth; i++) {
    out->Append("  ");
  }
}

// Detects if a field should trigger wrapping of the parent object.
bool ShouldCauseWrapping(reflection::BaseType type) {
  switch (type) {
    case BaseType::Vector:
    case BaseType::Obj:
      return true;
    default:
      return false;
  }
}

// Renders a string fields with appropriate escaping and/or conversion to a
// vector.
void RenderString(std::string_view str, const JsonOptions &json_options,
                  FastStringBuilder *out) {
  const bool valid_utf8 = aos::util::ValidateUtf8(str);
  if (json_options.use_standard_json && !valid_utf8) {
    out->Append("[ ");
    for (size_t i = 0; i < str.size(); ++i) {
      if (i != 0) {
        out->Append(", ");
      }
      out->AppendInt(static_cast<uint8_t>(str[i]));
    }
    out->Append(" ]");
  } else {
    out->AppendChar('"');
    for (char c : str) {
      switch (c) {
        case '"':
          out->Append("\\\"");
          break;
        case '\\':
          out->Append("\\\\");
          break;
        case '\b':
          out->Append("\\b");
          break;
        case '\f':
          out->Append("\\f");
          break;
        case '\n':
          out->Append("\\n");
          break;
        case '\r':
          out->Append("\\r");
          break;
        case '\t':
          out->Append("\\t");
          break;
        default:
          out->AppendChar(c);
      }
    }
    out->AppendChar('"');
  }
}

// Print field in flatbuffer table. Field must be populated.
template <typename ObjT>
void FieldToString(
    const ObjT *table, const reflection::Field *field,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Object>> *objects,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Enum>> *enums,
    FastStringBuilder *out, JsonOptions json_options, int tree_depth) {
  const reflection::Type *type = field->type();

  switch (type->base_type()) {
    case BaseType::Bool:
    case BaseType::UByte:
    case BaseType::Byte:
    case BaseType::Short:
    case BaseType::UShort:
    case BaseType::Int:
    case BaseType::UInt:
    case BaseType::Long:
    case BaseType::ULong:
      IntOrEnumToString(GetAnyFieldI(*table, *field), type, enums, out,
                        json_options.use_hex);
      break;
    case BaseType::Float:
    case BaseType::Double:
      FloatToString(GetAnyFieldF(*table, *field), type->base_type(), out,
                    json_options);
      break;
    case BaseType::String:
      if constexpr (std::is_same<flatbuffers::Table, ObjT>()) {
        RenderString(flatbuffers::GetFieldS(*table, *field)->string_view(),
                     json_options, out);
      } else {
        out->Append("null");
      }
      break;
    case BaseType::Vector: {
      if constexpr (std::is_same<flatbuffers::Table, ObjT>()) {
        const flatbuffers::VectorOfAny *vector =
            flatbuffers::GetFieldAnyV(*table, *field);
        reflection::BaseType elem_type = type->element();

        if (vector->size() > json_options.max_vector_size) {
          out->Append("[ \"... ");
          out->AppendInt(vector->size());
          out->Append(" elements ...\" ]");
          break;
        }

        bool wrap = false;
        const int child_tree_depth = tree_depth + 1;

        if (json_options.multi_line) {
          wrap = ShouldCauseWrapping(elem_type);
        }

        out->AppendChar('[');
        if (!wrap) {
          out->AppendChar(' ');
        }
        for (flatbuffers::uoffset_t i = 0; i < vector->size(); ++i) {
          if (i != 0) {
            if (wrap) {
              out->Append(",");
            } else {
              out->Append(", ");
            }
          }
          if (wrap) {
            AddWrapping(out, child_tree_depth);
          }
          if (flatbuffers::IsInteger(elem_type)) {
            IntOrEnumToString(
                flatbuffers::GetAnyVectorElemI(vector, elem_type, i), type,
                enums, out, json_options.use_hex);
          } else if (flatbuffers::IsFloat(elem_type)) {
            FloatToString(flatbuffers::GetAnyVectorElemF(vector, elem_type, i),
                          elem_type, out, json_options);
          } else if (elem_type == BaseType::String) {
            RenderString(flatbuffers::GetAnyVectorElemS(vector, elem_type, i),
                         json_options, out);
          } else if (elem_type == BaseType::Obj) {
            if (type->index() > -1 &&
                type->index() < (int32_t)objects->size()) {
              if (objects->Get(type->index())->is_struct()) {
                ObjectToString(
                    objects->Get(type->index()), objects, enums,
                    flatbuffers::GetAnyVectorElemAddressOf<
                        const flatbuffers::Struct>(
                        vector, i, objects->Get(type->index())->bytesize()),
                    out, json_options, child_tree_depth);
              } else {
                ObjectToString(objects->Get(type->index()), objects, enums,
                               flatbuffers::GetAnyVectorElemPointer<
                                   const flatbuffers::Table>(vector, i),
                               out, json_options, child_tree_depth);
              }
            }
          }
        }
        if (wrap) {
          AddWrapping(out, tree_depth);
        } else {
          out->AppendChar(' ');
        }
        out->AppendChar(']');
      } else {
        out->Append("null");
      }
    } break;
    case BaseType::Obj: {
      if (type->index() > -1 && type->index() < (int32_t)objects->size()) {
        if (objects->Get(type->index())->is_struct()) {
          ObjectToString(objects->Get(type->index()), objects, enums,
                         flatbuffers::GetFieldStruct(*table, *field), out,
                         json_options, tree_depth);
        } else if constexpr (std::is_same<flatbuffers::Table, ObjT>()) {
          ObjectToString(objects->Get(type->index()), objects, enums,
                         flatbuffers::GetFieldT(*table, *field), out,
                         json_options, tree_depth);
        }
      } else {
        out->Append("null");
      }
    } break;
    default:
      out->Append("null");
  }
}

// Prints flatbuffer table or struct given list of possible child objects and
// enums. Prints "null" if the child object type is not found.
template <typename ObjT>
void ObjectToString(
    const reflection::Object *obj,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Object>> *objects,
    const flatbuffers::Vector<flatbuffers::Offset<reflection::Enum>> *enums,
    const ObjT *object, FastStringBuilder *out, JsonOptions json_options,
    int tree_depth) {
  static_assert(std::is_same<flatbuffers::Table, ObjT>() ||
                    std::is_same<flatbuffers::Struct, ObjT>(),
                "Type must be either flatbuffer table or struct");
  bool print_sep = false;

  const int child_tree_depth = tree_depth + 1;

  bool wrap = false;
  if (json_options.max_multi_line) {
    wrap = true;
  } else if (json_options.multi_line) {
    // Check whether this object has objects, vectors, or floats inside of it
    for (const reflection::Field *field : *obj->fields()) {
      if (ShouldCauseWrapping(field->type()->base_type())) {
        wrap = true;
        break;
      }
    }
  }

  out->AppendChar('{');
  if (!wrap) {
    out->AppendChar(' ');
  }

  // Sort fields by field id to ensure they are output in the same order defined
  // by the flatbuffer schema.
  std::vector<const reflection::Field *> sorted_fields(obj->fields()->begin(),
                                                       obj->fields()->end());
  std::sort(sorted_fields.begin(), sorted_fields.end(),
            [](const reflection::Field *a, const reflection::Field *b) {
              return a->id() < b->id();
            });

  for (const reflection::Field *field : sorted_fields) {
    // Check whether this object has the field populated (even for structs,
    // which should have all fields populated)
    if (object->GetAddressOf(field->offset())) {
      if (print_sep) {
        if (wrap) {
          out->Append(",");
        } else {
          out->Append(", ");
        }
      } else {
        print_sep = true;
      }

      if (wrap) {
        AddWrapping(out, child_tree_depth);
      }

      out->AppendChar('"');
      out->Append(field->name()->string_view());
      out->Append("\": ");
      FieldToString(object, field, objects, enums, out, json_options,
                    child_tree_depth);
    }
  }

  if (wrap) {
    AddWrapping(out, tree_depth);
  } else {
    out->AppendChar(' ');
  }

  out->AppendChar('}');
}

}  // namespace

std::string FlatbufferToJson(const reflection::Schema *schema,
                             const uint8_t *data, JsonOptions json_options) {
  FastStringBuilder builder;
  FlatbufferToJson(&builder, schema, data, json_options);
  return builder.MoveResult();
}

void FlatbufferToJson(FastStringBuilder *builder,
                      const reflection::Schema *schema, const uint8_t *data,
                      JsonOptions json_options) {
  ABSL_CHECK(schema != nullptr) << ": Need to provide a schema";
  ABSL_CHECK(builder != nullptr) << ": Need to provide an output builder";

  // It is pretty common to get passed in a nullptr when a test fails.  Rather
  // than CHECK, return a more user friendly result.
  if (data == nullptr) {
    builder->Append("null");
    return;
  }

  const flatbuffers::Table *table = flatbuffers::GetAnyRoot(data);

  const reflection::Object *obj = schema->root_table();

  ObjectToString(obj, schema->objects(), schema->enums(), table, builder,
                 json_options);
}

}  // namespace aos
