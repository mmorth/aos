#include "aos/flatbuffer_utils.h"

#include "absl/log/absl_check.h"
#include "absl/log/absl_log.h"
#include "flatbuffers/minireflect.h"
#include "flatbuffers/reflection_generated.h"

namespace aos {

FlatbufferType::FlatbufferType(const flatbuffers::TypeTable *type_table) {
  ABSL_CHECK(type_table != nullptr);
  type_table_ = type_table;
}
FlatbufferType::FlatbufferType(const reflection::Schema *schema) {
  ABSL_CHECK(schema != nullptr);
  schema_ = schema;
  ABSL_DCHECK(schema->root_table() != nullptr);
  object_ = schema->root_table();
}

FlatbufferType::FlatbufferType(const reflection::Schema *schema,
                               const reflection::Object *object) {
  ABSL_CHECK(schema != nullptr);
  schema_ = schema;
  ABSL_DCHECK(object != nullptr);
  object_ = object;
}

FlatbufferType::FlatbufferType(const reflection::Schema *schema,
                               const reflection::Enum *fb_enum) {
  ABSL_CHECK(schema != nullptr);
  schema_ = schema;
  ABSL_CHECK(fb_enum != nullptr);
  enum_ = fb_enum;
}

bool FlatbufferType::IsSequence() const {
  if (type_table_) {
    return type_table_->st != flatbuffers::ST_ENUM;
  }
  if (object_) {
    return true;
  }
  if (enum_) {
    return enum_->is_union();
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

bool FlatbufferType::IsTable() const {
  if (type_table_) {
    return type_table_->st == flatbuffers::ST_TABLE;
  }
  if (object_) {
    return !object_->is_struct();
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

bool FlatbufferType::IsStruct() const {
  if (type_table_) {
    return type_table_->st == flatbuffers::ST_STRUCT;
  }
  if (object_) {
    return object_->is_struct();
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

bool FlatbufferType::IsEnum() const {
  if (type_table_) {
    return type_table_->st == flatbuffers::ST_ENUM;
  }
  if (object_) {
    return false;
  }
  if (enum_) {
    return !enum_->is_union();
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

bool FlatbufferType::FieldIsSequence(int index) const {
  ABSL_DCHECK(IsSequence());
  if (type_table_) {
    ABSL_DCHECK_LT(static_cast<size_t>(index), type_table_->num_elems);
    const flatbuffers::TypeCode &type_code = type_table_->type_codes[index];
    if (type_code.base_type != flatbuffers::ET_SEQUENCE) {
      return false;
    }
    ABSL_DCHECK(FieldType(index).IsSequence());
    return true;
  }
  if (object_ || enum_) {
    const reflection::BaseType base_type = ReflectionElementBaseType(index);
    return base_type == reflection::BaseType::Obj ||
           base_type == reflection::BaseType::Union;
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

bool FlatbufferType::FieldIsEnum(int index) const {
  ABSL_DCHECK(IsSequence());
  if (type_table_) {
    ABSL_DCHECK_LT(static_cast<size_t>(index), type_table_->num_elems);
    const flatbuffers::TypeCode &type_code = type_table_->type_codes[index];
    if (type_code.base_type == flatbuffers::ET_SEQUENCE) {
      return false;
    }
    if (type_code.sequence_ref == -1) {
      // Not an enum.
      return false;
    }
    ABSL_DCHECK(FieldType(index).IsEnum());
    return true;
  }
  if (object_ || enum_) {
    const reflection::BaseType base_type = ReflectionElementBaseType(index);
    if (base_type == reflection::BaseType::Obj ||
        base_type == reflection::BaseType::Union) {
      return false;
    }
    return ReflectionType(index)->index() >= 0;
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

std::optional<int64_t> FlatbufferType::EnumValue(std::string_view name) const {
  ABSL_DCHECK(IsEnum());
  ABSL_DCHECK(!object_);
  if (type_table_) {
    for (size_t i = 0; i < type_table_->num_elems; ++i) {
      if (name == type_table_->names[i]) {
        if (type_table_->values) {
          return type_table_->values[i];
        } else {
          return i;
        }
      }
    }
    return std::nullopt;
  }
  if (enum_) {
    for (size_t i = 0; i < enum_->values()->size(); ++i) {
      const auto *const value = enum_->values()->Get(i);
      if (name == value->name()->string_view()) {
        return value->value();
      }
    }
    return std::nullopt;
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

bool FlatbufferType::FieldIsRepeating(int index) const {
  ABSL_DCHECK(IsSequence());
  if (type_table_) {
    ABSL_DCHECK_LT(static_cast<size_t>(index), type_table_->num_elems);
    const flatbuffers::TypeCode &type_code = type_table_->type_codes[index];
    return type_code.is_repeating;
  }
  if (object_ || enum_) {
    const reflection::BaseType type = ReflectionType(index)->base_type();
    ABSL_CHECK(type != reflection::BaseType::None);
    return type == reflection::BaseType::Vector ||
           type == reflection::BaseType::Array;
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

int FlatbufferType::FieldIndex(std::string_view field_name) const {
  ABSL_DCHECK(IsSequence());
  if (type_table_) {
    for (size_t i = 0; i < type_table_->num_elems; ++i) {
      if (field_name == std::string_view(type_table_->names[i])) {
        return i;
      }
    }
    return -1;
  }
  if (object_) {
    for (size_t i = 0; i < object_->fields()->size(); ++i) {
      const reflection::Field *const field = object_->fields()->Get(i);
      if (field_name == field->name()->string_view()) {
        return field->id();
      }
    }
    return -1;
  }
  if (enum_) {
    for (size_t i = 0; i < enum_->values()->size(); ++i) {
      const reflection::EnumVal *const value = enum_->values()->Get(i);
      if (field_name == value->name()->string_view()) {
        return value->value();
      }
    }
    return -1;
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

std::string_view FlatbufferType::FieldName(int index) const {
  ABSL_DCHECK(IsSequence());
  if (type_table_) {
    ABSL_DCHECK_LT(static_cast<size_t>(index), type_table_->num_elems);
    return type_table_->names[index];
  }
  if (object_) {
    return ReflectionObjectField(index)->name()->string_view();
  }
  if (enum_) {
    return ReflectionEnumValue(index)->name()->string_view();
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

namespace {

flatbuffers::ElementaryType ElementaryTypeFromBaseType(
    reflection::BaseType base_type) {
  switch (base_type) {
    case reflection::BaseType::None:
      ABSL_LOG(FATAL) << "Invalid schema";
    case reflection::BaseType::UType:
      return flatbuffers::ElementaryType::ET_UTYPE;
    case reflection::BaseType::Bool:
      return flatbuffers::ElementaryType::ET_BOOL;
    case reflection::BaseType::Byte:
      return flatbuffers::ElementaryType::ET_CHAR;
    case reflection::BaseType::UByte:
      return flatbuffers::ElementaryType::ET_UCHAR;
    case reflection::BaseType::Short:
      return flatbuffers::ElementaryType::ET_SHORT;
    case reflection::BaseType::UShort:
      return flatbuffers::ElementaryType::ET_USHORT;
    case reflection::BaseType::Int:
      return flatbuffers::ElementaryType::ET_INT;
    case reflection::BaseType::UInt:
      return flatbuffers::ElementaryType::ET_UINT;
    case reflection::BaseType::Long:
      return flatbuffers::ElementaryType::ET_LONG;
    case reflection::BaseType::ULong:
      return flatbuffers::ElementaryType::ET_ULONG;
    case reflection::BaseType::Float:
      return flatbuffers::ElementaryType::ET_FLOAT;
    case reflection::BaseType::Double:
      return flatbuffers::ElementaryType::ET_DOUBLE;
    case reflection::BaseType::String:
      return flatbuffers::ElementaryType::ET_STRING;
    case reflection::BaseType::Obj:
      return flatbuffers::ElementaryType::ET_SEQUENCE;
    case reflection::BaseType::Union:
      return flatbuffers::ElementaryType::ET_SEQUENCE;
    default:
      ABSL_LOG(FATAL) << "Unknown BaseType";
  }
}

}  // namespace

flatbuffers::ElementaryType FlatbufferType::FieldElementaryType(
    int index) const {
  ABSL_DCHECK(IsSequence());
  if (type_table_) {
    ABSL_DCHECK_LT(static_cast<size_t>(index), type_table_->num_elems);
    const flatbuffers::TypeCode &type_code = type_table_->type_codes[index];
    return static_cast<flatbuffers::ElementaryType>(type_code.base_type);
  }
  if (object_ || enum_) {
    return ElementaryTypeFromBaseType(ReflectionElementBaseType(index));
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

namespace {

size_t BaseTypeInlineSize(reflection::BaseType base_type) {
  switch (base_type) {
    case reflection::BaseType::None:
      ABSL_LOG(FATAL) << "Invalid schema";
    case reflection::BaseType::UType:
    case reflection::BaseType::Bool:
    case reflection::BaseType::Byte:
    case reflection::BaseType::UByte:
      return 1;
    case reflection::BaseType::Short:
    case reflection::BaseType::UShort:
      return 2;
    case reflection::BaseType::Int:
    case reflection::BaseType::UInt:
    case reflection::BaseType::Float:
    case reflection::BaseType::String:
      return 4;
    case reflection::BaseType::Long:
    case reflection::BaseType::ULong:
    case reflection::BaseType::Double:
      return 8;
    case reflection::BaseType::Union:
      return 4;
    default:
      ABSL_LOG(FATAL) << "Unknown BaseType";
  }
}

}  // namespace

size_t FlatbufferType::InlineSize() const {
  ABSL_DCHECK(IsSequence());
  if (type_table_) {
    return flatbuffers::InlineSize(flatbuffers::ElementaryType::ET_SEQUENCE,
                                   type_table_);
  }
  if (object_) {
    return object_->is_struct() ? object_->bytesize() : /*offset size*/ 4u;
  }
  if (enum_) {
    return BaseTypeInlineSize(enum_->underlying_type()->base_type());
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

// Returns the required alignment for this type.
size_t FlatbufferType::Alignment() const {
  if (type_table_) {
    // Attempt to derive alignment as max alignment of the members.
    size_t alignment = 1u;
    for (size_t field_index = 0;
         field_index < static_cast<size_t>(NumberFields()); ++field_index) {
      alignment = std::max(alignment, FieldInlineAlignment(field_index));
    }
    return alignment;
  }
  if (object_) {
    return object_->minalign();
  }
  // We don't do a great job of supporting unions in general, and as of this
  // writing did not try to look up what the alignment rules for unions were.
  ABSL_LOG(FATAL) << "Unimplemented";
}

size_t FlatbufferType::FieldInlineAlignment(size_t field_index) const {
  if (FieldIsSequence(field_index) && FieldType(field_index).IsStruct()) {
    return FieldType(field_index).Alignment();
  }
  return FieldInlineSize(field_index);
}

size_t FlatbufferType::StructFieldOffset(int index) const {
  ABSL_DCHECK(IsStruct());
  if (type_table_) {
    return type_table_->values[index];
  }
  if (object_) {
    return ReflectionObjectField(index)->offset();
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

size_t FlatbufferType::FieldInlineSize(int index) const {
  ABSL_DCHECK(IsSequence());
  if (type_table_) {
    return flatbuffers::InlineSize(
        FieldElementaryType(index),
        FieldIsSequence(index) ? FieldType(index).type_table_ : nullptr);
  }
  if (object_ || enum_) {
    const reflection::Type *const type = ReflectionType(index);
    const reflection::BaseType element_base_type =
        ReflectionElementBaseType(index);
    int element_size;
    if (element_base_type == reflection::BaseType::Obj) {
      const FlatbufferType field_type = FieldType(index);
      if (field_type.object_ && field_type.object_->is_struct()) {
        element_size = field_type.object_->bytesize();
      } else {
        element_size = 4;
      }
    } else {
      element_size = BaseTypeInlineSize(element_base_type);
    }
    if (type->base_type() == reflection::BaseType::Array) {
      return element_size * type->fixed_length();
    }
    return element_size;
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

int FlatbufferType::NumberFields() const {
  ABSL_DCHECK(IsSequence());
  if (type_table_) {
    return type_table_->num_elems;
  }
  if (object_) {
    return object_->fields()->size();
  }
  if (enum_) {
    return enum_->values()->size();
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

FlatbufferType FlatbufferType::FieldType(int index) const {
  ABSL_DCHECK(IsSequence());
  if (type_table_) {
    ABSL_DCHECK_LT(static_cast<size_t>(index), type_table_->num_elems);
    const flatbuffers::TypeCode &type_code = type_table_->type_codes[index];
    ABSL_CHECK_GE(type_code.sequence_ref, 0);
    // type_refs can be shorter than num_elems, but not longer, so this is still
    // a valid sanity check.
    ABSL_DCHECK_LT(static_cast<size_t>(type_code.sequence_ref),
                   type_table_->num_elems);
    const flatbuffers::TypeFunction type_function =
        type_table_->type_refs[type_code.sequence_ref];
    return FlatbufferType(type_function());
  }
  if (object_ || enum_) {
    const reflection::BaseType base_type = ReflectionElementBaseType(index);
    const int object_index = ReflectionType(index)->index();
    ABSL_CHECK(object_index >= 0) << ": Invalid schema";
    if (base_type == reflection::BaseType::Obj ||
        base_type == reflection::BaseType::Union) {
      ABSL_DCHECK_LT(static_cast<size_t>(object_index),
                     schema_->objects()->size());
      return FlatbufferType(schema_, schema_->objects()->Get(object_index));
    } else {
      ABSL_DCHECK_LT(static_cast<size_t>(object_index),
                     schema_->enums()->size());
      return FlatbufferType(schema_, schema_->enums()->Get(object_index));
    }
  }
  ABSL_LOG(FATAL) << "Unimplemented";
}

const reflection::Type *FlatbufferType::ReflectionType(int index) const {
  if (object_) {
    return ReflectionObjectField(index)->type();
  } else {
    return ReflectionEnumValue(index)->union_type();
  }
}

const reflection::Field *FlatbufferType::ReflectionObjectField(
    int index) const {
  ABSL_DCHECK(object_);
  const auto result = std::find_if(
      object_->fields()->begin(), object_->fields()->end(),
      [index](const reflection::Field *field) { return field->id() == index; });
  ABSL_DCHECK(result != object_->fields()->end());
  return *result;
}

const reflection::EnumVal *FlatbufferType::ReflectionEnumValue(
    int index) const {
  ABSL_DCHECK(enum_);
  const auto result =
      std::find_if(enum_->values()->begin(), enum_->values()->end(),
                   [index](const reflection::EnumVal *value) {
                     return value->value() == index;
                   });
  ABSL_DCHECK(result != enum_->values()->end());
  return *result;
}

reflection::BaseType FlatbufferType::ReflectionElementBaseType(
    int index) const {
  const reflection::Type *const type = ReflectionType(index);
  reflection::BaseType base_type = type->base_type();
  if (base_type == reflection::BaseType::Vector ||
      base_type == reflection::BaseType::Array) {
    base_type = type->element();
  }
  ABSL_CHECK(base_type != reflection::BaseType::None);
  return base_type;
}

}  // namespace aos
