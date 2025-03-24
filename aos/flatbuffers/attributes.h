#ifndef AOS_FLATBUFFERS_ATTRIBUTES_H_
#define AOS_FLATBUFFERS_ATTRIBUTES_H_
#include "flatbuffers/reflection_generated.h"
namespace aos::fbs {
// Returns the flatbuffer field or object attribute with the specified name, if
// available.
template <typename T>
std::optional<std::string_view> GetAttribute(const T *entity,
                                             std::string_view attribute) {
  if (!entity->has_attributes()) {
    return std::nullopt;
  }
  const reflection::KeyValue *kv =
      entity->attributes()->LookupByKey(attribute.data());
  if (kv == nullptr) {
    return std::nullopt;
  }
  return kv->value()->string_view();
}
}  // namespace aos::fbs
#endif  // AOS_FLATBUFFERS_ATTRIBUTES_H_
