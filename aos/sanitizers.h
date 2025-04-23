#ifndef AOS_SANITIZERS_H_
#define AOS_SANITIZERS_H_

// Normalize evertyh
#if defined(__has_feature)
#if __has_feature(address_sanitizer)
#define __SANITIZE_ADDRESS__ 1
#endif
#endif

#if defined(__has_feature)
#if __has_feature(memory_sanitizer)
#define __SANITIZE_MEMORY__ 1
#endif
#endif

#if defined(__SANITIZE_ADDRESS__)
#define AOS_SANITIZE_ADDRESS 1
#endif

#if defined(__SANITIZE_MEMORY__)
#define AOS_SANITIZE_MEMORY 1
#endif

#endif  // AOS_SANITIZERS_H_
