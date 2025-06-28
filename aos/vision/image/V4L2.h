#ifndef AOS_CAMERA_V4L2_H_
#define AOS_CAMERA_V4L2_H_

// This file handles including everything needed to use V4L2 and has some
// utility functions.

#include <asm/types.h> /* for videodev2.h */
#include <linux/videodev2.h>
#include <sys/ioctl.h>

namespace camera {

static inline int xioctl(int fd, int request, void *arg) {
  int r;
  do {
    r = ioctl(fd, request, reinterpret_cast<uintptr_t>(arg));
  } while (r == -1 && errno == EINTR);
  return r;
}

}  // namespace camera

#endif
