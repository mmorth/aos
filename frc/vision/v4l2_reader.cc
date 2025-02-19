#include "frc/vision/v4l2_reader.h"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "absl/flags/flag.h"

ABSL_FLAG(bool, ignore_timestamps, false,
          "Don't require timestamps on images.  Used to allow webcams");
ABSL_FLAG(uint32_t, imagewidth, 640,
          "Image capture resolution width in pixels.");
ABSL_FLAG(uint32_t, imageheight, 480,
          "Image capture resolution height in pixels.");
namespace frc::vision {

V4L2ReaderBase::V4L2ReaderBase(aos::EventLoop *event_loop,
                               std::string_view device_name,
                               std::string_view image_channel)
    : fd_(open(device_name.data(), O_RDWR | O_NONBLOCK)),
      event_loop_(event_loop),
      image_channel_(image_channel) {
  PCHECK(fd_.get() != -1) << " Failed to open device " << device_name;

  // Figure out if we are multi-planar or not.
  {
    struct v4l2_capability capability;
    memset(&capability, 0, sizeof(capability));
    PCHECK(Ioctl(VIDIOC_QUERYCAP, &capability) == 0);

    LOG(INFO) << "Opening " << device_name;
    LOG(INFO) << "  driver " << capability.driver;
    LOG(INFO) << "  card " << capability.card;
    LOG(INFO) << "  bus_info " << capability.bus_info;
    if (capability.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
      LOG(INFO) << "  Multi-planar";
      multiplanar_ = true;
    }
  }

  // First, clean up after anybody else who left the device streaming.
  StreamOff();
}

void V4L2ReaderBase::StreamOn() {
  {
    struct v4l2_requestbuffers request;
    memset(&request, 0, sizeof(request));
    request.count = buffers_.size();
    request.type = multiplanar() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                                 : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_USERPTR;
    PCHECK(Ioctl(VIDIOC_REQBUFS, &request) == 0);
    CHECK_EQ(request.count, buffers_.size())
        << ": Kernel refused to give us the number of buffers we asked for";
  }

  {
    struct v4l2_format format;
    memset(&format, 0, sizeof(format));
    format.type = multiplanar() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                                : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    PCHECK(Ioctl(VIDIOC_G_FMT, &format) == 0);

    if (multiplanar()) {
      cols_ = format.fmt.pix_mp.width;
      rows_ = format.fmt.pix_mp.height;
      LOG(INFO) << "Format is " << cols_ << ", " << rows_;
      if (format.fmt.pix_mp.pixelformat == V4L2_PIX_FMT_MJPEG) {
        CHECK_EQ(static_cast<int>(format.fmt.pix_mp.plane_fmt[0].bytesperline),
                 0);
        format_ = ImageFormat::MJPEG;
      } else if (format.fmt.pix_mp.pixelformat == V4L2_PIX_FMT_YUYV) {
        CHECK_EQ(static_cast<int>(format.fmt.pix_mp.plane_fmt[0].bytesperline),
                 cols_ * 2 /* bytes per pixel */);
        format_ = ImageFormat::YUYV422;
      } else {
        LOG(FATAL) << ": Invalid pixel format";
      }

      CHECK_EQ(format.fmt.pix_mp.num_planes, 1u);

      CHECK_EQ(format.fmt.pix_mp.plane_fmt[0].sizeimage, ImageSize());
    } else {
      cols_ = format.fmt.pix.width;
      rows_ = format.fmt.pix.height;
      LOG(INFO) << "Format is " << cols_ << ", " << rows_;
      if (format.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG) {
        CHECK_EQ(static_cast<int>(format.fmt.pix.bytesperline), 0);
        format_ = ImageFormat::MJPEG;
      } else if (format.fmt.pix_mp.pixelformat == V4L2_PIX_FMT_YUYV) {
        CHECK_EQ(static_cast<int>(format.fmt.pix.bytesperline),
                 cols_ * 2 /* bytes per pixel */);
        format_ = ImageFormat::YUYV422;
      } else {
        LOG(FATAL) << ": Invalid pixel format";
      }
      CHECK_EQ(format.fmt.pix.sizeimage, ImageSize());
    }
  }

  for (size_t i = 0; i < buffers_.size(); ++i) {
    buffers_[i].sender = event_loop_->MakeSender<CameraImage>(image_channel_);
    MarkBufferToBeEnqueued(i);
  }
  int type = multiplanar() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                           : V4L2_BUF_TYPE_VIDEO_CAPTURE;
  PCHECK(Ioctl(VIDIOC_STREAMON, &type) == 0);
}

void V4L2ReaderBase::MarkBufferToBeEnqueued(int buffer_index) {
  ReinitializeBuffer(buffer_index);
  EnqueueBuffer(buffer_index);
}

void V4L2ReaderBase::MaybeEnqueue() {
  // First, enqueue any old buffer we already have. This is the one which
  // may have been sent.
  if (saved_buffer_) {
    MarkBufferToBeEnqueued(saved_buffer_.index);
    saved_buffer_.Clear();
  }
  ftrace_.FormatMessage("Enqueued previous buffer %d", saved_buffer_.index);
}

bool V4L2ReaderBase::ReadLatestImage() {
  MaybeEnqueue();

  while (true) {
    const BufferInfo previous_buffer = saved_buffer_;
    saved_buffer_ = DequeueBuffer();
    ftrace_.FormatMessage("Dequeued %d", saved_buffer_.index);
    if (saved_buffer_) {
      // We got a new buffer. Return the previous one (if relevant) and keep
      // going.
      if (previous_buffer) {
        ftrace_.FormatMessage("Previous %d", previous_buffer.index);
        MarkBufferToBeEnqueued(previous_buffer.index);
      }
      continue;
    }
    if (!previous_buffer) {
      // There were no images to read. Return an indication of that.
      ftrace_.FormatMessage("No images to read");
      return false;
    }
    // We didn't get a new one, but we already got one in a previous
    // iteration, which means we found an image so return it.
    ftrace_.FormatMessage("Got saved buffer %d", saved_buffer_.index);
    saved_buffer_ = previous_buffer;
    buffers_[saved_buffer_.index].PrepareMessage(
        rows_, cols_, format_, saved_buffer_.memory_size,
        saved_buffer_.valid_size, saved_buffer_.monotonic_eof);
    return true;
  }
}

void V4L2ReaderBase::SendLatestImage() {
  buffers_[saved_buffer_.index].Send();

  MarkBufferToBeEnqueued(saved_buffer_.index);
  saved_buffer_.Clear();
}

void V4L2ReaderBase::SetExposure(size_t duration) {
  v4l2_control manual_control;
  manual_control.id = V4L2_CID_EXPOSURE_AUTO;
  manual_control.value = V4L2_EXPOSURE_MANUAL;
  PCHECK(Ioctl(VIDIOC_S_CTRL, &manual_control) == 0);

  v4l2_control exposure_control;
  exposure_control.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  exposure_control.value = static_cast<int>(duration);  // 100 micro s units
  PCHECK(Ioctl(VIDIOC_S_CTRL, &exposure_control) == 0);
}

void V4L2ReaderBase::UseAutoExposure() {
  v4l2_control control;
  control.id = V4L2_CID_EXPOSURE_AUTO;
  control.value = V4L2_EXPOSURE_AUTO;
  PCHECK(Ioctl(VIDIOC_S_CTRL, &control) == 0);
}

void V4L2ReaderBase::Buffer::InitializeMessage(size_t max_image_size) {
  message_offset = flatbuffers::Offset<CameraImage>();
  builder = aos::Sender<CameraImage>::Builder();
  builder = sender.MakeBuilder();
  // The kernel has an undocumented requirement that the buffer is aligned
  // to 128 bytes. If you give it a nonaligned pointer, it will return EINVAL
  // and only print something in dmesg with the relevant dynamic debug
  // prints turned on.
  builder.fbb()->StartIndeterminateVector(max_image_size, 1, 128,
                                          &data_pointer);
  CHECK_EQ(reinterpret_cast<uintptr_t>(data_pointer) % 128, 0u)
      << ": Flatbuffers failed to align things as requested";
  CHECK_EQ(max_image_size % 128, 0u)
      << ": Image size must be a multiple of 128";
}

void V4L2ReaderBase::Buffer::PrepareMessage(
    int rows, int cols, ImageFormat format, size_t memory_size,
    size_t valid_size, aos::monotonic_clock::time_point monotonic_eof) {
  // EndIndeterminateVector wants the data to be a length which is a multiple of
  // 4 (the size of uoffset_t).  The code responsible for packing the vector
  // will make sure the starting address is aligned.
  const auto data_offset =
      builder.fbb()->EndIndeterminateVector(memory_size, 1);

  CHECK(data_pointer != nullptr);
  data_pointer = nullptr;

  // Now, trim any extra off the end of the vector by changing the length.
  flatbuffers::uoffset_t *length_offset =
      flatbuffers::GetMutableTemporaryPointer(
          *builder.fbb(),
          flatbuffers::Offset<flatbuffers::uoffset_t>(data_offset));

  *length_offset = valid_size;

  auto image_builder = builder.MakeBuilder<CameraImage>();
  image_builder.add_data(data_offset);
  image_builder.add_format(format);
  image_builder.add_rows(rows);
  image_builder.add_cols(cols);
  image_builder.add_monotonic_timestamp_ns(
      std::chrono::nanoseconds(monotonic_eof.time_since_epoch()).count());
  message_offset = image_builder.Finish();
}

int V4L2ReaderBase::Ioctl(unsigned long number, void *arg) {
  return ioctl(fd_.get(), number, arg);
}

V4L2ReaderBase::BufferInfo V4L2ReaderBase::DequeueBuffer() {
  struct v4l2_buffer buffer;
  memset(&buffer, 0, sizeof(buffer));
  buffer.memory = V4L2_MEMORY_USERPTR;
  size_t memory_size;
  size_t valid_size;
  if (multiplanar()) {
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    struct v4l2_plane planes[1];
    std::memset(planes, 0, sizeof(planes));
    buffer.m.planes = planes;
    buffer.length = 1;
    const int result = Ioctl(VIDIOC_DQBUF, &buffer);
    if (result == -1 && errno == EAGAIN) {
      return BufferInfo();
    }
    PCHECK(result == 0) << ": VIDIOC_DQBUF failed";
    CHECK_LT(buffer.index, buffers_.size());

    CHECK_EQ(reinterpret_cast<uintptr_t>(buffers_[buffer.index].data_pointer),
             planes[0].m.userptr);

    CHECK_EQ(ImageSize(), planes[0].length);
    memory_size = planes[0].length;
  } else {
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    const int result = Ioctl(VIDIOC_DQBUF, &buffer);
    if (result == -1 && errno == EAGAIN) {
      return BufferInfo();
    }
    PCHECK(result == 0) << ": VIDIOC_DQBUF failed";
    CHECK_LT(buffer.index, buffers_.size());
    CHECK_EQ(reinterpret_cast<uintptr_t>(buffers_[buffer.index].data_pointer),
             buffer.m.userptr);
    CHECK_EQ(ImageSize(), buffer.length);
    memory_size = buffer.length;
  }
  CHECK(buffer.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC);
  if (!absl::GetFlag(FLAGS_ignore_timestamps)) {
    // Require that we have good timestamp on images
    CHECK_EQ(buffer.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK,
             static_cast<uint32_t>(V4L2_BUF_FLAG_TSTAMP_SRC_EOF));
  }

  if (format_ == ImageFormat::MJPEG) {
    // The flatbuffer vector needs to have the starting address aligned to 4
    // bytes.  Move the memory block so that the starting address is 4 byte
    // aligned from the end.
    const size_t aligned_bytes_used = (buffer.bytesused + 3) & ~0x3;

    memmove(
        buffers_[buffer.index].data_pointer + memory_size - aligned_bytes_used,
        buffers_[buffer.index].data_pointer, buffer.bytesused);

    // Update the size now that we know we don't need it all.
    memory_size = aligned_bytes_used;
    valid_size = buffer.bytesused;
  } else {
    CHECK_EQ(memory_size, buffer.bytesused);
    valid_size = memory_size;
  }

  return {static_cast<int>(buffer.index), static_cast<int>(memory_size),
          static_cast<int>(valid_size),
          aos::time::from_timeval(buffer.timestamp)};
}

void V4L2ReaderBase::EnqueueBuffer(int buffer_number) {
  CHECK_GE(buffer_number, 0);
  CHECK_LT(buffer_number, static_cast<int>(buffers_.size()));
  CHECK(buffers_[buffer_number].data_pointer != nullptr);

  struct v4l2_buffer buffer;
  struct v4l2_plane planes[1];
  memset(&buffer, 0, sizeof(buffer));
  memset(&planes, 0, sizeof(planes));
  buffer.memory = V4L2_MEMORY_USERPTR;
  buffer.index = buffer_number;
  if (multiplanar()) {
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buffer.m.planes = planes;
    buffer.length = 1;
    planes[0].m.userptr =
        reinterpret_cast<uintptr_t>(buffers_[buffer_number].data_pointer);
    planes[0].length = ImageSize();
    planes[0].bytesused = planes[0].length;
  } else {
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.m.userptr =
        reinterpret_cast<uintptr_t>(buffers_[buffer_number].data_pointer);
    buffer.length = ImageSize();
  }

  PCHECK(Ioctl(VIDIOC_QBUF, &buffer) == 0);
}

void V4L2ReaderBase::StreamOff() {
  int type = multiplanar() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                           : V4L2_BUF_TYPE_VIDEO_CAPTURE;
  const int result = Ioctl(VIDIOC_STREAMOFF, &type);
  if (result == 0) {
    return;
  }
  // Some devices (like Alex's webcam) return this if streaming isn't
  // currently on, unlike what the documentations says should happen.
  if (errno == EBUSY) {
    return;
  }
  PLOG(FATAL) << "VIDIOC_STREAMOFF failed";
}

V4L2Reader::V4L2Reader(aos::EventLoop *event_loop, std::string_view device_name,
                       std::string_view image_channel)
    : V4L2ReaderBase(event_loop, device_name, image_channel) {
  // Don't know why this magic call to SetExposure is required (before the
  // camera settings are configured) to make things work on boot of the pi, but
  // it seems to be-- without it, the image exposure is wrong (too dark). Note--
  // any valid value seems to work-- just choosing 1 for now

  SetExposure(1);

  struct v4l2_format format;
  memset(&format, 0, sizeof(format));
  format.type = multiplanar() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                              : V4L2_BUF_TYPE_VIDEO_CAPTURE;

  const int kWidth = absl::GetFlag(FLAGS_imagewidth);
  const int kHeight = absl::GetFlag(FLAGS_imageheight);
  format.fmt.pix.width = kWidth;
  format.fmt.pix.height = kHeight;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  // This means we want to capture from a progressive (non-interlaced)
  // source.
  format.fmt.pix.field = V4L2_FIELD_NONE;
  PCHECK(Ioctl(VIDIOC_S_FMT, &format) == 0);
  CHECK_EQ(static_cast<int>(format.fmt.pix.width), kWidth);
  CHECK_EQ(static_cast<int>(format.fmt.pix.height), kHeight);
  CHECK_EQ(static_cast<int>(format.fmt.pix.bytesperline),
           kWidth * 2 /* bytes per pixel */);
  CHECK_EQ(format.fmt.pix.sizeimage, ImageSize(kHeight, kWidth));

  StreamOn();
}

MjpegV4L2Reader::MjpegV4L2Reader(aos::EventLoop *event_loop,
                                 aos::internal::EPoll *epoll,
                                 std::string_view device_name,
                                 std::string_view image_channel)
    : V4L2ReaderBase(event_loop, device_name, image_channel), epoll_(epoll) {
  // Don't know why this magic call to SetExposure is required (before the
  // camera settings are configured) to make things work on boot of the pi, but
  // it seems to be-- without it, the image exposure is wrong (too dark). Note--
  // any valid value seems to work-- just choosing 1 for now
  SetExposure(1);

  struct v4l2_format format;
  memset(&format, 0, sizeof(format));
  format.type = multiplanar() ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                              : V4L2_BUF_TYPE_VIDEO_CAPTURE;

  const int kWidth = absl::GetFlag(FLAGS_imagewidth);
  const int kHeight = absl::GetFlag(FLAGS_imageheight);
  format.fmt.pix.width = kWidth;
  format.fmt.pix.height = kHeight;
  format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  // This means we want to capture from a progressive (non-interlaced)
  // source.
  format.fmt.pix.field = V4L2_FIELD_NONE;
  PCHECK(Ioctl(VIDIOC_S_FMT, &format) == 0);
  CHECK_EQ(static_cast<int>(format.fmt.pix.width), kWidth);
  CHECK_EQ(static_cast<int>(format.fmt.pix.height), kHeight);
  CHECK_EQ(static_cast<int>(format.fmt.pix.bytesperline), 0);
  CHECK_EQ(format.fmt.pix.sizeimage, ImageSize(kHeight, kWidth));

  StreamOn();
  epoll_->OnReadable(fd().get(), [this]() {
    if (!ReadLatestImage()) {
      return;
    }

    SendLatestImage();
  });
}

MjpegV4L2Reader::~MjpegV4L2Reader() { epoll_->DeleteFd(fd().get()); }

RockchipV4L2Reader::RockchipV4L2Reader(aos::EventLoop *event_loop,
                                       aos::internal::EPoll *epoll,
                                       std::string_view device_name,
                                       std::string_view image_sensor_subdev,
                                       std::string_view image_channel)
    : V4L2ReaderBase(event_loop, device_name, image_channel),
      epoll_(epoll),
      image_sensor_fd_(open(image_sensor_subdev.data(), O_RDWR | O_NONBLOCK)),
      buffer_requeuer_([this](int buffer) { EnqueueBuffer(buffer); },
                       kEnqueueFifoPriority) {
  PCHECK(image_sensor_fd_.get() != -1)
      << " Failed to open device " << device_name;
  StreamOn();
  epoll_->OnReadable(fd().get(), [this]() { OnImageReady(); });
}

RockchipV4L2Reader::~RockchipV4L2Reader() { epoll_->DeleteFd(fd().get()); }

void RockchipV4L2Reader::MarkBufferToBeEnqueued(int buffer) {
  ReinitializeBuffer(buffer);
  buffer_requeuer_.Push(buffer);
}

void RockchipV4L2Reader::OnImageReady() {
  if (!ReadLatestImage()) {
    return;
  }

  SendLatestImage();
}

int RockchipV4L2Reader::ImageSensorIoctl(unsigned long number, void *arg) {
  return ioctl(image_sensor_fd_.get(), number, arg);
}

void RockchipV4L2Reader::SetExposure(size_t duration) {
  v4l2_control exposure_control;
  exposure_control.id = V4L2_CID_EXPOSURE;
  exposure_control.value = static_cast<int>(duration);
  PCHECK(ImageSensorIoctl(VIDIOC_S_CTRL, &exposure_control) == 0);
}

void RockchipV4L2Reader::SetGain(size_t gain) {
  v4l2_control gain_control;
  gain_control.id = V4L2_CID_GAIN;
  gain_control.value = static_cast<int>(gain);
  PCHECK(ImageSensorIoctl(VIDIOC_S_CTRL, &gain_control) == 0);
}

void RockchipV4L2Reader::SetGainExt(size_t gain) {
  struct v4l2_ext_controls controls;
  memset(&controls, 0, sizeof(controls));
  struct v4l2_ext_control control[1];
  memset(&control, 0, sizeof(control));

  controls.ctrl_class = V4L2_CTRL_CLASS_IMAGE_SOURCE;
  controls.count = 1;
  controls.controls = control;
  control[0].id = V4L2_CID_ANALOGUE_GAIN;
  control[0].value = gain;

  PCHECK(ImageSensorIoctl(VIDIOC_S_EXT_CTRLS, &controls) == 0);
}

void RockchipV4L2Reader::SetVerticalBlanking(size_t vblank) {
  struct v4l2_ext_controls controls;
  memset(&controls, 0, sizeof(controls));
  struct v4l2_ext_control control[1];
  memset(&control, 0, sizeof(control));

  controls.ctrl_class = V4L2_CTRL_CLASS_IMAGE_SOURCE;
  controls.count = 1;
  controls.controls = control;
  control[0].id = V4L2_CID_VBLANK;
  control[0].value = vblank;

  PCHECK(ImageSensorIoctl(VIDIOC_S_EXT_CTRLS, &controls) == 0);
}

}  // namespace frc::vision
