#include "frc/vision/foxglove_image_converter_lib.h"

#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

ABSL_FLAG(int32_t, jpeg_quality, 60,
          "Compression quality of JPEGs, 0-100; lower numbers mean lower "
          "quality and resulting image sizes.");
ABSL_FLAG(uint32_t, max_period_ms, 100,
          "Fastest period at which to convert images, to limit CPU usage.");

namespace frc::vision {
std::string_view ExtensionForCompression(ImageCompression compression) {
  switch (compression) {
    case ImageCompression::kJpeg:
      return "jpeg";
    case ImageCompression::kPng:
      return "png";
  }
}

flatbuffers::Offset<foxglove::CompressedImage> CompressImage(
    const cv::Mat image, const aos::monotonic_clock::time_point eof,
    flatbuffers::FlatBufferBuilder *fbb, ImageCompression compression) {
  std::string_view format = ExtensionForCompression(compression);
  // imencode doesn't let us pass in anything other than an std::vector, and
  // performance isn't yet a big enough issue to try to avoid the copy.
  std::vector<uint8_t> buffer;
  CHECK(cv::imencode(
      absl::StrCat(".", format), image, buffer,
      {cv::IMWRITE_JPEG_QUALITY, absl::GetFlag(FLAGS_jpeg_quality)}));
  const flatbuffers::Offset<flatbuffers::Vector<uint8_t>> data_offset =
      fbb->CreateVector(buffer);
  const struct timespec timestamp_t = aos::time::to_timespec(eof);
  const foxglove::Time time{static_cast<uint32_t>(timestamp_t.tv_sec),
                            static_cast<uint32_t>(timestamp_t.tv_nsec)};
  const flatbuffers::Offset<flatbuffers::String> format_offset =
      fbb->CreateString(format);
  foxglove::CompressedImage::Builder builder(*fbb);
  builder.add_timestamp(&time);
  builder.add_data(data_offset);
  builder.add_format(format_offset);
  return builder.Finish();
}

FoxgloveImageConverter::FoxgloveImageConverter(aos::EventLoop *event_loop,
                                               std::string_view input_channel,
                                               std::string_view output_channel,
                                               ImageCompression compression)
    : event_loop_(event_loop),
      image_callback_(
          event_loop_, input_channel,
          [this, compression](const cv::Mat image,
                              const aos::monotonic_clock::time_point eof) {
            if (event_loop_->monotonic_now() >
                (std::chrono::milliseconds(absl::GetFlag(FLAGS_max_period_ms)) +
                 sender_.monotonic_sent_time())) {
              auto builder = sender_.MakeBuilder();
              builder.CheckOk(builder.Send(
                  CompressImage(image, eof, builder.fbb(), compression)));
            }
          }),
      sender_(
          event_loop_->MakeSender<foxglove::CompressedImage>(output_channel)) {}
}  // namespace frc::vision
