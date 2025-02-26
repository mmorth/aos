#include "absl/flags/flag.h"

#include "aos/events/shm_event_loop.h"
#include "aos/init.h"
#include "frc/constants/constants_sender_lib.h"
#include "frc/vision/camera_constants_generated.h"
#include "frc/vision/v4l2_reader.h"

ABSL_FLAG(std::string, config, "aos_config.json",
          "Path to the config file to use.");
ABSL_FLAG(std::string, channel, "/camera", "What camera channel to use.");
ABSL_FLAG(std::string, viddevice, "/dev/video0", "What video device to use.");
ABSL_FLAG(int32_t, exposure, -1,
          "Exposure time, in 100us increments; 0 implies auto exposure; -1 "
          "defers to the constants file.");

namespace frc::vision {

void CameraReaderMain() {
  aos::FlatbufferDetachedBuffer<aos::Configuration> config =
      aos::configuration::ReadConfig(absl::GetFlag(FLAGS_config));

  frc::constants::WaitForConstants<CameraConstants>(&config.message());

  aos::ShmEventLoop event_loop(&config.message());

  const frc::constants::ConstantsFetcher<CameraConstants> calibration_data(
      &event_loop);

  CHECK(calibration_data.constants().has_default_camera_stream_settings())
      << ": Must provide camera stream settings for camera initialization.";

  const CameraStreamSettings *const stream_settings =
      calibration_data.constants().default_camera_stream_settings();

  MjpegV4L2Reader v4l2_reader(&event_loop, event_loop.epoll(),
                              absl::GetFlag(FLAGS_viddevice),
                              absl::GetFlag(FLAGS_channel), stream_settings);
  // If the exposure flag is set to override the constants, use it.
  const int32_t exposure_flag = absl::GetFlag(FLAGS_exposure);
  if (exposure_flag >= 0) {
    if (exposure_flag > 0) {
      LOG(INFO) << "Setting camera to Manual Exposure mode with exposure = "
                << exposure_flag << " or "
                << static_cast<double>(exposure_flag) / 10.0 << " ms";
      v4l2_reader.SetExposure(exposure_flag);
    } else {
      LOG(INFO) << "Setting camera to use Auto Exposure";
      v4l2_reader.UseAutoExposure();
    }
  }

  event_loop.Run();
}

}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  frc::vision::CameraReaderMain();
}
