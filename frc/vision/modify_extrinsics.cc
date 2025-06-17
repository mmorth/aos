#include <cmath>
#include <filesystem>
#include <regex>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_format.h"

#include "aos/configuration.h"
#include "aos/events/event_loop.h"
#include "aos/flatbuffer_merge.h"
#include "aos/init.h"
#include "aos/network/team_number.h"
#include "aos/time/time.h"
#include "aos/util/file.h"
#include "frc/vision/calibration_generated.h"
#include "frc/vision/vision_util_lib.h"

// This is a helper program to build and rename calibration files
// You can:
// (1) pass it in a new set of orin #, team #, camera #, to rename the file
// (2) Pass in extrinsics to set the extrinsics
// By default, writes to /tmp, but will write to calib_files folder if
// full path is given and calibration_folder is blank

ABSL_FLAG(std::string, orig_calib_file, "",
          "Intrinsics to use for estimating board pose prior to solving "
          "for the new intrinsics.");
ABSL_FLAG(std::string, calibration_folder, "/tmp",
          "Folder to place calibration files.");
ABSL_FLAG(std::string, node_name, "",
          "Node name to use, e.g. orin1, imu; unchanged if blank");
ABSL_FLAG(int32_t, team_number, -1, "Team number to use; unchanged if -1");
ABSL_FLAG(int32_t, camera_number, -1, "Camera number to use; unchanged if -1");

ABSL_FLAG(bool, set_extrinsics, true, "Set to false to ignore extrinsic data");
ABSL_FLAG(bool, use_inches, true,
          "Whether to use inches as units (meters if false)");
ABSL_FLAG(bool, use_degrees, true,
          "Whether to use degrees as units (radians if false)");
ABSL_FLAG(double, camera_x, 0.0, "x location of camera");
ABSL_FLAG(double, camera_y, 0.0, "y location of camera");
ABSL_FLAG(double, camera_z, 0.0, "z location of camera");
// Don't currently allow for roll of cameras
ABSL_FLAG(double, camera_yaw, 0.0, "yaw of camera about robot z axis");
ABSL_FLAG(double, camera_pitch, 0.0,
          "pitch of camera relative to robot y axis");
// TODO: This could be done by setting the pixel size and using the intrinsics
ABSL_FLAG(double, focal_length, 0.002, "Focal length in meters");

namespace frc::vision {
namespace {

// TODO: Put this in vision_util_lib?  Except, it depends on Eigen
std::vector<float> MatrixToVector(const Eigen::Matrix<double, 4, 4> &H) {
  std::vector<float> data;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      data.push_back(H(row, col));
    }
  }
  return data;
}

// Merge the original calibration file with all its changes
aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> BuildCalibration(
    const frc::vision::calibration::CameraCalibration *calibration,
    std::string node_name, uint16_t camera_number, uint16_t team_number) {
  aos::FlatbufferDetachedBuffer<frc::vision::calibration::CameraCalibration>
      cal_copy = aos::RecursiveCopyFlatBuffer(calibration);

  flatbuffers::FlatBufferBuilder fbb;
  flatbuffers::Offset<flatbuffers::String> node_name_offset =
      fbb.CreateString(absl::StrFormat("%s", node_name.c_str()));

  // If we're told to set the extrinsics, clear old and add in new
  flatbuffers::Offset<calibration::TransformationMatrix>
      fixed_extrinsics_offset;
  if (absl::GetFlag(FLAGS_set_extrinsics)) {
    cal_copy.mutable_message()->clear_fixed_extrinsics();
    Eigen::Affine3d extrinsic_matrix;
    // Convert to metric
    double translation_scale = (absl::GetFlag(FLAGS_use_inches) ? 0.0254 : 1.0);
    Eigen::Translation3d translation(
        absl::GetFlag(FLAGS_camera_x) * translation_scale,
        absl::GetFlag(FLAGS_camera_y) * translation_scale,
        absl::GetFlag(FLAGS_camera_z) * translation_scale);

    // convert to radians
    double angle_scale =
        (absl::GetFlag(FLAGS_use_degrees) ? M_PI / 180.0 : 1.0);
    // The rotation that takes robot coordinates (x forward, z up) to camera
    // coordiantes (z forward, x right)
    Eigen::Quaterniond R_robo_cam =
        Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(
        absl::GetFlag(FLAGS_camera_pitch) * angle_scale,
        Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(absl::GetFlag(FLAGS_camera_yaw) * angle_scale,
                               Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond rotation = yawAngle * pitchAngle * R_robo_cam;
    Eigen::Vector3d focal_length_offset =
        (rotation *
         Eigen::Translation3d(0.0, 0.0, absl::GetFlag(FLAGS_focal_length)))
            .translation();
    translation = translation * Eigen::Translation3d(focal_length_offset);

    extrinsic_matrix = translation * rotation;
    flatbuffers::Offset<flatbuffers::Vector<float>> data_offset =
        fbb.CreateVector(
            frc::vision::MatrixToVector(extrinsic_matrix.matrix()));
    calibration::TransformationMatrix::Builder matrix_builder(fbb);
    matrix_builder.add_data(data_offset);
    fixed_extrinsics_offset = matrix_builder.Finish();
  }

  calibration::CameraCalibration::Builder camera_calibration_builder(fbb);
  camera_calibration_builder.add_node_name(node_name_offset);
  camera_calibration_builder.add_team_number(team_number);
  if (absl::GetFlag(FLAGS_set_extrinsics)) {
    camera_calibration_builder.add_fixed_extrinsics(fixed_extrinsics_offset);
  }
  camera_calibration_builder.add_camera_number(camera_number);
  fbb.Finish(camera_calibration_builder.Finish());

  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration> updated_cal =
      fbb.Release();

  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
      merged_calibration =
          aos::MergeFlatBuffers(&cal_copy.message(), &updated_cal.message());
  return merged_calibration;
}

void Main(std::string orig_calib_filename) {
  LOG(INFO) << "Reading from file: " << orig_calib_filename;
  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
      base_calibration =
          aos::JsonFileToFlatbuffer<calibration::CameraCalibration>(
              orig_calib_filename);

  // Populate the new variables from command-line or from base_calibration
  std::string node_name = (absl::GetFlag(FLAGS_node_name) == ""
                               ? base_calibration.message().node_name()->str()
                               : absl::GetFlag(FLAGS_node_name));
  int team_number = (absl::GetFlag(FLAGS_team_number) == -1
                         ? base_calibration.message().team_number()
                         : absl::GetFlag(FLAGS_team_number));
  int camera_number = (absl::GetFlag(FLAGS_camera_number) == -1
                           ? base_calibration.message().camera_number()
                           : absl::GetFlag(FLAGS_camera_number));
  aos::FlatbufferDetachedBuffer<calibration::CameraCalibration>
      new_calibration = BuildCalibration(&base_calibration.message(), node_name,
                                         camera_number, team_number);

  // Set a new timestamp on the file, but leave calibration_timestamp unchanged
  const aos::realtime_clock::time_point realtime_now =
      aos::realtime_clock::now();
  std::stringstream time_ss;
  time_ss << realtime_now;
  // Use the camera_id that is set in the json file (not the filename)
  std::string camera_id = base_calibration.message().camera_id()->str();

  const std::string dirname =
      (absl::GetFlag(FLAGS_calibration_folder) == ""
           ? std::filesystem::path(orig_calib_filename).parent_path().string()
           : absl::GetFlag(FLAGS_calibration_folder));
  const std::string new_calib_filename = frc::vision::CalibrationFilename(
      dirname, node_name.c_str(), team_number, camera_number, camera_id.c_str(),
      time_ss.str());

  VLOG(1) << "From: " << orig_calib_filename << " -> "
          << aos::FlatbufferToJson(base_calibration, {.multi_line = true});

  VLOG(1) << "Writing: " << new_calib_filename << " -> "
          << aos::FlatbufferToJson(new_calibration, {.multi_line = true});

  LOG(INFO) << "Writing to file: " << new_calib_filename;
  aos::util::WriteStringToFileOrDie(
      new_calib_filename,
      aos::FlatbufferToJson(new_calibration, {.multi_line = true}));
}

}  // namespace
}  // namespace frc::vision

int main(int argc, char **argv) {
  aos::InitGoogle(&argc, &argv);
  CHECK(argc == 2) << "Must supply a starting calibration filename";
  std::string filename = argv[1];
  frc::vision::Main(filename);
}
