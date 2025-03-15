#include "frc/vision/charuco_lib.h"

#include <chrono>
#include <functional>
#include <string_view>

#include "absl/flags/declare.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "aos/events/event_loop.h"
#include "aos/flatbuffers.h"
#include "aos/network/team_number.h"
#include "frc/control_loops/quaternion_utils.h"
#include "frc/vision/vision_generated.h"

ABSL_FLAG(std::string, board_template_path, "",
          "If specified, write an image to the specified path for the "
          "charuco board pattern.");
ABSL_FLAG(bool, coarse_pattern, true,
          "If true, use coarse arucos; else, use fine");
ABSL_FLAG(uint32_t, gray_threshold, 0,
          "If > 0, threshold image based on this grayscale value");
ABSL_FLAG(bool, twenty_inch_large_board, false,
          "If true, use the large calibration board from "
          "etsy.com/listing/1820746969/charuco-calibration-target");
ABSL_FLAG(bool, large_board, true, "If true, use the large calibration board.");
// Rule of thumb for # of points from paper: https://arxiv.org/pdf/1907.04096
// is to add 5x constraints for each new unknown.
// Each image adds 6 unknowns (3D rotation + translation), so we need to add
// 30 constraints.  Each charuco corner provides 2 constraints (DOF), so we
// need 15 charucos per image.
ABSL_FLAG(
    uint32_t, min_charucos, 15,
    "The mininum number of aruco targets in charuco board required to match.");
ABSL_FLAG(uint32_t, min_id, 0, "Minimum valid charuco id");
ABSL_FLAG(uint32_t, max_diamonds, 0,
          "Maximum number of diamonds to see.  Set to 0 for no limit");
ABSL_FLAG(uint32_t, max_id, 15, "Maximum valid charuco id");
ABSL_FLAG(uint32_t, disable_delay, 100,
          "Time after an issue to disable tracing at.");
ABSL_FLAG(
    bool, draw_axes, false,
    "Whether to draw axes on the resulting data-- warning, may cause crashes.");
ABSL_FLAG(bool, visualize, false, "Whether to visualize the resulting data.");
ABSL_DECLARE_FLAG(bool, enable_ftrace);

ABSL_FLAG(bool, poll_images, false,
          "If true, poll for images instead of watching");

namespace frc::vision {
namespace chrono = std::chrono;
using aos::monotonic_clock;

CameraCalibration::CameraCalibration(
    const calibration::CameraCalibration *calibration)
    : intrinsics_([calibration]() {
        CHECK(calibration != nullptr);
        const cv::Mat result(3, 3, CV_32F,
                             const_cast<void *>(static_cast<const void *>(
                                 calibration->intrinsics()->data())));
        CHECK_EQ(result.total(), calibration->intrinsics()->size());
        return result;
      }()),
      intrinsics_eigen_([this]() {
        cv::Mat camera_intrinsics = intrinsics_;
        Eigen::Matrix3d result;
        cv::cv2eigen(camera_intrinsics, result);
        return result;
      }()),
      // TODO: Need to clean-up any other dist_coeffs hardcoded to 5 parameters
      dist_coeffs_([calibration]() {
        const cv::Mat result(calibration->dist_coeffs()->size(), 1, CV_32F,
                             const_cast<void *>(static_cast<const void *>(
                                 calibration->dist_coeffs()->data())));
        CHECK_EQ(result.total(), calibration->dist_coeffs()->size());
        return result;
      }()) {}

CameraImageCallback::CameraImageCallback(
    aos::EventLoop *event_loop, std::string_view channel,
    std::function<void(const CameraImage &, monotonic_clock::time_point)>
        &&handle_image_fn,
    monotonic_clock::duration max_age)
    : event_loop_(event_loop),
      server_fetcher_(
          event_loop_->MakeFetcher<aos::message_bridge::ServerStatistics>(
              "/aos")),
      source_node_(aos::configuration::GetNode(
          event_loop_->configuration(),
          event_loop_->GetChannel<CameraImage>(channel)
              ->source_node()
              ->string_view())),
      handle_image_(std::move(handle_image_fn)),
      timer_fn_(event_loop->AddTimer([this]() { DisableTracing(); })),
      max_age_(max_age) {
  if (absl::GetFlag(FLAGS_poll_images)) {
    image_fetcher_ = event_loop_->MakeFetcher<CameraImage>(channel);
    auto tmr = event_loop_->AddTimer([this]() {
      if (image_fetcher_.Fetch()) {
        HandleImage(*image_fetcher_.get());
      }
    });

    event_loop_->OnRun([this, tmr]() {
      tmr->Schedule(
          event_loop_->monotonic_now() + std::chrono::milliseconds(100),
          std::chrono::milliseconds(100));
    });

  } else {
    event_loop_->MakeWatcher(
        channel, [this](const CameraImage &image) { HandleImage(image); });
  }
}

void CameraImageCallback::HandleImage(const CameraImage &image) {
  const monotonic_clock::time_point eof_source_node =
      monotonic_clock::time_point(
          chrono::nanoseconds(image.monotonic_timestamp_ns()));
  chrono::nanoseconds offset{0};
  if (source_node_ != event_loop_->node()) {
    server_fetcher_.Fetch();
    if (!server_fetcher_.get()) {
      return;
    }

    // If we are viewing this image from another node, convert to our
    // monotonic clock.
    const aos::message_bridge::ServerConnection *server_connection = nullptr;

    for (const aos::message_bridge::ServerConnection *connection :
         *server_fetcher_->connections()) {
      CHECK(connection->has_node());
      if (connection->node()->name()->string_view() ==
          source_node_->name()->string_view()) {
        server_connection = connection;
        break;
      }
    }

    CHECK(server_connection != nullptr) << ": Failed to find client";
    if (!server_connection->has_monotonic_offset()) {
      VLOG(1) << "No offset yet.";
      return;
    }
    offset = chrono::nanoseconds(server_connection->monotonic_offset());
  }

  const monotonic_clock::time_point eof = eof_source_node - offset;

  const monotonic_clock::duration age = event_loop_->monotonic_now() - eof;
  const double age_double =
      std::chrono::duration_cast<std::chrono::duration<double>>(age).count();
  if (age > max_age_) {
    if (absl::GetFlag(FLAGS_enable_ftrace)) {
      ftrace_.FormatMessage("Too late receiving image, age: %f\n", age_double);
      if (absl::GetFlag(FLAGS_disable_delay) > 0) {
        if (!disabling_) {
          timer_fn_->Schedule(
              event_loop_->monotonic_now() +
              chrono::milliseconds(absl::GetFlag(FLAGS_disable_delay)));
          disabling_ = true;
        }
      } else {
        DisableTracing();
      }
    }
    VLOG(2) << "Age: " << age_double << ", getting behind, skipping";
    return;
  }

  ftrace_.FormatMessage("Starting conversion\n");
  handle_image_(image, eof);
}

void CameraImageCallback::DisableTracing() {
  disabling_ = false;
  ftrace_.TurnOffOrDie();
}

ImageCallback::ImageCallback(
    aos::EventLoop *event_loop, std::string_view channel,
    std::function<void(cv::Mat, monotonic_clock::time_point)> &&handle_image_fn,
    monotonic_clock::duration max_age)
    : handle_image_(std::move(handle_image_fn)),
      camera_image_callback_(
          event_loop, channel,
          [this](const CameraImage &image, monotonic_clock::time_point eof) {
            switch (image.format()) {
              case vision::ImageFormat::MONO8: {
                CHECK(format_ == Format::GRAYSCALE);
                cv::Mat gray_image(cv::Size(image.cols(), image.rows()),
                                   CV_8UC1, (void *)image.data()->data());
                const cv::Size image_size(image.cols(), image.rows());
                handle_image_(gray_image, eof);
              } break;
              case vision::ImageFormat::MONO16:
                LOG(FATAL) << "Unsupported image format MONO16";
                break;
              case vision::ImageFormat::YUYV422: {
                // Create color image:
                cv::Mat image_color_mat(cv::Size(image.cols(), image.rows()),
                                        CV_8UC2, (void *)image.data()->data());
                const cv::Size image_size(image.cols(), image.rows());
                switch (format_) {
                  case Format::GRAYSCALE: {
                    cv::Mat gray_image(image_size, CV_8UC3);
                    cv::cvtColor(image_color_mat, gray_image,
                                 cv::COLOR_YUV2GRAY_YUYV);
                    handle_image_(gray_image, eof);
                  } break;
                  case Format::BGR: {
                    cv::Mat rgb_image(image_size, CV_8UC3);
                    cv::cvtColor(image_color_mat, rgb_image,
                                 cv::COLOR_YUV2BGR_YUYV);
                    handle_image_(rgb_image, eof);
                  } break;
                  case Format::YUYV2: {
                    handle_image_(image_color_mat, eof);
                  };
                }
              } break;
              case vision::ImageFormat::BGR8: {
                CHECK(format_ == Format::BGR);
                cv::Mat bgr_image(cv::Size(image.cols(), image.rows()), CV_8UC3,
                                  (void *)image.data()->data());
                const cv::Size image_size(image.cols(), image.rows());
                handle_image_(bgr_image, eof);
              } break;
              case vision::ImageFormat::BGRA8:
                LOG(FATAL) << "Unsupported image format MONO16";
                return;
              case vision::ImageFormat::MJPEG:
                LOG(FATAL) << "Unsupported image format MJPEG";
                return;
            }
          },
          max_age) {}

cv::Ptr<cv::aruco::CharucoBoard> MakeCharucoBoard(
    cv::Size board_size, float square_length, float marker_length,
    cv::Ptr<cv::aruco::Dictionary> dictionary) {
#if CV_VERSION_MINOR >= 9
  return cv::makePtr<cv::aruco::CharucoBoard>(board_size, square_length,
                                              marker_length, *dictionary);

#else
  return cv::aruco::CharucoBoard::create(board_size.width, board_size.height,
                                         square_length, marker_length,
                                         dictionary);
#endif
}

void CharucoExtractor::SetupTargetData() {
  marker_length_ = 0.146;
  square_length_ = 0.2;

  // Only charuco board has a board associated with it
  board_ = static_cast<cv::Ptr<cv::aruco::CharucoBoard>>(NULL);

  if (target_type_ == TargetType::kCharuco ||
      target_type_ == TargetType::kAruco) {
    dictionary_ =
#if CV_VERSION_MINOR >= 9
        cv::makePtr<cv::aruco::Dictionary>
#endif
        (cv::aruco::getPredefinedDictionary(
            absl::GetFlag(FLAGS_twenty_inch_large_board)
                ? cv::aruco::DICT_4X4_250
                : (absl::GetFlag(FLAGS_large_board)
                       ? cv::aruco::DICT_5X5_250
                       : cv::aruco::DICT_6X6_250)));
    if (target_type_ == TargetType::kCharuco) {
      LOG(INFO) << "Using "
                << (absl::GetFlag(FLAGS_twenty_inch_large_board)
                        ? "20\" large"
                        : (absl::GetFlag(FLAGS_large_board) ? "large"
                                                            : "small"))
                << " charuco board with "
                << (absl::GetFlag(FLAGS_coarse_pattern) ? "coarse" : "fine")
                << " pattern";
      if (absl::GetFlag(FLAGS_twenty_inch_large_board)) {
        board_ = MakeCharucoBoard(cv::Size(15, 15), 0.03, 0.022, dictionary_);
      } else if (absl::GetFlag(FLAGS_large_board)) {
        if (absl::GetFlag(FLAGS_coarse_pattern)) {
          board_ =
              MakeCharucoBoard(cv::Size(12, 9), 0.06, 0.04666, dictionary_);
        } else {
          board_ =
              MakeCharucoBoard(cv::Size(25, 18), 0.03, 0.0233, dictionary_);
        }
      } else {
        if (absl::GetFlag(FLAGS_coarse_pattern)) {
          board_ = MakeCharucoBoard(cv::Size(7, 5), 0.04, 0.025, dictionary_);
        } else {
          // TODO(jim): Need to figure out what
          // size is for small board, fine pattern
          board_ = MakeCharucoBoard(cv::Size(7, 5), 0.03, 0.0233, dictionary_);
        }
      }
      if (!absl::GetFlag(FLAGS_board_template_path).empty()) {
        cv::Mat board_image;
#if CV_VERSION_MINOR >= 9
        board_->generateImage(
#else
        board_->draw(
#endif
            cv::Size(600, 500), board_image, 10, 1);
        cv::imwrite(absl::GetFlag(FLAGS_board_template_path), board_image);
      }
    }
  } else if (target_type_ == TargetType::kCharucoDiamond) {
    marker_length_ = 0.15;
    square_length_ = 0.2;
    dictionary_ = cv::makePtr<cv::aruco::Dictionary>(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250));
  } else if (target_type_ == TargetType::kAprilTag) {
    marker_length_ = 0.1016;
    square_length_ = 0.1524;
    dictionary_ = cv::makePtr<cv::aruco::Dictionary>(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5));
  } else {
    // Bail out if it's not a supported target
    LOG(FATAL) << "Target type undefined: "
               << static_cast<uint8_t>(target_type_);
  }
}

void CharucoExtractor::DrawTargetPoses(cv::Mat rgb_image,
                                       std::vector<cv::Vec3d> rvecs,
                                       std::vector<cv::Vec3d> tvecs) {
  if (!absl::GetFlag(FLAGS_draw_axes)) {
    // Don't draw anything unless draw_axes is set to true
    return;
  }

  const Eigen::Matrix<double, 3, 4> camera_projection =
      Eigen::Matrix<double, 3, 4>::Identity();

  int x_coord = 10;
  int y_coord = 0;
  // draw axis for each marker
  for (uint i = 0; i < rvecs.size(); i++) {
    Eigen::Vector3d rvec_eigen, tvec_eigen;
    cv::cv2eigen(rvecs[i], rvec_eigen);
    cv::cv2eigen(tvecs[i], tvec_eigen);

    Eigen::Quaternion<double> rotation(
        frc::controls::ToQuaternionFromRotationVector(rvec_eigen));
    Eigen::Translation3d translation(tvec_eigen);

    const Eigen::Affine3d board_to_camera = translation * rotation;

    Eigen::Vector3d result = calibration_.CameraIntrinsicsEigen() *
                             camera_projection * board_to_camera *
                             Eigen::Vector3d::Zero();

    // Found that drawAxis hangs if you try to draw with z values too
    // small (trying to draw axes at inifinity)
    // TODO<Jim>: Either track this down or reimplement drawAxes
    if (result.z() < 0.01) {
      LOG(INFO) << "Skipping, due to z value too small: " << result.z();
    } else if (absl::GetFlag(FLAGS_draw_axes) == true) {
      result /= result.z();
      if (target_type_ == TargetType::kCharuco ||
          target_type_ == TargetType::kAprilTag) {
        cv::drawFrameAxes(rgb_image, calibration_.CameraIntrinsics(),
                          calibration_.CameraDistCoeffs(), rvecs[i], tvecs[i],
                          square_length_);
      } else {
        cv::drawFrameAxes(rgb_image, calibration_.CameraIntrinsics(),
                          calibration_.CameraDistCoeffs(), rvecs[i], tvecs[i],
                          square_length_);
      }
    }
    std::stringstream ss;
    ss << "tvec[" << i << "] = " << tvecs[i];
    y_coord += 25;
    cv::putText(rgb_image, ss.str(), cv::Point(x_coord, y_coord),
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
    ss.str("");
    ss << "rvec[" << i << "] = " << rvecs[i];
    y_coord += 25;
    cv::putText(rgb_image, ss.str(), cv::Point(x_coord, y_coord),
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
  }
}

void CharucoExtractor::PackPoseResults(
    std::vector<cv::Vec3d> &rvecs, std::vector<cv::Vec3d> &tvecs,
    std::vector<Eigen::Vector3d> *rvecs_eigen,
    std::vector<Eigen::Vector3d> *tvecs_eigen) {
  for (cv::Vec3d rvec : rvecs) {
    Eigen::Vector3d rvec_eigen = Eigen::Vector3d::Zero();
    cv::cv2eigen(rvec, rvec_eigen);
    rvecs_eigen->emplace_back(rvec_eigen);
  }

  for (cv::Vec3d tvec : tvecs) {
    Eigen::Vector3d tvec_eigen = Eigen::Vector3d::Zero();
    cv::cv2eigen(tvec, tvec_eigen);
    tvecs_eigen->emplace_back(tvec_eigen);
  }
}

CharucoExtractor::CharucoExtractor(
    const calibration::CameraCalibration *calibration,
    const TargetType target_type)
    : event_loop_(nullptr),
      target_type_(target_type),
      calibration_(calibration) {
  VLOG(2) << "Configuring CharucoExtractor without event_loop";
  SetupTargetData();
  VLOG(2) << "Camera matrix:\n" << calibration_.CameraIntrinsics();
  VLOG(2) << "Distortion Coefficients:\n" << calibration_.CameraDistCoeffs();
}

CharucoExtractor::CharucoExtractor(
    aos::EventLoop *event_loop,
    const calibration::CameraCalibration *calibration,
    const TargetType target_type, std::string_view image_channel,
    std::function<void(cv::Mat, monotonic_clock::time_point,
                       std::vector<cv::Vec4i>,
                       std::vector<std::vector<cv::Point2f>>, bool,
                       std::vector<Eigen::Vector3d>,
                       std::vector<Eigen::Vector3d>)> &&handle_charuco_fn)
    : event_loop_(event_loop),
      target_type_(target_type),
      image_channel_(image_channel),
      calibration_(calibration),
      handle_charuco_(std::move(handle_charuco_fn)) {
  SetupTargetData();

  LOG(INFO) << "Camera matrix " << calibration_.CameraIntrinsics();
  LOG(INFO) << "Distortion Coefficients " << calibration_.CameraDistCoeffs();
  LOG(INFO) << "Connecting to channel " << image_channel_;
}

void CharucoExtractor::HandleImage(cv::Mat rgb_image,
                                   const monotonic_clock::time_point eof) {
  // Set up the variables we'll use in the callback function
  bool valid = false;
  // ids and corners for final, refined board / marker detections
  // Using Vec4i type since it supports Charuco Diamonds
  // And overloading it using 1st int in Vec4i for others target types
  std::vector<cv::Vec4i> result_ids;
  std::vector<std::vector<cv::Point2f>> result_corners;

  // Return a list of poses; for Charuco Board there will be just one
  std::vector<Eigen::Vector3d> rvecs_eigen;
  std::vector<Eigen::Vector3d> tvecs_eigen;
  ProcessImage(rgb_image, eof, event_loop_->monotonic_now(), result_ids,
               result_corners, valid, rvecs_eigen, tvecs_eigen);

  handle_charuco_(rgb_image, eof, result_ids, result_corners, valid,
                  rvecs_eigen, tvecs_eigen);
}

void CharucoExtractor::ProcessImage(
    cv::Mat rgb_image, const monotonic_clock::time_point eof,
    const monotonic_clock::time_point current_time,
    std::vector<cv::Vec4i> &result_ids,
    std::vector<std::vector<cv::Point2f>> &result_corners, bool &valid,
    std::vector<Eigen::Vector3d> &rvecs_eigen,
    std::vector<Eigen::Vector3d> &tvecs_eigen) {
  const double age_double =
      std::chrono::duration_cast<std::chrono::duration<double>>(current_time -
                                                                eof)
          .count();

  // Have found this useful if there is blurry / noisy images
  if (absl::GetFlag(FLAGS_gray_threshold) > 0) {
    cv::Mat gray;
    cv::cvtColor(rgb_image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat thresh;
    cv::threshold(gray, thresh, absl::GetFlag(FLAGS_gray_threshold), 255,
                  cv::THRESH_BINARY);
    cv::cvtColor(thresh, rgb_image, cv::COLOR_GRAY2RGB);
  }

  // ids and corners for initial aruco marker detections
  std::vector<int> marker_ids;
  std::vector<std::vector<cv::Point2f>> marker_corners;

  // Do initial marker detection; this is the same for all target types
  cv::Ptr<cv::aruco::DetectorParameters> detector_params =
      cv::makePtr<cv::aruco::DetectorParameters>();
  detector_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;

  cv::aruco::detectMarkers(rgb_image, dictionary_, marker_corners, marker_ids,
                           detector_params);
  if (absl::GetFlag(FLAGS_draw_axes)) {
    cv::aruco::drawDetectedMarkers(rgb_image, marker_corners, marker_ids);
  }

  VLOG(2) << "Handle Image, with target type = "
          << static_cast<uint8_t>(target_type_) << " and " << marker_ids.size()
          << " markers detected initially";

  if (marker_ids.size() == 0) {
    VLOG(2) << "Didn't find any markers";
  } else {
    if (target_type_ == TargetType::kCharuco) {
      std::vector<int> charuco_ids;
      std::vector<cv::Point2f> charuco_corners;

      // If enough aruco markers detected for the Charuco board
      if (marker_ids.size() >= absl::GetFlag(FLAGS_min_charucos)) {
        // Run everything twice, once with the calibration, and once
        // without. This lets us both collect data to calibrate the
        // intrinsics of the camera (to determine the intrinsics from
        // multiple samples), and also to use data from a previous/stored
        // calibration to determine a more accurate pose in real time (used
        // for extrinsics calibration)
        cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids,
                                             rgb_image, board_, charuco_corners,
                                             charuco_ids);

        std::vector<cv::Point2f> charuco_corners_with_calibration;
        std::vector<int> charuco_ids_with_calibration;

        // This call uses a previous intrinsic calibration to get more
        // accurate marker locations, for a better pose estimate
        cv::aruco::interpolateCornersCharuco(
            marker_corners, marker_ids, rgb_image, board_,
            charuco_corners_with_calibration, charuco_ids_with_calibration,
            calibration_.CameraIntrinsics(), calibration_.CameraDistCoeffs());

        if (charuco_ids.size() >= absl::GetFlag(FLAGS_min_charucos)) {
          if (absl::GetFlag(FLAGS_draw_axes)) {
            cv::aruco::drawDetectedCornersCharuco(
                rgb_image, charuco_corners, charuco_ids, cv::Scalar(255, 0, 0));
          }

          cv::Vec3d rvec, tvec;
          valid = cv::aruco::estimatePoseCharucoBoard(
              charuco_corners_with_calibration, charuco_ids_with_calibration,
              board_, calibration_.CameraIntrinsics(),
              calibration_.CameraDistCoeffs(), rvec, tvec);

          // if charuco pose is valid, return pose, with ids and corners
          if (valid) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            rvecs.emplace_back(rvec);
            tvecs.emplace_back(tvec);
            DrawTargetPoses(rgb_image, rvecs, tvecs);

            PackPoseResults(rvecs, tvecs, &rvecs_eigen, &tvecs_eigen);
            // Store the corners without calibration, since we use them to
            // do calibration
            result_corners.emplace_back(charuco_corners);
            for (auto id : charuco_ids) {
              result_ids.emplace_back(cv::Vec4i{id, 0, 0, 0});
            }
          } else {
            VLOG(2) << "Age: " << age_double << ", invalid charuco board pose";
          }
        } else {
          VLOG(2) << "Age: " << age_double << ", not enough charuco IDs, got "
                  << charuco_ids.size() << ", needed "
                  << absl::GetFlag(FLAGS_min_charucos);
        }
      } else {
        VLOG(2) << "Age: " << age_double
                << ", not enough marker IDs for charuco board, got "
                << marker_ids.size() << ", needed "
                << absl::GetFlag(FLAGS_min_charucos);
      }
    } else if (target_type_ == TargetType::kAruco ||
               target_type_ == TargetType::kAprilTag) {
      // estimate pose for arucos doesn't return valid, so marking true
      valid = true;
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(
          marker_corners, square_length_, calibration_.CameraIntrinsics(),
          calibration_.CameraDistCoeffs(), rvecs, tvecs);
      DrawTargetPoses(rgb_image, rvecs, tvecs);
      PackPoseResults(rvecs, tvecs, &rvecs_eigen, &tvecs_eigen);

      for (uint i = 0; i < marker_ids.size(); i++) {
        result_ids.emplace_back(cv::Vec4i{marker_ids[i], 0, 0, 0});
      }
      result_corners = marker_corners;
    } else if (target_type_ == TargetType::kCharucoDiamond) {
      // Extract the diamonds associated with the markers
      std::vector<cv::Vec4i> diamond_ids;
      std::vector<std::vector<cv::Point2f>> diamond_corners;
      cv::aruco::detectCharucoDiamond(rgb_image, marker_corners, marker_ids,
                                      square_length_ / marker_length_,
                                      diamond_corners, diamond_ids);

      // Check that we have an acceptable number of diamonds detected.
      // Should be at least one, and no more than FLAGS_max_diamonds.
      // Different calibration routines will require different values for this
      if (diamond_ids.size() > 0 &&
          (absl::GetFlag(FLAGS_max_diamonds) == 0 ||
           diamond_ids.size() <= absl::GetFlag(FLAGS_max_diamonds))) {
        // TODO<Jim>: Could probably make this check more general than
        // requiring range of ids
        bool all_valid_ids = true;
        for (uint i = 0; i < 4; i++) {
          uint id = diamond_ids[0][i];
          if ((id < absl::GetFlag(FLAGS_min_id)) ||
              (id > absl::GetFlag(FLAGS_max_id))) {
            all_valid_ids = false;
            LOG(INFO) << "Got invalid charuco id: " << id;
          }
        }
        if (all_valid_ids) {
          if (absl::GetFlag(FLAGS_draw_axes)) {
            cv::aruco::drawDetectedDiamonds(rgb_image, diamond_corners,
                                            diamond_ids);
          }
          // estimate pose for diamonds doesn't return valid, so marking
          // true
          valid = true;
          std::vector<cv::Vec3d> rvecs, tvecs;
          cv::aruco::estimatePoseSingleMarkers(
              diamond_corners, square_length_, calibration_.CameraIntrinsics(),
              calibration_.CameraDistCoeffs(), rvecs, tvecs);

          DrawTargetPoses(rgb_image, rvecs, tvecs);
          PackPoseResults(rvecs, tvecs, &rvecs_eigen, &tvecs_eigen);
          result_ids = diamond_ids;
          result_corners = diamond_corners;
        } else {
          LOG(INFO) << "Not all charuco ids were valid, so skipping";
        }
      } else {
        if (diamond_ids.size() == 0) {
          // OK to not see any markers sometimes
          VLOG(2) << "Found aruco markers, but no valid charuco diamond "
                     "targets";
        } else {
          VLOG(2) << "Found too many number of diamond markers, which likely "
                     "means false positives were detected: "
                  << diamond_ids.size() << " > "
                  << absl::GetFlag(FLAGS_max_diamonds);
        }
      }
    } else {
      LOG(FATAL) << "Unknown target type: "
                 << static_cast<uint8_t>(target_type_);
    }
  }
}

flatbuffers::Offset<foxglove::ImageAnnotations> BuildAnnotations(
    flatbuffers::FlatBufferBuilder *fbb,
    const aos::monotonic_clock::time_point monotonic_now,
    const std::vector<std::vector<cv::Point2f>> &corners,
    const std::vector<double> rgba_color, const double thickness,
    const foxglove::PointsAnnotationType line_type) {
  std::vector<flatbuffers::Offset<foxglove::PointsAnnotation>> rectangles;
  for (const std::vector<cv::Point2f> &rectangle : corners) {
    rectangles.push_back(BuildPointsAnnotation(
        fbb, monotonic_now, rectangle, rgba_color, thickness, line_type));
  }

  const auto rectangles_offset = fbb->CreateVector(rectangles);
  foxglove::ImageAnnotations::Builder annotation_builder(*fbb);
  annotation_builder.add_points(rectangles_offset);
  return annotation_builder.Finish();
}

flatbuffers::Offset<foxglove::PointsAnnotation> BuildPointsAnnotation(
    flatbuffers::FlatBufferBuilder *fbb,
    const aos::monotonic_clock::time_point monotonic_now,
    const std::vector<cv::Point2f> &corners,
    const std::vector<double> rgba_color, const double thickness,
    const foxglove::PointsAnnotationType line_type) {
  const struct timespec now_t = aos::time::to_timespec(monotonic_now);
  foxglove::Time time{static_cast<uint32_t>(now_t.tv_sec),
                      static_cast<uint32_t>(now_t.tv_nsec)};
  const flatbuffers::Offset<foxglove::Color> color_offset =
      foxglove::CreateColor(*fbb, rgba_color[0], rgba_color[1], rgba_color[2],
                            rgba_color[3]);
  std::vector<flatbuffers::Offset<foxglove::Point2>> points_offsets;
  for (const cv::Point2f &point : corners) {
    points_offsets.push_back(foxglove::CreatePoint2(*fbb, point.x, point.y));
  }
  const flatbuffers::Offset<
      flatbuffers::Vector<flatbuffers::Offset<foxglove::Point2>>>
      points_offset = fbb->CreateVector(points_offsets);
  std::vector<flatbuffers::Offset<foxglove::Color>> color_offsets(
      points_offsets.size(), color_offset);
  auto colors_offset = fbb->CreateVector(color_offsets);
  foxglove::PointsAnnotation::Builder points_builder(*fbb);
  points_builder.add_timestamp(&time);
  points_builder.add_type(line_type);
  points_builder.add_points(points_offset);
  points_builder.add_outline_color(color_offset);
  points_builder.add_outline_colors(colors_offset);
  points_builder.add_thickness(thickness);

  return points_builder.Finish();
}

TargetType TargetTypeFromString(std::string_view str) {
  if (str == "aruco") {
    return TargetType::kAruco;
  } else if (str == "charuco") {
    return TargetType::kCharuco;
  } else if (str == "charuco_diamond") {
    return TargetType::kCharucoDiamond;
  } else if (str == "apriltag") {
    return TargetType::kAprilTag;
  } else {
    LOG(FATAL) << "Unknown target type: " << str
               << ", expected: apriltag|aruco|charuco|charuco_diamond";
  }
}

std::ostream &operator<<(std::ostream &os, TargetType target_type) {
  switch (target_type) {
    case TargetType::kAruco:
      os << "aruco";
      break;
    case TargetType::kCharuco:
      os << "charuco";
      break;
    case TargetType::kCharucoDiamond:
      os << "charuco_diamond";
      break;
    case TargetType::kAprilTag:
      os << "apriltag";
      break;
  }
  return os;
}

}  // namespace frc::vision
