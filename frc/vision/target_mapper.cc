#include "frc/vision/target_mapper.h"

#include "absl/strings/str_format.h"

#include "frc/control_loops/control_loop.h"
#include "frc/vision/ceres/pose_graph_3d_error_term.h"
#include "frc/vision/geometry.h"
#include "frc/vision/vision_util_lib.h"

ABSL_FLAG(uint64_t, max_num_iterations, 100,
          "Maximum number of iterations for the ceres solver");
ABSL_FLAG(double, distortion_noise_scalar, 1.0,
          "Scale the target pose distortion factor by this when computing "
          "the noise.");
ABSL_FLAG(
    int32_t, frozen_target_id, 1,
    "Freeze the pose of this target so the map can have one fixed point.");
ABSL_FLAG(int32_t, min_target_id, 1, "Minimum target id to solve for");
ABSL_FLAG(int32_t, max_target_id, 30, "Maximum target id to solve for");
ABSL_FLAG(bool, visualize_solver, false,
          "If true, visualize the solving process.");
// This does seem like a strict threshold for a constaint to be considered an
// outlier, but most inliers were very close together and this is what worked
// best for map solving.
ABSL_FLAG(double, outlier_std_devs, 1.0,
          "Number of standard deviations above average error needed for a "
          "constraint to be considered an outlier and get removed.");
ABSL_FLAG(bool, do_map_fitting, false,
          "Whether to do a final fit of the solved map to the original map");

namespace frc::vision {
ceres::examples::VectorOfConstraints DataAdapter::MatchTargetDetections(
    const std::vector<DataAdapter::TimestampedDetection>
        &timestamped_target_detections,
    aos::distributed_clock::duration max_dt) {
  CHECK_GE(timestamped_target_detections.size(), 2ul)
      << "Must have at least 2 detections";

  // Match consecutive detections
  ceres::examples::VectorOfConstraints target_constraints;
  for (auto detection = timestamped_target_detections.begin() + 1;
       detection < timestamped_target_detections.end(); detection++) {
    for (int past = 1;
         past <=
         std::min<int>(4, detection - timestamped_target_detections.begin());
         ++past) {
      auto last_detection = detection - past;

      // Skip two consecutive detections of the same target, because the solver
      // doesn't allow this
      if (detection->id == last_detection->id) {
        continue;
      }

      // Don't take into account constraints too far apart in time, because the
      // recording device could have moved too much
      if ((detection->time - last_detection->time) > max_dt) {
        continue;
      }

      auto confidence = ComputeConfidence(*last_detection, *detection);
      target_constraints.emplace_back(
          ComputeTargetConstraint(*last_detection, *detection, confidence));
    }
  }

  return target_constraints;
}

TargetMapper::ConfidenceMatrix DataAdapter::ComputeConfidence(
    const TimestampedDetection &detection_start,
    const TimestampedDetection &detection_end) {
  constexpr size_t kX = 0;
  constexpr size_t kY = 1;
  constexpr size_t kZ = 2;
  constexpr size_t kOrientation1 = 3;
  constexpr size_t kOrientation2 = 4;
  constexpr size_t kOrientation3 = 5;

  // Uncertainty matrix between start and end
  TargetMapper::ConfidenceMatrix P = TargetMapper::ConfidenceMatrix::Zero();

  {
    // Noise for odometry-based robot position measurements
    TargetMapper::ConfidenceMatrix Q_odometry =
        TargetMapper::ConfidenceMatrix::Zero();
    Q_odometry(kX, kX) = std::pow(0.045, 2);
    Q_odometry(kY, kY) = std::pow(0.045, 2);
    Q_odometry(kZ, kZ) = std::pow(0.045, 2);
    Q_odometry(kOrientation1, kOrientation1) = std::pow(0.01, 2);
    Q_odometry(kOrientation2, kOrientation2) = std::pow(0.01, 2);
    Q_odometry(kOrientation3, kOrientation3) = std::pow(0.01, 2);

    // Add uncertainty for robot position measurements from start to end
    int iterations = (detection_end.time - detection_start.time) /
                     frc::controls::kLoopFrequency;
    P += static_cast<double>(iterations) * Q_odometry;
  }

  {
    // Noise for vision-based target localizations. Multiplying this matrix by
    // the distance from camera to target squared results in the uncertainty
    // in that measurement
    TargetMapper::ConfidenceMatrix Q_vision =
        TargetMapper::ConfidenceMatrix::Zero();
    Q_vision(kX, kX) = std::pow(0.045, 2);
    Q_vision(kY, kY) = std::pow(0.045, 2);
    Q_vision(kZ, kZ) = std::pow(0.045, 2);
    Q_vision(kOrientation1, kOrientation1) = std::pow(0.02, 2);
    Q_vision(kOrientation2, kOrientation2) = std::pow(0.02, 2);
    Q_vision(kOrientation3, kOrientation3) = std::pow(0.02, 2);

    // Add uncertainty for the 2 vision measurements (1 at start and 1 at end)
    P += Q_vision *
         std::pow(detection_start.distance_from_camera *
                      (1.0 + absl::GetFlag(FLAGS_distortion_noise_scalar) *
                                 detection_start.distortion_factor),
                  2);
    P += Q_vision *
         std::pow(detection_end.distance_from_camera *
                      (1.0 + absl::GetFlag(FLAGS_distortion_noise_scalar) *
                                 detection_end.distortion_factor),
                  2);
  }

  return P.inverse();
}

ceres::examples::Constraint3d DataAdapter::ComputeTargetConstraint(
    const TimestampedDetection &target_detection_start,
    const TimestampedDetection &target_detection_end,
    const TargetMapper::ConfidenceMatrix &confidence) {
  // Compute the relative pose (constraint) between the two targets
  Eigen::Affine3d H_targetstart_targetend =
      target_detection_start.H_robot_target.inverse() *
      target_detection_end.H_robot_target;
  ceres::examples::Pose3d target_constraint =
      PoseUtils::Affine3dToPose3d(H_targetstart_targetend);

  const auto constraint_3d =
      ceres::examples::Constraint3d{target_detection_start.id,
                                    target_detection_end.id,
                                    {target_constraint.p, target_constraint.q},
                                    confidence};

  VLOG(2) << "Computed constraint: " << constraint_3d;
  return constraint_3d;
}

TargetMapper::TargetMapper(
    std::string_view target_poses_path,
    const ceres::examples::VectorOfConstraints &target_constraints)
    : target_constraints_(target_constraints),
      T_frozen_actual_(Eigen::Vector3d::Zero()),
      R_frozen_actual_(Eigen::Quaterniond::Identity()),
      vis_robot_(cv::Size(kImageWidth_, kImageHeight_)) {
  // Compute focal length so that image shows field with viewpoint at 10m above
  // it (default for viewer)
  const double focal_length = kImageWidth_ * 10.0 / kFieldWidth_;
  vis_robot_.SetDefaultViewpoint(kImageWidth_, focal_length);

  aos::FlatbufferDetachedBuffer<TargetMap> target_map =
      aos::JsonFileToFlatbuffer<TargetMap>(target_poses_path);
  for (const auto *target_pose_fbs : *target_map.message().target_poses()) {
    ideal_target_poses_[target_pose_fbs->id()] =
        TargetPoseFromFbs(*target_pose_fbs).pose;
  }
  target_poses_ = ideal_target_poses_;
  CountConstraints();
}

TargetMapper::TargetMapper(
    const ceres::examples::MapOfPoses &target_poses,
    const ceres::examples::VectorOfConstraints &target_constraints)
    : ideal_target_poses_(target_poses),
      target_poses_(ideal_target_poses_),
      target_constraints_(target_constraints),
      T_frozen_actual_(Eigen::Vector3d::Zero()),
      R_frozen_actual_(Eigen::Quaterniond::Identity()),
      vis_robot_(cv::Size(kImageWidth_, kImageHeight_)) {
  CountConstraints();
}

flatbuffers::Offset<TargetPoseFbs> TargetMapper::TargetPoseToFbs(
    const TargetMapper::TargetPose &target_pose,
    flatbuffers::FlatBufferBuilder *fbb) {
  const auto position_offset =
      CreatePosition(*fbb, target_pose.pose.p(0), target_pose.pose.p(1),
                     target_pose.pose.p(2));
  const auto orientation_offset =
      CreateQuaternion(*fbb, target_pose.pose.q.w(), target_pose.pose.q.x(),
                       target_pose.pose.q.y(), target_pose.pose.q.z());
  return CreateTargetPoseFbs(*fbb, target_pose.id, position_offset,
                             orientation_offset);
}

TargetMapper::TargetPose TargetMapper::TargetPoseFromFbs(
    const TargetPoseFbs &target_pose_fbs) {
  return {.id = static_cast<TargetMapper::TargetId>(target_pose_fbs.id()),
          .pose = ceres::examples::Pose3d{
              Eigen::Vector3d(target_pose_fbs.position()->x(),
                              target_pose_fbs.position()->y(),
                              target_pose_fbs.position()->z()),
              Eigen::Quaterniond(target_pose_fbs.orientation()->w(),
                                 target_pose_fbs.orientation()->x(),
                                 target_pose_fbs.orientation()->y(),
                                 target_pose_fbs.orientation()->z())
                  .normalized()}};
}

namespace {
std::pair<TargetMapper::TargetId, TargetMapper::TargetId> MakeIdPair(
    const ceres::examples::Constraint3d &constraint) {
  auto min_id = std::min(constraint.id_begin, constraint.id_end);
  auto max_id = std::max(constraint.id_begin, constraint.id_end);
  return std::make_pair(min_id, max_id);
}
}  // namespace

void TargetMapper::CountConstraints() {
  for (const auto &constraint : target_constraints_) {
    auto id_pair = MakeIdPair(constraint);
    if (constraint_counts_.count(id_pair) == 0) {
      constraint_counts_[id_pair] = 0;
    }
    constraint_counts_[id_pair]++;
  }
}

std::optional<TargetMapper::TargetPose> TargetMapper::GetTargetPoseById(
    std::vector<TargetMapper::TargetPose> target_poses, TargetId target_id) {
  for (auto target_pose : target_poses) {
    if (target_pose.id == target_id) {
      return target_pose;
    }
  }

  return std::nullopt;
}

std::optional<TargetMapper::TargetPose> TargetMapper::GetTargetPoseById(
    TargetId target_id) const {
  if (target_poses_.count(target_id) > 0) {
    return TargetMapper::TargetPose{target_id, target_poses_.at(target_id)};
  }

  return std::nullopt;
}

// Taken from ceres/examples/slam/pose_graph_3d/pose_graph_3d.cc
// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void TargetMapper::BuildTargetPoseOptimizationProblem(
    const ceres::examples::VectorOfConstraints &constraints,
    ceres::examples::MapOfPoses *poses, ceres::Problem *problem) {
  CHECK(poses != nullptr);
  CHECK(problem != nullptr);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  ceres::LossFunction *loss_function = new ceres::HuberLoss(2.0);
  ceres::Manifold *quaternion_local_parameterization =
      new ceres::EigenQuaternionManifold();

  int min_constraint_id = std::numeric_limits<int>::max();
  int max_constraint_id = std::numeric_limits<int>::min();

  for (ceres::examples::VectorOfConstraints::const_iterator constraints_iter =
           constraints.begin();
       constraints_iter != constraints.end(); ++constraints_iter) {
    const ceres::examples::Constraint3d &constraint = *constraints_iter;

    ceres::examples::MapOfPoses::iterator pose_begin_iter =
        poses->find(constraint.id_begin);
    CHECK(pose_begin_iter != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
    ceres::examples::MapOfPoses::iterator pose_end_iter =
        poses->find(constraint.id_end);
    CHECK(pose_end_iter != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";

    const Eigen::Matrix<double, 6, 6> sqrt_information =
        constraint.information.llt().matrixL();

    auto id_pair = MakeIdPair(constraint);
    CHECK_GT(constraint_counts_.count(id_pair), 0ul)
        << "Should have counted constraints for " << id_pair.first << "->"
        << id_pair.second;

    VLOG(1) << "Adding constraint pair: " << id_pair.first << " and "
            << id_pair.second;
    // Store min & max id's; assumes first id < second id
    if (id_pair.first < min_constraint_id) {
      min_constraint_id = id_pair.first;
    }
    if (id_pair.second > max_constraint_id) {
      max_constraint_id = id_pair.second;
    }
    // Normalize constraint cost by occurrences
    size_t constraint_count = constraint_counts_[id_pair];
    // Scale all costs so the total cost comes out to more reasonable numbers
    constexpr double kGlobalWeight = 1000.0;
    double constraint_weight =
        kGlobalWeight / static_cast<double>(constraint_count);

    // Ceres will take ownership of the pointer.
    ceres::CostFunction *cost_function =
        ceres::examples::PoseGraph3dErrorTerm::Create(
            constraint.t_be, sqrt_information, constraint_weight);

    problem->AddResidualBlock(cost_function, loss_function,
                              pose_begin_iter->second.p.data(),
                              pose_begin_iter->second.q.coeffs().data(),
                              pose_end_iter->second.p.data(),
                              pose_end_iter->second.q.coeffs().data());

    problem->SetManifold(pose_begin_iter->second.q.coeffs().data(),
                         quaternion_local_parameterization);
    problem->SetManifold(pose_end_iter->second.q.coeffs().data(),
                         quaternion_local_parameterization);
  }

  // The pose graph optimization problem has six DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can
  // apply a rigid body transformation to all the nodes and the optimization
  // problem will still have the exact same cost. The Levenberg-Marquardt
  // algorithm has internal damping which mitigates this issue, but it is
  // better to properly constrain the gauge freedom. This can be done by
  // setting one of the poses as constant so the optimizer cannot change it.
  CHECK_NE(poses->count(absl::GetFlag(FLAGS_frozen_target_id)), 0ul)
      << "Got no poses for frozen target id "
      << absl::GetFlag(FLAGS_frozen_target_id);
  CHECK_GE(absl::GetFlag(FLAGS_frozen_target_id), min_constraint_id)
      << "target to freeze index " << absl::GetFlag(FLAGS_frozen_target_id)
      << " must be in range of constraints, > " << min_constraint_id;
  CHECK_LE(absl::GetFlag(FLAGS_frozen_target_id), max_constraint_id)
      << "target to freeze index " << absl::GetFlag(FLAGS_frozen_target_id)
      << " must be in range of constraints, < " << max_constraint_id;
  ceres::examples::MapOfPoses::iterator pose_start_iter =
      poses->find(absl::GetFlag(FLAGS_frozen_target_id));
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
  problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
}

std::unique_ptr<ceres::CostFunction>
TargetMapper::BuildMapFittingOptimizationProblem(ceres::Problem *problem) {
  // Set up robot visualization.
  vis_robot_.ClearImage();

  const size_t num_targets =
      absl::GetFlag(FLAGS_max_target_id) - absl::GetFlag(FLAGS_min_target_id);
  // Translation and rotation error for each target
  const size_t num_residuals = num_targets * 6;
  // Set up the only cost function (also known as residual). This uses
  // auto-differentiation to obtain the derivative (jacobian).
  std::unique_ptr<ceres::CostFunction> cost_function = std::make_unique<
      ceres::AutoDiffCostFunction<TargetMapper, ceres::DYNAMIC, 3, 4>>(
      this, num_residuals, ceres::DO_NOT_TAKE_OWNERSHIP);

  ceres::LossFunction *loss_function = new ceres::HuberLoss(2.0);
  ceres::Manifold *quaternion_local_parameterization =
      new ceres::EigenQuaternionManifold();

  problem->AddResidualBlock(cost_function.get(), loss_function,
                            T_frozen_actual_.vector().data(),
                            R_frozen_actual_.coeffs().data());
  problem->SetManifold(R_frozen_actual_.coeffs().data(),
                       quaternion_local_parameterization);
  return cost_function;
}

void TargetMapper::DisplayConstraintGraph() {
  vis_robot_.ClearImage();
  for (auto constraint : constraint_counts_) {
    Eigen::Vector3d start_line =
        PoseUtils::Pose3dToAffine3d(
            ideal_target_poses_.at(constraint.first.first))
            .translation();
    Eigen::Vector3d end_line =
        PoseUtils::Pose3dToAffine3d(
            ideal_target_poses_.at(constraint.first.second))
            .translation();
    // Weight the green intensity by # of constraints
    // TODO: This could be improved
    int color_scale =
        50 + std::min(155, static_cast<int>(constraint.second * 155.0 / 200.0));
    vis_robot_.DrawLine(start_line, end_line, cv::Scalar(0, color_scale, 0));
  }

  for (const auto &[id, solved_pose] : target_poses_) {
    Eigen::Affine3d H_world_ideal =
        PoseUtils::Pose3dToAffine3d(ideal_target_poses_.at(id));
    vis_robot_.DrawFrameAxes(H_world_ideal, std::to_string(id),
                             cv::Scalar(255, 255, 255));
  }
  cv::imshow("Constraint graph", vis_robot_.image_);
  cv::waitKey(0);
}

void TargetMapper::DisplaySolvedVsInitial() {
  vis_robot_.ClearImage();
  for (const auto &[id, solved_pose] : target_poses_) {
    Eigen::Affine3d H_world_initial =
        PoseUtils::Pose3dToAffine3d(ideal_target_poses_.at(id));
    vis_robot_.DrawFrameAxes(H_world_initial, std::to_string(id),
                             cv::Scalar(0, 0, 255));
    Eigen::Affine3d H_world_solved = PoseUtils::Pose3dToAffine3d(solved_pose);
    vis_robot_.DrawFrameAxes(H_world_solved, std::to_string(id) + "-est",
                             cv::Scalar(255, 255, 255));
  }
  cv::imshow("Solved vs. Initial", vis_robot_.image_);
  cv::waitKey(0);
}

// Taken from ceres/examples/slam/pose_graph_3d/pose_graph_3d.cc
bool TargetMapper::SolveOptimizationProblem(ceres::Problem *problem) {
  CHECK(problem != nullptr);

  ceres::Solver::Options options;
  options.max_num_iterations = absl::GetFlag(FLAGS_max_num_iterations);
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  LOG(INFO) << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

void TargetMapper::Solve(std::string_view field_name,
                         std::optional<std::string_view> output_dir) {
  ceres::Problem target_pose_problem_1;
  BuildTargetPoseOptimizationProblem(target_constraints_, &target_poses_,
                                     &target_pose_problem_1);
  CHECK(SolveOptimizationProblem(&target_pose_problem_1))
      << "The target pose solve 1 was not successful, exiting.";
  if (absl::GetFlag(FLAGS_visualize_solver)) {
    LOG(INFO) << "Displaying constraint graph before removing outliers";
    DisplayConstraintGraph();
    DisplaySolvedVsInitial();
  }

  RemoveOutlierConstraints();

  // Solve again once we've thrown out bad constraints
  ceres::Problem target_pose_problem_2;
  BuildTargetPoseOptimizationProblem(target_constraints_, &target_poses_,
                                     &target_pose_problem_2);
  CHECK(SolveOptimizationProblem(&target_pose_problem_2))
      << "The target pose solve 2 was not successful, exiting.";
  if (absl::GetFlag(FLAGS_visualize_solver)) {
    LOG(INFO) << "Displaying constraint graph before removing outliers";
    DisplayConstraintGraph();
    DisplaySolvedVsInitial();
  }

  if (absl::GetFlag(FLAGS_do_map_fitting)) {
    LOG(INFO) << "Solving the overall map's best alignment to the previous map";
    ceres::Problem map_fitting_problem(
        {.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP});
    std::unique_ptr<ceres::CostFunction> map_fitting_cost_function =
        BuildMapFittingOptimizationProblem(&map_fitting_problem);
    CHECK(SolveOptimizationProblem(&map_fitting_problem))
        << "The map fitting solve was not successful, exiting.";
    map_fitting_cost_function.release();

    Eigen::Affine3d H_frozen_actual = T_frozen_actual_ * R_frozen_actual_;
    LOG(INFO) << "H_frozen_actual: "
              << PoseUtils::Affine3dToPose3d(H_frozen_actual);

    auto H_world_frozen = PoseUtils::Pose3dToAffine3d(
        target_poses_[absl::GetFlag(FLAGS_frozen_target_id)]);
    auto H_world_frozenactual = H_world_frozen * H_frozen_actual;

    // Offset the solved poses to become the actual ones
    for (auto &[id, pose] : target_poses_) {
      // Don't offset targets we didn't solve for
      if (id < absl::GetFlag(FLAGS_min_target_id) ||
          id > absl::GetFlag(FLAGS_max_target_id)) {
        continue;
      }

      // Take the delta between the frozen target and the solved target, and put
      // that on top of the actual pose of the frozen target
      auto H_world_solved = PoseUtils::Pose3dToAffine3d(pose);
      auto H_frozen_solved = H_world_frozen.inverse() * H_world_solved;
      auto H_world_actual = H_world_frozenactual * H_frozen_solved;
      pose = PoseUtils::Affine3dToPose3d(H_world_actual);
    }
  }

  auto map_json = MapToJson(field_name);
  VLOG(1) << "Solved target poses: " << map_json;

  if (output_dir.has_value()) {
    std::string output_path =
        absl::StrCat(output_dir.value(), "/", field_name, ".json");
    LOG(INFO) << "Writing map to file: " << output_path;
    aos::util::WriteStringToFileOrDie(output_path, map_json);
  }

  for (TargetId id_start = absl::GetFlag(FLAGS_min_target_id);
       id_start < absl::GetFlag(FLAGS_max_target_id); id_start++) {
    for (TargetId id_end = id_start + 1;
         id_end <= absl::GetFlag(FLAGS_max_target_id); id_end++) {
      auto H_start_end =
          PoseUtils::Pose3dToAffine3d(target_poses_.at(id_start)).inverse() *
          PoseUtils::Pose3dToAffine3d(target_poses_.at(id_end));
      auto constraint = PoseUtils::Affine3dToPose3d(H_start_end);
      VLOG(1) << id_start << "->" << id_end << ": " << constraint.p.norm()
              << " meters";
    }
  }
}

std::string TargetMapper::MapToJson(std::string_view field_name) const {
  flatbuffers::FlatBufferBuilder fbb;

  // Convert poses to flatbuffers
  std::vector<flatbuffers::Offset<TargetPoseFbs>> target_poses_fbs;
  for (const auto &[id, pose] : target_poses_) {
    target_poses_fbs.emplace_back(
        TargetPoseToFbs(TargetPose{.id = id, .pose = pose}, &fbb));
  }

  const auto field_name_offset = fbb.CreateString(field_name);
  flatbuffers::Offset<TargetMap> target_map_offset = CreateTargetMap(
      fbb, fbb.CreateVector(target_poses_fbs), field_name_offset);

  return aos::FlatbufferToJson(
      flatbuffers::GetMutableTemporaryPointer(fbb, target_map_offset),
      {.multi_line = true});
}

namespace {
// Hacks to extract a double from a scalar, which is either a ceres jet or a
// double. Only used for debugging and displaying.
template <typename S>
double ScalarToDouble(S s) {
  const double *ptr = reinterpret_cast<double *>(&s);
  return *ptr;
}

template <typename S>
Eigen::Affine3d ScalarAffineToDouble(Eigen::Transform<S, 3, Eigen::Affine> H) {
  Eigen::Affine3d H_double;
  for (size_t i = 0; i < H.rows(); i++) {
    for (size_t j = 0; j < H.cols(); j++) {
      H_double(i, j) = ScalarToDouble(H(i, j));
    }
  }
  return H_double;
}

}  // namespace

template <typename S>
bool TargetMapper::operator()(const S *const translation,
                              const S *const rotation, S *residual) const {
  using Affine3s = Eigen::Transform<S, 3, Eigen::Affine>;
  Eigen::Quaternion<S> R_frozen_actual(rotation[3], rotation[1], rotation[2],
                                       rotation[0]);
  Eigen::Translation<S, 3> T_frozen_actual(translation[0], translation[1],
                                           translation[2]);
  // Actual target pose in the frame of the fixed pose.
  Affine3s H_frozen_actual = T_frozen_actual * R_frozen_actual;
  VLOG(2) << "H_frozen_actual: "
          << PoseUtils::Affine3dToPose3d(ScalarAffineToDouble(H_frozen_actual));

  Affine3s H_world_frozen =
      PoseUtils::Pose3dToAffine3d(
          target_poses_.at(absl::GetFlag(FLAGS_frozen_target_id)))
          .cast<S>();
  Affine3s H_world_frozenactual = H_world_frozen * H_frozen_actual;

  size_t residual_index = 0;
  if (absl::GetFlag(FLAGS_visualize_solver)) {
    vis_robot_.ClearImage();
  }

  for (const auto &[id, solved_pose] : target_poses_) {
    if (id < absl::GetFlag(FLAGS_min_target_id) ||
        id > absl::GetFlag(FLAGS_max_target_id)) {
      continue;
    }

    Affine3s H_world_ideal =
        PoseUtils::Pose3dToAffine3d(ideal_target_poses_.at(id)).cast<S>();
    Affine3s H_world_solved =
        PoseUtils::Pose3dToAffine3d(solved_pose).cast<S>();
    // Take the delta between the frozen target and the solved target, and put
    // that on top of the actual pose of the frozen target
    auto H_frozen_solved = H_world_frozen.inverse() * H_world_solved;
    auto H_world_actual = H_world_frozenactual * H_frozen_solved;
    VLOG(2) << id << ": " << H_world_actual.translation();
    Affine3s H_ideal_actual = H_world_ideal.inverse() * H_world_actual;
    auto T_ideal_actual = H_ideal_actual.translation();
    VLOG(2) << "T_ideal_actual: " << T_ideal_actual;
    VLOG(2);
    auto R_ideal_actual = Eigen::AngleAxis<S>(H_ideal_actual.rotation());

    // Weight translation errors higher than rotation.
    // 1 m in position error = 0.01 radian (or ~0.573 degrees)
    constexpr double kTranslationScalar = 1000.0;
    constexpr double kRotationScalar = 100.0;

    // Penalize based on how much our actual poses matches the ideal
    // ones. We've already solved for the relative poses, now figure out
    // where all of them fit in the world.
    residual[residual_index++] = kTranslationScalar * T_ideal_actual(0);
    residual[residual_index++] = kTranslationScalar * T_ideal_actual(1);
    residual[residual_index++] = kTranslationScalar * T_ideal_actual(2);
    residual[residual_index++] =
        kRotationScalar * R_ideal_actual.angle() * R_ideal_actual.axis().x();
    residual[residual_index++] =
        kRotationScalar * R_ideal_actual.angle() * R_ideal_actual.axis().y();
    residual[residual_index++] =
        kRotationScalar * R_ideal_actual.angle() * R_ideal_actual.axis().z();

    if (absl::GetFlag(FLAGS_visualize_solver)) {
      LOG(INFO) << std::to_string(id) + std::string("-est") << " at "
                << ScalarAffineToDouble(H_world_actual).matrix();
      vis_robot_.DrawFrameAxes(ScalarAffineToDouble(H_world_actual),
                               std::to_string(id) + std::string("-est"),
                               cv::Scalar(0, 255, 0));
      vis_robot_.DrawFrameAxes(ScalarAffineToDouble(H_world_ideal),
                               std::to_string(id), cv::Scalar(255, 255, 255));
    }
  }
  if (absl::GetFlag(FLAGS_visualize_solver)) {
    cv::imshow("Target maps", vis_robot_.image_);
    cv::waitKey(0);
  }

  // Ceres can't handle residual values of exactly zero
  for (size_t i = 0; i < residual_index; i++) {
    if (residual[i] == S(0)) {
      residual[i] = S(1e-9);
    }
  }

  return true;
}

TargetMapper::PoseError TargetMapper::ComputeError(
    const ceres::examples::Constraint3d &constraint) const {
  // Compute the difference between the map-based transform of the end target
  // in the start target frame, to the one from this constraint
  auto H_start_end_map =
      PoseUtils::Pose3dToAffine3d(target_poses_.at(constraint.id_begin))
          .inverse() *
      PoseUtils::Pose3dToAffine3d(target_poses_.at(constraint.id_end));
  auto H_start_end_constraint = PoseUtils::Pose3dToAffine3d(constraint.t_be);
  ceres::examples::Pose3d delta_pose = PoseUtils::Affine3dToPose3d(
      H_start_end_map.inverse() * H_start_end_constraint);
  double distance = delta_pose.p.norm();
  Eigen::AngleAxisd err_angle(delta_pose.q);
  double angle = std::abs(err_angle.angle());
  return {.angle = angle, .distance = distance};
}

TargetMapper::Stats TargetMapper::ComputeStats() const {
  Stats stats{.avg_err = {.angle = 0.0, .distance = 0.0},
              .std_dev = {.angle = 0.0, .distance = 0.0},
              .max_err = {.angle = 0.0, .distance = 0.0}};

  for (const auto &constraint : target_constraints_) {
    PoseError err = ComputeError(constraint);

    // Update our statistics
    stats.avg_err.distance += err.distance;
    if (err.distance > stats.max_err.distance) {
      stats.max_err.distance = err.distance;
    }

    stats.avg_err.angle += err.angle;
    if (err.angle > stats.max_err.angle) {
      stats.max_err.angle = err.angle;
    }
  }

  stats.avg_err.distance /= static_cast<double>(target_constraints_.size());
  stats.avg_err.angle /= static_cast<double>(target_constraints_.size());

  for (const auto &constraint : target_constraints_) {
    PoseError err = ComputeError(constraint);

    // Update our statistics
    stats.std_dev.distance +=
        std::pow(err.distance - stats.avg_err.distance, 2);

    stats.std_dev.angle += std::pow(err.angle - stats.avg_err.angle, 2);
  }

  stats.std_dev.distance = std::sqrt(
      stats.std_dev.distance / static_cast<double>(target_constraints_.size()));
  stats.std_dev.angle = std::sqrt(
      stats.std_dev.angle / static_cast<double>(target_constraints_.size()));

  return stats;
}

void TargetMapper::RemoveOutlierConstraints() {
  stats_with_outliers_ = ComputeStats();
  size_t original_size = target_constraints_.size();
  target_constraints_.erase(
      std::remove_if(
          target_constraints_.begin(), target_constraints_.end(),
          [&](const auto &constraint) {
            PoseError err = ComputeError(constraint);
            // Remove constraints with errors significantly above
            // the average
            if (err.distance > stats_with_outliers_.avg_err.distance +
                                   absl::GetFlag(FLAGS_outlier_std_devs) *
                                       stats_with_outliers_.std_dev.distance) {
              return true;
            }
            if (err.angle > stats_with_outliers_.avg_err.angle +
                                absl::GetFlag(FLAGS_outlier_std_devs) *
                                    stats_with_outliers_.std_dev.angle) {
              return true;
            }
            return false;
          }),
      target_constraints_.end());

  LOG(INFO) << "Removed " << (original_size - target_constraints_.size())
            << " outlier constraints out of " << original_size << " total";
}

void TargetMapper::DumpStats(std::string_view path) const {
  LOG(INFO) << "Dumping mapping stats to " << path;
  Stats stats = ComputeStats();
  std::ofstream fout(path.data());
  fout << "Stats after outlier rejection: " << std::endl;
  fout << "Average error - angle: " << stats.avg_err.angle
       << ", distance: " << stats.avg_err.distance << std::endl
       << std::endl;
  fout << "Standard deviation - angle: " << stats.std_dev.angle
       << ", distance: " << stats.std_dev.distance << std::endl
       << std::endl;
  fout << "Max error - angle: " << stats.max_err.angle
       << ", distance: " << stats.max_err.distance << std::endl;

  fout << std::endl << "Stats before outlier rejection:" << std::endl;
  fout << "Average error - angle: " << stats_with_outliers_.avg_err.angle
       << ", distance: " << stats_with_outliers_.avg_err.distance << std::endl
       << std::endl;
  fout << "Standard deviation - angle: " << stats_with_outliers_.std_dev.angle
       << ", distance: " << stats_with_outliers_.std_dev.distance << std::endl
       << std::endl;
  fout << "Max error - angle: " << stats_with_outliers_.max_err.angle
       << ", distance: " << stats_with_outliers_.max_err.distance << std::endl;

  fout.flush();
  fout.close();
}

void TargetMapper::DumpConstraints(std::string_view path) const {
  LOG(INFO) << "Dumping target constraints to " << path;
  std::ofstream fout(path.data());
  for (const auto &constraint : target_constraints_) {
    fout << absl::StrCat("", constraint) << std::endl;
  }
  fout.flush();
  fout.close();
}

void TargetMapper::PrintDiffs() const {
  for (int id = absl::GetFlag(FLAGS_min_target_id);
       id <= absl::GetFlag(FLAGS_max_target_id); id++) {
    Eigen::Affine3d H_world_ideal =
        PoseUtils::Pose3dToAffine3d(ideal_target_poses_.at(id));
    Eigen::Affine3d H_world_solved =
        PoseUtils::Pose3dToAffine3d(target_poses_.at(id));
    Eigen::Affine3d H_ideal_solved = H_world_ideal.inverse() * H_world_solved;
    Eigen::Vector3d rpy = PoseUtils::RotationMatrixToEulerAngles(
                              H_ideal_solved.rotation().matrix()) *
                          180.0 / M_PI;
    Eigen::Vector3d trans = H_ideal_solved.translation();

    LOG(INFO) << "\nOffset from ideal to solved for target " << id
              << " (in m, deg)"
              << "\n  x: " << trans(0) << ", y: " << trans(1)
              << ", z: " << trans(2) << ", \n  roll: " << rpy(0)
              << ", pitch: " << rpy(1) << ", yaw: " << rpy(2) << "\n";
  }
}

}  // namespace frc::vision
