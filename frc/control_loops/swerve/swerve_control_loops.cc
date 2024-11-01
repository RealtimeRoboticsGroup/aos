#include "frc/control_loops/swerve/swerve_control_loops.h"

ABSL_FLAG(int, swerve_priority, 1, "");
ABSL_FLAG(int, swerve_iters, 1, "");

namespace frc::control_loops::swerve {

SwerveControlLoops::SwerveControlLoops(
    ::aos::EventLoop *event_loop,
    const frc::control_loops::
        StaticZeroingSingleDOFProfiledSubsystemCommonParams *,
    const SwerveZeroing *zeroing_params,
    const NaiveEstimator::Parameters &params, const ::std::string &name)
    : frc::controls::ControlLoop<Goal, Position, StatusStatic, OutputStatic>(
          event_loop, name),
      can_position_fetcher_(
          event_loop->MakeFetcher<CanPosition>("/drivetrain")),
      gyro_fetcher_(event_loop->MakeFetcher<::frc::sensors::GyroReading>(
          "/drivetrain")),
      naive_estimator_(zeroing_params, params),
      velocity_controller_(LinearVelocityController::MakeParameters(params),
                           params),
      inverse_kinematics_(params),
      velocity_ekf_(params) {
  if (absl::GetFlag(FLAGS_swerve_priority) > 0) {
    event_loop->SetRuntimeRealtimePriority(
        absl::GetFlag(FLAGS_swerve_priority));
  }
}

void SwerveControlLoops::RunIteration(
    const Goal *goal, const Position *position,
    aos::Sender<OutputStatic>::StaticBuilder *output_builder,
    aos::Sender<StatusStatic>::StaticBuilder *status_builder) {
  ++iteration_counter_;
  if (iteration_counter_ % absl::GetFlag(FLAGS_swerve_iters) != 0) {
    return;
  }
  const aos::monotonic_clock::time_point profiling_start_time =
      aos::monotonic_clock::now();
  const aos::monotonic_clock::time_point now =
      event_loop()->context().monotonic_event_time;

  gyro_fetcher_.Fetch();

  can_position_fetcher_.Fetch();
  std::optional<NaiveEstimator::State> current_state;
  if (gyro_fetcher_.get() != nullptr &&
      can_position_fetcher_.get() != nullptr) {
    double gyro_rate = gyro_fetcher_->velocity();
    if (!yaw_gyro_zero_.has_value()) {
      yaw_gyro_zeroer_.AddData(gyro_rate);
      // Maximum variation to allow in the gyro when zeroing.
      constexpr double kMaxYawGyroZeroingRange = 0.15;
      if (yaw_gyro_zeroer_.full() &&
          yaw_gyro_zeroer_.GetRange() < kMaxYawGyroZeroingRange) {
        yaw_gyro_zero_ = yaw_gyro_zeroer_.GetAverage()(0);
        VLOG(1) << "Zeroed to " << *yaw_gyro_zero_ << " Range "
                << yaw_gyro_zeroer_.GetRange();
      }
    }
    if (yaw_gyro_zero_.has_value()) {
      gyro_rate = gyro_rate - yaw_gyro_zero_.value();
    } else {
      gyro_rate = 0.0;
    }

    current_state = naive_estimator_.Update(
        now, position, can_position_fetcher_.get(), gyro_rate);
    velocity_ekf_.Update(
        now,
        Eigen::Matrix<Scalar, 4, 1>{{current_state.value()(States::kThetas0)},
                                    {current_state.value()(States::kThetas1)},
                                    {current_state.value()(States::kThetas2)},
                                    {current_state.value()(States::kThetas3)}},
        {
            aos::monotonic_clock::time_point{
                std::chrono::nanoseconds(can_position_fetcher_->front_left()
                                             ->translation()
                                             ->timestamp())},
            aos::monotonic_clock::time_point{
                std::chrono::nanoseconds(can_position_fetcher_->front_right()
                                             ->translation()
                                             ->timestamp())},
            aos::monotonic_clock::time_point{
                std::chrono::nanoseconds(can_position_fetcher_->back_left()
                                             ->translation()
                                             ->timestamp())},
            aos::monotonic_clock::time_point{
                std::chrono::nanoseconds(can_position_fetcher_->back_right()
                                             ->translation()
                                             ->timestamp())},
        },
        Eigen::Matrix<Scalar, 4, 1>{
            {can_position_fetcher_->front_left()->translation()->position()},
            {can_position_fetcher_->front_right()->translation()->position()},
            {can_position_fetcher_->back_left()->translation()->position()},
            {can_position_fetcher_->back_right()->translation()->position()}},
        gyro_rate, U_,
        (status_builder != nullptr) ? status_builder->get()->add_velocity_ekf()
                                    : nullptr);
  }

  const aos::monotonic_clock::time_point estimation_done =
      aos::monotonic_clock::now();
  U_.setZero();
  std::optional<LinearVelocityController::ControllerResult> controller_result;
  if (goal != nullptr && current_state.has_value()) {
    CHECK_NE(goal->has_linear_velocity_goal(), goal->has_joystick_goal());
    if (goal->has_linear_velocity_goal()) {
      NaiveEstimator::State goal_state =
          ToEigenOrDie<12, 1>(*goal->linear_velocity_goal()->state())
              .cast<Scalar>();
      controller_result = velocity_controller_.RunRawController(
          current_state.value(), goal_state,
          ToEigenOrDie<8, 1>(*goal->linear_velocity_goal()->input())
              .cast<Scalar>());
    } else if (goal->has_joystick_goal()) {
      NaiveEstimator::State kinematics_state = current_state.value();
      kinematics_state(NaiveEstimator::States::kVx) =
          goal->joystick_goal()->vx();
      kinematics_state(NaiveEstimator::States::kVy) =
          goal->joystick_goal()->vy();
      kinematics_state(NaiveEstimator::States::kOmega) =
          goal->joystick_goal()->omega();
      controller_result = velocity_controller_.RunRawController(
          current_state.value(), inverse_kinematics_.Solve(kinematics_state),
          Eigen::Matrix<Scalar, 8, 1>::Zero());
    } else {
      LOG(FATAL) << "Unreachable";
    }
    U_ = controller_result->U.cast<float>();
  }

  const aos::monotonic_clock::time_point controller_done =
      aos::monotonic_clock::now();

  if (output_builder != nullptr) {
    OutputStatic *output = output_builder->get();

    {
      auto module_output = output->add_front_left_output();
      module_output->set_rotation_current(U_(InputStates::kIs0));
      module_output->set_translation_current(U_(InputStates::kId0));
    }
    {
      auto module_output = output->add_front_right_output();
      module_output->set_rotation_current(U_(InputStates::kIs1));
      module_output->set_translation_current(U_(InputStates::kId1));
    }
    {
      auto module_output = output->add_back_left_output();
      module_output->set_rotation_current(U_(InputStates::kIs2));
      module_output->set_translation_current(U_(InputStates::kId2));
    }
    {
      auto module_output = output->add_back_right_output();
      module_output->set_rotation_current(U_(InputStates::kIs3));
      module_output->set_translation_current(U_(InputStates::kId3));
    }

    // Ignore the return value of Send
    output_builder->CheckOk(output_builder->Send());
  } else {
    U_.setZero();
  }

  if (status_builder != nullptr) {
    StatusStatic *status = status_builder->get();

    if (current_state.has_value()) {
      naive_estimator_.PopulateStatus(status->add_naive_estimator());
    }

    if (controller_result.has_value()) {
      auto controller_status = status->add_linear_controller();
      CHECK(FromEigen(controller_result->debug.goal.cast<double>(),
                      controller_status->add_goal_state()));
      CHECK(FromEigen(controller_result->debug.U_ff.cast<double>(),
                      controller_status->add_feedforwards_currents()));
      CHECK(FromEigen(controller_result->debug.U_feedback.cast<double>(),
                      controller_status->add_feedback_currents()));
      CHECK(FromEigen(
          controller_result->debug.feedback_contributions.cast<double>(),
          controller_status->add_feedback_contributions()));
      controller_status->set_sb02od_result(
          controller_result->debug.sb02od_exit_code);
    }

    // Ignore the return value of Send
    status_builder->CheckOk(status_builder->Send());
  }

  const aos::monotonic_clock::time_point sends_done =
      aos::monotonic_clock::now();

  VLOG(1) << "Loop took "
          << aos::time::DurationInSeconds(sends_done - profiling_start_time)
          << " sec total. "
          << aos::time::DurationInSeconds(estimation_done -
                                          profiling_start_time)
          << " sec in estimation "
          << aos::time::DurationInSeconds(controller_done - estimation_done)
          << " in controller "
          << aos::time::DurationInSeconds(sends_done - controller_done)
          << " sec cleaning up";
}

}  // namespace frc::control_loops::swerve
