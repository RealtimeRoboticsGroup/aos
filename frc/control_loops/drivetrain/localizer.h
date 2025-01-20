#ifndef FRC_CONTROL_LOOPS_DRIVETRAIN_FIELD_ESTIMATOR_H_
#define FRC_CONTROL_LOOPS_DRIVETRAIN_FIELD_ESTIMATOR_H_

#include "aos/events/event_loop.h"
#include "frc/control_loops/drivetrain/drivetrain_config.h"
#include "frc/control_loops/drivetrain/drivetrain_status_generated.h"
#include "frc/control_loops/drivetrain/hybrid_ekf.h"
#include "frc/control_loops/pose.h"

namespace frc::control_loops::drivetrain {

// An interface for target selection. This provides an object that will take in
// state updates and then determine what poes we should be driving to.
class TargetSelectorInterface {
 public:
  typedef RobotSide Side;
  virtual ~TargetSelectorInterface() {}
  // Take the state as [x, y, theta, left_vel, right_vel]
  // If unable to determine what target to go for, returns false. If a viable
  // target is selected, then returns true and sets target_pose.
  // command_speed is the goal speed of the current drivetrain, generally
  // generated from the throttle and meant to signify driver intent.
  // TODO(james): Some implementations may also want a drivetrain goal so that
  // driver intent can be divined more directly.
  virtual bool UpdateSelection(const ::Eigen::Matrix<double, 5, 1> &state,
                               double command_speed) = 0;
  // Gets the current target pose. Should only be called if UpdateSelection has
  // returned true.
  virtual TypedPose<double> TargetPose() const = 0;
  // For the "radii" below, we have two possible modes:
  // 1) Akin to 2019, we can place with either edge of the game piece, so
  //    the line following code will have to automatically detect which edge
  //    (right or left) to aim to have intersect the target.
  // 2) As in 2023, the game piece itself is offset in the robot and so we care
  //    which of left vs. right we are using.
  // In situation (1), SignedRadii() should return false and the *Radius()
  // functions should return a non-negative number (technically I think the
  // math may work for negative numbers, but may have weird implications
  // physically...). For (2) SignedRadii()
  // should return true and the sign of the *Radius() functions will be
  // respected by the line following code.
  virtual bool SignedRadii() const = 0;
  // The "radius" of the target--for y2019, we wanted to drive in so that a disc
  // with radius r would hit the plane of the target at an offset of exactly r
  // from the TargetPose--this is distinct from wanting the center of the
  // robot to project straight onto the center of the target.
  virtual double TargetRadius() const = 0;
  // the "radius" of the robot/game piece to place.
  virtual double GamePieceRadius() const = 0;
  // Which direction we want the robot to drive to get to the target.
  virtual Side DriveDirection() const = 0;
  // Indicates that the line following *must* drive to the currently selected
  // target, regardless of any hysteresis we try to use to protect the driver.
  virtual bool ForceReselectTarget() const = 0;
};

// Defines an interface for classes that provide field-global localization.
class LocalizerInterface {
 public:
  typedef HybridEkf<double> Ekf;
  typedef typename Ekf::StateIdx StateIdx;

  virtual ~LocalizerInterface() {}

  // Perform a single step of the filter, using the information that is
  // available on every drivetrain iteration.
  // The user should pass in the U that the real system experienced from the
  // previous timestep until now; internally, any filters will first perform a
  // prediction step to get the estimate at time now, and then will apply
  // corrections based on the encoder/gyro/accelerometer values from time now.
  // TODO(james): Consider letting implementations subscribe to the sensor
  // values themselves, and then only passing in U. This requires more
  // coordination on timing, however.
  virtual void Update(const ::Eigen::Matrix<double, 2, 1> &U,
                      ::aos::monotonic_clock::time_point now,
                      double left_encoder, double right_encoder,
                      double gyro_rate, const Eigen::Vector3d &accel) = 0;
  virtual void Reset(aos::monotonic_clock::time_point t,
                     const Ekf::State &state) = 0;
  // Reset the absolute position of the estimator.
  virtual void ResetPosition(::aos::monotonic_clock::time_point t, double x,
                             double y, double theta, double theta_uncertainty,
                             bool reset_theta) = 0;
  flatbuffers::Offset<LocalizerState> PopulateStatus(
      flatbuffers::FlatBufferBuilder *fbb) {
    LocalizerState::Builder builder(*fbb);
    builder.add_x(x());
    builder.add_y(y());
    builder.add_theta(theta());
    builder.add_left_velocity(left_velocity());
    builder.add_right_velocity(right_velocity());
    builder.add_left_encoder(left_encoder());
    builder.add_right_encoder(right_encoder());
    builder.add_left_voltage_error(left_voltage_error());
    builder.add_right_voltage_error(right_voltage_error());
    builder.add_angular_error(angular_error());
    builder.add_longitudinal_velocity_offset(longitudinal_velocity_offset());
    builder.add_lateral_velocity(lateral_velocity());
    return builder.Finish();
  }
  virtual Ekf::State Xhat() const = 0;
  // There are several subtly different norms floating around for state
  // matrices. In order to avoid that mess, we just provide direct accessors for
  // the values that most people care about.
  double x() const { return Xhat()(StateIdx::kX); }
  double y() const { return Xhat()(StateIdx::kY); }
  double theta() const { return Xhat()(StateIdx::kTheta); }
  double left_velocity() const { return Xhat()(StateIdx::kLeftVelocity); }
  double right_velocity() const { return Xhat()(StateIdx::kRightVelocity); }
  double left_encoder() const { return Xhat()(StateIdx::kLeftEncoder); }
  double right_encoder() const { return Xhat()(StateIdx::kRightEncoder); }
  double left_voltage_error() const {
    return Xhat()(StateIdx::kLeftVoltageError);
  }
  double right_voltage_error() const {
    return Xhat()(StateIdx::kRightVoltageError);
  }
  double angular_error() const { return Xhat()(StateIdx::kAngularError); }
  double longitudinal_velocity_offset() const {
    return Xhat()(StateIdx::kLongitudinalVelocityOffset);
  }
  double lateral_velocity() const { return Xhat()(StateIdx::kLateralVelocity); }

  virtual TargetSelectorInterface *target_selector() = 0;
};

// A target selector, primarily for testing purposes, that just lets a user
// manually set the target selector state.
class TrivialTargetSelector : public TargetSelectorInterface {
 public:
  virtual ~TrivialTargetSelector() {}
  bool UpdateSelection(const ::Eigen::Matrix<double, 5, 1> &, double) override {
    return has_target_;
  }
  TypedPose<double> TargetPose() const override { return pose_; }
  bool SignedRadii() const override { return signed_radii_; }
  double TargetRadius() const override { return target_radius_; }
  double GamePieceRadius() const override { return game_piece_radius_; }
  Side DriveDirection() const override { return drive_direction_; }
  bool ForceReselectTarget() const override { return force_reselect_; }

  void set_pose(const TypedPose<double> &pose) { pose_ = pose; }
  void set_target_radius(double radius) { target_radius_ = radius; }
  void set_game_piece_radius(double radius) { game_piece_radius_ = radius; }
  void set_has_target(bool has_target) { has_target_ = has_target; }
  void set_drive_direction(Side side) { drive_direction_ = side; }
  void set_force_reselect(bool force_reselect) {
    force_reselect_ = force_reselect;
  }
  bool has_target() const { return has_target_; }

 private:
  bool has_target_ = true;
  bool force_reselect_ = false;
  TypedPose<double> pose_;
  bool signed_radii_ = false;
  double target_radius_ = 0.0;
  double game_piece_radius_ = 0.0;
  Side drive_direction_ = Side::DONT_CARE;
};

// Uses the generic HybridEkf implementation to provide a basic field estimator.
// This provides no method for using cameras or the such to get global
// measurements and just assumes that you can dead-reckon perfectly.
class DeadReckonEkf : public LocalizerInterface {
 public:
  DeadReckonEkf(::aos::EventLoop *event_loop,
                const DrivetrainConfig<double> &dt_config)
      : ekf_(dt_config) {
    event_loop->OnRun([this, event_loop]() {
      ekf_.ResetInitialState(event_loop->monotonic_now(), Ekf::State::Zero(),
                             ekf_.P());
    });
    target_selector_.set_has_target(false);
  }

  virtual ~DeadReckonEkf() {}

  void Update(const ::Eigen::Matrix<double, 2, 1> &U,
              ::aos::monotonic_clock::time_point now, double left_encoder,
              double right_encoder, double gyro_rate,
              const Eigen::Vector3d &accel) override {
    ekf_.UpdateEncodersAndGyro(left_encoder, right_encoder, gyro_rate, U, accel,
                               now);
  }

  void Reset(aos::monotonic_clock::time_point t,
             const HybridEkf<double>::State &state) override {
    ekf_.ResetInitialState(t, state, ekf_.P());
  }

  void ResetPosition(::aos::monotonic_clock::time_point t, double x, double y,
                     double theta, double /*theta_override*/,
                     bool /*reset_theta*/) override {
    const double left_encoder = ekf_.X_hat(StateIdx::kLeftEncoder);
    const double right_encoder = ekf_.X_hat(StateIdx::kRightEncoder);
    ekf_.ResetInitialState(t,
                           (Ekf::State() << x, y, theta, left_encoder, 0,
                            right_encoder, 0, 0, 0, 0, 0, 0)
                               .finished(),
                           ekf_.P());
  }

  Ekf::State Xhat() const override { return ekf_.X_hat(); }

  TrivialTargetSelector *target_selector() override {
    return &target_selector_;
  }

 private:
  Ekf ekf_;
  TrivialTargetSelector target_selector_;
};

}  // namespace frc::control_loops::drivetrain

#endif  // FRC_CONTROL_LOOPS_DRIVETRAIN_FIELD_ESTIMATOR_H_
