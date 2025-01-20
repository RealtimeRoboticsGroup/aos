#ifndef FRC_CONTROL_LOOPS_PROFILED_SUBSYSTEM_H_
#define FRC_CONTROL_LOOPS_PROFILED_SUBSYSTEM_H_

#include <array>
#include <chrono>
#include <memory>
#include <utility>

#include "Eigen/Dense"

#include "aos/util/trapezoid_profile.h"
#include "frc/constants.h"
#include "frc/control_loops/control_loop.h"
#include "frc/control_loops/control_loops_generated.h"
#include "frc/control_loops/profiled_subsystem_generated.h"
#include "frc/control_loops/simple_capped_state_feedback_loop.h"
#include "frc/control_loops/state_feedback_loop.h"
#include "frc/zeroing/pot_and_index.h"
#include "frc/zeroing/zeroing.h"

namespace frc::control_loops {

// TODO(Brian): Use a tuple instead of an array to support heterogeneous zeroing
// styles.
template <int number_of_states, int number_of_axes,
          class ZeroingEstimator =
              ::frc::zeroing::PotAndIndexPulseZeroingEstimator,
          int number_of_inputs = number_of_axes,
          int number_of_outputs = number_of_axes>
class ProfiledSubsystem {
 public:
  ProfiledSubsystem(
      ::std::unique_ptr<::frc::control_loops::SimpleCappedStateFeedbackLoop<
          number_of_states, number_of_inputs, number_of_outputs>>
          loop,
      ::std::array<ZeroingEstimator, number_of_axes> &&estimators)
      : loop_(::std::move(loop)), estimators_(::std::move(estimators)) {
    zeroed_.fill(false);
    unprofiled_goal_.setZero();
    X_hat_.setZero();
  }

  // Returns whether an error has occured
  bool error() const {
    for (const auto &estimator : estimators_) {
      if (estimator.error()) {
        return true;
      }
    }
    return false;
  }

  void Reset() {
    zeroed_.fill(false);
    initialized_ = false;
    for (auto &estimator : estimators_) {
      estimator.Reset();
    }
    should_reset_ = true;
  }

  // Returns the controller.
  const StateFeedbackLoop<number_of_states, number_of_inputs,
                          number_of_outputs> &
  controller() const {
    return *loop_;
  }

  int controller_index() const { return loop_->index(); }

  void set_controller_index(int index) { loop_->set_index(index); }

  // Returns whether the estimators have been initialized and zeroed.
  bool initialized() const { return initialized_; }

  bool zeroed() const {
    for (int i = 0; i < number_of_axes; ++i) {
      if (!zeroed_[i]) {
        return false;
      }
    }
    return true;
  }

  bool zeroed(int index) const { return zeroed_[index]; };

  // Returns the filtered goal.
  const Eigen::Matrix<double, number_of_states, 1> &goal() const {
    return loop_->R();
  }
  double goal(int row, int col) const { return loop_->R(row, col); }

  // Returns the unprofiled goal.
  const Eigen::Matrix<double, number_of_states, 1> &unprofiled_goal() const {
    return unprofiled_goal_;
  }
  double unprofiled_goal(int row, int col) const {
    return unprofiled_goal_(row, col);
  }

  // Returns the current state estimate after the most recent Correct.  This
  // does not change when Predict is run.
  const Eigen::Matrix<double, number_of_states, 1> &X_hat() const {
    return X_hat_;
  }
  double X_hat(int row, int col) const { return X_hat()(row, col); }
  // Returns a mutable reference to the current state of the actual kalman
  // filter state.  Note: changing this won't change X_hat() immediately.
  double &mutable_X_hat(int row, int col) const {
    return loop_->mutable_X_hat(row, col);
  }

  // Returns the current internal estimator state for logging.
  flatbuffers::Offset<typename ZeroingEstimator::State> EstimatorState(
      flatbuffers::FlatBufferBuilder *fbb, int index) {
    return estimators_[index].GetEstimatorState(fbb);
  }

  // Sets the maximum voltage that will be commanded by the loop.
  void set_max_voltage(::std::array<double, number_of_inputs> voltages) {
    for (int i = 0; i < number_of_inputs; ++i) {
      loop_->set_max_voltage(i, voltages[i]);
    }
  }

 protected:
  void set_zeroed(int index, bool val) { zeroed_[index] = val; }

  ::std::unique_ptr<::frc::control_loops::SimpleCappedStateFeedbackLoop<
      number_of_states, number_of_inputs, number_of_outputs>>
      loop_;

  // The goal that the profile tries to reach.
  Eigen::Matrix<double, number_of_states, 1> unprofiled_goal_;

  Eigen::Matrix<double, number_of_states, 1> X_hat_;

  bool initialized_ = false;

  // If true, the subclass should reset in Update.  It should then clear this
  // flag.
  bool should_reset_ = true;

  ::std::array<ZeroingEstimator, number_of_axes> estimators_;

 private:
  ::std::array<bool, number_of_axes> zeroed_;
};

template <typename ZeroingEstimator =
              ::frc::zeroing::PotAndIndexPulseZeroingEstimator,
          class Profile = aos::util::TrapezoidProfile>
class SingleDOFProfiledSubsystem
    : public ::frc::control_loops::ProfiledSubsystem<3, 1, ZeroingEstimator> {
 public:
  SingleDOFProfiledSubsystem(
      ::std::unique_ptr<SimpleCappedStateFeedbackLoop<3, 1, 1>> loop,
      const typename ZeroingEstimator::ZeroingConstants &zeroing_constants,
      const ::frc::constants::Range &range, double default_angular_velocity,
      double default_angular_acceleration);

  // Updates our estimator with the latest position.
  void Correct(const typename ZeroingEstimator::Position &position);
  // Runs the controller and profile generator for a cycle.  This is equivilent
  // to calling UpdateObserver(UpdateController()) with the rest of the syntax
  // actually right.
  double Update(bool disabled);
  // Just computes the controller and pushes the feed forwards forwards 1 step.
  double UpdateController(bool disabled);
  // Updates the observer with the computed U.
  // Note: if this is the only method called, ForceGoal should also be called to
  // move the state to match.
  void UpdateObserver(double voltage);

  // Fills out the ProfiledJointStatus structure with the current state.
  template <class StatusTypeBuilder>
  StatusTypeBuilder BuildStatus(flatbuffers::FlatBufferBuilder *fbb);

  // Forces the current goal to the provided goal, bypassing the profiler.
  void ForceGoal(double goal, double goal_velocity = 0.0);
  // Sets whether to use the trapezoidal profiler or whether to just bypass it
  // and pass the unprofiled goal through directly.
  void set_enable_profile(bool enable) { enable_profile_ = enable; }
  // Sets the unprofiled goal.  The profiler will generate a profile to go to
  // this goal.
  void set_unprofiled_goal(double unprofiled_goal,
                           double unprofiled_goal_velocity = 0.0,
                           bool print = false);
  // Limits our profiles to a max velocity and acceleration for proper motion.
  void AdjustProfile(const ::frc::ProfileParameters *profile_parameters);
  void AdjustProfile(double max_angular_velocity,
                     double max_angular_acceleration);

  // Returns true if we have exceeded any hard limits.
  bool CheckHardLimits();

  // Returns the requested voltage.
  double voltage() const { return this->loop_->U(0, 0); }

  // Returns the current position or velocity.
  double position() const { return this->Y_(0, 0); }

  // For testing:
  // Triggers an estimator error.
  void TriggerEstimatorError() { this->estimators_[0].TriggerError(); }

  const ::frc::constants::Range &range() const { return range_; }

  double default_velocity() const { return default_velocity_; }
  double default_acceleration() const { return default_acceleration_; }

  // Returns a pointer to the profile in use.
  Profile *mutable_profile() { return &profile_; }

 protected:
  // Limits the provided goal to the soft limits.  Prints "name" when it fails
  // to aid debugging.
  virtual void CapGoal(const char *name, Eigen::Matrix<double, 3, 1> *goal,
                       bool print = false);

 private:
  void UpdateOffset(double offset);

  Profile profile_;
  bool enable_profile_ = true;

  // Current measurement.
  Eigen::Matrix<double, 1, 1> Y_;
  // Current offset.  Y_ = offset_ + raw_sensor;
  Eigen::Matrix<double, 1, 1> offset_;

  const ::frc::constants::Range range_;

  const double default_velocity_;
  const double default_acceleration_;

  double last_position_ = 0;
};

namespace internal {

double UseUnlessZero(double target_value, double default_value);

}  // namespace internal

template <class ZeroingEstimator, class Profile>
SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::
    SingleDOFProfiledSubsystem(
        ::std::unique_ptr<SimpleCappedStateFeedbackLoop<3, 1, 1>> loop,
        const typename ZeroingEstimator::ZeroingConstants &zeroing_constants,
        const ::frc::constants::Range &range, double default_velocity,
        double default_acceleration)
    : ProfiledSubsystem<3, 1, ZeroingEstimator>(
          ::std::move(loop), {{ZeroingEstimator(zeroing_constants)}}),
      profile_(this->loop_->plant().coefficients().dt),
      range_(range),
      default_velocity_(default_velocity),
      default_acceleration_(default_acceleration) {
  Y_.setZero();
  offset_.setZero();
}

template <class ZeroingEstimator, class Profile>
void SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::UpdateOffset(
    double offset) {
  const double doffset = offset - offset_(0, 0);
  AOS_LOG(INFO, "Adjusting offset from %f to %f\n", offset_(0, 0), offset);

  this->loop_->mutable_X_hat()(0, 0) += doffset;
  this->Y_(0, 0) += doffset;
  last_position_ += doffset;
  this->loop_->mutable_R(0, 0) += doffset;

  profile_.MoveGoal(doffset);
  offset_(0, 0) = offset;

  CapGoal("R", &this->loop_->mutable_R());
}

template <class ZeroingEstimator, class Profile>
template <class StatusTypeBuilder>
StatusTypeBuilder
SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::BuildStatus(
    flatbuffers::FlatBufferBuilder *fbb) {
  flatbuffers::Offset<typename ZeroingEstimator::State> estimator_state =
      this->EstimatorState(fbb, 0);

  StatusTypeBuilder builder(*fbb);

  builder.add_zeroed(this->zeroed());
  // We don't know, so default to the bad case.

  builder.add_position(this->X_hat_(0, 0));
  builder.add_velocity(this->X_hat_(1, 0));
  builder.add_goal_position(this->goal(0, 0));
  builder.add_goal_velocity(this->goal(1, 0));
  builder.add_unprofiled_goal_position(this->unprofiled_goal(0, 0));
  builder.add_unprofiled_goal_velocity(this->unprofiled_goal(1, 0));
  builder.add_voltage_error(this->X_hat_(2, 0));
  builder.add_calculated_velocity(
      (position() - last_position_) /
      ::aos::time::DurationInSeconds(this->loop_->plant().coefficients().dt));

  builder.add_estimator_state(estimator_state);

  Eigen::Matrix<double, 3, 1> error = this->controller().error();
  builder.add_position_power(this->controller().controller().K(0, 0) *
                             error(0, 0));
  builder.add_velocity_power(this->controller().controller().K(0, 1) *
                             error(1, 0));
  return builder;
}

template <class ZeroingEstimator, class Profile>
void SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::Correct(
    const typename ZeroingEstimator::Position &new_position) {
  this->estimators_[0].UpdateEstimate(new_position);

  if (this->estimators_[0].error()) {
    AOS_LOG(ERROR, "zeroing error\n");
    this->X_hat_ = this->loop_->X_hat();
    return;
  }

  if (!this->initialized_) {
    if (this->estimators_[0].offset_ready()) {
      UpdateOffset(this->estimators_[0].offset());
      this->initialized_ = true;
    }
  }

  if (!this->zeroed(0) && this->estimators_[0].zeroed()) {
    UpdateOffset(this->estimators_[0].offset());
    this->set_zeroed(0, true);
  }

  last_position_ = position();
  this->Y_ << new_position.encoder();
  this->Y_ += this->offset_;
  this->loop_->Correct(Y_);
  this->X_hat_ = this->loop_->X_hat();
}

template <class ZeroingEstimator, class Profile>
void SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::CapGoal(
    const char *name, Eigen::Matrix<double, 3, 1> *goal, bool print) {
  // Limit the goal to min/max allowable positions.
  if ((*goal)(0, 0) > range_.upper) {
    if (print) {
      AOS_LOG(WARNING, "Goal %s above limit, %f > %f\n", name, (*goal)(0, 0),
              range_.upper);
    }
    (*goal)(0, 0) = range_.upper;
  }
  if ((*goal)(0, 0) < range_.lower) {
    if (print) {
      AOS_LOG(WARNING, "Goal %s below limit, %f < %f\n", name, (*goal)(0, 0),
              range_.lower);
    }
    (*goal)(0, 0) = range_.lower;
  }
}

template <class ZeroingEstimator, class Profile>
void SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::ForceGoal(
    double goal, double goal_velocity) {
  set_unprofiled_goal(goal, goal_velocity, false);
  this->loop_->mutable_R() = this->unprofiled_goal_;
  this->loop_->mutable_next_R() = this->loop_->R();

  const ::Eigen::Matrix<double, 3, 1> &R = this->loop_->R();
  this->profile_.MoveCurrentState(R.block<2, 1>(0, 0));
}

template <class ZeroingEstimator, class Profile>
void SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::set_unprofiled_goal(
    double unprofiled_goal, double unprofiled_goal_velocity, bool print) {
  this->unprofiled_goal_(0, 0) = unprofiled_goal;
  this->unprofiled_goal_(1, 0) = unprofiled_goal_velocity;
  this->unprofiled_goal_(2, 0) = 0.0;
  CapGoal("unprofiled R", &this->unprofiled_goal_, print);
}

template <class ZeroingEstimator, class Profile>
double SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::UpdateController(
    bool disable) {
  // TODO(austin): What do we want to do with the profile on reset?  Also, we
  // should probably reset R, the offset, the profile, etc.
  if (this->should_reset_) {
    this->loop_->mutable_X_hat(0, 0) = Y_(0, 0);
    this->loop_->mutable_X_hat(1, 0) = 0.0;
    this->loop_->mutable_X_hat(2, 0) = 0.0;
    this->X_hat_.setZero();
    this->should_reset_ = false;
  }

  if (!disable) {
    if (enable_profile_) {
      ::Eigen::Matrix<double, 2, 1> goal_state = profile_.Update(
          this->unprofiled_goal_(0, 0), this->unprofiled_goal_(1, 0));

      this->loop_->mutable_next_R(0, 0) = goal_state(0, 0);
      this->loop_->mutable_next_R(1, 0) = goal_state(1, 0);
      this->loop_->mutable_next_R(2, 0) = 0.0;
    } else {
      this->loop_->mutable_R() = this->unprofiled_goal_;
      this->loop_->mutable_next_R() = this->unprofiled_goal_;
      this->loop_->mutable_next_R(0, 0) +=
          this->unprofiled_goal_(1) *
          aos::time::DurationInSeconds(this->loop_->plant().coefficients().dt);
      CapGoal("R", &this->loop_->mutable_R());
    }
    CapGoal("next R", &this->loop_->mutable_next_R());
  }

  this->loop_->UpdateController(disable);

  if (!disable && this->loop_->U(0, 0) != this->loop_->U_uncapped(0, 0)) {
    const ::Eigen::Matrix<double, 3, 1> &R = this->loop_->R();
    profile_.MoveCurrentState(R.block<2, 1>(0, 0));
  }

  return this->loop_->U(0, 0);
}

template <class ZeroingEstimator, class Profile>
void SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::UpdateObserver(
    double voltage) {
  this->loop_->mutable_U(0, 0) = voltage;
  this->loop_->UpdateObserver(this->loop_->U(), this->loop_->plant().dt());
}

template <class ZeroingEstimator, class Profile>
double SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::Update(
    bool disable) {
  const double voltage = UpdateController(disable);
  UpdateObserver(voltage);
  return voltage;
}

template <class ZeroingEstimator, class Profile>
bool SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::CheckHardLimits() {
  // Returns whether hard limits have been exceeded.

  if (position() > range_.upper_hard || position() < range_.lower_hard) {
    AOS_LOG(
        ERROR,
        "SingleDOFProfiledSubsystem at %f out of bounds [%f, %f], ESTOPing\n",
        position(), range_.lower_hard, range_.upper_hard);
    return true;
  }

  return false;
}

template <class ZeroingEstimator, class Profile>
void SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::AdjustProfile(
    const ::frc::ProfileParameters *profile_parameters) {
  AdjustProfile(
      profile_parameters != nullptr ? profile_parameters->max_velocity() : 0.0,
      profile_parameters != nullptr ? profile_parameters->max_acceleration()
                                    : 0.0);
}

template <class ZeroingEstimator, class Profile>
void SingleDOFProfiledSubsystem<ZeroingEstimator, Profile>::AdjustProfile(
    double max_angular_velocity, double max_angular_acceleration) {
  profile_.set_maximum_velocity(
      internal::UseUnlessZero(max_angular_velocity, default_velocity_));
  profile_.set_maximum_acceleration(
      internal::UseUnlessZero(max_angular_acceleration, default_acceleration_));
}

}  // namespace frc::control_loops

#endif  // FRC_CONTROL_LOOPS_PROFILED_SUBSYSTEM_H_
