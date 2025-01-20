#ifndef FRC_CONTROL_LOOPS_DRIVETRAIN_HYBRID_EKF_H_
#define FRC_CONTROL_LOOPS_DRIVETRAIN_HYBRID_EKF_H_

#include <chrono>
#include <optional>

#include "Eigen/Dense"

#include "aos/commonmath.h"
#include "aos/containers/priority_queue.h"
#include "aos/util/math.h"
#include "frc/control_loops/c2d.h"
#include "frc/control_loops/drivetrain/drivetrain_config.h"
#include "frc/control_loops/runge_kutta.h"

namespace y2019::control_loops::testing {
class ParameterizedLocalizerTest;
}  // namespace y2019::control_loops::testing

namespace frc::control_loops::drivetrain {

namespace testing {
class HybridEkfTest;
}

// HybridEkf is an EKF for use in robot localization. It is currently
// coded for use with drivetrains in particular, and so the states and inputs
// are chosen as such.
// The "Hybrid" part of the name refers to the fact that it can take in
// measurements with variable time-steps.
// measurements can also have been taken in the past and we maintain a buffer
// so that we can replay the kalman filter whenever we get an old measurement.
// Currently, this class provides the necessary utilities for doing
// measurement updates with an encoder/gyro as well as a more generic
// update function that can be used for arbitrary nonlinear updates (presumably
// a camera update).
//
// Discussion of the model:
// In the current model, we try to rely primarily on IMU measurements for
// estimating robot state--we also need additional information (some combination
// of output voltages, encoders, and camera data) to help eliminate the biases
// that can accumulate due to integration of IMU data.
// We use IMU measurements as inputs rather than measurement outputs because
// that seemed to be easier to implement. I tried initially running with
// the IMU as a measurement, but it seemed to blow up the complexity of the
// model.
//
// On each prediction update, we take in inputs of the left/right voltages and
// the current measured longitudinal/lateral accelerations. In the current
// setup, the accelerometer readings will be used for estimating how the
// evolution of the longitudinal/lateral velocities. The voltages (and voltage
// errors) will solely be used for estimating the current rotational velocity of
// the robot (I do this because currently I suspect that the accelerometer is a
// much better indicator of current robot state than the voltages). We also
// deliberately decay all of the velocity estimates towards zero to help address
// potential accelerometer biases. We use two separate decay models:
// -The longitudinal velocity is modelled as decaying at a constant rate (see
//  the documentation on the VelocityAccel() method)--this needs a more
//  complex model because the robot will, under normal circumstances, be
//  travelling at non-zero velocities.
// -The lateral velocity is modelled as exponentially decaying towards zero.
//  This is simpler to model and should be reasonably valid, since we will
//  not *normally* be travelling sideways consistently (this assumption may
//  need to be revisited).
// -The "longitudinal velocity offset" (described below) also uses an
//  exponential decay, albeit with a different time constant. A future
//  improvement may remove the decay modelling on the longitudinal velocity
//  itself and instead use that decay model on the longitudinal velocity offset.
//  This would place a bit more trust in the encoder measurements but also
//  more correctly model situations where the robot is legitimately moving at
//  a certain velocity.
//
// For modelling how the drivetrain encoders evolve, and to help prevent the
// aforementioned decay functions from affecting legitimate high-velocity
// maneuvers too much, we have a "longitudinal velocity offset" term. This term
// models the difference between the actual longitudinal velocity of the robot
// (estimated by the average of the left/right velocities) and the velocity
// experienced by the wheels (which can be observed from the encoders more
// directly). Because we model this velocity offset as decaying towards zero,
// what this will do is allow the encoders to be a constant velocity off from
// the accelerometer updates for short periods of time but then gradually
// pull the "actual" longitudinal velocity offset towards that of the encoders,
// helping to reduce constant biases.
template <typename Scalar = double>
class HybridEkf {
 public:
  // An enum specifying what each index in the state vector is for.
  enum StateIdx {
    // Current X/Y position, in meters, of the robot.
    kX = 0,
    kY = 1,
    // Current heading of the robot.
    kTheta = 2,
    // Current estimated encoder reading of the left wheels, in meters.
    // Rezeroed once on startup.
    kLeftEncoder = 3,
    // Current estimated actual velocity of the left side of the robot, in m/s.
    kLeftVelocity = 4,
    // Same variables, for the right side of the robot.
    kRightEncoder = 5,
    kRightVelocity = 6,
    // Estimated offset to input voltage. Used as a generic error term, Volts.
    kLeftVoltageError = 7,
    kRightVoltageError = 8,
    // These error terms are used to estimate the difference between the actual
    // movement of the drivetrain and that implied by the wheel odometry.
    // Angular error effectively estimates a constant angular rate offset of the
    // encoders relative to the actual rotation of the robot.
    // Semi-arbitrary units (we don't bother accounting for robot radius in
    // this).
    kAngularError = 9,
    // Estimate of slip between the drivetrain wheels and the actual
    // forwards/backwards velocity of the robot, in m/s.
    // I.e., (left velocity + right velocity) / 2.0 = (left wheel velocity +
    //        right wheel velocity) / 2.0 + longitudinal velocity offset
    kLongitudinalVelocityOffset = 10,
    // Current estimate of the lateral velocity of the robot, in m/s.
    // Positive implies the robot is moving to its left.
    kLateralVelocity = 11,
  };
  static constexpr int kNStates = 12;
  enum InputIdx {
    // Left/right drivetrain voltages.
    kLeftVoltage = 0,
    kRightVoltage = 1,
    // Current accelerometer readings, in m/s/s, along the longitudinal and
    // lateral axes of the robot. Should be projected onto the X/Y plane, to
    // compensate for tilt of the robot before being passed to this filter. The
    // HybridEkf has no knowledge of the current pitch/roll of the robot, and so
    // can't do anything to compensate for it.
    kLongitudinalAccel = 2,
    kLateralAccel = 3,
  };

  static constexpr int kNInputs = 4;
  // Number of previous samples to save.
  static constexpr int kSaveSamples = 200;
  // Whether we should completely rerun the entire stored history of
  // kSaveSamples on every correction. Enabling this will increase overall CPU
  // usage substantially; however, leaving it disabled makes it so that we are
  // less likely to notice if processing camera frames is causing delays in the
  // drivetrain.
  // If we are having CPU issues, we have three easy avenues to improve things:
  // (1) Reduce kSaveSamples (e.g., if all camera frames arive within
  //     100 ms, then we can reduce kSaveSamples to be 25 (125 ms of samples)).
  // (2) Don't actually rely on the ability to insert corrections into the
  //     timeline.
  // (3) Set this to false.
  static constexpr bool kFullRewindOnEverySample = false;
  // Assume that all correction steps will have kNOutputs
  // dimensions.
  // TODO(james): Relax this assumption; relaxing it requires
  // figuring out how to deal with storing variable size
  // observation matrices, though.
  static constexpr int kNOutputs = 3;
  // Time constant to use for estimating how the longitudinal/lateral velocity
  // offsets decay, in seconds.
  static constexpr double kVelocityOffsetTimeConstant = 1.0;
  static constexpr double kLateralVelocityTimeConstant = 1.0;

  // The maximum allowable timestep--we use this to check for situations where
  // measurement updates come in too infrequently and this might cause the
  // integrator and discretization in the prediction step to be overly
  // aggressive.
  static constexpr std::chrono::milliseconds kMaxTimestep{20};
  // Inputs are [left_volts, right_volts]
  typedef Eigen::Matrix<Scalar, kNInputs, 1> Input;
  // Outputs are either:
  // [left_encoder, right_encoder, gyro_vel]; or [heading, distance, skew] to
  // some target. This makes it so we don't have to figure out how we store
  // variable-size measurement updates.
  typedef Eigen::Matrix<Scalar, kNOutputs, 1> Output;
  typedef Eigen::Matrix<Scalar, kNStates, kNStates> StateSquare;
  // State contains the states defined by the StateIdx enum. See comments there.
  typedef Eigen::Matrix<Scalar, kNStates, 1> State;

  // The following classes exist to allow us to support doing corections in the
  // past by rewinding the EKF, calling the appropriate H and dhdx functions,
  // and then playing everything back. Originally, this simply used
  // std::function's, but doing so causes us to perform dynamic memory
  // allocation in the core of the drivetrain control loop.
  //
  // The ExpectedObservationFunctor class serves to provide an interface for the
  // actual H and dH/dX that the EKF itself needs. Most implementations end up
  // just using this; in the degenerate case, ExpectedObservationFunctor could
  // be implemented as a class that simply stores two std::functions and calls
  // them when H() and DHDX() are called.
  //
  // The ObserveDeletion() and deleted() methods exist for sanity checking--we
  // don't rely on them to do any work, but in order to ensure that memory is
  // being managed correctly, we have the HybridEkf call ObserveDeletion() when
  // it no longer needs an instance of the object.
  class ExpectedObservationFunctor {
   public:
    virtual ~ExpectedObservationFunctor() = default;
    // Return the expected measurement of the system for a given state and plant
    // input.
    virtual Output H(const State &state, const Input &input) = 0;
    // Return the derivative of H() with respect to the state, given the current
    // state.
    virtual Eigen::Matrix<Scalar, kNOutputs, kNStates> DHDX(
        const State &state) = 0;
    virtual void ObserveDeletion() {
      CHECK(!deleted_);
      deleted_ = true;
    }
    bool deleted() const { return deleted_; }

   private:
    bool deleted_ = false;
  };

  // The ExpectedObservationBuilder creates a new ExpectedObservationFunctor.
  // This is used for situations where in order to know what the correction
  // methods even are we need to know the state at some time in the past. This
  // is only used in the y2019 code and we've generally stopped using this
  // pattern.
  class ExpectedObservationBuilder {
   public:
    virtual ~ExpectedObservationBuilder() = default;
    // The lifetime of the returned object should last at least until
    // ObserveDeletion() is called on said object.
    virtual ExpectedObservationFunctor *MakeExpectedObservations(
        const State &state, const StateSquare &P) = 0;
    void ObserveDeletion() {
      CHECK(!deleted_);
      deleted_ = true;
    }
    bool deleted() const { return deleted_; }

   private:
    bool deleted_ = false;
  };

  // The ExpectedObservationAllocator provides a utility class which manages the
  // memory for a single type of correction step for a given localizer.
  // Using the knowledge that at most kSaveSamples ExpectedObservation* objects
  // can be referenced by the HybridEkf at any given time, this keeps an
  // internal queue that more than mirrors the HybridEkf's internal queue, using
  // the oldest spots in the queue to construct new ExpectedObservation*'s.
  // This can be used with T as either a ExpectedObservationBuilder or
  // ExpectedObservationFunctor. The appropriate Correct function will then be
  // called in place of calling HybridEkf::Correct directly. Note that unless T
  // implements both the Builder and Functor (which is generally discouraged),
  // only one of the Correct* functions will build.
  template <typename T>
  class ExpectedObservationAllocator {
   public:
    ExpectedObservationAllocator(HybridEkf *ekf) : ekf_(ekf) {}
    void CorrectKnownH(const Output &z, const Input *U, T H,
                       const Eigen::Matrix<Scalar, kNOutputs, kNOutputs> &R,
                       aos::monotonic_clock::time_point t) {
      if (functors_.full()) {
        CHECK(functors_.begin()->functor->deleted());
      }
      auto pushed = functors_.PushFromBottom(Pair{t, std::move(H)});
      if (pushed == functors_.end()) {
        VLOG(1) << "Observation dropped off bottom of queue.";
        return;
      }
      ekf_->Correct(z, U, nullptr, &pushed->functor.value(), R, t);
    }
    void CorrectKnownHBuilder(
        const Output &z, const Input *U, T builder,
        const Eigen::Matrix<Scalar, kNOutputs, kNOutputs> &R,
        aos::monotonic_clock::time_point t) {
      if (functors_.full()) {
        CHECK(functors_.begin()->functor->deleted());
      }
      auto pushed = functors_.PushFromBottom(Pair{t, std::move(builder)});
      if (pushed == functors_.end()) {
        VLOG(1) << "Observation dropped off bottom of queue.";
        return;
      }
      ekf_->Correct(z, U, &pushed->functor.value(), nullptr, R, t);
    }

   private:
    struct Pair {
      aos::monotonic_clock::time_point t;
      std::optional<T> functor;
      friend bool operator<(const Pair &l, const Pair &r) { return l.t < r.t; }
    };

    HybridEkf *const ekf_;
    aos::PriorityQueue<Pair, kSaveSamples + 1, std::less<Pair>> functors_;
  };

  // A simple implementation of ExpectedObservationFunctor for an LTI correction
  // step. Does not store any external references, so overrides
  // ObserveDeletion() to do nothing.
  class LinearH : public ExpectedObservationFunctor {
   public:
    LinearH(const Eigen::Matrix<Scalar, kNOutputs, kNStates> &H) : H_(H) {}
    virtual ~LinearH() = default;
    Output H(const State &state, const Input &) final { return H_ * state; }
    Eigen::Matrix<Scalar, kNOutputs, kNStates> DHDX(const State &) final {
      return H_;
    }
    void ObserveDeletion() {}

   private:
    const Eigen::Matrix<Scalar, kNOutputs, kNStates> H_;
  };

  // Constructs a HybridEkf for a particular drivetrain.
  // Currently, we use the drivetrain config for modelling constants
  // (continuous time A and B matrices) and for the noise matrices for the
  // encoders/gyro.
  // If force_dt is set, then all predict steps will use a dt of force_dt.
  // This can be used in situations where there is no reliable clock guiding
  // the measurement updates, but the source is coming in at a reasonably
  // consistent period.
  HybridEkf(const DrivetrainConfig<double> &dt_config,
            std::optional<std::chrono::nanoseconds> force_dt = std::nullopt)
      : dt_config_(dt_config),
        velocity_drivetrain_coefficients_(
            dt_config.make_hybrid_drivetrain_velocity_loop()
                .plant()
                .coefficients()),
        force_dt_(force_dt) {
    InitializeMatrices();
  }

  // Set the initial guess of the state. Can only be called once, and before
  // any measurement updates have occurred.
  void ResetInitialState(::aos::monotonic_clock::time_point t,
                         const State &state, const StateSquare &P) {
    observations_.clear();
    X_hat_ = state;
    P_ = P;
    observations_.PushFromBottom({
        t,
        t,
        X_hat_,
        P_,
        Input::Zero(),
        Output::Zero(),
        nullptr,
        &H_encoders_and_gyro_.value(),
        Eigen::Matrix<Scalar, kNOutputs, kNOutputs>::Identity(),
        StateSquare::Identity(),
        StateSquare::Zero(),
        std::chrono::seconds(0),
        State::Zero(),
    });
  }

  // Correct with:
  // A measurement z at time t with z = h(X_hat, U) + v where v has noise
  // covariance R.
  // Input U is applied from the previous timestep until time t.
  // If t is later than any previous measurements, then U must be provided.
  // If the measurement falls between two previous measurements, then U
  // can be provided or not; if U is not provided, then it is filled in based
  // on an assumption that the voltage was held constant between the time steps.
  // TODO(james): Is it necessary to explicitly to provide a version with H as a
  // matrix for linear cases?
  void Correct(const Output &z, const Input *U,
               ExpectedObservationBuilder *observation_builder,
               ExpectedObservationFunctor *expected_observations,
               const Eigen::Matrix<Scalar, kNOutputs, kNOutputs> &R,
               aos::monotonic_clock::time_point t);

  // A utility function for specifically updating with encoder and gyro
  // measurements.
  void UpdateEncodersAndGyro(const std::optional<Scalar> left_encoder,
                             const std::optional<Scalar> right_encoder,
                             const Scalar gyro_rate,
                             const Eigen::Matrix<Scalar, 2, 1> &voltage,
                             const Eigen::Matrix<Scalar, 3, 1> &accel,
                             aos::monotonic_clock::time_point t) {
    Input U;
    U.template block<2, 1>(0, 0) = voltage;
    U.template block<2, 1>(kLongitudinalAccel, 0) =
        accel.template block<2, 1>(0, 0);
    RawUpdateEncodersAndGyro(left_encoder, right_encoder, gyro_rate, U, t);
  }
  // Version of UpdateEncodersAndGyro that takes a input matrix rather than
  // taking in a voltage/acceleration separately.
  void RawUpdateEncodersAndGyro(const std::optional<Scalar> left_encoder,
                                const std::optional<Scalar> right_encoder,
                                const Scalar gyro_rate, const Input &U,
                                aos::monotonic_clock::time_point t) {
    // Because the check below for have_zeroed_encoders_ will add an
    // Observation, do a check here to ensure that initialization has been
    // performed and so there is at least one observation.
    CHECK(!observations_.empty());
    if (!have_zeroed_encoders_) {
      // This logic handles ensuring that on the first encoder reading, we
      // update the internal state for the encoders to match the reading.
      // Otherwise, if we restart the drivetrain without restarting
      // wpilib_interface, then we can get some obnoxious initial corrections
      // that mess up the localization.
      State newstate = X_hat_;
      have_zeroed_encoders_ = true;
      if (left_encoder.has_value()) {
        newstate(kLeftEncoder) = left_encoder.value();
      } else {
        have_zeroed_encoders_ = false;
      }
      if (right_encoder.has_value()) {
        newstate(kRightEncoder) = right_encoder.value();
      } else {
        have_zeroed_encoders_ = false;
      }
      newstate(kLeftVoltageError) = 0.0;
      newstate(kRightVoltageError) = 0.0;
      newstate(kAngularError) = 0.0;
      newstate(kLongitudinalVelocityOffset) = 0.0;
      newstate(kLateralVelocity) = 0.0;
      ResetInitialState(t, newstate, P_);
    }

    Output z(left_encoder.value_or(0.0), right_encoder.value_or(0.0),
             gyro_rate);

    Eigen::Matrix<Scalar, kNOutputs, kNOutputs> R;
    R.setZero();
    R.diagonal() << encoder_noise_, encoder_noise_, gyro_noise_;
    CHECK(H_encoders_and_gyro_.has_value());
    CHECK(H_gyro_only_.has_value());
    LinearH *H = &H_encoders_and_gyro_.value();
    if (!left_encoder.has_value() || !right_encoder.has_value()) {
      H = &H_gyro_only_.value();
    }
    Correct(z, &U, nullptr, H, R, t);
  }

  // Sundry accessor:
  State X_hat() const { return X_hat_; }
  Scalar X_hat(long i) const { return X_hat_(i); }
  StateSquare P() const { return P_; }
  aos::monotonic_clock::time_point latest_t() const {
    return observations_.top().t;
  }

  static Scalar CalcLongitudinalVelocity(const State &X) {
    return (X(kLeftVelocity) + X(kRightVelocity)) / 2.0;
  }

  Scalar CalcYawRate(const State &X) const {
    return (X(kRightVelocity) - X(kLeftVelocity)) / 2.0 /
           dt_config_.robot_radius;
  }

  // Returns the last state before the specified time.
  // Returns nullopt if time is older than the oldest measurement.
  std::optional<State> LastStateBeforeTime(
      aos::monotonic_clock::time_point time) {
    if (observations_.empty() || observations_.begin()->t > time) {
      return std::nullopt;
    }
    for (const auto &observation : observations_) {
      if (observation.t > time) {
        // Note that observation.X_hat actually references the _previous_ X_hat.
        return observation.X_hat;
      }
    }
    return X_hat();
  }
  std::optional<State> OldestState() {
    if (observations_.empty()) {
      return std::nullopt;
    }
    return observations_.begin()->X_hat;
  }

  // Returns the most recent input vector.
  Input MostRecentInput() {
    CHECK(!observations_.empty());
    Input U = observations_.top().U;
    return U;
  }

  void set_ignore_accel(bool ignore_accel) { ignore_accel_ = ignore_accel; }

 private:
  struct Observation {
    Observation(aos::monotonic_clock::time_point t,
                aos::monotonic_clock::time_point prev_t, State X_hat,
                StateSquare P, Input U, Output z,
                ExpectedObservationBuilder *make_h,
                ExpectedObservationFunctor *h,
                Eigen::Matrix<Scalar, kNOutputs, kNOutputs> R, StateSquare A_d,
                StateSquare Q_d,
                aos::monotonic_clock::duration discretization_time,
                State predict_update)
        : t(t),
          prev_t(prev_t),
          X_hat(X_hat),
          P(P),
          U(U),
          z(z),
          make_h(make_h),
          h(h),
          R(R),
          A_d(A_d),
          Q_d(Q_d),
          discretization_time(discretization_time),
          predict_update(predict_update) {}
    Observation(const Observation &) = delete;
    Observation &operator=(const Observation &) = delete;
    // Move-construct an observation by copying over the contents of the struct
    // and then clearing the old Observation's pointers so that it doesn't try
    // to clean things up.
    Observation(Observation &&o)
        : Observation(o.t, o.prev_t, o.X_hat, o.P, o.U, o.z, o.make_h, o.h, o.R,
                      o.A_d, o.Q_d, o.discretization_time, o.predict_update) {
      o.make_h = nullptr;
      o.h = nullptr;
    }
    Observation &operator=(Observation &&observation) = delete;
    ~Observation() {
      // Observe h being deleted first, since make_h may own its memory.
      // Shouldn't actually matter, though.
      if (h != nullptr) {
        h->ObserveDeletion();
      }
      if (make_h != nullptr) {
        make_h->ObserveDeletion();
      }
    }
    // Time when the observation was taken.
    aos::monotonic_clock::time_point t;
    // Time that the previous observation was taken:
    aos::monotonic_clock::time_point prev_t;
    // Estimate of state at previous observation time t, after accounting for
    // the previous observation.
    State X_hat;
    // Noise matrix corresponding to X_hat_.
    StateSquare P;
    // The input applied from previous observation until time t.
    Input U;
    // Measurement taken at that time.
    Output z;
    // A function to create h and dhdx from a given position/covariance
    // estimate. This is used by the camera to make it so that we only have to
    // match targets once.
    // Only called if h and dhdx are empty.
    ExpectedObservationBuilder *make_h = nullptr;
    // A function to calculate the expected output at a given state/input.
    // TODO(james): For encoders/gyro, it is linear and the function call may
    // be expensive. Potential source of optimization.
    ExpectedObservationFunctor *h = nullptr;
    // The measurement noise matrix.
    Eigen::Matrix<Scalar, kNOutputs, kNOutputs> R;

    // Discretized A and Q to use on this update step. These will only be
    // recalculated if the timestep changes.
    StateSquare A_d;
    StateSquare Q_d;
    aos::monotonic_clock::duration discretization_time;

    // A cached value indicating how much we change X_hat in the prediction step
    // of this Observation.
    State predict_update;

    // In order to sort the observations in the PriorityQueue object, we
    // need a comparison function.
    friend bool operator<(const Observation &l, const Observation &r) {
      return l.t < r.t;
    }
  };

  void InitializeMatrices();

  // These constants and functions define how the longitudinal velocity
  // (the average of the left and right velocities) decays. We model it as
  // decaying at a constant rate, except very near zero where the decay rate is
  // exponential (this is more numerically stable than just using a constant
  // rate the whole time). We use this model rather than a simpler exponential
  // decay because an exponential decay will result in the robot's velocity
  // estimate consistently being far too low when at high velocities, and since
  // the acceleromater-based estimate of the velocity will only drift at a
  // relatively slow rate and doesn't get worse at higher velocities, we can
  // safely decay pretty slowly.
  static constexpr double kMaxVelocityAccel = 0.005;
  static constexpr double kMaxVelocityGain = 1.0;
  static Scalar VelocityAccel(Scalar velocity) {
    return -std::clamp(kMaxVelocityGain * velocity, -kMaxVelocityAccel,
                       kMaxVelocityAccel);
  }

  static Scalar VelocityAccelDiff(Scalar velocity) {
    return (std::abs(kMaxVelocityGain * velocity) > kMaxVelocityAccel)
               ? 0.0
               : -kMaxVelocityGain;
  }

  // Returns the "A" matrix for a given state. See DiffEq for discussion of
  // ignore_accel.
  StateSquare AForState(const State &X, bool ignore_accel) const {
    // Calculate the A matrix for a given state. Note that A = partial Xdot /
    // partial X. This is distinct from saying that Xdot = A * X. This is
    // particularly relevant for the (kX, kTheta) members which otherwise seem
    // odd.
    StateSquare A_continuous = A_continuous_;
    const Scalar theta = X(kTheta);
    const Scalar stheta = std::sin(theta);
    const Scalar ctheta = std::cos(theta);
    const Scalar lng_vel = CalcLongitudinalVelocity(X);
    const Scalar lat_vel = X(kLateralVelocity);
    const Scalar diameter = 2.0 * dt_config_.robot_radius;
    const Scalar yaw_rate = CalcYawRate(X);
    // X and Y derivatives
    A_continuous(kX, kTheta) = -stheta * lng_vel - ctheta * lat_vel;
    A_continuous(kX, kLeftVelocity) = ctheta / 2.0;
    A_continuous(kX, kRightVelocity) = ctheta / 2.0;
    A_continuous(kX, kLateralVelocity) = -stheta;
    A_continuous(kY, kTheta) = ctheta * lng_vel - stheta * lat_vel;
    A_continuous(kY, kLeftVelocity) = stheta / 2.0;
    A_continuous(kY, kRightVelocity) = stheta / 2.0;
    A_continuous(kY, kLateralVelocity) = ctheta;

    if (!ignore_accel) {
      const Eigen::Matrix<Scalar, 1, kNStates> lng_vel_row =
          (A_continuous.row(kLeftVelocity) + A_continuous.row(kRightVelocity)) /
          2.0;
      A_continuous.row(kLeftVelocity) -= lng_vel_row;
      A_continuous.row(kRightVelocity) -= lng_vel_row;
      // Terms to account for centripetal accelerations.
      // lateral centripetal accel = -yaw_rate * lng_vel
      A_continuous(kLateralVelocity, kLeftVelocity) +=
          X(kLeftVelocity) / diameter;
      A_continuous(kLateralVelocity, kRightVelocity) +=
          -X(kRightVelocity) / diameter;
      A_continuous(kRightVelocity, kLateralVelocity) += yaw_rate;
      A_continuous(kLeftVelocity, kLateralVelocity) += yaw_rate;
      const Scalar dlng_accel_dwheel_vel = X(kLateralVelocity) / diameter;
      A_continuous(kRightVelocity, kRightVelocity) += dlng_accel_dwheel_vel;
      A_continuous(kLeftVelocity, kRightVelocity) += dlng_accel_dwheel_vel;
      A_continuous(kRightVelocity, kLeftVelocity) += -dlng_accel_dwheel_vel;
      A_continuous(kLeftVelocity, kLeftVelocity) += -dlng_accel_dwheel_vel;

      A_continuous(kRightVelocity, kRightVelocity) +=
          VelocityAccelDiff(lng_vel) / 2.0;
      A_continuous(kRightVelocity, kLeftVelocity) +=
          VelocityAccelDiff(lng_vel) / 2.0;
      A_continuous(kLeftVelocity, kRightVelocity) +=
          VelocityAccelDiff(lng_vel) / 2.0;
      A_continuous(kLeftVelocity, kLeftVelocity) +=
          VelocityAccelDiff(lng_vel) / 2.0;
    }
    return A_continuous;
  }

  // Returns dX / dt given X and U. If ignore_accel is set, then we ignore the
  // accelerometer-based components of U (this is solely used in testing).
  State DiffEq(const State &X, const Input &U, bool ignore_accel) const {
    State Xdot = A_continuous_ * X + B_continuous_ * U;
    // And then we need to add on the terms for the x/y change:
    const Scalar theta = X(kTheta);
    const Scalar lng_vel = CalcLongitudinalVelocity(X);
    const Scalar lat_vel = X(kLateralVelocity);
    const Scalar stheta = std::sin(theta);
    const Scalar ctheta = std::cos(theta);
    Xdot(kX) = ctheta * lng_vel - stheta * lat_vel;
    Xdot(kY) = stheta * lng_vel + ctheta * lat_vel;

    const Scalar yaw_rate = CalcYawRate(X);
    const Scalar expected_lat_accel = lng_vel * yaw_rate;
    const Scalar expected_lng_accel =
        CalcLongitudinalVelocity(Xdot) - yaw_rate * lat_vel;
    const Scalar lng_accel_offset = U(kLongitudinalAccel) - expected_lng_accel;
    constexpr double kAccelWeight = 1.0;
    if (!ignore_accel) {
      Xdot(kLeftVelocity) += kAccelWeight * lng_accel_offset;
      Xdot(kRightVelocity) += kAccelWeight * lng_accel_offset;
      Xdot(kLateralVelocity) += U(kLateralAccel) - expected_lat_accel;

      Xdot(kRightVelocity) += VelocityAccel(lng_vel);
      Xdot(kLeftVelocity) += VelocityAccel(lng_vel);
    }
    return Xdot;
  }

  void PredictImpl(Observation *obs, std::chrono::nanoseconds dt, State *state,
                   StateSquare *P) {
    if (force_dt_.has_value()) {
      dt = force_dt_.value();
    }
    // Only recalculate the discretization if the timestep has changed.
    // Technically, this isn't quite correct, since the discretization will
    // change depending on the current state. However, the slight loss of
    // precision seems acceptable for the sake of significantly reducing CPU
    // usage.
    if (obs->discretization_time != dt) {
      // TODO(james): By far the biggest CPU sink in the localization appears to
      // be this discretization--it's possible the spline code spikes higher,
      // but it doesn't create anywhere near the same sustained load. There
      // are a few potential options for optimizing this code, but none of
      // them are entirely trivial, e.g. we could:
      // -Reduce the number of states (this function grows at O(kNStates^3))
      // -Adjust the discretization function itself (there're a few things we
      //  can tune there).
      // -Try to come up with some sort of lookup table or other way of
      //  pre-calculating A_d and Q_d.
      // I also have to figure out how much we care about the precision of
      // some of these values--I don't think we care much, but we probably
      // do want to maintain some of the structure of the matrices.
      const StateSquare A_c = AForState(*state, ignore_accel_);
      controls::DiscretizeQAFast(Q_continuous_, A_c, dt, &obs->Q_d, &obs->A_d);
      obs->discretization_time = dt;

      obs->predict_update =
          RungeKuttaU(
              [this](const State &X, const Input &U) {
                return DiffEq(X, U, ignore_accel_);
              },
              *state, obs->U, aos::time::DurationInSeconds(dt)) -
          *state;
    }

    *state += obs->predict_update;

    StateSquare Ptemp = obs->A_d * *P * obs->A_d.transpose() + obs->Q_d;
    *P = Ptemp;
  }

  void CorrectImpl(Observation *obs, State *state, StateSquare *P) {
    const Eigen::Matrix<Scalar, kNOutputs, kNStates> H = obs->h->DHDX(*state);
    // Note: Technically, this does calculate P * H.transpose() twice. However,
    // when I was mucking around with some things, I found that in practice
    // putting everything into one expression and letting Eigen optimize it
    // directly actually improved performance relative to precalculating P *
    // H.transpose().
    const Eigen::Matrix<Scalar, kNStates, kNOutputs> K =
        *P * H.transpose() * (H * *P * H.transpose() + obs->R).inverse();
    const StateSquare Ptemp = (StateSquare::Identity() - K * H) * *P;
    *P = Ptemp;
    *state += K * (obs->z - obs->h->H(*state, obs->U));
  }

  void ProcessObservation(Observation *obs, const std::chrono::nanoseconds dt,
                          State *state, StateSquare *P) {
    *state = obs->X_hat;
    *P = obs->P;
    if (dt.count() != 0 && dt < kMaxTimestep) {
      PredictImpl(obs, dt, state, P);
    }
    if (obs->h == nullptr) {
      CHECK(obs->make_h != nullptr);
      obs->h = obs->make_h->MakeExpectedObservations(*state, *P);
      CHECK(obs->h != nullptr);
    }
    CorrectImpl(obs, state, P);
  }

  DrivetrainConfig<double> dt_config_;
  State X_hat_;
  StateFeedbackHybridPlantCoefficients<2, 2, 2, double>
      velocity_drivetrain_coefficients_;
  std::optional<std::chrono::nanoseconds> force_dt_;
  StateSquare A_continuous_;
  StateSquare Q_continuous_;
  StateSquare P_;
  std::optional<LinearH> H_encoders_and_gyro_;
  std::optional<LinearH> H_gyro_only_;
  Scalar encoder_noise_, gyro_noise_;
  Eigen::Matrix<Scalar, kNStates, kNInputs> B_continuous_;

  bool have_zeroed_encoders_ = false;

  // Whether to pay attention to accelerometer readings to compensate for wheel
  // slip.
  bool ignore_accel_ = false;

  aos::PriorityQueue<Observation, kSaveSamples, std::less<Observation>>
      observations_;

  friend class testing::HybridEkfTest;
  friend class y2019::control_loops::testing::ParameterizedLocalizerTest;
};  // class HybridEkf

template <typename Scalar>
void HybridEkf<Scalar>::Correct(
    const Output &z, const Input *U,
    ExpectedObservationBuilder *observation_builder,
    ExpectedObservationFunctor *expected_observations,
    const Eigen::Matrix<Scalar, kNOutputs, kNOutputs> &R,
    aos::monotonic_clock::time_point t) {
  CHECK(!observations_.empty());
  if (!observations_.full() && t < observations_.begin()->t) {
    AOS_LOG(ERROR,
            "Dropped an observation that was received before we "
            "initialized.\n");
    return;
  }
  auto cur_it = observations_.PushFromBottom(
      {t, t, State::Zero(), StateSquare::Zero(), Input::Zero(), z,
       observation_builder, expected_observations, R, StateSquare::Identity(),
       StateSquare::Zero(), std::chrono::seconds(0), State::Zero()});
  if (cur_it == observations_.end()) {
    VLOG(1) << "Camera dropped off of end with time of "
            << aos::time::DurationInSeconds(t.time_since_epoch())
            << "s; earliest observation in "
               "queue has time of "
            << aos::time::DurationInSeconds(
                   observations_.begin()->t.time_since_epoch())
            << "s.\n";
    return;
  }
  // Now we populate any state information that depends on where the
  // observation was inserted into the queue. X_hat and P must be populated
  // from the values present in the observation *following* this one in
  // the queue (note that the X_hat and P that we store in each observation
  // is the values that they held after accounting for the previous
  // measurement and before accounting for the time between the previous and
  // current measurement). If we appended to the end of the queue, then
  // we need to pull from X_hat_ and P_ specifically.
  // Furthermore, for U:
  // -If the observation was inserted at the end, then the user must've
  //  provided U and we use it.
  // -Otherwise, only grab U if necessary.
  auto next_it = cur_it;
  ++next_it;
  if (next_it == observations_.end()) {
    cur_it->X_hat = X_hat_;
    cur_it->P = P_;
    // Note that if next_it == observations_.end(), then because we already
    // checked for !observations_.empty(), we are guaranteed to have
    // valid prev_it.
    auto prev_it = cur_it;
    --prev_it;
    cur_it->prev_t = prev_it->t;
    // TODO(james): Figure out a saner way of handling this.
    CHECK(U != nullptr);
    cur_it->U = *U;
  } else {
    cur_it->X_hat = next_it->X_hat;
    cur_it->P = next_it->P;
    cur_it->prev_t = next_it->prev_t;
    next_it->prev_t = cur_it->t;
    cur_it->U = (U == nullptr) ? next_it->U : *U;
  }

  if (kFullRewindOnEverySample) {
    next_it = observations_.begin();
    cur_it = next_it++;
  }

  // Now we need to rerun the predict step from the previous to the new
  // observation as well as every following correct/predict up to the current
  // time.
  while (true) {
    // We use X_hat_ and P_ to store the intermediate states, and then
    // once we reach the end they will all be up-to-date.
    ProcessObservation(&*cur_it, cur_it->t - cur_it->prev_t, &X_hat_, &P_);
    // TOOD(james): Note that this can be triggered when there are extremely
    // small values in P_. This is particularly likely if Scalar is just float
    // and we are performing zero-time updates where the predict step never
    // runs.
    CHECK(X_hat_.allFinite());
    if (next_it != observations_.end()) {
      next_it->X_hat = X_hat_;
      next_it->P = P_;
    } else {
      break;
    }
    ++cur_it;
    ++next_it;
  }
}

template <typename Scalar>
void HybridEkf<Scalar>::InitializeMatrices() {
  A_continuous_.setZero();
  const Scalar diameter = 2.0 * dt_config_.robot_radius;
  // Theta derivative
  A_continuous_(kTheta, kLeftVelocity) = -1.0 / diameter;
  A_continuous_(kTheta, kRightVelocity) = 1.0 / diameter;

  // Encoder derivatives
  A_continuous_(kLeftEncoder, kLeftVelocity) = 1.0;
  A_continuous_(kLeftEncoder, kAngularError) = 1.0;
  A_continuous_(kLeftEncoder, kLongitudinalVelocityOffset) = -1.0;
  A_continuous_(kRightEncoder, kRightVelocity) = 1.0;
  A_continuous_(kRightEncoder, kAngularError) = -1.0;
  A_continuous_(kRightEncoder, kLongitudinalVelocityOffset) = -1.0;

  // Pull velocity derivatives from velocity matrices.
  // Note that this looks really awkward (doesn't use
  // Eigen blocks) because someone decided that the full
  // drivetrain Kalman Filter should have a weird convention.
  // TODO(james): Support shifting drivetrains with changing A_continuous
  const auto &vel_coefs = velocity_drivetrain_coefficients_;
  A_continuous_(kLeftVelocity, kLeftVelocity) = vel_coefs.A_continuous(0, 0);
  A_continuous_(kLeftVelocity, kRightVelocity) = vel_coefs.A_continuous(0, 1);
  A_continuous_(kRightVelocity, kLeftVelocity) = vel_coefs.A_continuous(1, 0);
  A_continuous_(kRightVelocity, kRightVelocity) = vel_coefs.A_continuous(1, 1);

  A_continuous_(kLongitudinalVelocityOffset, kLongitudinalVelocityOffset) =
      -1.0 / kVelocityOffsetTimeConstant;
  A_continuous_(kLateralVelocity, kLateralVelocity) =
      -1.0 / kLateralVelocityTimeConstant;

  // TODO(james): Decide what to do about these terms. They don't really matter
  // too much when we have accelerometer readings available.
  B_continuous_.setZero();
  B_continuous_.template block<1, 2>(kLeftVelocity, kLeftVoltage) =
      vel_coefs.B_continuous.row(0).template cast<Scalar>();
  B_continuous_.template block<1, 2>(kRightVelocity, kLeftVoltage) =
      vel_coefs.B_continuous.row(1).template cast<Scalar>();
  A_continuous_.template block<kNStates, 2>(0, kLeftVoltageError) =
      B_continuous_.template block<kNStates, 2>(0, kLeftVoltage);

  Q_continuous_.setZero();
  // TODO(james): Improve estimates of process noise--e.g., X/Y noise can
  // probably be reduced when we are stopped because you rarely jump randomly.
  // Or maybe it's more appropriate to scale wheelspeed noise with wheelspeed,
  // since the wheels aren't likely to slip much stopped.
  Q_continuous_(kX, kX) = 0.002;
  Q_continuous_(kY, kY) = 0.002;
  Q_continuous_(kTheta, kTheta) = 0.0001;
  Q_continuous_(kLeftEncoder, kLeftEncoder) = std::pow(0.15, 2.0);
  Q_continuous_(kRightEncoder, kRightEncoder) = std::pow(0.15, 2.0);
  Q_continuous_(kLeftVelocity, kLeftVelocity) = std::pow(0.1, 2.0);
  Q_continuous_(kRightVelocity, kRightVelocity) = std::pow(0.1, 2.0);
  Q_continuous_(kLeftVoltageError, kLeftVoltageError) = std::pow(10.0, 2.0);
  Q_continuous_(kRightVoltageError, kRightVoltageError) = std::pow(10.0, 2.0);
  Q_continuous_(kAngularError, kAngularError) = std::pow(2.0, 2.0);
  // This noise value largely governs whether we will trust the encoders or
  // accelerometer more for estimating the robot position.
  // Note that this also affects how we interpret camera measurements,
  // particularly when using a heading/distance/skew measurement--if the
  // noise on these numbers is particularly high, then we can end up with weird
  // dynamics where a camera update both shifts our X/Y position and adjusts our
  // velocity estimates substantially, causing the camera updates to create
  // "momentum" and if we don't trust the encoders enough, then we have no way
  // of determining that the velocity updates are bogus. This also interacts
  // with kVelocityOffsetTimeConstant.
  Q_continuous_(kLongitudinalVelocityOffset, kLongitudinalVelocityOffset) =
      std::pow(0.01, 2.0);
  Q_continuous_(kLateralVelocity, kLateralVelocity) = std::pow(0.01, 2.0);

  {
    Eigen::Matrix<Scalar, kNOutputs, kNStates> H_encoders_and_gyro;
    H_encoders_and_gyro.setZero();
    // Gyro rate is just the difference between right/left side speeds:
    H_encoders_and_gyro(2, kLeftVelocity) = -1.0 / diameter;
    H_encoders_and_gyro(2, kRightVelocity) = 1.0 / diameter;
    H_gyro_only_.emplace(H_encoders_and_gyro);
    // Encoders are stored directly in the state matrix, so are a minor
    // transform away.
    H_encoders_and_gyro(0, kLeftEncoder) = 1.0;
    H_encoders_and_gyro(1, kRightEncoder) = 1.0;
    H_encoders_and_gyro_.emplace(H_encoders_and_gyro);
  }

  encoder_noise_ = 5e-9;
  gyro_noise_ = 1e-13;

  X_hat_.setZero();
  P_.setZero();
}

}  // namespace frc::control_loops::drivetrain

#endif  // FRC_CONTROL_LOOPS_DRIVETRAIN_HYBRID_EKF_H_
