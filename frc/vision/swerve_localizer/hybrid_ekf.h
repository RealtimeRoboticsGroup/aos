#ifndef FRC_VISION_SWERVE_LOCALIZER_HYBRID_EKF_H_
#define FRC_VISION_SWERVE_LOCALIZER_HYBRID_EKF_H_

#include <chrono>
#include <optional>

#include "Eigen/Dense"

#include "aos/commonmath.h"
#include "aos/containers/priority_queue.h"
#include "aos/util/math.h"
#include "frc/control_loops/c2d.h"
#include "frc/control_loops/runge_kutta.h"

namespace frc::vision::swerve_localizer {

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
// Currently, this class provides the necessary utilities for arbitrary
// nonlinear updates (presumably a camera update).
//
// Discussion of the model:
// We essentially just assume that the reported velocity is right, and integrate
// it up.
//
// On each prediction update, we take in inputs of the absolute robot velocity,
// and integrate it up.
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
  };
  static constexpr int kNStates = 3;
  enum InputIdx {
    kVx = 0,
    kVy = 1,
    kOmega = 2,
  };

  static constexpr int kNInputs = 3;
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

  // The maximum allowable timestep--we use this to check for situations where
  // measurement updates come in too infrequently and this might cause the
  // integrator and discretization in the prediction step to be overly
  // aggressive.
  static constexpr std::chrono::milliseconds kMaxTimestep{120};
  // Inputs are [vx, vy, omega]
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
  // was only used in the 2019 code and we've generally stopped using this
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
  // ExpectedObservationFunctor. The appropriate AddObservation function will
  // then be called in place of calling HybridEkf::AddObservation directly. Note
  // that unless T implements both the Builder and Functor (which is generally
  // discouraged), only one of the AddObservation* functions will build.
  template <typename T>
  class ExpectedObservationAllocator {
   public:
    ExpectedObservationAllocator(HybridEkf *ekf) : ekf_(ekf) {}
    void CorrectKnownH(const std::optional<Output> z, const Input *U, T H,
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
      ekf_->AddObservation(z, U, nullptr, &pushed->functor.value(), R, t);
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
  HybridEkf(std::optional<std::chrono::nanoseconds> force_dt = std::nullopt)
      : force_dt_(force_dt) {
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
        std::nullopt,
        nullptr,
        nullptr,
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
  void AddObservation(const std::optional<Output> z, const Input *U,
                      ExpectedObservationBuilder *observation_builder,
                      ExpectedObservationFunctor *expected_observations,
                      const Eigen::Matrix<Scalar, kNOutputs, kNOutputs> &R,
                      aos::monotonic_clock::time_point t);

  // A utility function for specifically updating with encoder and gyro
  // measurements.
  void UpdateSpeeds(double vx, double vy, double omega,
                    aos::monotonic_clock::time_point t) {
    Input U;
    U(0, 0) = vx;
    U(1, 0) = vy;
    U(2, 0) = omega;
    AddObservation(std::nullopt, &U, nullptr, nullptr,
                   Eigen::Matrix<Scalar, kNOutputs, kNOutputs>::Zero(), t);
  }

  // Sundry accessor:
  State X_hat() const { return X_hat_; }
  Scalar X_hat(long i) const { return X_hat_(i); }
  StateSquare P() const { return P_; }
  aos::monotonic_clock::time_point latest_t() const {
    return observations_.top().t;
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

  // Returns the last state before the specified time.
  // Returns nullopt if time is older than the oldest measurement.
  std::optional<Input> LastInputBeforeTime(
      aos::monotonic_clock::time_point time) {
    if (observations_.empty() || observations_.begin()->t > time) {
      return std::nullopt;
    }
    for (const auto &observation : observations_) {
      if (observation.t > time) {
        // Note that observation.X_hat actually references the _previous_ X_hat.
        return observation.U;
      }
    }
    return MostRecentInput();
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

 private:
  struct Observation {
    Observation(aos::monotonic_clock::time_point t,
                aos::monotonic_clock::time_point prev_t, State X_hat,
                StateSquare P, Input U, std::optional<Output> z,
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
    // Measurement taken at that time.  If this isn't populated, no measurement
    // occured.
    std::optional<Output> z;
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

  // Returns the "A" matrix for a given state. See DiffEq for discussion of
  // ignore_accel.
  StateSquare AForState(const State & /*X*/) const {
    return StateSquare::Zero();
  }

  // Returns dX / dt given X and U. If ignore_accel is set, then we ignore the
  // accelerometer-based components of U (this is solely used in testing).
  State DiffEq(const State &X, const Input &U) const {
    State Xdot = A_continuous_ * X + B_continuous_ * U;
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
      const StateSquare A_c = AForState(*state);
      controls::DiscretizeQAFast(Q_continuous_, A_c, dt, &obs->Q_d, &obs->A_d);
      obs->discretization_time = dt;

      obs->predict_update =
          control_loops::RungeKuttaU(
              [this](const State &X, const Input &U) { return DiffEq(X, U); },
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
    *state += K * (obs->z.value() - obs->h->H(*state, obs->U));
  }

  void ProcessObservation(Observation *obs, const std::chrono::nanoseconds dt,
                          State *state, StateSquare *P) {
    *state = obs->X_hat;
    *P = obs->P;
    if (dt.count() != 0 && dt < kMaxTimestep) {
      PredictImpl(obs, dt, state, P);
    }
    if (obs->z.has_value()) {
      if (obs->h == nullptr) {
        CHECK(obs->make_h != nullptr);
        obs->h = obs->make_h->MakeExpectedObservations(*state, *P);
        CHECK(obs->h != nullptr);
      }
      CorrectImpl(obs, state, P);
    }
  }

  State X_hat_;
  std::optional<std::chrono::nanoseconds> force_dt_;
  StateSquare A_continuous_;
  StateSquare Q_continuous_;
  StateSquare P_;
  Eigen::Matrix<Scalar, kNStates, kNInputs> B_continuous_;

  bool have_zeroed_encoders_ = false;

  aos::PriorityQueue<Observation, kSaveSamples, std::less<Observation>>
      observations_;

  friend class testing::HybridEkfTest;
};  // class HybridEkf

template <typename Scalar>
void HybridEkf<Scalar>::AddObservation(
    const std::optional<Output> z, const Input *U,
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
  B_continuous_ = Eigen::Matrix<Scalar, kNStates, kNInputs>::Identity();

  Q_continuous_.setZero();
  // TODO(james): Improve estimates of process noise--e.g., X/Y noise can
  // probably be reduced when we are stopped because you rarely jump randomly.
  // Or maybe it's more appropriate to scale wheelspeed noise with wheelspeed,
  // since the wheels aren't likely to slip much stopped.
  Q_continuous_(kX, kX) = 0.04;
  Q_continuous_(kY, kY) = 0.04;
  Q_continuous_(kTheta, kTheta) = 0.01;

  X_hat_.setZero();
  P_.setZero();
}

}  // namespace frc::vision::swerve_localizer

#endif  // FRC_VISION_SWERVE_LOCALIZER_HYBRID_EKF_H_
