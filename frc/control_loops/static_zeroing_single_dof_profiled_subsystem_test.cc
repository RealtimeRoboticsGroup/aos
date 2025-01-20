#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem.h"

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "flatbuffers/flatbuffers.h"
#include "gtest/gtest.h"

#include "frc/control_loops/capped_test_plant.h"
#include "frc/control_loops/control_loop.h"
#include "frc/control_loops/control_loop_test.h"
#include "frc/control_loops/position_sensor_sim.h"
#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem_test_absolute_encoder_status_generated.h"
#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem_test_absolute_position_generated.h"
#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem_test_integral_plant.h"
#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem_test_plant.h"
#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem_test_pot_and_absolute_encoder_status_generated.h"
#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem_test_pot_and_absolute_position_generated.h"
#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem_test_subsystem_goal_generated.h"
#include "frc/control_loops/static_zeroing_single_dof_profiled_subsystem_test_subsystem_output_generated.h"
#include "frc/zeroing/absolute_encoder.h"
#include "frc/zeroing/continuous_absolute_encoder.h"
#include "frc/zeroing/pot_and_absolute_encoder.h"
#include "frc/zeroing/zeroing.h"

using ::frc::control_loops::PositionSensorSimulator;

namespace frc::control_loops {
namespace {
constexpr double kNoiseScalar = 0.01;

namespace chrono = ::std::chrono;
using ::aos::monotonic_clock;

using SZSDPS_PotAndAbsEncoder = StaticZeroingSingleDOFProfiledSubsystem<
    ::frc::zeroing::PotAndAbsoluteEncoderZeroingEstimator,
    ::frc::control_loops::PotAndAbsoluteEncoderProfiledJointStatus>;

using SZSDPS_AbsEncoder = StaticZeroingSingleDOFProfiledSubsystem<
    ::frc::zeroing::AbsoluteEncoderZeroingEstimator,
    ::frc::control_loops::AbsoluteEncoderProfiledJointStatus>;

using SZSDPS_Continuous = StaticZeroingSingleDOFProfiledSubsystem<
    ::frc::zeroing::ContinuousAbsoluteEncoderZeroingEstimator,
    ::frc::control_loops::AbsoluteEncoderProfiledJointStatus>;

using FBB = flatbuffers::FlatBufferBuilder;

struct PotAndAbsoluteEncoderTestParams {
  typedef zeroing::testing::SubsystemPotAndAbsoluteEncoderProfiledJointStatus
      Status;
  typedef zeroing::testing::SubsystemPotAndAbsolutePosition Position;
  typedef PotAndAbsolutePosition RealPosition;
  typedef ::frc::control_loops::zeroing::testing::SubsystemGoal Goal;
  typedef ::frc::control_loops::zeroing::testing::SubsystemOutput Output;
  static constexpr bool kRespectsHardstops = true;
  static constexpr bool kIsContinuous = false;
};

struct AbsoluteEncoderTestParams {
  typedef zeroing::testing::SubsystemAbsoluteEncoderProfiledJointStatus Status;
  typedef zeroing::testing::SubsystemAbsolutePosition Position;
  typedef AbsolutePosition RealPosition;
  typedef zeroing::testing::SubsystemGoal Goal;
  typedef zeroing::testing::SubsystemOutput Output;
  static constexpr bool kRespectsHardstops = true;
  static constexpr bool kIsContinuous = false;
};

struct ContinuousAbsoluteEncoderTestParams {
  typedef zeroing::testing::SubsystemAbsoluteEncoderProfiledJointStatus Status;
  typedef zeroing::testing::SubsystemAbsolutePosition Position;
  typedef AbsolutePosition RealPosition;
  typedef zeroing::testing::SubsystemGoal Goal;
  typedef zeroing::testing::SubsystemOutput Output;
  static constexpr bool kRespectsHardstops = false;
  static constexpr bool kIsContinuous = true;
};

typedef ::testing::Types<
    ::std::pair<SZSDPS_AbsEncoder, AbsoluteEncoderTestParams>,
    ::std::pair<SZSDPS_PotAndAbsEncoder, PotAndAbsoluteEncoderTestParams>,
    ::std::pair<SZSDPS_Continuous, ContinuousAbsoluteEncoderTestParams>>
    TestTypes;

constexpr double kZeroingVoltage = 2.5;
constexpr double kOperatingVoltage = 12.0;
const int kZeroingSampleSize = 200;

constexpr double kEncoderIndexDifference = 1.0;

template <typename ZeroingEstimator>
struct TestIntakeSystemValues {
  static const typename ZeroingEstimator::ZeroingConstants kZeroing;
  static const ::frc::constants::Range kRange;

  static const StaticZeroingSingleDOFProfiledSubsystemParams<ZeroingEstimator>
  make_params();
};

template <>
const frc::zeroing::PotAndAbsoluteEncoderZeroingEstimator::ZeroingConstants
    TestIntakeSystemValues<
        frc::zeroing::PotAndAbsoluteEncoderZeroingEstimator>::kZeroing{
        {}, kZeroingSampleSize, kEncoderIndexDifference, 0, 0.0005, 20, 1.9};

template <>
const frc::zeroing::AbsoluteEncoderZeroingEstimator::ZeroingConstants
    TestIntakeSystemValues<
        frc::zeroing::AbsoluteEncoderZeroingEstimator>::kZeroing{
        {}, kZeroingSampleSize, kEncoderIndexDifference, 0.0, 0.2, 0.0005, 20,
        1.9};

template <>
const frc::zeroing::ContinuousAbsoluteEncoderZeroingEstimator::ZeroingConstants
    TestIntakeSystemValues<
        frc::zeroing::ContinuousAbsoluteEncoderZeroingEstimator>::kZeroing{
        {}, kZeroingSampleSize, kEncoderIndexDifference, 0.0, 0.0005, 20, 1.9};

template <>
const ::frc::constants::Range TestIntakeSystemValues<
    frc::zeroing::PotAndAbsoluteEncoderZeroingEstimator>::kRange{
    .lower_hard = -0.01, .upper_hard = 0.250, .lower = 0.01, .upper = 0.235};
template <>
const ::frc::constants::Range TestIntakeSystemValues<
    frc::zeroing::AbsoluteEncoderZeroingEstimator>::kRange{
    .lower_hard = -0.01, .upper_hard = 0.250, .lower = 0.01, .upper = 0.235};
template <>
const ::frc::constants::Range TestIntakeSystemValues<
    frc::zeroing::ContinuousAbsoluteEncoderZeroingEstimator>::kRange{
    .lower_hard = -3.01, .upper_hard = 3.1, .lower = -3.00, .upper = 3.0};

template <typename ZeroingEstimator>
const StaticZeroingSingleDOFProfiledSubsystemParams<ZeroingEstimator>
TestIntakeSystemValues<ZeroingEstimator>::make_params() {
  StaticZeroingSingleDOFProfiledSubsystemParams<ZeroingEstimator> params{
      kZeroingVoltage,
      kOperatingVoltage,
      {{}, 0.1, 1.0},
      {{}, 0.3, 5.0},
      TestIntakeSystemValues::kRange,
      TestIntakeSystemValues::kZeroing,
      &MakeIntegralTestIntakeSystemLoop};
  return params;
}

}  // namespace

template <typename SZSDPS, typename TestParams>
class TestIntakeSystemSimulation {
 public:
  typedef typename TestParams::Goal GoalType;
  typedef typename TestParams::Status StatusType;
  typedef typename TestParams::Position PositionType;
  typedef typename TestParams::RealPosition RealPositionType;
  typedef typename TestParams::Output OutputType;
  const ::frc::constants::Range kRange =
      TestIntakeSystemValues<typename SZSDPS::ZeroingEstimator>::kRange;

  TestIntakeSystemSimulation(::aos::EventLoop *event_loop,
                             chrono::nanoseconds dt)
      : event_loop_(event_loop),
        dt_(dt),
        subsystem_position_sender_(
            event_loop_->MakeSender<PositionType>("/loop")),
        subsystem_status_fetcher_(
            event_loop_->MakeFetcher<StatusType>("/loop")),
        subsystem_output_fetcher_(
            event_loop_->MakeFetcher<OutputType>("/loop")),
        subsystem_plant_(new CappedTestPlant(
            ::frc::control_loops::MakeTestIntakeSystemPlant())),
        subsystem_sensor_sim_(kEncoderIndexDifference) {
    // Start the subsystem out in the middle by default.
    InitializeSubsystemPosition((kRange.lower + kRange.upper) / 2.0);

    event_loop_->AddPhasedLoop(
        [this](int) {
          // Skip this the first time.
          if (!first_) {
            this->Simulate();
          }
          first_ = false;
          this->SendPositionMessage();
        },
        dt);
  }

  void InitializeSubsystemPosition(double start_pos) {
    this->subsystem_plant_->mutable_X(0, 0) = start_pos;
    this->subsystem_plant_->mutable_X(1, 0) = 0.0;

    this->InitializeSensorSim(start_pos);
  }

  void InitializeSensorSim(double start_pos);

  double subsystem_position() const { return subsystem_plant_->X(0, 0); }
  double subsystem_velocity() const { return subsystem_plant_->X(1, 0); }

  // Sets the difference between the commanded and applied powers.
  // This lets us test that the integrators work.
  void set_subsystem_voltage_offset(double voltage_offset) {
    this->subsystem_plant_->set_voltage_offset(voltage_offset);
  }

  // Sends a queue message with the position.
  void SendPositionMessage() {
    typename ::aos::Sender<PositionType>::Builder position =
        subsystem_position_sender_.MakeBuilder();

    auto real_position_builder =
        position.template MakeBuilder<RealPositionType>();
    flatbuffers::Offset<RealPositionType> position_offset =
        this->subsystem_sensor_sim_
            .template GetSensorValues<typename RealPositionType::Builder>(
                &real_position_builder);
    auto position_builder = position.template MakeBuilder<PositionType>();
    position_builder.add_position(position_offset);
    CHECK_EQ(position.Send(position_builder.Finish()),
             aos::RawSender::Error::kOk);
  }

  void set_peak_subsystem_acceleration(double value) {
    peak_subsystem_acceleration_ = value;
  }
  void set_peak_subsystem_velocity(double value) {
    peak_subsystem_velocity_ = value;
  }

  // Simulates the subsystem for a single timestep.
  void Simulate() {
    EXPECT_TRUE(subsystem_output_fetcher_.Fetch());
    EXPECT_TRUE(subsystem_status_fetcher_.Fetch());

    const double begin_subsystem_velocity = subsystem_velocity();

    const double voltage_check_subsystem =
        (static_cast<typename SZSDPS::State>(
             subsystem_status_fetcher_->status()->state()) ==
         SZSDPS::State::RUNNING)
            ? kOperatingVoltage
            : kZeroingVoltage;

    EXPECT_LE(::std::abs(subsystem_output_fetcher_->output()),
              voltage_check_subsystem);

    ::Eigen::Matrix<double, 1, 1> subsystem_U;
    subsystem_U << subsystem_output_fetcher_->output() +
                       subsystem_plant_->voltage_offset();
    subsystem_plant_->Update(subsystem_U);

    const double position_subsystem = subsystem_plant_->Y(0, 0);

    subsystem_sensor_sim_.MoveTo(position_subsystem);

    EXPECT_GE(position_subsystem, kRange.lower_hard);
    EXPECT_LE(position_subsystem, kRange.upper_hard);

    const double loop_time = ::aos::time::DurationInSeconds(dt_);
    const double subsystem_acceleration =
        (subsystem_velocity() - begin_subsystem_velocity) / loop_time;
    EXPECT_NEAR(subsystem_acceleration, 0.0, peak_subsystem_acceleration_);
    EXPECT_NEAR(subsystem_velocity(), 0.0, peak_subsystem_velocity_);
  }

 private:
  ::aos::EventLoop *event_loop_;
  chrono::nanoseconds dt_;

  bool first_ = true;

  typename ::aos::Sender<PositionType> subsystem_position_sender_;
  typename ::aos::Fetcher<StatusType> subsystem_status_fetcher_;
  typename ::aos::Fetcher<OutputType> subsystem_output_fetcher_;

  ::std::unique_ptr<CappedTestPlant> subsystem_plant_;
  PositionSensorSimulator subsystem_sensor_sim_;

  // The acceleration limits to check for while moving.
  double peak_subsystem_acceleration_ = 1e10;
  // The velocity limits to check for while moving.
  double peak_subsystem_velocity_ = 1e10;

  flatbuffers::FlatBufferBuilder fbb;
};

template <>
void TestIntakeSystemSimulation<
    SZSDPS_PotAndAbsEncoder,
    PotAndAbsoluteEncoderTestParams>::InitializeSensorSim(double start_pos) {
  subsystem_sensor_sim_.Initialize(
      start_pos, kNoiseScalar, 0.0,
      TestIntakeSystemValues<
          typename SZSDPS_PotAndAbsEncoder::ZeroingEstimator>::kZeroing
          .measured_absolute_position);
}

template <>
void TestIntakeSystemSimulation<SZSDPS_AbsEncoder, AbsoluteEncoderTestParams>::
    InitializeSensorSim(double start_pos) {
  subsystem_sensor_sim_.Initialize(
      start_pos, kNoiseScalar, 0.0,
      TestIntakeSystemValues<
          typename SZSDPS_PotAndAbsEncoder::ZeroingEstimator>::kZeroing
          .measured_absolute_position);
}

template <>
void TestIntakeSystemSimulation<SZSDPS_Continuous,
                                ContinuousAbsoluteEncoderTestParams>::
    InitializeSensorSim(double start_pos) {
  subsystem_sensor_sim_.Initialize(
      start_pos, kNoiseScalar, 0.0,
      TestIntakeSystemValues<typename SZSDPS_Continuous::ZeroingEstimator>::
          kZeroing.measured_absolute_position);
}

// Class to represent a module using a subsystem.  This lets us use event loops
// to wrap it.
template <typename TestParams, typename SZSDPS>
class Subsystem
    : public ::frc::controls::ControlLoop<
          typename TestParams::Goal, typename TestParams::Position,
          typename TestParams::Status, typename TestParams::Output> {
 public:
  typedef typename TestParams::Goal GoalType;
  typedef typename TestParams::Status StatusType;
  typedef typename TestParams::Position PositionType;
  typedef typename TestParams::Output OutputType;

  Subsystem(::aos::EventLoop *event_loop, const ::std::string &name)
      : frc::controls::ControlLoop<
            typename TestParams::Goal, typename TestParams::Position,
            typename TestParams::Status, typename TestParams::Output>(
            event_loop, name),
        subsystem_(TestIntakeSystemValues<
                   typename SZSDPS::ZeroingEstimator>::make_params()) {}

  void RunIteration(const GoalType *unsafe_goal, const PositionType *position,
                    typename ::aos::Sender<OutputType>::Builder *output,
                    typename ::aos::Sender<StatusType>::Builder *status) {
    if (this->WasReset()) {
      AOS_LOG(ERROR, "WPILib reset, restarting\n");
      subsystem_.Reset();
    }

    // Convert one goal type to another...
    // TODO(austin): This mallocs...
    FBB fbb;
    ProfileParametersBuilder params_builder(fbb);
    if (unsafe_goal != nullptr) {
      if (unsafe_goal->profile_params() != nullptr) {
        params_builder.add_max_velocity(
            unsafe_goal->profile_params()->max_velocity());
        params_builder.add_max_acceleration(
            unsafe_goal->profile_params()->max_acceleration());
      }

      const auto params_builder_offset = params_builder.Finish();
      StaticZeroingSingleDOFProfiledSubsystemGoalBuilder goal_builder(fbb);
      goal_builder.add_unsafe_goal(unsafe_goal->unsafe_goal());
      if (unsafe_goal->has_goal_velocity()) {
        goal_builder.add_goal_velocity(unsafe_goal->goal_velocity());
      }
      if (unsafe_goal->has_ignore_profile()) {
        goal_builder.add_ignore_profile(unsafe_goal->ignore_profile());
      }
      goal_builder.add_profile_params(params_builder_offset);
      fbb.Finish(goal_builder.Finish());
    } else {
      params_builder.add_max_velocity(0.0);
      params_builder.add_max_acceleration(0.0);
      const auto params_builder_offset = params_builder.Finish();
      StaticZeroingSingleDOFProfiledSubsystemGoalBuilder goal_builder(fbb);
      goal_builder.add_profile_params(params_builder_offset);
      fbb.Finish(goal_builder.Finish());
    }

    double output_voltage;

    auto status_offset = subsystem_.Iterate(
        unsafe_goal == nullptr
            ? nullptr
            : flatbuffers::GetRoot<StaticZeroingSingleDOFProfiledSubsystemGoal>(
                  fbb.GetBufferPointer()),
        position->position(), output == nullptr ? nullptr : &output_voltage,
        status->fbb());
    typename StatusType::Builder subsystem_status_builder =
        status->template MakeBuilder<StatusType>();

    subsystem_status_builder.add_status(status_offset);
    CHECK_EQ(status->Send(subsystem_status_builder.Finish()),
             aos::RawSender::Error::kOk);
    if (output != nullptr) {
      typename OutputType::Builder output_builder =
          output->template MakeBuilder<OutputType>();
      output_builder.add_output(output_voltage);
      CHECK_EQ(output->Send(output_builder.Finish()),
               aos::RawSender::Error::kOk);
    }
  }

  SZSDPS *subsystem() { return &subsystem_; }

 private:
  SZSDPS subsystem_;
};

template <typename TSZSDPS>
class IntakeSystemTest : public ::frc::testing::ControlLoopTest {
 protected:
  using SZSDPS = typename TSZSDPS::first_type;
  using TestParams = typename TSZSDPS::second_type;
  using ZeroingEstimator = typename SZSDPS::ZeroingEstimator;
  using ProfiledJointStatus = typename SZSDPS::ProfiledJointStatus;

  typedef typename TestParams::Goal GoalType;
  typedef typename TestParams::Status StatusType;
  typedef typename TestParams::Position PositionType;
  typedef typename TestParams::Output OutputType;

  static constexpr bool kRespectsHardstops = TestParams::kRespectsHardstops;
  static constexpr bool kIsContinuous = TestParams::kIsContinuous;
  const ::frc::constants::Range kRange =
      TestIntakeSystemValues<ZeroingEstimator>::kRange;

  IntakeSystemTest()
      : ::frc::testing::ControlLoopTest(
            aos::configuration::ReadConfig("frc/control_loops/"
                                           "static_zeroing_single_dof_profiled_"
                                           "subsystem_test_config.json"),
            chrono::microseconds(5050)),
        test_event_loop_(MakeEventLoop("test")),
        subsystem_goal_sender_(test_event_loop_->MakeSender<GoalType>("/loop")),
        subsystem_goal_fetcher_(
            test_event_loop_->MakeFetcher<GoalType>("/loop")),
        subsystem_status_fetcher_(
            test_event_loop_->MakeFetcher<StatusType>("/loop")),
        subsystem_event_loop_(MakeEventLoop("subsystem")),
        subsystem_(subsystem_event_loop_.get(), "/loop"),
        subsystem_plant_event_loop_(MakeEventLoop("plant")),
        subsystem_plant_(subsystem_plant_event_loop_.get(), dt()) {}

  void VerifyNearGoal() {
    subsystem_goal_fetcher_.Fetch();
    EXPECT_TRUE(subsystem_goal_fetcher_.get() != nullptr);
    EXPECT_TRUE(subsystem_status_fetcher_.Fetch());

    EXPECT_NEAR(subsystem_goal_fetcher_->unsafe_goal(),
                subsystem_status_fetcher_->status()->position(), 0.001)
        << aos::FlatbufferToJson(subsystem_status_fetcher_.get());
    EXPECT_NEAR(subsystem_goal_fetcher_->unsafe_goal(),
                subsystem_plant_.subsystem_position(), 0.001);
    EXPECT_NEAR(subsystem_status_fetcher_->status()->velocity(), 0, 0.001);
  }

  SZSDPS *subsystem() { return subsystem_.subsystem(); }

  void set_peak_subsystem_acceleration(double value) {
    subsystem_plant_.set_peak_subsystem_acceleration(value);
  }
  void set_peak_subsystem_velocity(double value) {
    subsystem_plant_.set_peak_subsystem_velocity(value);
  }

  ::std::unique_ptr<::aos::EventLoop> test_event_loop_;
  ::aos::Sender<GoalType> subsystem_goal_sender_;
  ::aos::Fetcher<GoalType> subsystem_goal_fetcher_;
  ::aos::Fetcher<StatusType> subsystem_status_fetcher_;

  // Create a control loop and simulation.
  ::std::unique_ptr<::aos::EventLoop> subsystem_event_loop_;
  Subsystem<TestParams, SZSDPS> subsystem_;

  ::std::unique_ptr<::aos::EventLoop> subsystem_plant_event_loop_;
  TestIntakeSystemSimulation<SZSDPS, TestParams> subsystem_plant_;
};

TYPED_TEST_SUITE_P(IntakeSystemTest);

// Tests that the subsystem does nothing when the goal is zero.
TYPED_TEST_P(IntakeSystemTest, DoesNothing) {
  this->SetEnabled(true);
  // Intake system uses 0.05 to test for 0.
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    EXPECT_EQ(message.Send(
                  zeroing::testing::CreateSubsystemGoal(*message.fbb(), 0.05)),
              aos::RawSender::Error::kOk);
  }
  this->RunFor(chrono::seconds(5));

  this->VerifyNearGoal();
}

// Tests that the subsystem loop can reach a goal.
TYPED_TEST_P(IntakeSystemTest, ReachesGoal) {
  this->SetEnabled(true);
  // Set a reasonable goal.
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    auto profile_builder =
        message.template MakeBuilder<frc::ProfileParameters>();
    profile_builder.add_max_velocity(1);
    profile_builder.add_max_acceleration(0.5);
    EXPECT_EQ(message.Send(zeroing::testing::CreateSubsystemGoal(
                  *message.fbb(), 0.10, profile_builder.Finish())),
              aos::RawSender::Error::kOk);
  }

  // Give it a lot of time to get there.
  this->RunFor(chrono::seconds(8));

  this->VerifyNearGoal();
}

// Tests that the subsystem loop can reach a goal.
TYPED_TEST_P(IntakeSystemTest, ContinuousReachesGoal) {
  if (!this->kIsContinuous) {
    return;
  }
  // Set a reasonable goal.
  auto send_goal = [this](double goal_pos) {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    auto profile_builder =
        message.template MakeBuilder<frc::ProfileParameters>();
    profile_builder.add_max_velocity(1);
    profile_builder.add_max_acceleration(0.5);
    EXPECT_EQ(message.Send(zeroing::testing::CreateSubsystemGoal(
                  *message.fbb(), goal_pos, profile_builder.Finish())),
              aos::RawSender::Error::kOk);
  };
  // Deliberately start the subsystem at an offset from zero so that we can
  // observe that we are not able to zero the "true" absolute position of the
  // subsystem.
  this->subsystem_plant_.InitializeSubsystemPosition(kEncoderIndexDifference +
                                                     0.01);
  EXPECT_FLOAT_EQ(1.01, this->subsystem_plant_.subsystem_position())
      << "Sanity check of initial system state failed.";
  this->SetEnabled(true);
  auto verify_near_value = [this](double goal, std::string_view message) {
    SCOPED_TRACE(message);
    EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());

    // Because the subsystem starts at a position of 1.01 and we only have
    // an absolute encoder, the status will always output positions that are
    // 1 period behind the "actual" position.
    const double expected_status_offset = -kEncoderIndexDifference;
    EXPECT_NEAR(goal + expected_status_offset,
                this->subsystem_status_fetcher_->status()->position(), 0.001)
        << aos::FlatbufferToJson(this->subsystem_status_fetcher_.get());
    EXPECT_NEAR(goal, this->subsystem_plant_.subsystem_position(), 0.001)
        << aos::FlatbufferToJson(this->subsystem_status_fetcher_.get());
    EXPECT_NEAR(this->subsystem_status_fetcher_->status()->velocity(), 0,
                0.001);
  };

  // Note that while the continuous subsystem controller does not know which
  // revolution it started on, it does not attempt to wrap requested goals.
  send_goal(0.9);
  this->RunFor(chrono::seconds(8));
  verify_near_value(1.9, "initial goal");

  send_goal(1.1);
  this->RunFor(chrono::seconds(8));
  verify_near_value(2.1, "increment");

  // Sending a goal that is offset by 1 should result in us driving the
  // subsystem by one period.
  send_goal(0.1);
  this->RunFor(chrono::seconds(8));
  verify_near_value(1.1, "offset by one period");
  // Check that we can handle negative goals.
  send_goal(-0.9);
  this->RunFor(chrono::seconds(8));
  verify_near_value(0.1, "send negative goal");
}

// Tests that the subsystem loop can reach a goal when the profile is disabled.
TYPED_TEST_P(IntakeSystemTest, FunctionsWhenProfileDisabled) {
  this->SetEnabled(true);
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    auto profile_builder =
        message.template MakeBuilder<frc::ProfileParameters>();
    // By setting NaN for the profile, we would cause the entire system to fail
    // or blow up if it is not ignoring the profile correctly.
    profile_builder.add_max_velocity(std::numeric_limits<double>::quiet_NaN());
    profile_builder.add_max_acceleration(
        std::numeric_limits<double>::quiet_NaN());
    EXPECT_EQ(message.Send(zeroing::testing::CreateSubsystemGoal(
                  *message.fbb(), 0.10, profile_builder.Finish(), 0.0, true)),
              aos::RawSender::Error::kOk);
  }

  // Give it a lot of time to get there.
  this->RunFor(chrono::seconds(8));

  this->VerifyNearGoal();
}

// Tests that the subsystem loop can maintain a velocity when using the
// goal_velocity setting.
TYPED_TEST_P(IntakeSystemTest, MaintainConstantVelocityWithoutProfile) {
  this->SetEnabled(true);

  const double kStartingGoal = -0.10;
  const double kVelocity = 0.05;
  this->test_event_loop_->AddPhasedLoop(
      [this, kStartingGoal, kVelocity](int) {
        auto message = this->subsystem_goal_sender_.MakeBuilder();
        auto profile_builder =
            message.template MakeBuilder<frc::ProfileParameters>();
        profile_builder.add_max_velocity(0);
        profile_builder.add_max_acceleration(0);
        EXPECT_EQ(
            message.Send(zeroing::testing::CreateSubsystemGoal(
                *message.fbb(),
                kStartingGoal + aos::time::DurationInSeconds(
                                    this->monotonic_now().time_since_epoch()) *
                                    kVelocity,
                profile_builder.Finish(), kVelocity, true)),
            aos::RawSender::Error::kOk);
      },
      this->dt());

  const double kRunTimeSec = 4;
  // Give time for the system to settle down--it should've been running at a
  // constant velocity the whole time, once it converged.
  this->RunFor(chrono::seconds(static_cast<int>(kRunTimeSec)));

  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());

  EXPECT_NEAR(kStartingGoal + kVelocity * kRunTimeSec,
              this->subsystem_status_fetcher_->status()->position(), 0.001);
  EXPECT_NEAR(kStartingGoal + kVelocity * kRunTimeSec,
              this->subsystem_plant_.subsystem_position(), 0.001);
  EXPECT_NEAR(kVelocity, this->subsystem_status_fetcher_->status()->velocity(),
              0.001);
}

// Makes sure that the voltage on a motor is properly pulled back after
// saturation such that we don't get weird or bad (e.g. oscillating) behaviour.
TYPED_TEST_P(IntakeSystemTest, SaturationTest) {
  this->SetEnabled(true);
  // Zero it before we move.
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    EXPECT_EQ(message.Send(zeroing::testing::CreateSubsystemGoal(
                  *message.fbb(), this->kRange.upper)),
              aos::RawSender::Error::kOk);
  }
  this->RunFor(chrono::seconds(20));
  this->VerifyNearGoal();

  // Try a low acceleration move with a high max velocity and verify the
  // acceleration is capped like expected.
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    auto profile_builder =
        message.template MakeBuilder<frc::ProfileParameters>();
    profile_builder.add_max_velocity(20.0);
    profile_builder.add_max_acceleration(0.1);
    EXPECT_EQ(
        message.Send(zeroing::testing::CreateSubsystemGoal(
            *message.fbb(), this->kRange.lower, profile_builder.Finish())),
        aos::RawSender::Error::kOk);
  }
  this->set_peak_subsystem_velocity(23.0);
  this->set_peak_subsystem_acceleration(0.2);

  this->RunFor(chrono::seconds(20));
  this->VerifyNearGoal();

  // Now do a high acceleration move with a low velocity limit.
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    auto profile_builder =
        message.template MakeBuilder<frc::ProfileParameters>();
    profile_builder.add_max_velocity(0.1);
    profile_builder.add_max_acceleration(100.0);
    EXPECT_EQ(
        message.Send(zeroing::testing::CreateSubsystemGoal(
            *message.fbb(), this->kRange.upper, profile_builder.Finish())),
        aos::RawSender::Error::kOk);
  }

  this->set_peak_subsystem_velocity(0.2);
  this->set_peak_subsystem_acceleration(103);
  this->RunFor(chrono::seconds(static_cast<int>(
      std::ceil((this->kRange.upper - this->kRange.lower) / 0.1 * 1.1))));

  this->VerifyNearGoal();
}

// Tests that the subsystem loop doesn't try and go beyond it's physical range
// of the mechanisms.
TYPED_TEST_P(IntakeSystemTest, RespectsRange) {
  this->SetEnabled(true);

  // Set some ridiculous goals to test upper limits.
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    auto profile_builder =
        message.template MakeBuilder<frc::ProfileParameters>();
    profile_builder.add_max_velocity(1.0);
    profile_builder.add_max_acceleration(0.5);
    EXPECT_EQ(message.Send(zeroing::testing::CreateSubsystemGoal(
                  *message.fbb(), 100.0, profile_builder.Finish())),
              aos::RawSender::Error::kOk);
  }
  this->RunFor(chrono::seconds(10));

  // Check that we are near our soft limit.
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(this->kRange.upper,
              this->subsystem_status_fetcher_->status()->position(), 0.001);

  // Set some ridiculous goals to test lower limits.
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    auto profile_builder =
        message.template MakeBuilder<frc::ProfileParameters>();
    profile_builder.add_max_velocity(1.0);
    profile_builder.add_max_acceleration(0.5);
    EXPECT_EQ(message.Send(zeroing::testing::CreateSubsystemGoal(
                  *message.fbb(), -100.0, profile_builder.Finish())),
              aos::RawSender::Error::kOk);
  }

  this->RunFor(chrono::seconds(20));

  // Check that we are near our soft limit.
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(this->kRange.lower,
              this->subsystem_status_fetcher_->status()->position(), 0.001);
}

// Tests that the subsystem loop zeroes when run for a while.
TYPED_TEST_P(IntakeSystemTest, ZeroTest) {
  this->SetEnabled(true);

  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    auto profile_builder =
        message.template MakeBuilder<frc::ProfileParameters>();
    profile_builder.add_max_velocity(1.0);
    profile_builder.add_max_acceleration(0.5);
    EXPECT_EQ(
        message.Send(zeroing::testing::CreateSubsystemGoal(
            *message.fbb(), this->kRange.upper, profile_builder.Finish())),
        aos::RawSender::Error::kOk);
  }

  this->RunFor(chrono::seconds(10));

  this->VerifyNearGoal();
}

// Tests that the loop zeroes when run for a while without a goal.
TYPED_TEST_P(IntakeSystemTest, ZeroNoGoal) {
  this->SetEnabled(true);
  this->RunFor(chrono::seconds(5));

  EXPECT_EQ(TestFixture::SZSDPS::State::RUNNING, this->subsystem()->state());
}

TYPED_TEST_P(IntakeSystemTest, LowerHardstopStartup) {
  if (!this->kRespectsHardstops) {
    return;
  }
  this->SetEnabled(true);
  this->subsystem_plant_.InitializeSubsystemPosition(this->kRange.lower_hard);
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    EXPECT_EQ(message.Send(zeroing::testing::CreateSubsystemGoal(
                  *message.fbb(), this->kRange.upper)),
              aos::RawSender::Error::kOk);
  }
  this->RunFor(chrono::seconds(10));

  this->VerifyNearGoal();
}

// Tests that starting at the upper hardstops doesn't cause an abort.
TYPED_TEST_P(IntakeSystemTest, UpperHardstopStartup) {
  if (!this->kRespectsHardstops) {
    return;
  }
  this->SetEnabled(true);

  this->subsystem_plant_.InitializeSubsystemPosition(this->kRange.upper_hard);
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    EXPECT_EQ(message.Send(zeroing::testing::CreateSubsystemGoal(
                  *message.fbb(), this->kRange.upper)),
              aos::RawSender::Error::kOk);
  }
  this->RunFor(chrono::seconds(10));

  this->VerifyNearGoal();
}

// Tests that resetting WPILib results in a rezero.
TYPED_TEST_P(IntakeSystemTest, ResetTest) {
  this->SetEnabled(true);

  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    EXPECT_EQ(message.Send(
                  zeroing::testing::CreateSubsystemGoal(*message.fbb(), 0.1)),
              aos::RawSender::Error::kOk);
  }
  this->RunFor(chrono::seconds(10));

  EXPECT_EQ(TestFixture::SZSDPS::State::RUNNING, this->subsystem()->state());

  this->VerifyNearGoal();
  this->SimulateSensorReset();
  this->RunFor(chrono::milliseconds(100));

  EXPECT_EQ(TestFixture::SZSDPS::State::UNINITIALIZED,
            this->subsystem()->state());

  this->RunFor(chrono::seconds(10));

  EXPECT_EQ(TestFixture::SZSDPS::State::RUNNING, this->subsystem()->state());
  this->VerifyNearGoal();
}

// Tests that the internal goals don't change while disabled.
TYPED_TEST_P(IntakeSystemTest, DisabledGoalTest) {
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    EXPECT_EQ(message.Send(zeroing::testing::CreateSubsystemGoal(
                  *message.fbb(), this->kRange.lower + 0.03)),
              aos::RawSender::Error::kOk);
  }

  // Checks that the subsystem has not moved from its starting position at 0
  this->RunFor(chrono::milliseconds(100));
  EXPECT_EQ(0.0, this->subsystem()->goal(0));

  // Now make sure they move correctly
  this->SetEnabled(true);
  this->RunFor(chrono::seconds(4));
  EXPECT_NE(0.0, this->subsystem()->goal(0));
}

// Tests that zeroing while disabled works.
TYPED_TEST_P(IntakeSystemTest, DisabledZeroTest) {
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    EXPECT_EQ(message.Send(zeroing::testing::CreateSubsystemGoal(
                  *message.fbb(), this->kRange.lower)),
              aos::RawSender::Error::kOk);
  }

  // Run disabled for 2 seconds
  this->RunFor(chrono::seconds(2));
  EXPECT_EQ(TestFixture::SZSDPS::State::RUNNING, this->subsystem()->state());

  this->SetEnabled(true);
  this->RunFor(chrono::seconds(12));

  this->VerifyNearGoal();
}

// Tests that set_min_position limits range properly
TYPED_TEST_P(IntakeSystemTest, MinPositionTest) {
  this->SetEnabled(true);
  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    EXPECT_EQ(message.Send(zeroing::testing::CreateSubsystemGoal(
                  *message.fbb(), this->kRange.lower_hard)),
              aos::RawSender::Error::kOk);
  }
  this->RunFor(chrono::seconds(12));

  // Check that this->kRange.lower is used as the default min position
  EXPECT_EQ(this->subsystem()->goal(0), this->kRange.lower);
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(this->kRange.lower,
              this->subsystem_status_fetcher_->status()->position(), 0.001);

  // Set min position and check that the subsystem increases to that position
  this->subsystem()->set_min_position(this->kRange.lower + 0.05);
  this->RunFor(chrono::seconds(2));
  EXPECT_EQ(this->subsystem()->goal(0), this->kRange.lower + 0.05);
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(this->kRange.lower + 0.05,
              this->subsystem_status_fetcher_->status()->position(), 0.001);

  // Clear min position and check that the subsystem returns to
  // this->kRange.lower
  this->subsystem()->clear_min_position();
  this->RunFor(chrono::seconds(2));
  EXPECT_EQ(this->subsystem()->goal(0), this->kRange.lower);
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(this->kRange.lower,
              this->subsystem_status_fetcher_->status()->position(), 0.001);
}

// Tests that set_max_position limits range properly
TYPED_TEST_P(IntakeSystemTest, MaxPositionTest) {
  this->SetEnabled(true);

  {
    auto message = this->subsystem_goal_sender_.MakeBuilder();
    EXPECT_EQ(message.Send(zeroing::testing::CreateSubsystemGoal(
                  *message.fbb(), this->kRange.upper_hard)),
              aos::RawSender::Error::kOk);
  }
  this->RunFor(chrono::seconds(12));

  // Check that this->kRange.upper is used as the default max position
  EXPECT_EQ(this->subsystem()->goal(0), this->kRange.upper);
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(this->kRange.upper,
              this->subsystem_status_fetcher_->status()->position(), 0.001);

  // Set max position and check that the subsystem lowers to that position
  this->subsystem()->set_max_position(this->kRange.upper - 0.05);
  this->RunFor(chrono::seconds(2));
  EXPECT_EQ(this->subsystem()->goal(0), this->kRange.upper - 0.05);
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(this->kRange.upper - 0.05,
              this->subsystem_status_fetcher_->status()->position(), 0.001);

  // Clear max position and check that the subsystem returns to
  // this->kRange.upper
  this->subsystem()->clear_max_position();
  this->RunFor(chrono::seconds(2));
  EXPECT_EQ(this->subsystem()->goal(0), this->kRange.upper);
  EXPECT_TRUE(this->subsystem_status_fetcher_.Fetch());
  EXPECT_NEAR(this->kRange.upper,
              this->subsystem_status_fetcher_->status()->position(), 0.001);
}

// Tests that the subsystem maintains its current position when sent a null goal
TYPED_TEST_P(IntakeSystemTest, NullGoalTest) {
  this->SetEnabled(true);

  this->subsystem_plant_.InitializeSubsystemPosition(this->kRange.upper);

  this->RunFor(chrono::seconds(5));

  EXPECT_NEAR(this->kRange.upper, this->subsystem_plant_.subsystem_position(),
              0.001);
  EXPECT_NEAR(this->subsystem_plant_.subsystem_velocity(), 0, 0.001);
}

// Tests that the subsystem estops when a zeroing error occurs
TYPED_TEST_P(IntakeSystemTest, ZeroingErrorTest) {
  this->SetEnabled(true);
  this->RunFor(chrono::seconds(2));

  EXPECT_EQ(this->subsystem()->state(), TestFixture::SZSDPS::State::RUNNING);
  this->subsystem()->TriggerEstimatorError();
  this->RunFor(this->dt());
  EXPECT_EQ(this->subsystem()->state(), TestFixture::SZSDPS::State::ESTOP);
}

REGISTER_TYPED_TEST_SUITE_P(IntakeSystemTest, DoesNothing, ReachesGoal,
                            FunctionsWhenProfileDisabled,
                            MaintainConstantVelocityWithoutProfile,
                            SaturationTest, RespectsRange, ZeroTest, ZeroNoGoal,
                            LowerHardstopStartup, UpperHardstopStartup,
                            ResetTest, DisabledGoalTest, DisabledZeroTest,
                            MinPositionTest, MaxPositionTest, NullGoalTest,
                            ZeroingErrorTest, ContinuousReachesGoal);
INSTANTIATE_TYPED_TEST_SUITE_P(My, IntakeSystemTest, TestTypes);

}  // namespace frc::control_loops
