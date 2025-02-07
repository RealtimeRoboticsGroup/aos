#!/usr/bin/python3

# Generates profiled subsystem for use in
# static_zeroing_single_dof_profiled_subsystem_test

from frc.control_loops.python import control_loop
from frc.control_loops.python import linear_system
import numpy
import sys
import gflags
import glog

FLAGS = gflags.FLAGS

try:
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass

kIntake = linear_system.LinearSystemParams(name='Elevator',
                                           motor=control_loop.NMotor(
                                               control_loop.Falcon(), 2),
                                           G=(14. / 48.),
                                           radius=1.7 * 0.0254 / 2.0,
                                           mass=(14.5 + 3.5) / 2.2,
                                           q_pos=0.10,
                                           q_vel=5.5,
                                           kalman_q_pos=0.12,
                                           kalman_q_vel=2.00,
                                           kalman_q_voltage=40.0,
                                           kalman_r_position=0.05)


def main(argv):
    max_velocity = 3.0
    max_acceleration = 40.0
    if FLAGS.plot:
        R = numpy.matrix([[1.0], [0.0]])
        #linear_system.PlotMotion(kIntake,
        #                         R,
        #                         max_velocity=max_velocity
        #                         max_acceleration=max_acceleration)
        linear_system.PlotKick(kIntake, R)

    plant = linear_system.LinearSystem(kIntake)

    # volts / meter -> volts / rotation
    # 1 rotation * kIntake.G * radius * 2.0 * pi -> meters

    # dx/dt = A * X + B * U

    # d [x]  = [a b] * [x] + [e] * U
    # d [v]  = [c d] * [v] + [f] * U

    # Have: m/(s*v)
    # Want: Volts/rotation/sec
    Kv = -plant.B_continuous[1, 0] / plant.A_continuous[1, 1]
    meters_per_rotation = (kIntake.G * kIntake.radius * 2.0 * numpy.pi)

    Ka = plant.B_continuous[1, 0]

    # meters/rotation -> 1.0 / (kIntake.G * kIntake.radius * 2.0 * math.pi)
    glog.info('Kp: %s volts/rotation', plant.K[0, 0] * meters_per_rotation)
    glog.info('Kd: %s volts / (rotation * sec)',
              plant.K[0, 1] * meters_per_rotation)
    glog.info('Kv: %s volts / (rotation * sec)', meters_per_rotation / Kv)
    glog.info('Ka: %s volts / (rotation * sec * sec)',
              meters_per_rotation / Ka)

    glog.info('Accel: %s rotation / (sec * sec)',
              max_acceleration / meters_per_rotation)
    glog.info('Vel: %s rotation / sec', max_velocity / meters_per_rotation)

    glog.info('Kg: %s volts',
              (kIntake.mass - 5.0 / 2.2) * 9.8 * kIntake.radius * kIntake.G /
              kIntake.motor.Kt * kIntake.motor.resistance)

    if FLAGS.plot:
        return

    # Write the generated constants out to a file.
    if len(argv) != 7:
        glog.fatal(
            'Expected .h, .cc, and .json filenames and .json file name for the \
            static_zeroing_single_dof_profiled_subsystem_test and integral \
            static_zeroing_single_dof_profiled_subsystem_test.')
    else:
        namespaces = ['frc', 'control_loops']
        linear_system.WriteLinearSystem(kIntake, argv[1:4], argv[4:7],
                                        namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
