#!/usr/bin/python3

from frc.control_loops.python import control_loop
from frc.control_loops.python import angular_system
import numpy
import sys
import gflags
import glog

FLAGS = gflags.FLAGS

try:
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass

kArm = angular_system.AngularSystemParams(name='Arm',
                                          motor=control_loop.Falcon(),
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
    max_velocity = 10.0
    max_acceleration = 90.0
    if FLAGS.plot:
        R = numpy.matrix([[numpy.pi / 2.0], [0.0]])
        #angular_system.PlotMotion(kArm,
        #                         R,
        #                         max_velocity=max_velocity
        #                         max_acceleration=max_acceleration)
        angular_system.PlotKick(kArm, R)

    plant = angular_system.LinearSystem(kArm)

    # volts / meter -> volts / rotation
    # 1 rotation * kArm.G -> output rotations

    # dx/dt = A * X + B * U

    # d [x]  = [a b] * [x] + [e] * U
    # d [v]  = [c d] * [v] + [f] * U

    # Have: radians/(s*v)
    # Want: Volts/(output rotation/sec)
    Kv = -plant.B_continuous[1, 0] / plant.A_continuous[1, 1]
    radians_per_motor_rotation = (kArm.G * 2.0 * numpy.pi)

    Ka = plant.B_continuous[1, 0]

    glog.info('Kp: %s volts/rotation', plant.K[0, 0] * radians_per_motor_rotation)
    glog.info('Kd: %s volts / (rotation * sec)',
              plant.K[0, 1] * radians_per_motor_rotation)
    glog.info('Kv: %s volts / (rotation * sec)', 1.0 / (rotations_per_motor_radian * Kv))
    glog.info('Ka: %s volts / (rotation * sec * sec)',
              1.0 / (rotations_per_motor_radian * Ka))

    glog.info('Accel: %s rotation / (sec * sec)',
              max_acceleration / radians_per_motor_rotation)
    glog.info('Vel: %s rotation / sec', max_velocity / radians_per_motor_rotation)

    glog.info('Kg: %s volts', (kArm.mass - 5.0 / 2.2) * 9.8 * kArm.radius *
              kArm.G / kArm.motor.Kt * kArm.motor.resistance)

    if FLAGS.plot:
        return

    # Write the generated constants out to a file.
    if len(argv) != 7:
        glog.fatal(
            'Expected .h, .cc, and .json filenames and .json file name for the \
            static_zeroing_single_dof_profiled_subsystem_test and integral \
            static_zeroing_single_dof_profiled_subsystem_test.'                                                                                                                                                                                                                                                                                                                           )
    else:
        namespaces = ['frc', 'control_loops']
        angular_system.WriteAngularSystem(kArm, argv[1:4], argv[4:7],
                                          namespaces)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
