#!/usr/bin/python3

from frc.control_loops.python import drivetrain
import sys

import gflags
import glog

FLAGS = gflags.FLAGS

kDrivetrain = drivetrain.DrivetrainParams(
    J=8.0,
    mass=100.0,
    robot_radius=0.616 / 2.0,
    wheel_radius=0.127 / 2.0 * 120.0 / 118.0,
    G_low=46.0 / 60.0 * 20.0 / 48.0 * 14.0 / 62.0,
    G_high=62.0 / 44.0 * 20.0 / 48.0 * 14.0 / 62.0,
    q_pos_low=0.12,
    q_pos_high=0.14,
    q_vel_low=1.0,
    q_vel_high=0.95,
    controller_poles=[0.82, 0.82])


def main(argv):
    argv = FLAGS(argv)
    glog.init()

    if len(argv) != 7:
        glog.error("Expected .h, .cc, and .json filenames")
    else:
        # Write the generated constants out to a file.
        drivetrain.WriteDrivetrain(argv[1:4],
                                   argv[4:7], ['motors', 'seems_reasonable'],
                                   kDrivetrain,
                                   scalar_type='float')


if __name__ == '__main__':
    sys.exit(main(sys.argv))
