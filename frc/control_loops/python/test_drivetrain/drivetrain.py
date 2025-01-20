#!/usr/bin/python3

from __future__ import print_function
from frc.control_loops.python import drivetrain
import sys

import gflags
import glog

FLAGS = gflags.FLAGS

gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')

# Values stolen from y2018 (most recent year with shifters).
kDrivetrain = drivetrain.DrivetrainParams(
    J=6.0,
    mass=68.0,
    robot_radius=0.616 / 2.0,
    wheel_radius=0.127 / 2.0 * 120.0 / 118.0,
    G_low=46.0 / 60.0 * 20.0 / 48.0 * 14.0 / 62.0,
    G_high=62.0 / 44.0 * 20.0 / 48.0 * 14.0 / 62.0,
    q_pos_low=0.12,
    q_pos_high=0.14,
    q_vel_low=1.0,
    q_vel_high=0.95,
    efficiency=0.70,
    has_imu=True,
    force=True,
    kf_q_voltage=13.0,
    controller_poles=[0.82, 0.82],
    robot_cg_offset=0.0,
)


def main(argv):
    argv = FLAGS(argv)
    glog.init()

    if FLAGS.plot:
        drivetrain.PlotDrivetrainMotions(kDrivetrain)
    elif len(argv) != 7:
        print("Expected .h, .cc, and .json file names.")
    else:
        # Write the generated constants out to a file.
        drivetrain.WriteDrivetrain(
            argv[1:4], argv[4:7],
            ['frc', 'control_loops', 'python', 'test_drivetrain'], kDrivetrain)


if __name__ == '__main__':
    sys.exit(main(sys.argv))
