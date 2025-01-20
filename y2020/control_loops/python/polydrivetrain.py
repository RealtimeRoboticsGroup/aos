#!/usr/bin/python3

import sys
from y2020.control_loops.python import drivetrain
from frc.control_loops.python import polydrivetrain

import gflags
import glog

__author__ = 'Austin Schuh (austin.linux@gmail.com)'

FLAGS = gflags.FLAGS

try:
    gflags.DEFINE_bool('plot', False, 'If true, plot the loop response.')
except gflags.DuplicateFlagError:
    pass


def main(argv):
    if FLAGS.plot:
        polydrivetrain.PlotPolyDrivetrainMotions(drivetrain.kDrivetrain)
    elif len(argv) != 10:
        glog.fatal('Expected .h, .cc, and .json filenames')
    else:
        polydrivetrain.WritePolyDrivetrain(argv[1:4], argv[4:7], argv[7:10],
                                           'y2020', drivetrain.kDrivetrain)


if __name__ == '__main__':
    argv = FLAGS(sys.argv)
    glog.init()
    sys.exit(main(argv))
