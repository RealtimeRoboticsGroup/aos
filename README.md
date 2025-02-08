# Build Status
[![Build status](https://badge.buildkite.com/fcbebde155a2437db2ddd5caf1c6d3d49e8302c621289e337e.svg?branch=main)](https://buildkite.com/realtimeroboticsgroup/ci)

# Introduction

Welcome to AOS's main code repository. It contains two things:
* AOS ("Autonomous Operating System") is an open source event-loop based framework for realtime robotic systems. You can learn more about it [here](documentation/aos/docs/index.md).
* A collection of software useful for robots, particularly the kind used in the [First Robotics Competition](https://www.firstinspires.org/robotics/frc).

Pleaes note: AOS moved here fairly recently and we are still cleaning up a few things, especially documentation.

# Contributing
All development of AOS is done on our gerrit instance at https://realtimeroboticsgroup.org/gerrit with github being a read-only mirror.  We are happy to add external contributors.  If you are interested, reach out to Austin Schuh and he will help you get access.  In case of disputes over if a patch should be taken or not, Austin has final say.

Submissions must be made under the terms of the following Developer Certificate of Origin.

By making a contribution to this project, I certify that:

        (a) The contribution was created in whole or in part by me and I
            have the right to submit it under the open source license
            indicated in the file; or

        (b) The contribution is based upon previous work that, to the best
            of my knowledge, is covered under an appropriate open source
            license and I have the right under that license to submit that
            work with modifications, whether created in whole or in part
            by me, under the same open source license (unless I am
            permitted to submit under a different license), as indicated
            in the file; or

        (c) The contribution was provided directly to me by some other
            person who certified (a), (b) or (c) and I have not modified
            it.

        (d) I understand and agree that this project and the contribution
            are public and that a record of the contribution (including all
            personal information I submit with it, including my sign-off) is
            maintained indefinitely and may be redistributed consistent with
            this project or the open source license(s) involved.

To do this, add the following to your commit message.  Gerrit will enforce that all commits have been signed off.

	Signed-off-by: Random J Developer <random@developer.example.org>

Git has support for adding Signed-off-by lines by using `git commit -s`, or you can setup a git commit hook to automatically sign off your commits.  [Stack Overflow](https://stackoverflow.com/questions/15015894/git-add-signed-off-by-line-using-format-signoff-not-working) has instructions for how to do this if you are interested.
