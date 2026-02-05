#!/usr/bin/env python3
# This script is meant to act as a test to confirm that our log_to_mcap converter produces
# a valid MCAP file. To do so, it first generates an AOS log, then converts it to MCAP, and
# then runs the "mcap doctor" tool on it to confirm compliance with the standard.
import argparse
import os
import re
import subprocess
import sys
import tempfile
import time
from typing import Sequence, Text

# Regexes for extracting the number of messages in the MCAP for specific channels.
LOG_MESSAGE_FBS_RE = re.compile(
    "\(2\) \s*/aos aos.logging.LogMessageFbs \s*(\d+) msgs")
CLOCK_TIMEPOINTS_RE = re.compile(
    "\(6\) \s*/clocks aos.ClockTimepoints \s*(\d+) msgs")

# Regexes for extracting time information about the MCAP.
INFO_DURATION_RE = re.compile("duration:\s+([\d\.hms]+)")
INFO_START_RE = re.compile("start:\s+([\d\.]+)")

# Duration and start time for when the MCAP has no messages in it.
INFO_NO_DURATION = "2562047h47m16.854775807s"
INFO_NO_START = "0.000000000"


def make_permutations(options):
    if len(options) == 0:
        return [[]]
    permutations = []
    for option in options[0]:
        for sub_permutations in make_permutations(options[1:]):
            permutations.append([option] + sub_permutations)
    return permutations


def generate_argument_permutations():
    arg_sets = [["--compress", "--nocompress"],
                ["--mode=flatbuffer", "--mode=json"],
                ["--canonical_channel_names", "--nocanonical_channel_names"],
                ["--mcap_chunk_size=1000", "--mcap_chunk_size=10000000"],
                ["--fetch=none", "--fetch=all", "--fetch=rewrite"],
                ["--include_channels=", "--include_channels=.*"],
                ["--drop_channels=", "--drop_channels=.*aos.examples.Pong"]]
    permutations = make_permutations(arg_sets)
    return permutations


def filter_stdout(stdout: str) -> str:
    """Filters currently-unhandled messages from the mcap CLI.

    We should probably fix these messages, but that can be a future effort.
    """
    lines = stdout.splitlines()
    # Ignore these kinds of messages for now:
    # Message.log_time X on "/test aos.examples.Pong" is less than the latest log time Y
    filtered_lines = filter(
        lambda line: "is less than the latest log time" not in line, lines)
    return "\n".join(filtered_lines)


def main(argv: Sequence[Text]):
    parser = argparse.ArgumentParser()
    parser.add_argument("--log_to_mcap",
                        required=True,
                        help="Path to log_to_mcap binary.")
    parser.add_argument("--mcap", required=True, help="Path to mcap binary.")
    parser.add_argument("--generate_log",
                        required=True,
                        help="Path to logfile generator.")
    args = parser.parse_args(argv)
    log_to_mcap_argument_permutations = generate_argument_permutations()
    for log_to_mcap_args in log_to_mcap_argument_permutations:
        with tempfile.TemporaryDirectory() as tmpdir:
            log_name = tmpdir + "/test_log/"
            mcap_name = tmpdir + "/log.mcap"
            print(f"Running with arguments: {log_to_mcap_args}")
            subprocess.run([args.generate_log, "--output_folder",
                            log_name]).check_returncode()
            # Run with a really small chunk size, to force a multi-chunk file.
            subprocess.run([
                args.log_to_mcap,
                "--mcap_chunk_size=1000",
                "--mode=json",
                # For simplicity, test with a single second delta for latched messages.
                "--rewrite_timestamp_delta_seconds=1",
                log_name,
            ] + log_to_mcap_args + [mcap_name]).check_returncode()
            # MCAP attempts to find $HOME/.mcap.yaml, and dies on $HOME not existing. So
            # give it an arbitrary config location (it seems to be fine with a non-existent config).
            doctor_result = subprocess.run([
                args.mcap, "doctor", mcap_name, "--config",
                tmpdir + "/.mcap.yaml"
            ],
                                           stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE,
                                           encoding='utf-8')
            print("STDOUT:", doctor_result.stdout)
            print("STDERR:", doctor_result.stderr)
            # mcap doctor doesn't actually return a non-zero exit code on certain failures...
            # See https://github.com/foxglove/mcap/issues/356
            if len(doctor_result.stderr) != 0:
                print("Didn't expect any stderr output.")
                return 1
            filtered_stdout = filter_stdout(doctor_result.stdout)
            if filtered_stdout != "Header.profile field \"x-aos\" is not a well-known profile.":
                print("Only expected one line of stdout. Got: ",
                      filtered_stdout)
                return 1
            doctor_result.check_returncode()

            # Validate that we dropped or fetched the messages appropriately.
            info = subprocess.check_output(
                [args.mcap, "info", mcap_name],
                env=os.environ.copy() | {
                    # For some reason `mcap info` requires HOME.
                    "HOME": "/nonexistent"
                }).decode("utf-8")

            # By default assume we dropped all the channels.
            expected_duration = INFO_NO_DURATION
            expected_start_time = INFO_NO_START

            # Validate that log_to_mcap can fetch messages appropriately. This is only possible when
            # we're not dropping all messages.
            if "--include_channels=" not in log_to_mcap_args:
                if "--fetch=all" in log_to_mcap_args:
                    # We expect this message to be in the log.
                    expected_num_log_message_fbs = 1
                    # ClockTimepoints messages start at time zero. So the MCAP starts then too. That
                    # means that a 10s log starting at time=10s gives us a ~20s MCAP file.
                    if "--drop_channels=" in log_to_mcap_args:
                        expected_duration = "19.99005s"
                    else:
                        expected_duration = "19.99s"
                    expected_start_time = "0.000000000"
                    expected_num_clock_timepoints = 20

                elif "--fetch=rewrite" in log_to_mcap_args:
                    # We expect this message to be in the MCAP, but the timestamp will be rewritten.
                    expected_num_log_message_fbs = 1
                    # Since we rewrite fetched messages' timestamps to ~1s before the start of the
                    # log, the 10s log turns into an 11s MCAP.
                    if "--drop_channels=" in log_to_mcap_args:
                        expected_duration = "10.99005s"
                    else:
                        expected_duration = "10.99s"
                    expected_start_time = "9.000000000"
                    # We only get ClockTimepoints for the duration of the log.
                    expected_num_clock_timepoints = 10

                else:
                    expected_num_log_message_fbs = 0
                    # No message fetching. We have a ~10s MCAP file.
                    if "--drop_channels=" in log_to_mcap_args:
                        expected_duration = "9.99005s"
                    else:
                        expected_duration = "9.99s"
                    expected_start_time = "10.000000000"
                    expected_num_clock_timepoints = 10

                log_message_fbs = LOG_MESSAGE_FBS_RE.findall(info)
                if log_message_fbs != [str(expected_num_log_message_fbs)]:
                    print(
                        "Expected {} LogMessageFbs messages. Found: {}".format(
                            expected_num_log_message_fbs, log_message_fbs))
                    print(info)
                    return 1

                clock_timepoints = CLOCK_TIMEPOINTS_RE.findall(info)
                if clock_timepoints != [str(expected_num_clock_timepoints)]:
                    print("Expected {} ClockTimepoints messages. Found: {}".
                          format(expected_num_clock_timepoints,
                                 clock_timepoints))
                    print(info)
                    return 1

            # Validate the start time and the duration of the MCAP.
            try:
                actual_duration = INFO_DURATION_RE.findall(info)[0]
                actual_start_time = INFO_START_RE.findall(info)[0]
            except IndexError:
                print("Failed to find duration and start time.")
                print(info)
                return 1

            if expected_duration != actual_duration:
                print(
                    f"Expected duration of {expected_duration}, but got {actual_duration}."
                )
                print(info)
                return 1
            if expected_start_time != actual_start_time:
                print(
                    f"Expected start_time of {expected_start_time}, but got {actual_start_time}."
                )
                print(info)
                return 1

            if not (match := re.search(r"channels: (\d+)", info)):
                print("Couldn't find the number of channels in:")
                print(info)
                return 1

            num_channels = int(match.group(1))
            # We expect one fewer channels when we drop the Pong channel.
            if "--include_channels=" in log_to_mcap_args:
                expected_num_channels = 0
            elif "--drop_channels=" in log_to_mcap_args:
                expected_num_channels = 10
            else:
                expected_num_channels = 9
            if num_channels != expected_num_channels:
                print(
                    f"Expected {expected_num_channels} channels, but found {num_channels} instead."
                )
                return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
