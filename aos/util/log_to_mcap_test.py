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
                ["--fetch", "--nofetch"],
                ["--include_channels=", "--include_channels=.*"],
                ["--drop_channels=", "--drop_channels=.*aos.examples.Pong"]]
    permutations = make_permutations(arg_sets)
    print(permutations)
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
            subprocess.run([args.generate_log, "--output_folder",
                            log_name]).check_returncode()
            # Run with a really small chunk size, to force a multi-chunk file.
            subprocess.run([
                args.log_to_mcap, "--mcap_chunk_size", "1000", "--mode",
                "json", log_name
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

            # Validate that we dropped the messages appropriately.
            info = subprocess.check_output(
                [args.mcap, "info", mcap_name],
                env=os.environ.copy() | {
                    # For some reason `mcap info` requires HOME.
                    "HOME": "/nonexistent"
                }).decode("utf-8")
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
