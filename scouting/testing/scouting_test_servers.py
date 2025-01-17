"""This library is here to run the various servers involved in the scouting app.

The servers are:
 - The fake TBA server
 - The actual web server
 - The postgres database
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
from typing import List

from rules_python.python.runfiles import runfiles

RUNFILES = runfiles.Create()


def wait_for_server(port: int):
    """Waits for the server at the specified port to respond to TCP connections."""
    while True:
        try:
            connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            connection.connect(("localhost", port))
            connection.close()
            break
        except ConnectionRefusedError:
            connection.close()
            time.sleep(0.01)


def create_db_config(tmpdir: Path) -> Path:
    config = tmpdir / "db_config.json"
    config.write_text(
        json.dumps({
            "username": "test",
            "password": "password",
            "port": 5432,
        }))
    return config


def create_tba_config(tmpdir: Path, year: int, event_code: str) -> Path:
    # Configure the scouting webserver to scrape data from our fake TBA server.
    config = tmpdir / "scouting_config.json"
    config.write_text(
        json.dumps({
            "api_key": "dummy_key_that_is_not_actually_used_in_this_test",
            "base_url": "http://localhost:7000",
            "year": year,
            "event_code": event_code,
        }))
    return config


def set_up_tba_api_dir(tmpdir: Path, year: int, event_code: str):
    tba_api_dir = tmpdir / "api" / "v3" / "event" / f"{year}{event_code}"
    tba_api_dir.mkdir(parents=True, exist_ok=True)
    (tba_api_dir / "matches").write_text(
        Path(
            RUNFILES.Rlocation(
                f"aos/scouting/scraping/test_data/{year}_{event_code}.json"
            )).read_text())


class Runner:
    """Helps manage the services we need for testing the scouting app."""

    def start(self,
              port: int,
              notify_fd: int = 0,
              year: int = 2016,
              event_code: str = "nytr"):
        """Starts the services needed for testing the scouting app.

        if notify_fd is set to a non-zero value, the string "READY" is written
        to that file descriptor once everything is set up.
        """

        self.tmpdir = Path(os.environ["TEST_TMPDIR"]) / "servers"
        self.tmpdir.mkdir(exist_ok=True)

        db_config = create_db_config(self.tmpdir)
        tba_config = create_tba_config(self.tmpdir,
                                       year=year,
                                       event_code=event_code)

        # The database needs to be running and addressable before the scouting
        # webserver can start.
        self.testdb_server = subprocess.Popen([
            RUNFILES.Rlocation(
                "aos/scouting/db/testdb_server/testdb_server_/testdb_server"
            )
        ])
        wait_for_server(5432)

        # Create a fake TBA server to serve the static match list.
        # Make sure it starts up first so that the webserver successfully
        # scrapes the TBA data on startup.
        set_up_tba_api_dir(self.tmpdir, year, event_code)
        self.fake_tba_api = subprocess.Popen(
            ["python3", "-m", "http.server", "7000"],
            cwd=self.tmpdir,
        )
        wait_for_server(7000)

        # Wait for the scouting webserver to start up.
        self.webserver = subprocess.Popen([
            RUNFILES.Rlocation("aos/scouting/scouting"),
            f"--port={port}",
            f"--db_config={db_config}",
            f"--tba_config={tba_config}",
        ])
        wait_for_server(port)

        if notify_fd:
            with os.fdopen(notify_fd, "w") as file:
                file.write("READY")

    def stop(self):
        """Stops the services needed for testing the scouting app."""
        servers = (self.webserver, self.testdb_server, self.fake_tba_api)
        for server in servers:
            server.terminate()
        for server in servers:
            server.wait()

        try:
            shutil.rmtree(self.tmpdir)
        except FileNotFoundError:
            pass


def discard_signal(signum, frame):
    """A NOP handler to ignore certain signals.

    We use signal.pause() to wait for a signal. That means we can't use the default handler. The
    default handler would tear the application down without stopping child processes.
    """
    pass


def main(argv: List[str]):
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",
                        type=int,
                        help="The port for the actual web server.")
    parser.add_argument(
        "--notify_fd",
        type=int,
        default=0,
        help=("If non-zero, indicates a file descriptor to which 'READY' is "
              "written when everything has started up."),
    )
    args = parser.parse_args(argv[1:])

    runner = Runner()
    runner.start(args.port, args.notify_fd)

    # Wait until we're asked to shut down via CTRL-C or SIGTERM.
    signal.signal(signal.SIGINT, discard_signal)
    signal.signal(signal.SIGTERM, discard_signal)
    signal.pause()

    runner.stop()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
