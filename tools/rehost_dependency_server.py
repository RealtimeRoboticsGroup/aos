#!/usr/bin/python3
"""A basic script to capture new dependencies files.

To use: run Bazel with an --experimental_downloader_config which directs
requests to localhost:8000 while this script is running.

This is intended for environments with a dedicated mirror server. This script is
run to capture new files added during development, and then all of the new files
are added to the mirror server once a change is ready for review. This
facilitates review of the files being added, and avoids adding files needed by
experiments which are abandoned (or further upgraded) before completion.

Bazel seems to overwhelm Python's HTTP server infrastructure sometimes. I had
success with --http_max_parallel_downloads=4, you may need a different value. If
this is too large, Bazel will exit with a generic "build interrupted" error. You
can try again, or make the value smaller if it's happening too often.
Oftentimes just trying multiple times is the fastest approach overall, because
it makes progress each time.

This script will likely not achieve native network performance, especially with
many downloads to a local mirror in parallel.

Note that this script can only catch dependencies which are actually downloaded
over the network. You may need to remove both the entire output tree
(bazel clean --expunge) and the content-addressable cache
($HOME/.cache/bazel/_bazel_$USER/cache) to ensure it catches everything.

Usage: rehost_dependency_server.py <TAR> <CONFIG>

  <TAR> is the .tar file to create/read/modify with new dependencies.

  <CONFIG> is the "normal" Bazel downloader_config file. This is NOT the one
           which directs to this script. This script will parse the normal one
           to understand which URLs to save vs not. Note that this parsing is
           somewhat limited and will mis-parse some downloader_configs.
"""

import contextlib
import http.server
import re
import shutil
import stat
import sys
import tarfile
import tempfile
import threading

import requests

TAR_BUFSIZE = 16 * 1024 * 1024


class RequestHandler(http.server.BaseHTTPRequestHandler):

    def __init__(self, tar_path, tar_lock, parsed_downloader_config, *args,
                 **kwargs):
        self._tar_path = tar_path
        self._tar_lock = tar_lock
        self._parsed_downloader_config = parsed_downloader_config
        super().__init__(*args, **kwargs)

    def rewrite_path(self, normalized_path):
        for orig_path, new_template in self._parsed_downloader_config:
            if m := orig_path.match(normalized_path):
                return m.expand(new_template)

    def do_GET(self):
        path = self.path
        if '?' in path:
            self.send_error(400, 'No query string allowed')
            return
        assert path[0] == '/'
        normalized_path = path[1:]
        upstream_url = 'https://' + normalized_path

        rewritten_path = self.rewrite_path(normalized_path)
        if rewritten_path is not None:
            if '://' not in rewritten_path:
                rewritten_path = 'http://' + rewritten_path
            if self.stream_response_from(rewritten_path):
                return
            if self.stream_response_from_tar(normalized_path):
                return
            if self.stream_response_from(upstream_url, normalized_path):
                self.log_message('Added file to tar: %s', normalized_path)
                return
        else:
            if self.stream_response_from(upstream_url):
                self.log_message(
                    'Did not match downloader_config, downloaded directly: %s',
                    normalized_path)
                return

        self.log_error('Failed to fetch from anywhere for: %s', upstream_url)
        self.send_error(500, 'Ran out of places to try')

    def stream_response_from_tar(self, normalized_path):
        with self._tar_lock.read(), contextlib.ExitStack() as exit_stack:
            try:
                tar = exit_stack.enter_context(
                    tarfile.open(self._tar_path, 'r'))
            except (FileNotFoundError, tarfile.ReadError):
                return False
            try:
                member = tar.getmember(normalized_path)
            except KeyError:
                return False
            self.send_response(200)
            self.end_headers()
            with tar.extractfile(member) as f:
                shutil.copyfileobj(f, self.wfile, TAR_BUFSIZE)
            return True

    def stream_response_from(self, url, tar_member_path=None):
        with requests.get(
                url,
                headers=
            {
                # The crates.io usage policy at <https://crates.io/data-access>
                # requires a User-Agent with contact info. In my experience,
                # they allow quite a few requests without one, but eventually
                # start returning 403 responses.
                'User-Agent':
                'Bazel dependency downloader, https://github.com/RealtimeRoboticsGroup/aos',
            },
                stream=True) as r, contextlib.ExitStack() as exit_stack:
            if r.status_code == 404:
                return False
            if r.status_code != 200:
                self.log_error('Response for %s: %s: %s', url, r, r, content)
                self.send_response(r.status_code)
                self.end_headers()
                return True

            temp_buffer = None
            if tar_member_path is not None:
                temp_buffer = exit_stack.enter_context(
                    tempfile.SpooledTemporaryFile(32 * TAR_BUFSIZE))
            self.send_response(200)
            self.end_headers()

            total_size = 0
            for chunk in r.iter_content(chunk_size=None):
                total_size += len(chunk)
                try:
                    self.wfile.write(chunk)
                except BrokenPipeError:
                    # This happens when Bazel aborts, usually because we
                    # responded too slowly. Finish downloading and saving
                    # the file regardless so it's faster next time.
                    pass
                if temp_buffer is not None:
                    temp_buffer.write(chunk)

            if tar_member_path is not None:
                assert temp_buffer.tell() == total_size
                temp_buffer.seek(0)
                tarinfo = tarfile.TarInfo(tar_member_path)
                tarinfo.size = total_size
                tarinfo.mode = stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH
                with self._tar_lock.write():
                    already_included = False
                    try:
                        with tarfile.open(self._tar_path, 'r') as tar:
                            try:
                                tar.getmember(tar_member_path)
                                already_included = True
                                self.log_message(
                                    'Rejecting duplicate tar member: %s',
                                    tar_member_path)
                            except KeyError:
                                pass
                    except FileNotFoundError:
                        self.log_message('No tar file yet')
                    if not already_included:
                        with tarfile.open(self._tar_path, 'a') as tar:
                            tar.addfile(tarinfo, temp_buffer)
            return True


class RWLock:

    def __init__(self):
        self._reader_count_lock = threading.Lock()
        self._reader_count = 0
        self._writer_lock = threading.Lock()

    @contextlib.contextmanager
    def read(self):
        try:
            with self._reader_count_lock:
                self._reader_count += 1
                if self._reader_count == 1:
                    self._writer_lock.acquire()
            yield
        finally:
            with self._reader_count_lock:
                self._reader_count -= 1
                if self._reader_count == 0:
                    self._writer_lock.release()

    def write(self):
        return self._writer_lock


class RequestHandlerFactory:

    def __init__(self, tar_path, parsed_downloader_config):
        self._tar_path = tar_path
        self._parsed_downloader_config = parsed_downloader_config
        self._tar_lock = RWLock()

    def __call__(self, *args, **kwargs):
        return RequestHandler(self._tar_path, self._tar_lock,
                              self._parsed_downloader_config, *args, **kwargs)


def main():
    tar_path = sys.argv[1]
    downloader_config = sys.argv[2]

    parsed_downloader_config = list()
    # The best documentation for parsing this seems to be the Bazel source:
    #   https://github.com/bazelbuild/bazel/blob/09c621e4cf5b968f4c6cdf905ab142d5961f9ddc/src/main/java/com/google/devtools/build/lib/bazel/repository/downloader/UrlRewriterConfig.java
    with open(downloader_config, 'r') as c:
        for line in c:
            split = re.split('\\s+', line.strip())
            if split[0] != 'rewrite':
                continue
            if len(split) != 3:
                print('Warning: unable to parse downloader_config line: %s' %
                      line,
                      file=sys.stderr)
                continue
            parsed_downloader_config.append(
                (re.compile(split[1]), split[2].replace('$', '\\')))
    if not parsed_downloader_config:
        print(
            'Warning: did not find any rewrite lines in downloader_config, not filtering anything',
            file=sys.stderr)

    http.server.ThreadingHTTPServer(
        ('localhost', 8000),
        RequestHandlerFactory(tar_path, parsed_downloader_config),
    ).serve_forever()


if __name__ == '__main__':
    main()
