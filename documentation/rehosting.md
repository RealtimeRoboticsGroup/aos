# Dependency Rehosting Guide

All Bazel external dependencies in this repository are downloaded through a mirror hosted at `realtimeroboticsgroup.org/build-dependencies`. This is enforced by `--downloader_config=tools/dependency_rewrite` in `.bazelrc`.

If you add or update an external dependency (via `MODULE.bazel` / Bzlmod or legacy `WORKSPACE` rules), Bazel will fail to fetch the files with a `404 Not Found` error because they are not yet present on the mirror.

## How to Rehost

To download and mirror a dependency file, you need write permissions on the GCP bucket `gs://austin-vpn-build-dependencies/`.

### For Users / Owners (with GCP write access)

Run the rehost script from the workspace root:

```bash
./tools/rehost.sh <domain-relative-url-of-dependency>
```

For example:

```bash
./tools/rehost.sh download.gnome.org/sources/glib/2.82/glib-2.82.2.tar.xz
```

If multiple files failed to download during a build, you can save the build output to a file and rehost them all at once:

```bash
./tools/rehost_all.sh <build_log_file>
```

### For AI Agents / Contributors (without GCP write access)

If you do not have write access to the GCP bucket (e.g. if you are an AI coding assistant), **you must ask the user to rehost the files for you**.

1. Identify all URLs that failed to download.
2. Present the exact `./tools/rehost.sh` commands to the user.
3. Ask the user to run the commands in their terminal and let you know when they are done.
