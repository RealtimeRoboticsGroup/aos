#!/usr/bin/env python3

from __future__ import annotations
import contextlib
import unicodedata
import jinja2
import pathlib
from absl import logging
import re
import os
import shlex
import subprocess
import apt_pkg
import sys
import functools
import tempfile

apt_pkg.init_system()


@contextlib.contextmanager
def scoped_loopback(image):
    """Mounts an image as a loop back device."""
    result = subprocess.run(["sudo", "losetup", "--show", "-f", image],
                            check=True,
                            stdout=subprocess.PIPE)
    device = result.stdout.decode('utf-8').strip()
    logging.info("Mounted", image, "to", repr(device), file=sys.stderr)
    try:
        yield device
    finally:
        subprocess.run(["sudo", "losetup", "-d", device], check=True)


@contextlib.contextmanager
def scoped_mount(image):
    """Mounts an image as a partition."""
    partition = f"{image}.partition"
    try:
        os.mkdir(partition)
    except FileExistsError:
        pass

    result = subprocess.run(["sudo", "mount", "-o", "loop", image, partition],
                            check=True)

    try:
        yield partition
    finally:
        subprocess.run(
            ["sudo", "rm", f"{partition}/usr/bin/qemu-aarch64-static"])
        subprocess.run(["sudo", "umount", partition], check=True)


def mount_and_bash(image):
    """Helper function to just mount and open a bash interface
    To run from the CLI, call
    python3 -c "from rootfs_utils import *; mount_and_bash('/home/austin/local/frc4646/tegra-demo-distro/')"
    """
    with scoped_tmpdir_tegraflash_rootfs(image) as rootfs:
        with scoped_mount(rootfs) as partition:
            subprocess.run([
                "sudo", "cp", "/usr/bin/qemu-aarch64-static",
                f"{partition}/usr/bin/"
            ],
                           check=True)

            target(partition, ["/bin/bash"])


@contextlib.contextmanager
def scoped_tmpdir_tegraflash_rootfs(image):
    """Extracts a tegraflash image and mounts the root filesystem out of it.

    For use with "with", and removes it when it is done.
    """
    with tempfile.TemporaryDirectory() as tempdir:
        extracted_tegraflash = os.path.join(tempdir, 'tegraflash')
        os.mkdir(extracted_tegraflash)

        logging.info(f'Extracting original image to {extracted_tegraflash}')
        subprocess.run([
            "tar",
            "xf",
            image,
            "-C",
            extracted_tegraflash,
        ],
                       check=True)

        yield os.path.join(extracted_tegraflash, 'demo-image-base.ext4')


@contextlib.contextmanager
def scoped_tmpdir_tarball(image):
    """Extracts a tegraflash image and mounts the root filesystem out of it.

    For use with "with", and removes it when it is done.
    """
    with tempfile.TemporaryDirectory() as tempdir:
        logging.info(f'Extracting original image to {tempdir}')
        subprocess.run([
            "tar",
            "xf",
            image,
            "-C",
            tempdir,
        ], check=True)

        yield tempdir


def check_buildifier():
    """Checks if buildifier is in the path"""
    result = subprocess.run(["which", "buildifier"], stdout=subprocess.PIPE)
    if result.stdout.decode('utf-8') == "":
        return False
    else:
        return True


def check_required_deps(deps):
    """Checks if the provided list of dependencies is installed."""
    missing_deps = []
    for dep in deps:
        result = subprocess.run(["dpkg-query", "-W", "-f='${Status}'", dep],
                                check=True,
                                stdout=subprocess.PIPE)

        if "install ok installed" not in result.stdout.decode('utf-8'):
            missing_deps.append(dep)

    if len(missing_deps) > 0:
        logging.error("Missing dependencies, please install:", file=sys.stderr)
        logging.error("sudo apt-get install", " ".join(missing_deps), file=sys.stderr)
        exit()


def target_unescaped(partition, cmd):
    """Runs a command as root with bash -c cmd, ie without escaping."""
    subprocess.run([
        "sudo", "chroot", "--userspec=0:0", partition, "qemu-aarch64-static",
        "/bin/bash", "-c", cmd
    ],
                   check=True)


def target(partition, cmd):
    """Runs a command as root with escaping."""
    target_unescaped(partition, shlex.join([shlex.quote(c) for c in cmd]))


def is_elf_file(filename):
    """
    Checks if the given file is an ELF file by examining its magic number.

    Args:
        filename: The path to the file.

    Returns:
        True if the file is an ELF file, False otherwise.
    """
    with open(filename, 'rb') as f:
        magic_bytes = f.read(4)
        return magic_bytes == b'\x7f\x45\x4c\x46'


def detect_static_library(filepath):
    """
    Autodetects if a given file is a static library on Linux by reading its magic number.

    Args:
        filepath: The path to the file.

    Returns:
        True if the file is a static library, False otherwise.
        Returns None if the file does not exist or an error occurs.
    """
    if not os.path.exists(filepath):
        return None

    try:
        with open(filepath, 'rb') as f:
            # Read the first 8 bytes (the magic number for ar archives)
            magic_number = f.read(8)

            # Check if the magic number matches the ar archive signature
            if magic_number == b'!<arch>\n':  # b' indicates bytes literal
                return True
            else:
                return False

    except OSError:
        logging.error(f"Error reading file: {filepath}")
        return None


def parse_linker_script(script):
    """Parses a GNU linker script to determine the shared libraries included.

    Args:
      script: The linker script as a string.

    Returns:
      A list of shared library names.
    """

    shared_libs = []

    # Normalize the string to handle all kinds of whitespace characters
    script = unicodedata.normalize('NFKC', script)
    script = ' '.join(script.split())

    # Use a more general regex to capture anything within parentheses
    pattern = r'(?:INPUT|GROUP)\s*\(\s*(.+?)\s*\)'
    matches = re.findall(pattern, script)

    for match in matches:
        # Split the match into individual entries, handling AS_NEEDED clauses separately
        entries = re.findall(
            r'(?:AS_NEEDED\s*\(\s*(.+?)\s*\))|([\w\-\/\.]+\.(?:so|a)(?:\.[0-9]+)?|-l\w+)',
            match)
        for entry in entries:
            for lib in entry:
                if lib:
                    # If it's a -l flag, convert it to the corresponding library name
                    if lib.startswith('-l'):
                        # TODO(austin): This should really search for lib*.a, not just blindly match libgcc.
                        # That being said, this is the only use case, so it isn't worth being too generic yet.
                        if lib == '-lgcc':
                            continue
                        library_name = 'lib' + lib[2:] + '.so'
                    else:
                        library_name = lib

                    shared_libs.append(library_name.strip())

    return shared_libs


def read_linker_script(filename):
    """Reads a linker script from a file and returns its content as a string.

    Args:
      filename: The path to the linker script file.

    Returns:
      The content of the linker script as a string.
    """
    try:
        with open(filename, 'r') as f:
            script_content = f.read()
        return script_content
    except FileNotFoundError:
        logging.error(f"Error: Linker script file not found: {filename}")
        return None


class NameVersion:
    """Class representing a package name and optionally a version constraint. """

    def __init__(self, nameversion: str):
        # We are processing package names here like:
        #   python3:any
        #   python3-markdown (= 3.4.1-2)
        if '(' in nameversion:
            s = nameversion.split(' (')
            self.name = s[0].strip()

            v = s[1][:-1].split(' ')

            self.operator = v[0]
            self.version = v[1]
        else:
            self.name = nameversion.strip()
            self.operator = None
            self.version = None

        # Rip off :amd64 or :aarch64 from the name if it is here.
        if ':' in self.name:
            self.name = self.name.split(':')[0]

    def matches(self, other: NameVersion) -> bool:
        """If self meets the requirements defined by other."""
        if other.name != self.name:
            return False

        if other.operator is None:
            return True

        # libz1 is special and doesn't have a version...  Don't stress it for now until we learn why.
        if self.operator is None and self.name == 'libz1':
            return True

        vc = apt_pkg.version_compare(self.version, other.version)
        if vc < 0:
            return other.operator in ('<=', '<<')
        elif vc == 0:
            return other.operator in ('=', '>=', '<=')
        elif vc > 0:
            return other.operator in ('>=', '>>')

    def __repr__(self) -> str:
        if self.operator is not None:
            return f"NameVersion({self.name} ({self.operator} {self.version}))"
        else:
            return f"NameVersion({self.name})"


class Package:
    """Class representing a deian package."""

    def __init__(self, name: str, provides: str, version: str, depends: str,
                 files: list[str]):
        self.name = NameVersion(f"{name} (= {version})")

        self.provides = [self.name]

        if provides:
            for package_and_version in provides.split(","):
                self.provides.append(NameVersion(package_and_version))

        self.depends = []
        if depends:
            for package_and_version in depends.split(", "):
                if ' | ' in package_and_version:
                    oneof = []
                    for oneof_package_and_version in package_and_version.split(
                            ' | '):
                        oneof.append(NameVersion(oneof_package_and_version))
                    self.depends.append(oneof)
                else:
                    self.depends.append([NameVersion(package_and_version)])

        self.files = files

    def update_filetypes(self, directories: set[str], symlinks: dict[str,
                                                                     str]):
        if hasattr(self, 'directories') or hasattr(self, 'symlinks'):
            return

        self.directories = []
        self.symlinks = dict()
        files = []
        for f in self.files:
            if f in directories:
                self.directories.append(f)
            elif f in symlinks:
                self.symlinks[f] = symlinks[f]
            else:
                files.append(f)

        self.files = files

    def matches(self, other: NameVersion) -> bool:
        """If self meets the requirements defined by other."""
        return any(p.matches(other) for p in self.provides)

    def resolved_depends(self, packages: dict[Package]) -> list[Package]:
        result = set()

        # The dependencies are lists of lists of dependencies.  At least one
        # element from each inner list needs to match for it to be valid.  Most
        # of the dependencies are going to be a single element list.
        for p_or_list in self.depends:
            resolved_set = set()
            for oneof_package in p_or_list:
                if oneof_package.name not in packages:
                    continue

                resolved_oneof_package = packages[oneof_package.name]
                if resolved_oneof_package.matches(oneof_package):
                    resolved_set.add(resolved_oneof_package)

            if len(resolved_set) == 0:
                raise RuntimeError(
                    f"Failed to find dependencies for {p_or_list}: {repr(self)}"
                )

            result.update(resolved_set)

        return sorted(list(result), key=lambda x: x.name.name)

    def headers(self) -> list[str]:
        return [h for h in self.files if h.startswith('/usr/include')]

    def objects(self) -> list[str]:
        result = []
        for file in self.files:
            if not file.startswith('/usr'):
                continue

            # Gotta love GDB extensions ...libc.so....py.  Ignore them.
            if file.endswith('.py'):
                continue

            # We want to find things like libfoo.so.1.2.3.4.5.  The .so needs to be last.
            opath = file
            found_so = False
            while True:
                opath, ext = os.path.splitext(opath)
                if ext == '':
                    break
                elif ext == '.so':
                    found_so = True

            if found_so:
                result.append(file)

        return sorted(result)

    def __repr__(self) -> str:
        return f"{{ {repr(self.provides[0])}, \"provides\": {repr(self.provides[1:])}, \"depends\": {repr(self.depends)} }}"


class PkgConfig:
    """Represents a pkg-config file for a library."""

    def __init__(self, contents, package, filename):
        # The pkgconfig file format lets you specify variables and the expand
        # them into the various fields.  These are in the form
        #   asdf=15234
        self.variables = dict()

        self.package = package
        self.libs = []
        self.cflags = []
        self.requires = []
        for line in contents.split('\n'):
            line = line.strip()
            # Parse everything so we learn if a new field shows up we don't
            # know how to parse.
            if line == '':
                pass
            elif line[0] == '#':
                pass
            elif line.startswith('Name:'):
                self.name = self.expand(line.removeprefix('Name:').strip())
            elif line.startswith('Description:'):
                self.description = self.expand(
                    line.removeprefix('Description:').strip())
            elif line.startswith('Url:'):
                pass
            elif line.startswith('Version:'):
                self.version = self.expand(
                    line.removeprefix('Version:').strip())
            elif line.startswith('Libs:'):
                self.libs = self.expand(
                    line.removeprefix('Libs:').strip()).split()
            elif line.startswith('Cflags:'):
                self.cflags = self.expand(
                    line.removeprefix('Cflags:').strip()).split()
            elif line.startswith('URL:'):
                pass
            elif line.startswith('Cflags.private:'):
                pass
            elif line.startswith('Libs.Private:'):
                pass
            elif line.startswith('Requires:'):
                # Parse a Requires line of the form:
                # Requires: glib-2.0 >= 2.56.0, gobject-2.0
                self.requires += [
                    f.split()[0] for f in self.expand(
                        line.removeprefix('Requires:').strip()).split(',') if f
                ]
            elif line.startswith('Requires.private:'):
                # Parse a Requires.private line of the form:
                # Requires.private: gmodule-2.0
                self.requires += [
                    f.split()[0] for f in self.expand(
                        line.removeprefix('Requires.private:').strip()).split(
                            ',') if f
                ]
            elif line.startswith('Libs.private:'):
                pass
            elif line.startswith('Conflicts:'):
                pass
            elif re.match('^[-a-zA-Z_0-9]* *=.*$', line):
                split_line = re.split(' *= *', line)
                self.variables[split_line[0]] = self.expand(split_line[1])
            else:
                raise ValueError('Unknown line in pkgconfig file ' + filename +
                                 ": " + repr(line))

        if self.name is None:
            raise RuntimeError("Failed to find Name.")

    def expand(self, line: str) -> str:
        """ Expands a string with variable expansions in it like bash (${foo}). """
        for var in self.variables:
            line = line.replace('${' + var + '}', self.variables[var])
        return line


class Filesystem:
    """Represents a full root filesystem of packages, libraries, and pkgconf."""

    def __init__(self, partition, sudo_bash):
        self.partition = partition
        # TODO(austin): I really want to be able to run this on an amd64
        # filesystem too, which won't work with qemu-aarch64-static.  Pull it
        # into a library.
        result = subprocess.run(sudo_bash + [
            "-c",
            "dpkg-query -W -f='Version: ${Version}\nPackage: ${Package}\nProvides: ${Provides}\nDepends: ${Depends}\n${db-fsys:Files}--\n'"
        ],
                                check=True,
                                stdout=subprocess.PIPE)

        # Mapping from all package names (str) to their corresponding Package
        # objects for that package.
        self.packages = dict()

        package_in_progress = {'files': []}
        files = set()
        for line in result.stdout.decode('utf-8').strip().split('\n'):
            if line == '--':
                # We found the end of line deliminator, save the package and
                # clear everything out.
                new_package = Package(package_in_progress['Package'],
                                      package_in_progress['Provides'],
                                      package_in_progress['Version'],
                                      package_in_progress['Depends'],
                                      package_in_progress['files'])

                for provides in new_package.provides:
                    self.packages[provides.name] = new_package

                # Wipe everything so we detect if any fields are missing.
                package_in_progress = {'files': []}
            elif line.startswith("Version: "):
                package_in_progress['Version'] = line.removeprefix("Version: ")
            elif line.startswith("Package: "):
                package_in_progress['Package'] = line.removeprefix("Package: ")
            elif line.startswith("Provides: "):
                package_in_progress['Provides'] = line.removeprefix(
                    "Provides: ")
            elif line.startswith("Depends: "):
                package_in_progress['Depends'] = line.removeprefix("Depends: ")
            else:
                assert (line.startswith(' '))
                f = line.removeprefix(' ')
                if f not in [
                        # NVIDIA seems to muck up the redirects for these libraries.  Ignore them.
                        '/usr/lib/x86_64-linux-gnu/libEGL.so.1.1.0',
                        '/usr/lib/x86_64-linux-gnu/libGL.so.1.7.0',
                        '/usr/lib/x86_64-linux-gnu/libGLESv1_CM.so.1.2.0',
                        '/usr/lib/x86_64-linux-gnu/libGLESv2.so.2.1.0',
                        # This doesn't render well into bazel.
                        # And it isn't useful enough to be worth fighting.
                        '/lib/systemd/system/system-systemd\\x2dcryptsetup.slice',
                ]:
                    package_in_progress['files'].append(f)
                    files.add(f)

        self.directories = set()
        self.symlinks = dict()

        for root, walked_dirs, walked_files in os.walk(self.partition):
            for entry in walked_files + walked_dirs:
                full_target = os.path.join(root, entry)
                if pathlib.Path(full_target).is_symlink():
                    target = full_target.removeprefix(self.partition)
                    self.symlinks[target] = os.readlink(full_target)

        for file in files:
            full_target = f"{self.partition}/{file}"
            try:
                if pathlib.Path(full_target).is_symlink():
                    self.symlinks[file] = os.readlink(full_target)

                if pathlib.Path(full_target).is_dir():
                    self.directories.add(file)
            except PermissionError:
                # Assume it is a file...
                logging.error("Failed to read", file, file=sys.stderr)
                pass

            # Directories are all the things before the last /
            for parent in pathlib.Path(file).parents:
                self.directories.add(str(parent))

        # Now, populate self.files with a mapping from each file to the owning
        # package so we can do file ownership lookups.
        visited = set()
        self.files = dict()
        for package in self.packages.values():
            if package in visited:
                continue
            visited.add(package)

            for f in package.files:
                if f in self.directories:
                    continue

                if f in self.files:
                    logging.warning("Duplicate file",
                                 repr(f),
                                 ' current',
                                 package,
                                 ' already',
                                 self.files[f],
                                 file=sys.stderr)
                    if not f.startswith('/usr/share'):
                        assert (f not in self.files)
                self.files[f] = package

        # For each package, update the file list to track dependencies and symlinks correctly.
        for p in self.packages.values():
            p.update_filetypes(self.directories, self.symlinks)

        # Print out all the libraries and where they live as known to ldconfig
        result = subprocess.run(
            [
                '/usr/sbin/ldconfig', '-C',
                f'{self.partition}/etc/ld.so.cache', '-p'
            ],
            check=True,
            stdout=subprocess.PIPE,
        )

        self.ldconfig_cache = dict()
        for line in result.stdout.decode('utf-8').split('\n'):
            if line.startswith('\t'):
                logging.vlog(1, line)
                split_line = re.split(' \\(libc6,(AArch64|x86-64)\\) => ',
                                      line.strip())
                self.ldconfig_cache[split_line[0]] = split_line[2]

        self.pkgcfg = dict()
        for pkgconfig in [
                '/usr/local/lib/aarch64-linux-gnu/pkgconfig',
                '/usr/lib/x86_64-linux-gnu/pkgconfig',
                '/usr/local/lib/pkgconfig',
                '/usr/local/share/pkgconfig',
                '/usr/lib/aarch64-linux-gnu/pkgconfig',
                '/usr/lib/pkgconfig',
                '/usr/share/pkgconfig',
        ]:
            candidate_folder = f"{self.partition}/{pkgconfig}"
            if not os.path.exists(candidate_folder):
                continue

            for f in os.listdir(candidate_folder):
                full_filename = f"{candidate_folder}/{f}"
                if pathlib.Path(full_filename).is_dir():
                    continue
                if not f.endswith('.pc'):
                    continue

                package_name = f.removesuffix('.pc')

                with open(f"{candidate_folder}/{f}", "r") as file:
                    self.pkgcfg[package_name] = PkgConfig(
                        file.read(), self.files[f'{pkgconfig}/{f}'],
                        f"{candidate_folder}/{f}")

    def resolve_symlink(self, path: str) -> str:
        """ Implements symlink resolution using self.symlinks. """
        # Only need to support absolute links since we don't have a concept of cwd.

        # Implements the symlink algorithm in
        # https://android.googlesource.com/platform/bionic.git/+/android-4.0.1_r1/libc/bionic/realpath.c
        assert (path[0] == '/')

        left = path.split('/')[1:]
        logging.vlog(2, '  start: %s', left)

        if len(path) == 0:
            return path

        resolved = ['']

        while len(left) > 0:
            if left[0] == '.':
                left = left[1:]
                logging.vlog(2, '  .: %s %s', resolved, left)
            elif left[0] == '..':
                assert (len(resolved) >= 1)
                resolved = resolved[:-1]
                left = left[1:]
                logging.vlog(2, '  ..: %s %s', resolved, left)
            else:
                resolved.append(left[0])
                merged = '/'.join(resolved)
                if merged in self.symlinks:
                    symlink = self.symlinks[merged]
                    # Absolute symlink, blow away the previously accumulated path
                    if symlink[0] == '/':
                        resolved = ['']
                        left = symlink[1:].split('/') + left[1:]
                        logging.vlog(2, '  /: %s %s', resolved, left)
                    else:
                        # Relative symlink, replace the symlink name in the path with the newly found target.
                        resolved = resolved[:-1]
                        left = symlink.split('/') + left[1:]
                        logging.vlog(2, '  relative: %s %s', resolved, left)
                else:
                    left = left[1:]

        return '/'.join(resolved)

    def exists(self, path: str) -> bool:
        if path in self.files or path in self.symlinks or path in self.directories:
            return True
        return False

    def resolve_object(self,
                       obj: str,
                       requesting_obj: str | None = None,
                       runpaths: list[str] = []) -> str:
        if obj in self.ldconfig_cache:
            logging.debug(f'in cache {obj}, -> {self.ldconfig_cache[obj]}')
            return self.ldconfig_cache[obj]
        elif requesting_obj is not None:
            to_search = os.path.join(os.path.split(requesting_obj)[0], obj)
            if self.exists(to_search):
                return to_search

        for rpath in runpaths:
            to_search = os.path.join(rpath, obj)
            if self.exists(to_search):
                return to_search

        raise FileNotFoundError(obj)

    @functools.cache
    def object_dependencies(self, obj: str) -> str:
        # Step 1: is it a shared library?
        filename = f'{self.partition}/{obj}'
        soname = os.path.basename(obj)
        if not is_elf_file(filename):
            if detect_static_library(filename):
                return [], None, []

            return parse_linker_script(read_linker_script(filename)), soname, []

        result = subprocess.run(
            ['objdump', '-p', filename],
            check=True,
            stdout=subprocess.PIPE,
        )

        # Part of the example output.  We only want NEEDED from the dynamic section.
        #
        #    RELRO off    0x0000000000128af0 vaddr 0x0000000000128af0 paddr 0x0000000000128af0 align 2**0
        #          filesz 0x0000000000003510 memsz 0x0000000000003510 flags r--
        #
        # Dynamic Section:
        #   NEEDED               libtinfo.so.6
        #   NEEDED               libc.so.6
        #   RUNPATH              /usr/lib/x86_64-linux-gnu/systemd
        #   INIT                 0x000000000002f000
        #   FINI                 0x00000000000efb94

        deps = []
        runpaths = []
        for line in result.stdout.decode('utf-8').split('\n'):
            if 'NEEDED' in line:
                deps.append(line.strip().split()[1])
            elif 'SONAME' in line:
                soname = line.strip().split()[1]
            elif 'RUNPATH' in line:
                runpaths.append(line.strip().split()[1])

        return deps, soname, runpaths


def generate_build_file(filesystem, packages_to_eval, template_filename):
    # Now, we want to figure out what the dependencies of each of the packages are.
    # Generate the dependency tree starting from an initial list of packages.
    # Then, figure out how to link the .so's in.

    # Recursively walk the tree using dijkstra's algorithm to generate targets
    # for each set of headers.
    print('Walking tree for', [p.name.name for p in packages_to_eval],
          file=sys.stderr)

    rules = []
    objs_to_eval = []

    # Set of packages already generated in case our graph hits a package
    # multiple times.
    packages_visited_set = set()
    while packages_to_eval:
        next_package = packages_to_eval.pop()
        if next_package in packages_visited_set:
            continue
        packages_visited_set.add(next_package)

        hdrs = next_package.headers()
        objects = next_package.objects()

        deps = []
        for p in next_package.resolved_depends(filesystem.packages):
            if p not in packages_visited_set:
                packages_to_eval.append(p)

            # These two form a circular dependency...
            # Don't add them since libc6 has no headers in it.
            if next_package.name.name == 'libgcc-s1' and p.name.name == 'libc6':
                continue
            # Another circular dependency.  Break any one of the links, it doesn't matter.
            if next_package.name.name == 'dmsetup' and p.name.name == 'libdevmapper1.02.1':
                continue

            deps.append(p.name.name)

        logging.vlog(1, 'For package %s, found objects %s', next_package.name, objects)
        if objects:
            objs_to_eval += objects

        hdrs.sort()
        deps.sort()
        hdrs = [f'        "{h[1:]}",\n' for h in hdrs]
        hdrs_files = ''.join(hdrs)
        deps_joined = ''.join([f'        ":{d}-headers",\n' for d in deps])

        filegroup_srcs = ''.join(
            [f'        "{f[1:]}",\n' for f in next_package.files] +
            [f'        ":{d}-filegroup",\n' for d in deps])

        rules.append(
            f'filegroup(\n    name = "{next_package.name.name}-filegroup",\n    srcs = [\n{filegroup_srcs}    ],\n)'
        )
        rules.append(
            f'cc_library(\n    name = "{next_package.name.name}-headers",\n    hdrs = [\n{hdrs_files}    ],\n    visibility = ["//visibility:public"],\n    deps = [\n{deps_joined}    ],\n)'
        )

    obj_set = set()
    while objs_to_eval:
        obj = objs_to_eval.pop()
        if obj in obj_set:
            continue
        obj_set.add(obj)

        deps, soname, runpaths = filesystem.object_dependencies(obj)
        resolved_deps = []
        for d in deps:
            resolved_obj = filesystem.resolve_symlink(
                filesystem.resolve_object(d,
                                          requesting_obj=obj,
                                          runpaths=runpaths))
            resolved_deps.append(resolved_obj)
            if resolved_obj not in obj_set:
                objs_to_eval.append(resolved_obj)
            logging.vlog(2, '%s -> soname %s %s', obj, soname, resolved_obj)

        resolved_deps.sort()

        # Name it after the fully resolved name of the object.  This is unambiguous.
        rule_name = obj[1:].replace('/', '_')

        rule_deps = ''.join([
            '        ":{}-lib",\n'.format(d[1:].replace('/', '_'))
            # libc6 is a bit aggressive to link against...  And breaks aos's malloc hooks.
            for d in resolved_deps
            if not d.endswith('/libc.so.6') and not d.endswith('/libm.so.6')
        ])

        # We want to include the canonical name for a library.  This is what
        # other shared libraries and the linker will refer to it by.
        try:
            # We got a None soname when a static library comes through.  The
            # only one I've seen so far is libc_nonshared.a, which we want to ignore.
            if soname is not None:
                canonical_objs = [
                  filesystem.resolve_object(soname,
                                            requesting_obj=obj,
                                            runpaths=runpaths)[1:]
                ]
                joined_canonical_objs = "\", \"".join(canonical_objs)
                rules.append(
                    f'cc_library(\n    name = "{rule_name}-lib",\n    srcs = ["{joined_canonical_objs}"],\n    deps = [\n{rule_deps}    ],\n)'
                )
        except FileNotFoundError as e:
            # soname is busted.
            logging.warning('SONAME of %s for %s is not resolvable, ignoring',
                            soname, obj)
            rules.append(
                f'# SONAME of {soname} for {obj} is not resolvable, ignoring')


    standard_includes = set()
    standard_includes.add('/usr/include')
    standard_includes.add('/usr/include/aarch64-linux-gnu')
    standard_includes.add('/usr/include/x86-64-linux-gnu')
    for pkg in filesystem.pkgcfg:
        try:
            contents = filesystem.pkgcfg[pkg]
            resolved_libraries = [
                filesystem.resolve_symlink(
                    filesystem.resolve_object('lib' + f.removeprefix('-l') +
                                              '.so')) for f in contents.libs
                if f.startswith('-l')
            ]

            includes = []
            for flag in contents.cflags:
                if flag.startswith('-I/') and flag.removeprefix(
                        '-I') not in standard_includes:
                    includes.append(flag.removeprefix('-I/'))

            rule_deps = ''.join(
                sorted([
                    '        ":' + l[1:].replace('/', '_') + '-lib",\n'
                    for l in resolved_libraries
                ] + [f'        ":{contents.package.name.name}-headers",\n'] +
                       [f'        ":{dep}",\n' for dep in contents.requires]))
            includes.sort()
            if len(includes) > 0:
                includes_string = '    includes = ["' + '", "'.join(
                    includes) + '"],\n'
            else:
                includes_string = ''
            rules.append(
                f'# pkgconf -> {pkg}\ncc_library(\n    name = "{pkg}",\n{includes_string}    visibility = ["//visibility:public"],\n    deps = [\n{rule_deps}    ],\n)'
            )
            # Look up which package this is from to include the headers
            # Depend on all the libraries
            # Parse -I -> includes
        except FileNotFoundError as e:
            logging.warning('Failed to instantiate package %s %s', repr(pkg),
                            e)
            pass

    with open(template_filename, "r") as file:
        template = jinja2.Template(file.read())

    substitutions = {
        "RULES": '\n\n'.join(rules),
    }

    return template.render(substitutions)
