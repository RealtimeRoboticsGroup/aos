import sys
import unittest


class TestPipImports(unittest.TestCase):

    def test_version(self):
        """Validates that we are using the version specified in rules_python."""
        if sys.version_info[0:2] == (3, 13):
            self.assertEqual(3, sys.version_info[0])
            self.assertEqual(13, sys.version_info[1])
        else:
            self.assertEqual((3, 10, 18), sys.version_info[0:3])

    def test_imports(self):
        """Validates that we can import pip packages from pypi.org."""
        import numpy
        import scipy
        import matplotlib

        # Make sure we're sourcing numpy from the expected source. We could
        # pick any of the three we imported above.
        # This needs to support both the workspace and bzlmod paths as we
        # migrate.
        self.assertTrue(
            numpy.__file__.endswith("site-packages/numpy/__init__.py")
            and ("pip_deps_numpy" in numpy.__file__
                 or "rules_python++pip+pip_deps" in numpy.__file__),
            numpy.__file__)

    def test_load_shared_libraries_from_template(self):
        """Validates that we can find and load a shared library from every directory added to LD_LIBRARY_PATH in the template."""
        import ctypes
        import os
        import re
        import sys

        runfiles_dir = os.environ.get("RUNFILES_DIR")
        self.assertTrue(runfiles_dir, "RUNFILES_DIR is not set")

        base_path = None
        for ws in ["_main", "aos", "aos+"]:
            p = os.path.join(runfiles_dir, ws, "tools/python")
            if os.path.isdir(p):
                base_path = p
                break
        self.assertTrue(base_path, f"Could not find tools/python in runfiles")

        # Read template to extract the LD_LIBRARY_PATH additions
        template_path = os.path.join(base_path, "runtime_binary.sh.tpl")
        self.assertTrue(os.path.isfile(template_path),
                        f"Missing template: {template_path}")
        with open(template_path, "r") as f:
            template_content = f.read()

        # Parse LD_LIBRARY_PATH=":${BASE_PATH}/lib" and LD_LIBRARY_PATH+=":${SYSROOT_DIR}/..."
        patterns = re.findall(r'LD_LIBRARY_PATH\+?="([^"]+)"',
                              template_content)
        patterns = [
            p for p in patterns
            if p.startswith(':${BASE_PATH}') or p.startswith(':${SYSROOT_DIR}')
        ]
        self.assertTrue(patterns, "No LD_LIBRARY_PATH lines found in template")

        # Resolve variables based on the active Python installation and sysroot in runfiles
        python_base = sys.base_prefix
        parent_dir = os.path.dirname(python_base)

        sysroot_dir = None
        for name in ["amd64_debian_sysroot+", "amd64_debian_sysroot"]:
            p = os.path.join(parent_dir, name)
            if os.path.isdir(p):
                sysroot_dir = p
                break
        self.assertTrue(
            sysroot_dir,
            f"Could not locate sysroot directory in runfiles (parent: {parent_dir})"
        )

        tested_dirs = 0
        for pattern in patterns:
            # Clean up the pattern (e.g. remove leading colon and resolve variables)
            clean_pattern = pattern.lstrip(':')
            resolved = clean_pattern.replace('${BASE_PATH}',
                                             python_base).replace(
                                                 '${SYSROOT_DIR}', sysroot_dir)

            # Skip checking NCCL if NCCL_DIR_NAME is not resolved or doesn't exist
            if '${NCCL_DIR_NAME}' in resolved:
                # Upstream python test doesn't depend on nvidia/nccl wheels in runfiles, so skip
                continue

            self.assertTrue(os.path.isdir(resolved),
                            f"Expected directory to exist: {resolved}")

            # Find all shared library files in this directory
            so_files = []
            for entry in os.scandir(resolved):
                if entry.is_file() and (entry.name.endswith(".so")
                                        or ".so." in entry.name):
                    so_files.append(entry.path)

            self.assertTrue(
                so_files,
                f"No shared libraries (.so) found in directory: {resolved}")

            # Try to load at least one shared library from this directory
            loaded_any = False
            errors = []
            for so_file in so_files:
                try:
                    # Some libraries might fail to load because their dependencies are not in LD_LIBRARY_PATH yet,
                    # but we only need to successfully load at least one library to confirm the path works.
                    ctypes.CDLL(so_file)
                    loaded_any = True
                    break
                except OSError as e:
                    errors.append(
                        f"Failed to load {os.path.basename(so_file)}: {e}")

            self.assertTrue(
                loaded_any,
                f"Failed to load any shared library from directory {resolved}. Found files: {[os.path.basename(f) for f in so_files[:5]]}. Errors:\n"
                + "\n".join(errors))
            tested_dirs += 1

        # We must have tested at least 5 standard directories (python/lib, sysroot/lib, sysroot/usr/lib, sysroot/usr/lib/gvfs, etc.)
        self.assertGreaterEqual(
            tested_dirs, 5,
            f"Expected to test at least 5 LD_LIBRARY_PATH directories, but only tested {tested_dirs}"
        )

    def test_runtime_binary_paths_and_contents(self):
        """Validates that all generated runtime scripts have the correct paths and values."""
        import os
        import re

        runfiles_dir = os.environ.get("RUNFILES_DIR")
        self.assertTrue(runfiles_dir, "RUNFILES_DIR is not set")

        base_path = None
        for ws in ["_main", "aos", "aos+"]:
            p = os.path.join(runfiles_dir, ws, "tools/python")
            if os.path.isdir(p):
                base_path = p
                break
        self.assertTrue(
            base_path,
            f"Could not find tools/python in runfiles. RUNFILES_DIR={runfiles_dir}"
        )

        # Define expected variable values for each script
        expected_configs = {
            "runtime_binary_bzlmod_3_10.sh": {
                "PYTHON_VER":
                "3_10",
                "SHORT_VER":
                "310",
                "PYTHON_REPO_DIR":
                "rules_python++python+python_3_10_x86_64-unknown-linux-gnu",
                "SYSROOT_DIR_NAME":
                "amd64_debian_sysroot+",
                "NCCL_DIR_NAME":
                "rules_python++pip+aos_pip_deps_310_nvidia_nccl_cu12",
            },
            "runtime_binary_workspace_3_10.sh": {
                "PYTHON_VER": "3_10",
                "SHORT_VER": "310",
                "PYTHON_REPO_DIR": "python_3_10_x86_64-unknown-linux-gnu",
                "SYSROOT_DIR_NAME": "amd64_debian_sysroot",
                "NCCL_DIR_NAME": "pip_deps_nvidia_nccl_cu12",
            },
            "runtime_binary_bzlmod_3_13.sh": {
                "PYTHON_VER":
                "3_13",
                "SHORT_VER":
                "313",
                "PYTHON_REPO_DIR":
                "rules_python++python+python_3_13_x86_64-unknown-linux-gnu",
                "SYSROOT_DIR_NAME":
                "amd64_debian_sysroot+",
                "NCCL_DIR_NAME":
                "rules_python++pip+aos_pip_deps_313_nvidia_nccl_cu12",
            },
            "runtime_binary_workspace_3_13.sh": {
                "PYTHON_VER": "3_13",
                "SHORT_VER": "313",
                "PYTHON_REPO_DIR": "python_3_13_x86_64-unknown-linux-gnu",
                "SYSROOT_DIR_NAME": "amd64_debian_sysroot",
                "NCCL_DIR_NAME": "pip_deps_nvidia_nccl_cu12",
            },
        }

        # Check template contents
        template_path = os.path.join(base_path, "runtime_binary.sh.tpl")
        self.assertTrue(os.path.isfile(template_path),
                        f"Missing template: {template_path}")
        with open(template_path, "r") as f:
            template_content = f.read()

        # Check that expected paths are in the template
        expected_template_lines = [
            'LD_LIBRARY_PATH=":${BASE_PATH}/lib"',
            'LD_LIBRARY_PATH+=":${SYSROOT_DIR}/lib/x86_64-linux-gnu/"',
            'LD_LIBRARY_PATH+=":${SYSROOT_DIR}/usr/lib/x86_64-linux-gnu/"',
            'LD_LIBRARY_PATH+=":${SYSROOT_DIR}/usr/lib/"',
            'LD_LIBRARY_PATH+=":${SYSROOT_DIR}/usr/lib/x86_64-linux-gnu/gvfs/"',
            'if [[ -e "${BASE_PATH}/../${NCCL_DIR_NAME}" ]]; then',
            '  LD_LIBRARY_PATH+=":${BASE_PATH}/../${NCCL_DIR_NAME}/site-packages/nvidia/nccl/lib/"',
            'fi',
        ]
        for line in expected_template_lines:
            self.assertIn(line, template_content,
                          f"Template is missing expected path line: {line}")

        # Check each generated script matches the template with substituted variables
        for filename, expected_vars in expected_configs.items():
            script_path = os.path.join(base_path, filename)
            self.assertTrue(os.path.isfile(script_path),
                            f"Missing generated script: {script_path}")

            with open(script_path, "r") as f:
                script_content = f.read()

            # Parse variables from script using regex: readonly VAR="VALUE"
            parsed_vars = {}
            for var_name in expected_vars.keys():
                match = re.search(f'readonly {var_name}="([^"]+)"',
                                  script_content)
                self.assertTrue(
                    match, f"Could not find variable {var_name} in {filename}")
                parsed_vars[var_name] = match.group(1)

            self.assertEqual(parsed_vars, expected_vars,
                             f"Variable mismatch in {filename}")

            # Verify that the generated script content is identical to the template with the variables replaced
            expected_content = template_content
            for var_name, var_val in expected_vars.items():
                expected_content = expected_content.replace(
                    f"@{var_name}@", var_val)

            self.assertEqual(
                script_content, expected_content,
                f"Generated script {filename} does not match template substitutions"
            )


if __name__ == "__main__":
    unittest.main()
