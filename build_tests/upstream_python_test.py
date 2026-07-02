import sys
import unittest


class TestPipImports(unittest.TestCase):

    def test_version(self):
        """Validates that we are using the version specified in rules_python."""
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
                 or "rules_python++pip+aos_pip_deps" in numpy.__file__
                 or "rules_python++pip+pip_deps" in numpy.__file__),
            numpy.__file__)


if __name__ == "__main__":
    unittest.main()
