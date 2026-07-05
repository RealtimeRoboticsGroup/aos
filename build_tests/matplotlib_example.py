import sys
import os
import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

# If Gtk can't be initialized (e.g. on a headless CI server), skip the test.
initialized, argv = Gtk.init_check(sys.argv)
if not initialized:
    print("Gtk couldn't be initialized (likely headless). Skipping GUI test.",
          file=sys.stderr)
    sys.exit(0)

import matplotlib
# Set up the gtk backend before running matplotlib.
matplotlib.use("GTK3Agg")
import matplotlib.pyplot as plt

plt.plot([1, 2, 3, 4])
plt.ylabel("some numbers")

# If running under a test runner, exit immediately instead of blocking on user input.
if "BAZEL_TEST" not in os.environ:
    plt.show()
