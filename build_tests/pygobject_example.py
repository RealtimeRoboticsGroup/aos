import os
import sys
import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

# If Gtk can't be initialized (e.g. on a headless CI server), skip the test.
initialized, argv = Gtk.init_check(sys.argv)
if not initialized:
    print("Gtk couldn't be initialized (likely headless). Skipping GUI test.",
          file=sys.stderr)
    sys.exit(0)


class MyWindow(Gtk.Window):

    def __init__(self):
        super().__init__(title="Hello World")
        self.button = Gtk.Button(label="Click Here")
        self.button.connect("clicked", self.on_button_clicked)
        self.add(self.button)

    def on_button_clicked(self, widget):
        print("Hello World")


def main():
    win = MyWindow()
    win.connect("destroy", Gtk.main_quit)
    win.show_all()

    # If running under a test runner, exit immediately instead of blocking on user input.
    if "BAZEL_TEST" in os.environ:
        print("Running under Bazel test. Exiting immediately.")
        Gtk.main_quit()
    else:
        Gtk.main()


if __name__ == "__main__":
    main()
