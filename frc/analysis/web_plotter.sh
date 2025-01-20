# This script provides basic plotting of a logfile.
# Basic usage:
# $ bazel run -c opt //frc/analysis:web_plotter -- --node node_name /path/to/logfile
./aos/network/log_web_proxy_main --data_dir=frc/analysis $@
