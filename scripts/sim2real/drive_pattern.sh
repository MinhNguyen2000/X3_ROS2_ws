#!/usr/bin/env bash
# Fixed drive patterns for the odometry recordings and the scale check: drive_pattern.sh straight|turn|square [--wait SEC]
#
# Kept as the entry point named in docs/real_robot_run_protocol.md and odom_scale_check.py. The patterns used to be sent
# here with one `ros2 topic pub` per segment, whose closing zero commands could be lost before the driver had matched
# the new publisher (the robot kept moving after "done, robot stopped", 2026-11-02). drive_pattern.py now does it with
# one publisher that waits for the driver and checks the stop; see its docstring.
exec python3 "$(dirname "$0")/drive_pattern.py" "$@"
