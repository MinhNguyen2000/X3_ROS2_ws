#!/usr/bin/env bash
# Fixed drive patterns for the odometry recordings (docs/real_robot_run_protocol.md, ekf_compare.py) and the scale check.
#
#   drive_pattern.sh straight   3 m at 0.2 m/s (300 messages at 20 Hz)
#   drive_pattern.sh turn       one full turn at 0.3 rad/s, counter-clockwise (420 messages)
#   drive_pattern.sh square     two laps of a 1 m square: 8 x (1 m straight, 90 degree turn), 1 s stops in between
#
# The driver (mcnamu_driver.py) keeps the last cmd_vel until a new one arrives, so the robot keeps moving when the
# publishing ends. This script therefore always ends with zero commands, also on Ctrl-C, SIGTERM or an error, and
# starts with some. The e-stop is still the operator's. CMD_VEL_TOPIC overrides the topic (default /agent0/cmd_vel).
set -u
TOPIC=${CMD_VEL_TOPIC:-/agent0/cmd_vel}
TYPE=geometry_msgs/msg/TwistStamped
RATE=20

zero() {   # about 0.5 s of zero commands, long enough to be received even right after the publisher starts
    ros2 topic pub -r $RATE -t 10 "$TOPIC" $TYPE "{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}" >/dev/null 2>&1
}
send() {   # $1 messages of the twist $2 at $RATE Hz
    ros2 topic pub -r $RATE -t "$1" "$TOPIC" $TYPE "$2" >/dev/null 2>&1
}
trap zero EXIT
trap 'exit 130' INT TERM    # the EXIT trap then sends the zero commands

FORWARD="{twist: {linear: {x: 0.2}}}"
TURN="{twist: {angular: {z: 0.3}}}"

case "${1:-}" in
    straight)
        zero; echo "straight: 3 m"; send 300 "$FORWARD" ;;
    turn)
        zero; echo "turn: one full turn"; send 420 "$TURN" ;;
    square)
        zero
        for i in 1 2 3 4 5 6 7 8; do
            echo "square: side $i of 8"
            send 100 "$FORWARD"; zero; sleep 0.5
            send 105 "$TURN";    zero; sleep 0.5
        done ;;
    *)
        echo "usage: $0 straight|turn|square" >&2; exit 2 ;;
esac
echo "done, robot stopped"
