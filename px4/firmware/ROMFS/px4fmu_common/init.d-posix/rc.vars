#!/bin/sh

# Arguments passed to this script:
# $1: optional instance id
inst=0
[ -n "$1" ] && inst=$1

#global
#PX4_SIM_MODEL=iris
#PX4_ESTIMATOR=ekf2

#tcp port
simulator_opts="-c $((4560+inst))"

#udp port
#simulator_opts="-u $((14560+inst))"

udp_offboard_port_local=$((14580+inst))
udp_offboard_port_remote=$((14540+inst))
[ $inst -gt 9 ] && udp_offboard_port_remote=14549 # use the same ports for more than 10 instances to avoid port overlaps
udp_onboard_payload_port_local=$((14280+inst))
udp_onboard_payload_port_remote=$((14030+inst))
udp_gcs_port_local=$((18570+inst))
