#!/usr/bin/env bash

INSTA360_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$INSTA360_ROOT/scripts/cyclonedds_insta360.xml"
