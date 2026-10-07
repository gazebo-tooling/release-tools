#!/usr/bin/env bash
# Copyright (C) 2025 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# This script creates milestones in each Gazebo library and ros_gz. You'll
# need the `gh` Github CLI tool installed and configured properly with an
# account that has write permissions on all the repos specified below.
#
# Usage:
# ./create_milestones.bash milestone_description.json
#
# where milestone_description.json is json file that contains the title,
# description and due date of the milestone
# Example:
#
# {
#   "title": "Jetty Release"
#   "state": "open",
#   "description": "Track pull requests that are meant to go into the release. Note that this also includes pull requests that get merged into older branches and get forward merged."
#   "due_on": "2025-09-29T23:59:59Z"
# }
#
#
if [[ $# -lt 1 ]]; then
  echo "./create_milestones.bash FILE.json"
  exit 1
fi

if [[ ! -f $1 ]]; then
  echo "File \"$1\" does not exist"
  exit 1
fi

repos="
gz-cmake
gz-common
gz-fuel-tools
gz-gui
gz-launch
gz-math
gz-msgs
gz-physics
gz-plugin
gz-rendering
gz-sensors
gz-sim
gz-tools
gz-transport
gz-utils
sdformat
ros_gz
"

for repo in $repos; do
  echo "Creating milestone on gazebosim/$repo"
  result=$(cat $1 | gh api  --jq ".html_url" --method POST repos/gazebosim/$repo/milestones --input - 2>&1)
  retval=$?
  echo "  url: $result"
  if [[ retval -ne 0 ]]; then
    exit 1
  fi
done
