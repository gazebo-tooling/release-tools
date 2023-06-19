#!/bin/bash
# Launch all the suite of ros_gazebo_pkgs:
# Usage: ros_gz-release.py.bash <version>

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

set -e

if [[ $# -lt 2 ]]; then
    echo "$0 <version> <release_repo> <ros_distro> <token> 'other arguments used in release.py'"
    exit 1
fi

if [[ ${1%-*} != "${1}" ]]; then
  echo "Version should not contain a revision number. Use -r argument"
  exit 1
fi

"${SCRIPT_DIR}/_check_metadata.bash" "${2}" "${3}"

for p in ros-gz ros-gz-bridge ros-gz-image ros-gz-interfaces ros-gz-sim ros-gz-sim-demos; do
  ../release-bloom.py "${p}" $(for i in $@; do echo -n "$i "; done)
done
