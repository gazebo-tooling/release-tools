#!/bin/bash

# Prints sanitized contents of a Packages file from a debian repository to
# standard out. The output contains only the Package, Source, and Version
# fields, while retaining existing blank lines.
# This can be used to package versions across Ubuntu releases and architectures.
#
# Arguments
#
# 1. <package_repo>: Optional: stable / prerelease / nightly (defaults to stable)
# 2. <ubuntu_release>: Optional: jammy / noble / resolute / etc (defaults to noble)
# 3. <arch>: Optional: amd64 / arm64 / armhf (defaults to amd64)
#
# Usage
#
#   bash print_sanitized_packages.bash <package_repo> <ubuntu_release> <arch>
#
# For example
#
#  bash print_sanitized_packages.bash stable resolute arm64

PACKAGES_URL=${PACKAGES_URL:-packages.osrfoundation.org}
echo "Using $PACKAGES_URL"
PACKAGE_REPO=${1:-stable}
UBUNTU_RELEASE=${2:-noble}
ARCH=${3:-amd64}

COLUMN="        "
GREEN="\e[42m"
YELLOW="\e[43m"
RED="\e[101m"

DISTRO="ubuntu"

# The goal of the following command is to download the current Packages file,
# filter out all lines that are not blank or starting with
# 'Package:', 'Source:', and 'Version:'
# It currently filters out all non-empty lines not starting with 'P', 'S', or 'V'
# as well as lines starting with 'Pr' (Priority), 'Se' (Section), 'Si' (Size),
# and 'SH' (SHA1, SHA256). This appears to be sufficient to meet the stated goal.
# It also replaces instances of any string matching the UBUNTU_RELEASE variable
# with a '${UBUNTU_RELEASE}' to reduce the diff between output from different
# ubuntu releases.
curl http://${PACKAGES_URL}/gazebo/ubuntu-${PACKAGE_REPO}/dists/${UBUNTU_RELEASE}/main/binary-${ARCH}/Packages \
  | egrep -v '^[^PSV]' \
  | egrep -v '^Pr' \
  | egrep -v 'S[eiH]' \
  | sed -e "s@${UBUNTU_RELEASE}@\${UBUNTU_RELEASE}@g"

