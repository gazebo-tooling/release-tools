#!/bin/bash -x
set -e

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

#. ${SCRIPT_DIR}/lib/sdformat-homebrew-default.bash

# Step 1. Set up homebrew
curl -fsSL https://raw.github.com/mxcl/homebrew/go/install > install.rb
RUN_DIR=$(mktemp -d ${HOME}/jenkins.XXXX)
echo "Install into: ${RUN_DIR}"
# Configure PREFIX
sed -i -e "s#HOMEBREW_PREFIX = .*#HOMEBREW_PREFIX='${RUN_DIR}'#" install.rb
# Non interactive
sed -i -e "s#c = getc#c = 13#" install.rb
ruby install.rb

# Step 2. Install sdformat
${RUN_DIR}/bin/brew tap osrf/simulation
${RUN_DIR}/bin/brew install `${RUN_DIR}/bin/brew deps sdformat`

# Step 3. Clean up
# rm -fr ${RUN_DIR}
