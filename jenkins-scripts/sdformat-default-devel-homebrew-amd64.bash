#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

#export DISTRO=quantal

#. ${SCRIPT_DIR}/lib/sdformat-homebrew-default.bash

# Step 1. Set up homebrew
curl -fsSL https://raw.github.com/mxcl/homebrew/go/install > install.rb
RUN_DIR=$(mktemp -d ${HOME}/jenkins.XXXX)
sed "s:HOMEBREW_PREFIX.*:HOMEBREW_PREFIX=${RUN_DIR}::"
ruby -e install.rb

# Step 2. Build sdformat
${RUN_DIR}/bin/brew install sdformat

# Step 3. Clean up
rm -fr ${RUN_DIR}
