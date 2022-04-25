#!/usr/bin/env bash
# Copyright (C) 2022 Open Source Robotics Foundation
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

# DESCRIPTION
# ===========
# This script will migrate headers and references to those headers in the source files for:
# - ign(ition) -> gz
# - Ign(ition) -> Gz
#
# It will, for all subdirectories under `src`, `test`, `examples`, and `include`:
# - Move the files
# - Edit source files to update references/new paths
# - Edit CMakeLists files to update references/new paths
#
# It will also leave redirection aliases for tick-tocking in the existing ign(ition) directories;
# As well as create a top level CMakeLists file to install those redirection aliases.
#
# WARNING
# =======
# This script will NOT migrate variable and class names! It also won't migrate macro names with
# arguments. (It'll mainly try to target header guards for macro updates.)
#
# PRE-REQUISITES
# ==============
# Requires the 'gh' CLI to be installed.
#
# USAGE
# =====
# Place the script in the root of the repo to be migrated, and run:
# $ ./header_migration_script.sh
#
# Author: methylDragon


# METHODS AND CONSTANTS ============================================================================
export LICENSE="""\
/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the \"License\");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an \"AS IS\" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
"""


mv_headers() {
  # Create necessary directories
  dirname $1 | sed 's@include\(.*\)*/ignition@include\1/gz@g' | xargs -I {} mkdir -pv {}

  # Move anything in an include/.../ignition or include/ignition subdirectory
  # Also handles:
  #   IgnitionXXX -> GzXXX
  #   IgnXXX -> GzXXX
  if [[ $1 =~ include(.*)*/ignition ]] || [[ $1 =~ include(.*)*/Ign(ition)?[A-Z] ]] ; then
    echo $1 | sed \
      -e 's@include\(.*\)*/ignition@include\1/gz@g' \
      -e 's@include\(.*\)*/Ign\(ition\)\?\([A-Z]\)@include\1/Gz\3@g' \
       | xargs -I {} bash -c "git mv -f $1 {} && echo '[MOVED] $1 --> {}'" _
  fi

  # Leave redirection aliases
  #   Converting *.in into normal versions (spoofing configuration of *.in files)
  echo $1 | sed 's@\(.*\)\.in@\1@g' | xargs -I {} touch {}
} ; export -f mv_headers


migrate_sources() {  # Different variations of ignition/ign -> gz in source files
  # Note: DOES NOT MIGRATE CLASS OR VARIABLE NAMES

  if [[ $1 =~ \.c[^\.]*$|\.in[^\.]*$|\.h[^\.]*$ ]] ; then # Only do for source files
    echo "[MIGRATING SOURCES] $1"
    if [[ $1 != *.in ]] ; then                  # !! But not macros for .in files
      # Only migrate non-macro definition calls
      sed -i 's@\(#.*\) IGNITION_\([^(]*\)$@\1 GZ_\2@g' $1  # e.g. IGNITION_UTILS__XXX -> GZ_UTILS__XXX
      sed -i 's@\(#.*\) IGN_\([^(]*\)\$@\1 GZ_\2@g' $1       # e.g. IGN_UTILS__XXX -> GZ_UTILS__XXX
    fi

    # NOTE(CH3): We're not migrating class or variable names for now
    # sed -i 's@Ignition\([A-Z]\)@Gz\1@g' $1 # e.g. IgnitionFormatter -> GzFormatter

    sed -i 's@ignition/@gz/@g' $1   # e.g. include <ignition/utils/XXX> -> include <gz/utils/XXX>
    sed -i 's@ignition_@gz_@g' $1  # e.g. ignition_xxx -> gz_xxx
    sed -i 's@ign_@gz_@g' $1       # e.g. ign_xxx -> gz_xxx

    # Handle Edge Cases
    sed -i 's@${gz_headers}@${ign_headers}@g' $1
  fi
} ; export -f migrate_sources


migrate_cmake() {  # Different variations of ignition/ign -> gz in CMake files
  # This is special because we need to specifically avoid unmigrated ign-cmake macro calls

  if [[ $1 =~ CMakeLists.txt ]] ; then # Only do for CMakeLists files
    echo "[MIGRATING CMAKE] $1"

    sed -i 's@(ignition@(gz@g' $1   # e.g. add_subdirectory(ignition) -> add_subdirectory(gz)

    # NOTE(CH3):
    # ^\(?!ign\(?ition\)_\) ignores lines that start with ign(nition)_
    # Which should avoid changing any ign-cmake macro calls
    sed -i 's@^\(?!ign\(?ition\)_\)\(#.*\) IGNITION_@\1 GZ_@g' $1  # e.g. IGNITION_UTILS__XXX -> GZ_UTILS__XXX
    sed -i 's@^\(?!ign\(?ition\)_\)\(#.*\) IGN_@\1 GZ_@g' $1       # e.g. IGN_UTILS__XXX -> GZ_UTILS__XXX

    sed -i 's@^\(?!ign\(?ition\)_\)Ignition\([A-Z]\)@Gz\1@g' $1 # e.g. IgnitionFormatter -> GzFormatter

    sed -i 's@^\(?!ign\(?ition\)_\)(ignition@(gz@g' $1   # e.g. add_subdirectory(ignition) -> add_subdirectory(gz)
    sed -i 's@^\(?!ign\(?ition\)_\)ignition/@gz/@g' $1   # e.g. include <ignition/utils/XXX> -> include <gz/utils/XXX>
    sed -i 's@^\(?!ign\(?ition\)_\)ignition_@gz_/@g' $1  # e.g. ignition_xxx -> gz_xxx
    sed -i 's@^\(?!ign\(?ition\)_\)ign_@gz_/@g' $1       # e.g. ign_xxx -> gz_xxx
  fi
} ; export -f migrate_cmake


populate_redirection_alias() {
  echo "[REDIRECTING] $1"
  echo "$LICENSE" > $1
  echo "#include <$(echo $1 | sed \
    -e 's@.*/include/@@g' \
    -e 's@ignition/@gz/@g' \
    -e 's@*Ign(ition)@Gz@g')>" >> $1
} ; export -f populate_redirection_alias

# MAIN =============================================================================================
# Edit top level CMakeLists
sed -i 's@ign_configure_project(\(.*\))@ign_configure_project(\n  REPLACE_IGNITION_INCLUDE_PATH gz/utils\n  \1)@g' CMakeLists.txt

# Move headers
find . -regex '.*include\(.*\)*' -type f -print0 | xargs -0 -I {} bash -c 'mv_headers {}' _

# Create Export.hh and <lib>.hh
find . -regex './include.*/ignition/[^/]*' -type d -print0 \
  | xargs -0 -I {} bash -c 'touch "{}/Export.hh" \
                            && echo "[CREATED] {}/Export.hh"' _
find . -regex './include.*/ignition/[^/]*' -type d -print0 \
  | xargs -0 -I {} bash -c 'touch "$(sed "s@\(.*\)/\(.*\)\$@\1/\2/\2.hh@g" <<< {})" \
                            && echo "[CREATED] $(sed "s@\(.*\)/\(.*\)\$@\1/\2/\2.hh@g" <<< {})"'

# Cleanup dangling files
find . -regex ".*/include.*/ignition.*/CMakeLists\.txt" -type f -print0 | xargs -0 -I {} rm {}  # Remove dangling .in config files
find . -regex ".*/include.*/ignition.*/.*\.in" -type f -print0 | xargs -0 -I {} rm {}  # Remove dangling ignition CMakeLists

# Provision redirection aliases
find . -regex '.*include.*/ignition.*\.h.*\|.*include\(.*\)*/Ign\(ition\)?[A-Z].*\.h.*' -type f -print0 \
  | xargs -0 -I {} bash -c 'populate_redirection_alias {}' _

# Migrate Gz sources
find . -regex '.*/\[src\|test\|examples\]/.*\|.*include\(.*\)*/[gz|Gz].*' -type f -print0 | xargs -0 -I {} bash -c 'migrate_sources {}' _

# Migrate Gz CMake files
find . -regex '.*include.*/gz.*\|.*include\(.*\)*/Gz.*' -type f -print0 | xargs -0 -I {} bash -c 'migrate_cmake {}' _

# Add header level CMakeLists.txt
touch ./include/CMakeLists.txt
echo "add_subdirectory(gz)" > ./include/CMakeLists.txt
echo "install(DIRECTORY ignition DESTINATION \${IGN_INCLUDE_INSTALL_DIR_FULL})" >> ./include/CMakeLists.txt

# TODOs
# Parse libs from CLI
# Clone and autobranch/push
