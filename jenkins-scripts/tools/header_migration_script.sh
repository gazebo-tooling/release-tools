#!/usr/bin/env bash

migrate_ign_to_gz() {
  # Migrate ignition to gz
  if [[ $1 != *.in ]] ; then  # But not macros for .in files
    sed -i 's@\(#.*\) IGNITION_@\1 GZ_@g' $1  # e.g. IGNITION_UTILS__XXX -> GZ_UTILS__XXX
  fi

  sed -i 's@(ignition@(gz@g' $1          # e.g. add_subdirectory(ignition) -> add_subdirectory(gz)
  sed -i 's@ignition/@gz/@g' $1          # e.g. include <ignition/utils/XXX> -> include <gz/utils/XXX>

  # Do the moves
  dirname $1 | sed 's@include/ignition@include/gz@g' | xargs -I {} mkdir -pv {}
  echo $1 | sed 's@include/ignition@include/gz@g' | xargs -I {} git mv -f $1 {} && echo "Moved: $1 --> {}"

  # Leave redirection aliases
  # Convert *.in into normal versions
  echo $1 | sed 's@\(.*\)\.in@\1@g' | xargs -I {} touch {}
}
export -f migrate_ign_to_gz


redirection_alias() {
  text="""\
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

#include <$(echo $1 | sed "s@.*ignition/\(.*\)@gz/\1@g")>\
"""

 echo "$text" > $1
}
export -f redirection_alias

# MAIN =============================================================================================
cd ign-utils
git reset --hard

# Edit top level CMakeLists
sed -i 's@ign_configure_project(\(.*\))@ign_configure_project(\n  REPLACE_IGNITION_INCLUDE_PATH gz/utils\n  \1)@g' CMakeLists.txt

# Migrate
find . -regex './include/ignition.*' -type f -print0 | xargs -0 -I {} bash -c 'migrate_ign_to_gz {}' _

# Provision redirection aliases
find . -regex './include/ignition.*' -type f -print0 | xargs -0 -I {} bash -c 'redirection_alias {}' _
find . -regex "./include/ignition.*/CMakeLists.txt" -type f -print0 | xargs -0 -I {} rm {}  # Remove dangling ignition CMakeLists

# Add header level CMakeLists.txt
touch ./include/CMakeLists.txt
echo "add_subdirectory(gz)" > ./include/CMakeLists.txt
echo "install(DIRECTORY ignition DESTINATION \${IGN_INCLUDE_INSTALL_DIR_FULL})" >> ./include/CMakeLists.txt


# TODOs
# Make Export.hh and utils.hh
# Parse libs
# Clone and autobranch/push
