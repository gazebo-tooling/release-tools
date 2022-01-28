#!/usr/bin/env bash
# Copyright (C) 2012 Open Source Robotics Foundation
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

# The script will populate all changelogs in a OSRF release repository
#
# Use:
# $ cd $software-release
# $ ./changelog_spawn <version> [msg]

version=${1}
msg=${2}

if [[ $# -lt 1 ]]; then
    echo "changelog_spawn <version> [msg]"
    exit 1
fi

if [[ -z ${DEBEMAIL} || -z ${DEBFULLNAME} ]]; then
    echo "DEBEMAIL and/or DEBFULLNAME env variables are empty. Needed for changelogs"
    echo "Please add it to your bashrc"
    exit 1
fi

if [[ ${version%-*} == ${version} ]]; then
  echo "Version should contain a revision number"
  exit 1
fi

changelog_example=$(find . -name changelog | head -n 1)

if [[ -z ${changelog_example} ]]; then
    echo "Did not found any changelog files in subdirectories. Check your current path"
    exit 1
fi

pkg_name=$(dpkg-parsechangelog -l${changelog_example} | grep Source | awk '{ print $2 }')
msg_text=${msg:-"${pkg_name} ${version} release"}

echo "Changelogs: "
echo " - pkg     : ${pkg_name}"
echo " - version : ${version}"
echo " - msg     : ${msg_text}"
echo ""

changelog_files=$(find . -name changelog)

for f in $changelog_files; do
    # No changelog for debian by default
    ubuntu_distro=$(dpkg-parsechangelog -l${f} -S distribution)
    version_txt=${version}~${ubuntu_distro}
    if [[ ${f} == ./debian/* ]]; then
      echo " [!] skipped Debian ${ubuntu_distro} from toplevel releasing"
    else
      echo " [x] ${f} [$version_txt]"
    fi
    debchange --package ${pkg_name} \
	      --newversion ${version_txt}  \
	      --distribution ${ubuntu_distro} \
	      --force-distribution \
	      --changelog=${f} -- "${msg_text}" &> ${HOME}/.changelog_spawn.log
done

if [[ -d .git ]]; then
  git diff
  echo
  git status

  echo "All fine to commit? [enter] [control+c to abort]"
  read

  echo
  git commit -am "${msg_text}"
  echo "Commit done"
  git push

elif [[ -d .hg ]]; then
  hg diff
  echo
  hg status

  echo "All fine to commit? [enter] [control+c to abort]"
  read

  echo
  hg commit -m "${msg_text}"
  echo "Commit done"
  hg push -b .
fi
