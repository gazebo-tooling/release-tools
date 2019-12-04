# Copyright (C) 2019 Open Source Robotics Foundation
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

# The script will bump major version in a repo
#
# Use:
# $ cd $software-release
# $ ./bump_major_version.bash <old_version> <new_version>

old_version=${1}
new_version=${2}

if [[ $# -lt 1 ]]; then
    echo "bump_major_version.bash <old_version> <new_version>"
    exit 1
fi

if [[ ! -d ubuntu/ ]]; then
    echo "No ubuntu/ directory found. Are you in a -release repo?"
    exit 1
fi


if [[ $(find . -name *${old_version}-*.install | wc -l) -lt 1 ]]; then
    echo "Not found a single install file with the old_version number"
    exit 1
fi

# extract software name (prune for not in debian/, -quit to stop at first)
old_software_name=$(find . -path ./debian -prune -o -name changelog -exec dpkg-parsechangelog -l {} -S source \; -quit)
new_software_name=${old_software_name/${old_version}/${new_version}}

echo "Moving -release info"
echo "=============================="
echo " - From: ${old_software_name}"
echo " - To  : ${new_software_name}"
echo "============================="
echo

# Check origin install files for versioning
for f in $(find ubuntu/ -name *${old_version}*.install -type f); do
    echo " * Making unversioned file from ${f}"
    new_name=${f/${old_version}/${new_version}}
    git mv ${f} ${new_name}
    file_name=${f##*/}
    echo "   - Move symlinks"
    symlinks=$(find . -name ${file_name} -type l)
    for s in $symlinks; do
	echo "     - move ${s}"
	root_new_name="../../ubuntu/debian/${new_name##*/}"
	ln -sf ${root_new_name} ${s}
	if ! [ -e ${s} ]; then
	    ln -sf ../${root_new_name} ${s}
	fi
	git mv ${s} ${s%/*}/${new_name##*/}
    done
done
echo

# Renaming install files
echo " * Renaming install files"
for f in $(find . -name *${old_version}*.install); do
    new_name=${f/${old_version}/${new_version}}
    echo "    ${f} -> ${new_name}"
    git mv ${f} ${new_name}
done
echo

# Prepare changelog for changelog_spawn
echo " * Update changelogs"
for f in $(find . -name changelog -type f); do
    echo "     ${f}"
    msg_text=${msg:-"Stub to be removed after first entry"}
    distribution=$(dpkg-parsechangelog -l "${f}" -S distribution)
    debchange --package "${new_software_name}" \
	      --newversion "${old_version}.999.999-1~${distribution}"  \
	      --distribution ${distribution} \
	      --force-distribution \
	      --changelog="${f}" -- "${msg_text}"
    sed -i -e '7,$d' ${f}
done
echo

# Changes in control
for f in $(find . -name control -type f); do
    echo " * Changing values in control file"
    sed -i -e "s:${old_software_name}:${new_software_name}:g" "${f}"
    # Change short version of ign-too
    sed -i -e "s:${old_software_name/ignition/ign}:${new_software_name/ignition/ign}:g" "${f}"
    # Change bitbucket for github
    sed -i -e "s:https\://bitbucket.org/ignitionrobotics/:https\://github.com/ignition-release/:g" "${f}"
    sed -i -e "s:https\://bitbucket.org/osrf/:https\://github.com/ignition-release/:g" "${f}"
    echo " * Safety checks in control"
    # 1. Check for custom version modifiers in control file
    # 2. Breaks or replaces clause?
    if [[ -z $(grep Breaks ${f}) ]]; then
	echo "    ! FIXME: Breaks clause detected in control file ${f}"
    fi
    if [[ -z $(grep Replaces ${f}) ]]; then
	echo "    ! FIXME: Replaces clause detected in control file ${f}"
    fi
done
echo

for f in $(find . -name rules -type f); do
    echo " * Changing values in rules file"
    sed -i -e "s:${old_software_name}:${new_software_name}:g" "${f}"
    echo
done

# Reviewing all symlinks
echo " * Safety checks in symlinks"
for s in $(find . -type l); do
    if [[ ! -e ${s} ]]; then
	echo "    ! FIXME: Found broken link ${s}"
    fi
done
