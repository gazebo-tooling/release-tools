#!/bin/bash
#
# Please, see README for documentation
# 

if [[ ${#} -lt 2 ]]; then
    echo "Usage: ${0} <package> <git-repo> [release-version] [release-arch-version] [distro] [arch] [branch/tag]"
    exit -1
fi

export PACKAGE=${1}
export GIT_REPOSITORY=${2}
export RELEASE_VERSION=${3-1}
export RELEASE_ARCH_VERSION=${4-1}
export DISTRO=${5-precise}
export ARCH=${6-amd64}
export BRANCH=${7-default}
export WORKSPACE=/tmp/workspace

. prepare_env.sh

echo ""
echo "Sumary"
echo "---------------------"
echo " - Package         : ${PACKAGE}"
echo " - Git repo        : ${GIT_REPOSITORY}"
echo " - Release_version : ${RELEASE_VERSION}"
echo " - R_arch_version  : ${RELEASE_ARCH_VERSION}"
echo " - Distro          : ${DISTRO}"
echo " - Arch            : ${ARCH}"
echo " - Branch/Tag      : ${BRANCH}"
echo ""
echo " - Workspace       : ${WORKSPACE}"
echo ""

# prepare the workspace
set_up_workspace

# get release-tools
set_up_release_tools

# Be sure of not uploading anything
mkdir -p ${FAKE_HOME}/pbuilder/${DISTRO}_result/
sed -i -e "s:/var/packages/gazebo/ubuntu:${FAKE_HOME}/pbuilder/${DISTRO}_result/:g" ${SCRIPT_DIR}/lib/debian-git-debbuild.bash

echo "3. Calling jenkins script"
# pbuilder via sudo needs to own home
chmod +x ${SCRIPT_DIR}/debian-git-debbuild.bash
# root needs to own home for pbuilder
sudo chown -R root:root ${FAKE_HOME}
HOME=${FAKE_HOME} ${SCRIPT_DIR}/debian-git-debbuild.bash
