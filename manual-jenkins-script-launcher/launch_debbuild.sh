#!/bin/bash
#
# Please, see README for documentation
# 

if [[ ${#} -lt 2 ]]; then
    echo "Usage: ${0} <package> <version> [release-version] [distro] [arch] [source_uri] [release_repo_branch] [package_alias]"
    exit -1
fi

export PACKAGE=${1}
export VERSION=${2}
export RELEASE_VERSION=${3-1}
export DISTRO=${4-precise}
export ARCH=${5-amd64}
export RELEASE_REPO_BRANCH=${7-default}
export PACKAGE_ALIAS=${8-${PACKAGE}}
export SOURCE_TARBALL_URI=${6-http://gazebosim.org/assets/distributions/${PACKAGE_ALIAS}-${VERSION}.tar.bz2}
export WORKSPACE=/tmp/workspace

. prepare_env.sh

echo ""
echo "Sumary"
echo "---------------------"
echo " - Package         : ${PACKAGE}"
echo " - Version         : ${VERSION}"
echo " - Release_version : ${RELEASE_VERSION}"
echo " - Distro          : ${DISTRO}"
echo " - Arch            : ${ARCH}"
echo " - Source URI      : ${SOURCE_TARBALL_URI}"
echo " - Release branch  : ${RELEASE_REPO_BRANCH}"
echo ""
echo " - Workspace       : ${WORKSPACE}"
echo ""

# prepare the workspace
set_up_workspace

# get release-tools
set_up_release_tools

# Be sure of not uploading anything
sed -i -e 's:GNUPGHOME.*::g' ${SCRIPT_DIR}/lib/debbuild-base.bash
sed -i -e 's:scp.*::g' ${SCRIPT_DIR}/lib/debbuild-base.bash
mkdir -p ${FAKE_HOME}/pbuilder/${DISTRO}_result/
sed -i -e "s:/var/packages/gazebo/ubuntu:${FAKE_HOME}/pbuilder/${DISTRO}_result/:g" ${SCRIPT_DIR}/lib/debbuild-base.bash

echo "3. Calling jenkins script"
# pbuilder via sudo needs to own home
chmod +x ${SCRIPT_DIR}/multidistribution-debbuild.bash
# root needs to own home for pbuilder
sudo chown -R root:root ${FAKE_HOME}
HOME=${FAKE_HOME} ${SCRIPT_DIR}/multidistribution-debbuild.bash
