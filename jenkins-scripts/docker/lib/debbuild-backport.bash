#!/bin/bash -x

NIGHTLY_MODE=false
if [ "${UPLOAD_TO_REPO}" = "nightly" ]; then
   OSRF_REPOS_TO_USE="stable nightly"
   NIGHTLY_MODE=true
fi


# Do not use the subprocess_reaper in debbuild. Seems not as needed as in
# testing jobs and seems to be slow at the end of jenkins jobs
export ENABLE_REAPER=false

. ${SCRIPT_DIR}/lib/boilerplate_prepare.sh

cat > build.sh << DELIM
###################################################
# Make project-specific changes here
#
#!/usr/bin/env bash
set -ex

cd $WORKSPACE/build

export DEBFULLNAME="OSRF Jenkins"
export DEBEMAIL="build@osrfoundation.org"

echo '# BEGIN SECTION: generating the backport (${DEST_UBUNTU_DISTRO}/${ARCH})'
export COWBUILDER_BASE_PATH=/var/lib/pbuilder/${DEST_UBUNTU_DISTRO}-base.cow
mkdir -p /var/lib/pbuilder/
cowbuilder --create \
           --distribution ${DEST_UBUNTU_DISTRO} \
	   --components "main restricted universe multiverse" \
           --basepath=\${COWBUILDER_BASE_PATH}

BASEPATH=\${COWBUILDER_BASE_PATH} \
  backportpackage --dont-sign \
		  -b \
                  --builder cowbuilder \
		  -s ${SOURCE_UBUNTU_DISTRO} -d ${DEST_UBUNTU_DISTRO} \
		  -w . ${PACKAGE}
echo '# END SECTION'

echo '# BEGIN SECTION: export pkgs'
mkdir -p $WORKSPACE/pkgs
cp *.tar.* $WORKSPACE/pkgs
cp buildresult/*.dsc $WORKSPACE/pkgs
# debian is only generated in quilt format, native does not have it
cp ../*.debian.* $WORKSPACE/pkgs || true

PKGS=\`find buildresult/ -name '*.deb' || true\`

FOUND_PKG=0
for pkg in \${PKGS}; do
    echo "found \$pkg"
    # Check for correctly generated packages size > 3Kb
    [[ \$(find \$pkg -size +3k) ]] || echo "WARNING: empty package?"
    cp \${pkg} $WORKSPACE/pkgs
    FOUND_PKG=1
done
# check at least one upload
test \$FOUND_PKG -eq 1 || exit 1
echo '# END SECTION'
DELIM

# Workaround to invoke the --priveleged option in docker, needed for pbuilder
USE_GPU_DOCKER=true
OSRF_REPOS_TO_USE=${OSRF_REPOS_TO_USE:=stable}
DEPENDENCY_PKGS="devscripts \
		 ubuntu-dev-tools \
		 debhelper \
		 wget \
		 ca-certificates \
		 equivs \
		 dh-make \
		 cowbuilder"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
