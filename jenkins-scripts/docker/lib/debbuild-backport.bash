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
. ${SCRIPT_DIR}/lib/_common_scripts.bash

cat > build.sh << DELIM
$(generate_buildsh_header)

cd $WORKSPACE/build

export DEBFULLNAME="OSRF Jenkins"
export DEBEMAIL="build@osrfoundation.org"

echo '# BEGIN SECTION: generating the backport (${DEST_UBUNTU_DISTRO}/${ARCH})'
# Only the source package: the container is already a ${DEST_UBUNTU_DISTRO}
# system, so the binaries are built right here. A cowbuilder/pbuilder chroot
# would need a privileged container.
backportpackage --dont-sign \
                -s ${SOURCE_UBUNTU_DISTRO} -d ${DEST_UBUNTU_DISTRO} \
                -w . ${PACKAGE}
BACKPORT_CHANGES=\$(find . -maxdepth 1 -name '*_source.changes')
if [[ \$(echo "\${BACKPORT_CHANGES}" | wc -w) -ne 1 ]]; then
  echo "Expected one *_source.changes from backportpackage, found: \${BACKPORT_CHANGES}"
  exit 1
fi
BACKPORT_DSC=\${BACKPORT_CHANGES%_source.changes}.dsc
echo '# END SECTION'

echo '# BEGIN SECTION: building the backport'
dpkg-source -x \${BACKPORT_DSC} backport_src
cd backport_src
sudo apt-get update
sudo apt-get build-dep -y ./
dpkg-buildpackage -us -uc -b
cd ..
echo '# END SECTION'

echo '# BEGIN SECTION: export pkgs'
mkdir -p $WORKSPACE/pkgs
cp *.tar.* $WORKSPACE/pkgs
cp \${BACKPORT_DSC} $WORKSPACE/pkgs
# debian is only generated in quilt format, native does not have it
cp ../*.debian.* $WORKSPACE/pkgs || true

PKGS=\`find . -maxdepth 1 -name '*.deb' || true\`

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

OSRF_REPOS_TO_USE=${OSRF_REPOS_TO_USE:=stable}
DEPENDENCY_PKGS="devscripts \
                 ubuntu-dev-tools \
                 build-essential \
                 fakeroot \
                 debhelper \
                 wget \
                 ca-certificates \
                 equivs \
                 dh-make"

. ${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash
. ${SCRIPT_DIR}/lib/docker_run.bash
