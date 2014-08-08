#!/bin/bash -x
set -ex

if [[ -z ${DISTRO} ]]; then
    echo 'Error: $DISTRO parameter was not set' && exit 1
fi

if [[ -z ${ARCH} ]]; then
    echo 'Error: ARCH parameter was not set' && exit 1
fi

if [[ -z ${PACKAGE_ALIAS} ]]; then
    echo 'Error: PACKAGE_ALIAS parameter was not set' && exit 1
fi

NEEDED_HOST_PACKAGES="openssh-client"
QUERY_HOST_PACKAGES=$(dpkg-query -Wf'${db:Status-abbrev}' ${NEEDED_HOST_PACKAGES} 2>&1) || true
if [[ -n ${QUERY_HOST_PACKAGES} ]]; then
  sudo apt-get update
  sudo apt-get install -y ${NEEDED_HOST_PACKAGES}
fi

# Be sure we are not removing something not -nightly
if [ `echo $PACKAGE_ALIAS | sed -e 's:nightly::'` == $PACKAGE_ALIAS ]; then
   echo "Sanity check fail! Close to remove something with no nightly in the name" && exit 1
fi

# Remove all nightly version except latest three
ssh -o StrictHostKeyChecking=no -i $HOME/.ssh/id_rsa ubuntu@old.gazebosim.org "ls -t /var/www/assets/distributions/${PACKAGE_ALIAS}_*~${DISTRO}_${ARCH}.deb | sed -e '1,3d' | xargs -d '\n' rm -f"
