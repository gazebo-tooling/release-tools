#!/bin/bash -x

if [[ -z ${DISTRO} ]]; then
    echo 'Error: $DISTRO parameter was not set'
    exit 1
fi

NEEDED_HOST_PACKAGES="reprepro openssh"
QUERY_HOST_PACKAGES=$(dpkg-query -Wf'${db:Status-abbrev}' ${NEEDED_HOST_PACKAGES} 2>&1) || true
if [[ -n ${QUERY_HOST_PACKAGES} ]]; then
  sudo apt-get update
  sudo apt-get install -y ${NEEDED_HOST_PACKAGES}
fi

for pkg in `ls $WORKSPACE/pkgs/*.deb`; do
  GNUPGHOME=$HOME/.gnupg reprepro includedeb $DISTRO ${pkg}
  scp -o StrictHostKeyChecking=no -i $HOME/.ssh/id_rsa ${pkg} ubuntu@gazebosim.org:/var/www/assets/distributions
done

rm -fr $WORKSPACE/pkgs/*.deb
