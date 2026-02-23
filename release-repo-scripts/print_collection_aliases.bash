#!/usr/bin/env bash

set -e

Collection=${1}

if [[ $# -lt 1 ]]; then
    echo "print_collection_aliases <Collection>"
    exit 1
fi

collection_lower=$(echo ${Collection} | tr '[:upper:]' '[:lower:]')

for p in $(grep '^Package:' ubuntu/debian/control | sed -e 's@^Package: *@@')
do
  p_collection=$(echo $p | sed -e "s@gz-@gz-${collection_lower}-@")
  cat << DELIM

Package: ${p_collection}
Depends: ${p} (= \${binary:Version}), \${misc:Depends}
Architecture: all
Priority: optional
Section: metapackages
Description: alias package
 Provides a package for ${Collection} without exposing the version number.
DELIM

done
