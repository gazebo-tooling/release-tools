#!/bin/bash

if [[ $# -lt 2 ]]; then
    echo "Usage: <old.install.file> <new.name>"
    exit 1
fi

echo " + Move original install file"
hg mv ubuntu/debian/${1} ubuntu/debian/${2}
echo " - Move symlinks"
symlinks=$(find . -name ${1} -type l)
for s in $symlinks; do
    echo "  + move ${s}"
    ln -sf ../../ubuntu/debian/${2} ${s}
    hg mv ${s} ${s%/*}/${2}
done
