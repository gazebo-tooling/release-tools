#!/bin/bash

set -e

# 0. Pre checks
find . -name control -type f | while IFS= read -r f; do
  if grep "transitional package" "${f}" > /dev/null; then
    echo "There are transitional packages declared in ${f}"
    echo "Please remove them, probably not needed."
    exit 1
  fi
done

# 1. Block: change in the control file
find . -name control -type f | while IFS= read -r f; do
  if [[ $f == ./ubuntu/debian/tests/control ]]; then
    continue
  fi
  echo "Processing control file: ${f}"
  ignition_pkgs=$(grep-dctrl -sPackage -n '' ${f})
  echo " - Replace ignition ocurrences"
  # Rename packages and sources
  sed -i -e 's/Package: python3-ignition-gazebo/Package: python3-gz-sim/g' "${f}"
  sed -i -e 's/Package: ignition-gazebo/Package: gz-sim/g' "${f}"
  sed -i -e 's/Package: libignition-gazebo/Package: libgz-sim/g' "${f}"
  sed -i -e 's/Source: ignition-gazebo/Source: gz-sim/g' "${f}"

  sed -i -e 's/Package: python3-ignition/Package: python3-gz/g' "${f}"
  sed -i -e 's/Package: ignition/Package: gz/g' "${f}"
  sed -i -e 's/Package: libignition/Package: libgz/g' "${f}"
  sed -i -e 's/Source: ignition/Source: gz/g' "${f}"
  # Inject transitional (alias) packages
  echo " - Transitional packages in control file"
  for pkg in ${ignition_pkgs}; do
    echo "  * Processing transitional package for ${pkg}"
    new_pkg_name=${pkg/ignition/gz}
    new_pkg_name=${new_pkg_name/gazebo/sim}
    cat << EOF >> "${f}"

Package: ${pkg}
Depends: ${new_pkg_name}, \${misc:Depends}
Architecture: all
Priority: optional
Section: oldlibs
Description: transitional package
 This is a transitional package. It can safely be removed.
EOF
  done
  echo
done

# 5. Rename all ignition symlinks and destinations
find . -name '*ignition-gazebo*' -type l | while IFS= read -r f; do
  echo "Renaming ${f} -> ${f/ignition-gazebo/gz-sim}"
  link=$(readlink "$f")
  ln -sfT "${link//ignition-gazebo/gz-sim}" "${f}"
  git mv "${f}" "${f/ignition-gazebo/gz-sim}"
done

find . -name '*ignition*' -type l | while IFS= read -r f; do
  echo "Renaming ${f} -> ${f/ignition-/gz-}"
  link=$(readlink "$f")
  ln -sfT "${link//ignition-/gz-}" "${f}"
  git mv "${f}" "${f/ignition-/gz-}"
done

echo

# 6. Rename all ignition filenames not symlinks
find . -name '*ignition-gazebo*' -type f | while IFS= read -r f; do
  echo "Renaming ${f} -> ${f/ignition-gazebo/gz-sim}"
  git mv "${f}" "${f/ignition-gazebo/gz-sim}"
done

find . -name '*ignition*' -type f | while IFS= read -r f; do
  echo "Renaming ${f} -> ${f/ignition-/gz-}"
  git mv "${f}" "${f/ignition-/gz-}"
done

echo

# # 7. Revert Provides, Breaks, Replaces
# find . -name control -type f | while IFS= read -r f; do
#   sed -i -e 's:\(Breaks\:.*\)gz-\(.*\):\1ignition-\2:' "${f}"
#   sed -i -e 's:\(Provides\:.*\)gz-\(.*\):\1ignition-\2:' "${f}"
#   sed -i -e 's:\(Replaces\:.*\)gz-\(.*\):\1ignition-\2:' "${f}"
# done

echo "Do manually:"
echo " * Update depends from the same library"
echo " * Add breaks / replaces for previous nightly"
