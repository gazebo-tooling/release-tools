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

# we don't support buster anymore
if [[ -d debian/buster ]]; then
  git rm -fr debian/buster
fi

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
  sed -i -e 's/Package: ruby-ignition/Package: ruby-gz/g' "${f}"
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

# 2.0 Rename all ignition symlinks and destinations
find . -name '*ignition-gazebo*' -type l | while IFS= read -r f; do
  echo "Renaming ${f} -> ${f/ignition-gazebo/gz-sim}"
  link=$(readlink "$f")
  ln -sfT "${link//ignition-gazebo/gz-sim}" "${f}"
  git mv "${f}" "${f/ignition-gazebo/gz-sim}"
done

# 2. Copyright files
find . -name '*copyright*' -type f | while IFS= read -r f; do
  echo "Procesing copyright file: ${f}"
  sed -i -e 's:bitbucket.org/:github.org/:g' "${f}"
  sed -i -e 's:/osrf/:/ignitionrobotics/:g' "${f}"
  sed -i -e 's:ignitionrobotics:gazebosim:g' "${f}"
  sed -i -e 's:ignition:gz:g' "${f}"
  sed -i -e 's:ign_:gz-:g' "${f}"
  echo
done

# 3. Test file if needed
if [[ -f ubuntu/debian/tests/ ]]; then
  echo "Procesing copyright tests control"
  sed -i -e 's:ignition:gz:g' ubuntu/debian/tests/control
  echo
fi

# 4. Changelog files
find . -name 'changelog' -type f | while IFS= read -r f; do
  echo "Bumping changelog file ${f}"
  distro=$(dpkg-parsechangelog -l${f} -S distribution)
  version=$(dpkg-parsechangelog -l${f} -S Version  | sed 's:~.*::g')
  version_without_rev=${version/-*}
  bump_rev=$(( ${version/*-} + 1 ))
  dch --changelog "${f}" \
      --distribution="${distro}" \
      --force-distribution \
      --newversion "${version_without_rev}-${bump_rev}~${distro}" \
        "Change package source name"
  sed -i -e '1 s:ignition-gazebo:gz-sim:g' "${f}"
  sed -i -e '1 s:ignition:gz:g' "${f}"
done

echo

# 5. Rename all ignition symlinks and destinations
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

# 6. watch files
find . -name 'watch' -type f | while IFS= read -r f; do
  echo "Changeing watch file ${f}"
  sed -i -e 's:/osrf/:/ignitionrobotics/:g' "${f}"
  sed -i -e 's:ignitionrobotics:gazebosim:g' "${f}"
  sed -i -e 's:ignition:gz:g' "${f}"
done

echo

 # 7. Revert Provides, Breaks, Replaces
 find . -name control -type f | while IFS= read -r f; do
   sed -i -e 's:\(Breaks\:.*\)gz-\(.*\):\1ignition-\2:' "${f}"
   sed -i -e 's:\(Provides\:.*\)gz-\(.*\):\1ignition-\2:' "${f}"
   sed -i -e 's:\(Replaces\:.*\)gz-\(.*\):\1ignition-\2:' "${f}"
done

echo "Do manually:"
echo " * Update depends from the same library"
echo " * Check breaks / replaces for previous nightly"
echo " * Update rules"
