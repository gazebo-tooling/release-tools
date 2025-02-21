#!/bin/bash -e
#
if [[ -z $(ls -- *.xml) ]]; then
  echo "No .xml file founds. Generate them using:"
  echo "https://github.com/gazebo-tooling/release-tools/blob/master/jenkins-scripts/README.md#L11"
  exit 1
fi

# Some XML generated files include valid Groovy code using null
# like the _outdated_jobs job. Do not search for null unconditionally
not_null=$(grep -3 '>.*null.*<' -- *.xml || true)
if [[ -n ${not_null} ]]; then
  echo "Found a null value in a configuration file:"
  echo "${not_null}"
  echo "Probably a problem in the code"
  exit 1
fi

# Check for existing scripts. Lines:
# 1. lookg for ./scripts/. and exclude comment lnes
# 2. replaces %WORKSPACE% by .
# 3. grab only the path from ./scripts/
# 4. remove spurious "
# 5. sor and uniq to get clean output
for f in $(grep -Eh './scripts/.*' -- *.xml | grep -v '//' | \
           sed 's/%WORKSPACE%/./g' | \
           grep -Eh -o './scripts/.*' | awk '{print $1}' | \
           sed 's/"//g' | \
           sort | uniq); do
  if ! test -f "${f}"; then
    echo "${f} script not found in the repository"
  fi
done

# Check conda enviroments links
for f in $(ls *-c*win.xml); do
  CONDA_ENV_NAME=$(grep -oP 'CONDA_ENV_NAME=\K.*' $f)
  for link in "$(grep -Eh './scripts/conda/envs.*' $f | grep -v '//' | \
           sed 's/%WORKSPACE%/./g' | \
           sed "s/%CONDA_ENV_NAME%/${CONDA_ENV_NAME}/g" |\
           grep -Eh -o './scripts/.*' | awk '{print $1}' | \
           sed 's/"//g' | \
           uniq)"; do
    echo $link
    if ! test -d "${link}"; then
      echo "${link} conda env not found in the repository in file ${f}"
      exit 1
    fi
  done
done

abichecker_main=$(grep '<branch>main</branch>' -- *-abichecker-*.xml || true)
if [[ -n ${abichecker_main} ]]; then
  echo "Found a main branch in an abichecker job:"
  echo "${abichecker_main}"
  echo "Main branches are not target of abichecking. Fix the code."
  exit 1
fi

# Check authorized/expected github organizations and/or repositories
non_github_orgs=$(grep -R "https?://github.com/" -- *.xml | \
  grep -E -v '/gazebosim/|/gazebo-tooling/|/gazebo-release/|/gazebo-forks/' | \
  grep -E -v '/osrf/|/ros-simulation/|/ros2?-gbp/' | \
  grep -E -v '/j-rivero/ratt/' || true)
if [[ -n ${non_github_orgs} ]]; then
  echo "Unexpected github orgs in XML files:"
  echo "${non_github_orgs}"
  echo "either update this test or fix the DSL code"
  exit 1
fi

# Check that whiteListedTargetBranches are non empty in all generated gazebo_libs
# see https://github.com/gazebo-tooling/release-tools/pull/1144
# For other jobs the use case is valid since pr can be enabled on all branches
empty_branches_on_github_triggered=$(grep '<whiteListTargetBranches></whiteListTargetBranches>' \
                                      -- {gz_,sdformat}*{-abichecker-,-pr_any-}*.xml || true)
if [[ -n ${empty_branches_on_github_triggered} ]]; then
  echo "Unexpected whiteListTargetBranches without values. It will trigger all branches:"
  echo "${empty_branches_on_github_triggered}"
  exit 1
fi

# Filter out the previous auto jobs
filtered_dir=$(mktemp -d)
cp -- *-abichecker-*.xml "${filtered_dir}"
rm -f "${filtered_dir}"/*-ubuntu_auto*.xml
repeated=$(grep '\<branch>' "${filtered_dir}"/*-abichecker-*.xml | awk '{ print $2 }' | sort | uniq -d)
if [[ -n ${repeated} ]]; then
   echo "Found a duplicate in an abichecker branch:"
   echo "${repeated}"
   echo "please exclude one of the versions in the yaml file to reduce the server workload"
   exit 1
fi

avoid_infinite_build_archive=$(grep '<numToKeep>-1</numToKeep>' -- *.xml || true)
if [[ -n ${avoid_infinite_build_archive} ]]; then
  echo "Found a job setup to keep infinite number of builds. This is BAD"
  echo "${avoid_infinite_build_archive}"
  exit 1
fi
