#!/bin/bash -e
#
if [[ -z $(ls -- *.xml) ]]; then
  echo "No .xml file founds. Generate them using:"
  echo "https://github.com/gazebo-tooling/release-tools/blob/master/jenkins-scripts/README.md#L11"
  exit 1
fi

not_null=$(grep -3 'null' -- *.xml || true)
if [[ -n ${not_null} ]]; then
  echo "Found a null value in a configuration file:"
  echo "${not_null}"
  echo "Probably a problem in the code"
  exit 1
fi

# Check for existing scripts
for f in $(grep -Eh -o './scripts/.*' -- *.xml | awk '{print $1}' | sed 's/"//' | sort | uniq); do
  if ! test -f "${f}"; then
    echo "${f} script not found in the repository"
  fi
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
