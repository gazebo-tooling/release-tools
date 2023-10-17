#!/bin/bash -e
#
if [[ -z $(ls -- *.xml) ]]; then
  echo "No .xml file founds. Generate them using:"
  echo "https://github.com/gazebo-tooling/release-tools/blob/master/jenkins-scripts/README.md#L11"
fi

main=$(grep '<branch>main</branch>' *-abichecker-*.xml || true)
if [[ -n ${main} ]]; then
  echo "Found a main branch in an abichecker job:"
  echo "${main}"
  echo "Main branches are not target of abichecking. Fix the code."
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
