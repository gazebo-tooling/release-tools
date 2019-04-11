#!/bin/bash

views='ign-acropolis ign-blueprint'

echo "# Build Cop Report $(date +%Y-%m-%d)"
echo "## Aggregate Results as of $(date '+%Y-%m-%d %H:%M:%S')"


for v in ${views}; do
curl https://build.osrfoundation.org/view/"${v}"/api/json 2>/dev/null | VIEW=${v} python -c '\
import datetime, json, re, sys, os;
view=os.environ["VIEW"]

print("\n## [" + view + " builds](https://build.osrfoundation.org/view/" + view + "/)\n")
jobs = json.loads(sys.stdin.read())["jobs"];
print("| Type | Count | Percent | Change |")
print("|--|--|--|--|")
print("| total | %d | |  |" % len(jobs));
jobs_colors = {}
for c in ["yellow", "red", "aborted", "notbuilt"]:
    jc = [j for j in jobs if j["color"].startswith(c)]
    jobs_colors[c] = jc
    print("| %s | %d/%d | %.1f%% |  |" % (c, len(jc), len(jobs), 100*float(len(jc)) / len(jobs)))

# list of reasons build might be failing
alwaysFailsBecause = {}
alwaysFailsBecause["ign_gazebo-ign-1-win"] = \
  "Pending fixes to build script under the scope of [PR 789 in release-tools](https://bitbucket.org/osrf/release-tools/pull-requests/789/add-ign-gazebo-colcon-script-for-windows/diff)"
alwaysFailsBecause["ign_gazebo-ci-win"] = \
  "Pending fixes to build script under the scope of [PR 789 in release-tools](https://bitbucket.org/osrf/release-tools/pull-requests/789/add-ign-gazebo-colcon-script-for-windows/diff)"
alwaysFailsBecause["gazebo-performance-default-xenial-amd64"] = \
  "Performance problems documented in [gazebo issue 2320](https://bitbucket.org/osrf/gazebo/issues/2320/performance_transport_stress-test-times)"

header = "\n### Builds with no record of passing\n"
header_printed = False
for j in jobs_colors["red"]:
    name = j["name"]
    url = j["url"]
    if alwaysFailsBecause.has_key(name):
        if not header_printed:
            print(header)
            header_printed = True
        print("\n* %s\n" % alwaysFailsBecause[name])
        print("    * [![Build Status](%s/badge/icon)](%s) [%s](%s)\n" % (url, url, name, url))

header = "\n### Unexpected failures\n"
header_printed = False
for j in jobs_colors["red"]:
    name = j["name"]
    url = j["url"]
    if not alwaysFailsBecause.has_key(name):
        if not header_printed:
            print(header)
            header_printed = True
        print("\n* Assigned to\n")
        print("    * [![Build Status](%s/badge/icon)](%s) [%s](%s)\n" % (url, url, name, url))

yellow_jobs = list(jobs_colors["yellow"])
yellow_jobs.sort()
header = "\n### Unstable builds\n"
header_printed = False
for j in yellow_jobs:
    if not header_printed:
        print(header)
        header_printed = True
    name = j["name"]
    url = j["url"]
    print("* [![Build Status](%s/badge/icon)](%s) [%s](%s)\n" % (url, url, name, url))
'
done
