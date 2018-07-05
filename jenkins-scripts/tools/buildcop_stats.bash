#!/bin/bash
curl https://build.osrfoundation.org/view/main/view/BuildCopFail/api/json 2>/dev/null | python -c '\
import datetime, json, sys;
print("# Build Cop Report %s\n" % datetime.date.today())
print("## Aggregate Results as of %s\n" % datetime.datetime.now())
jobs = json.loads(sys.stdin.read())["jobs"];
print("| Type | Count | Percent | Change |")
print("|--|--|--|--|")
print("| total | %d | |  |" % len(jobs));
jobs_colors = {}
for c in ["blue", "yellow", "red", "aborted", "notbuilt"]:
    jc = [j for j in jobs if j["color"].startswith(c)]
    jobs_colors[c] = jc
    print("| %s | %d/%d | %.1f%% |  |" % (c, len(jc), len(jobs), 100*float(len(jc)) / len(jobs)))
print("\n## [Failing Builds](https://build.osrfoundation.org/view/main/view/BuildCopFail/)\n")

# list of reasons build might be failing
alwaysFailsBecause = {}
alwaysFailsBecause["gazebo-ci-default-windows7-amd64"] = \
  "Both pending some fixes under the scope of [PR 529 in release-tools](https://bitbucket.org/osrf/release-tools/pull-requests/529/fix-windows-gazebo-build/diff)"
alwaysFailsBecause["gazebo-ci-gazebo9-windows7-amd64"] = \
  "Both pending some fixes under the scope of [PR 529 in release-tools](https://bitbucket.org/osrf/release-tools/pull-requests/529/fix-windows-gazebo-build/diff)"
alwaysFailsBecause["gazebo-performance-default-xenial-amd64"] = \
  "Performance problems documented in [gazebo issue 2320](https://bitbucket.org/osrf/gazebo/issues/2320/performance_transport_stress-test-times)"
alwaysFailsBecause["ignition_gui-ci-default-windows7-amd64"]       = "Awaiting Windows implementation. In jrivero TODO list"
alwaysFailsBecause["ignition_rendering-ci-default-windows7-amd64"] = "Awaiting Windows implementation. In jrivero TODO list"
alwaysFailsBecause["ignition_sensors-ci-default-windows7-amd64"]   = "Awaiting Windows implementation. In jrivero TODO list"

print("\n### Builds with no record of passing\n")
for j in jobs_colors["red"]:
    name = j["name"]
    url = j["url"]
    if alwaysFailsBecause.has_key(name):
        print("\n* %s\n" % alwaysFailsBecause[name])
        print("    * [![Build Status](%s/badge/icon)](%s) [%s](%s)\n" % (url, url, name, url))

print("\n### Builds that have succeeded in the past, but are failing now\n")
print("\n* internal compiler error, started a new build\n")
print("\n* Jenkins disconnected before build could finish, started a new build\n")
print("\n* not sure\n")
for j in jobs_colors["red"]:
    name = j["name"]
    url = j["url"]
    if not alwaysFailsBecause.has_key(name):
        print("    * [![Build Status](%s/badge/icon)](%s) [%s](%s)\n" % (url, url, name, url))
'
