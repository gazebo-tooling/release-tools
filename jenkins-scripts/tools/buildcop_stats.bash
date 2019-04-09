#!/bin/bash
curl https://build.osrfoundation.org/view/ign-acropolis/api/json 2>/dev/null | python -c '\
import datetime, json, re, sys;
print("# Build Cop Report %s\n" % datetime.date.today())
print("## Aggregate Results as of %s\n" % datetime.datetime.now())
jobs = json.loads(sys.stdin.read())["jobs"];
print("| Type | Count | Percent | Change |")
print("|--|--|--|--|")
print("| total | %d | |  |" % len(jobs));
jobs_colors = {}
for c in ["yellow", "red", "aborted", "notbuilt"]:
    jc = [j for j in jobs if j["color"].startswith(c)]
    jobs_colors[c] = jc
    print("| %s | %d/%d | %.1f%% |  |" % (c, len(jc), len(jobs), 100*float(len(jc)) / len(jobs)))
print("\n## [ign-acropolis builds](https://build.osrfoundation.org/view/ign-acropolis/)\n")

# list of reasons build might be failing
alwaysFailsBecause = {}
alwaysFailsBecause["ign_gazebo-ign-1-win"] = \
  "Pending fixes to build script under the scope of [PR 789 in release-tools](https://bitbucket.org/osrf/release-tools/pull-requests/789/add-ign-gazebo-colcon-script-for-windows/diff)"
alwaysFailsBecause["ign_gazebo-ci-win"] = \
  "Pending fixes to build script under the scope of [PR 789 in release-tools](https://bitbucket.org/osrf/release-tools/pull-requests/789/add-ign-gazebo-colcon-script-for-windows/diff)"
alwaysFailsBecause["gazebo-performance-default-xenial-amd64"] = \
  "Performance problems documented in [gazebo issue 2320](https://bitbucket.org/osrf/gazebo/issues/2320/performance_transport_stress-test-times)"

print("\n### Builds with no record of passing\n")
for j in jobs_colors["red"]:
    name = j["name"]
    url = j["url"]
    if alwaysFailsBecause.has_key(name):
        print("\n* %s\n" % alwaysFailsBecause[name])
        print("    * [![Build Status](%s/badge/icon)](%s) [%s](%s)\n" % (url, url, name, url))

print("\n### Unexpected failures\n")
for j in jobs_colors["red"]:
    name = j["name"]
    url = j["url"]
    if not alwaysFailsBecause.has_key(name):
        print("\n* Assigned to\n")
        print("    * [![Build Status](%s/badge/icon)](%s) [%s](%s)\n" % (url, url, name, url))

print("\n## Statistics of non-blue builds by project name\n")
# sort by job name prefix
ci_jobs = {}
install_jobs = []
for job in jobs:
    ci_match = re.search("^.*-ci-", job["name"])
    ign_windows_match = re.search("^ign_", job["name"])
    install_match = re.search("^.*-install-", job["name"])
    gazebo_match = re.search("^gazebo", job["name"])
    if install_match:
        install_jobs.append(job)
    elif gazebo_match:
        gzci = "gazebo-ci-"
        if not gzci in ci_jobs:
            ci_jobs[gzci] = []
        ci_jobs[gzci].append(job)
    elif ci_match:
        ci_name = ci_match.group(0)
        if not ci_name in ci_jobs:
            ci_jobs[ci_name] = []
        ci_jobs[ci_name].append(job)
    elif ign_windows_match:
        win_name = ign_windows_match.group(0) + ".*-win"
        if not win_name in ci_jobs:
            ci_jobs[win_name] = []
        ci_jobs[win_name].append(job)
    else:
        print("Unmatched: %s" % (job["name"]))

ci_jobs_sorted = list(ci_jobs.keys())
ci_jobs_sorted.sort()
for job_name in ci_jobs_sorted:
    job_count = len(ci_jobs[job_name])
    header_printed = False
    print("\n## %s\n" % job_name)
    header = "| Type | Count | Percent | Change |\n"
    header += "|--|--|--|--|"
    for c in ["blue", "yellow", "red", "aborted", "notbuilt"]:
        jc = [j for j in ci_jobs[job_name] if j["color"].startswith(c)]
        if len(jc) > 0:
            if not header_printed:
                print(header)
                header_printed = True
            print("| %s | %d/%d | %.1f%% |  |" % (c, len(jc), job_count, 100*float(len(jc)) / job_count))

header_printed = False
header = "\n## Install jobs\n\n"
header += "| Type | Count | Percent | Change |\n"
header += "|--|--|--|--|"
for c in ["blue", "yellow", "red", "aborted", "notbuilt"]:
    jc = [j for j in install_jobs if j["color"].startswith(c)]
    if len(jc) > 0:
        if not header_printed:
            print(header)
            header_printed = True
        print("| %s | %d/%d | %.1f%% |  |" % (c, len(jc), len(install_jobs), 100*float(len(jc)) / len(install_jobs)))
'
