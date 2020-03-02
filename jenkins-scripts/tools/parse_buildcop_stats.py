#!/usr/bin/env python3
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
alwaysFailsBecause['ign_gazebo-ign-3-win'] = \
        "Dependencies are not being found within the build environment."
alwaysFailsBecause['ign_gazebo-ign-2-win'] = \
        "Dependencies are not being found within the build environment."

header = "\n### Builds with no record of passing\n"
header_printed = False
for j in jobs_colors["red"]:
    name = j["name"]
    url = j["url"]
    if name in alwaysFailsBecause:
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
    if not name in alwaysFailsBecause:
        if not header_printed:
            print(header)
            header_printed = True
        print("\n* Assigned to\n")
        print("    * [![Build Status](%s/badge/icon)](%s) [%s](%s)\n" % (url, url, name, url))

yellow_jobs = sorted(jobs_colors["yellow"], key=lambda j: j["name"])
header = "\n### Unstable builds\n"
header_printed = False
for j in yellow_jobs:
    if not header_printed:
        print(header)
        header_printed = True
    name = j["name"]
    url = j["url"]
    print("* [![Build Status](%s/badge/icon)](%s) [%s](%s)\n" % (url, url, name, url))

