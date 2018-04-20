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
for j in jobs_colors["red"]:
    name = j["name"]
    url = j["url"]
    print("* [![Build Status](%s/badge/icon)](%s) [%s](%s)\n" % (url, url, name, url))'
