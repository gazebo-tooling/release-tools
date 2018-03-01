#!/bin/bash
curl https://build.osrfoundation.org/view/main/view/BuildCopFail/api/json 2>/dev/null | python -c '\
import json, sys;
jobs = json.loads(sys.stdin.read())["jobs"];
print("| Type | Count | Percent | Change |")
print("|--|--|--|--|")
print("| total | %d | |  |" % len(jobs));
for c in ["blue", "yellow", "red", "aborted"]:
    jc = [j for j in jobs if j["color"].startswith(c)]
    print("| %s | %d/%d | %.1f%% |  |" % (c, len(jc), len(jobs), 100*float(len(jc)) / len(jobs)))'
