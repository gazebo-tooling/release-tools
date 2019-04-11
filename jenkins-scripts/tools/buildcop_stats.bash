#!/bin/bash

views='ign-acropolis ign-blueprint'

echo "# Build Cop Report $(date +%Y-%m-%d)"
echo "## Aggregate Results as of $(date '+%Y-%m-%d %H:%M:%S')"

for v in ${views}; do
  curl https://build.osrfoundation.org/view/"${v}"/api/json 2>/dev/null \
    | VIEW=${v} python parse_buildcop_stats.py
done
