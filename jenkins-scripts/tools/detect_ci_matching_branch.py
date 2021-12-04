#!/usr/bin/env python
from __future__ import print_function
import re
import sys

if len(sys.argv) != 2:
    print('need to branch name', file=sys.stderr)
    exit()
branchName = sys.argv[1]

pattern = 'ci_matching_branch/'
match = re.search(pattern, branchName)
if match:
    print(f"{branchName} matches {pattern}")
else:
    sys.exit(f"{branchName} does not match {pattern}")
