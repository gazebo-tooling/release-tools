#!/usr/bin/env python
from __future__ import print_function
import re
import sys

if len(sys.argv) != 2:
    print('need to specify branch name', file=sys.stderr)
    sys.exit(1)
branchName = sys.argv[1]

pattern = 'ci_matching_branch/'
match = re.search(pattern, branchName)
if match:
    print(branchName, "matches", pattern)
else:
    print("{} does not match {}".format(branchName, pattern))
    sys.exit(1)
