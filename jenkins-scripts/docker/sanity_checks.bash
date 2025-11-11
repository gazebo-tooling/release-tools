#!/bin/bash -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

#
# Check for existing scripts. Lines:
# 1. Find all .bash files excluding *_checks.bash
# 2. Search for occurrences of ${SCRIPT_DIR}/ in the files
# 3. Exclude lines containing '//' (comments)
# 4. Extract the paths containing ${SCRIPT_DIR}/
# 5. Remove spurious "
# 6. Sort and remove duplicates to get clean output
for f in $(find -type f -name '*.bash' ! -path '*_checks.bash' -exec grep -Eh '\${SCRIPT_DIR}/' {} \; | grep -v '//' | \
       grep -Eh -o '\${SCRIPT_DIR}/.*' | awk '{print $1}' | \
       sed 's/"//g' | \
       sort | uniq); do
  f=$(eval echo $f)
  if ! test -f ${f}; then
    echo "${f} script not found in the repository"
    exit 1
  fi
done
