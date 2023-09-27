# #!/usr/bin/env bash
#
set -euo pipefail

curr_dir="$(cd "$(dirname "$BASH_SOURCE")"; pwd -P)"
source "${curr_dir}"/bach.sh

releasepy_run="./release.py --dry-run --no-sanity-check"

test-releasepy-no() {
  @command /usr/bin/python3 ${curr_dir}/${releasepy_run} gz-non-existing 0.0.0 foo
}

test-ASSERT-FAIL-releasepy-no() {
  @stderr
}
