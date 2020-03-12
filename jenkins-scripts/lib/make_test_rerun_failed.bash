#!/bin/bash -x
set -e

# Run `make test`
# If it has any failures and RERUN_FAILED_TESTS is nonzero,
# then rerun the failed tests one time and merge the junit results
# The MAKE_TEST_RERUN_ARGS variable and any arguments to this script
# hold extra make ARGS, such as -R UNIT_*
# Note that flaky_junit_merge.py requires lxml python package
if ! make test ARGS="-VV ${MAKE_TEST_RERUN_ARGS} $@" \
    && [[ "${RERUN_FAILED_TESTS}" -gt 0 ]]; then
  mv test_results test_results_merged
  mkdir test_results
  # we can't just run ctest --rerun-failed
  # because that might not run the check_test_ran for failed tests
  echo Failed tests:
  ctest -N --rerun-failed
  FAILED_TESTS=$(ctest -N --rerun-failed \
    | grep 'Test  *#[0-9][0-9]*:' \
    | sed -e 's@^ *Test  *#[0-9]*: *@@' \
  )
  for i in ${FAILED_TESTS}; do
    make test ARGS="-VV -R ${i}\$$" || true
  done
  mkdir test_results_tmp
  for i in $(ls test_results); do
    echo looking for flaky tests in test_results_merged/$i and test_results/$i
    python3 ${WORKSPACE}/scripts/jenkins-scripts/tools/flaky_junit_merge.py \
      test_results_merged/$i test_results/$i \
      > test_results_tmp/$i
    mv test_results_tmp/$i test_results_merged
  done
  rm -fr test_results test_results_tmp
  mv test_results_merged test_results
fi
