#!/bin/bash -xe

test_dir=$(mktemp -d)
mkdir -p ${test_dir}/{focal,jammy,ubuntu}/debian
export _RELEASEPY_TEST_RELEASE_REPO=${test_dir}

exec_releasepy_test()
{
  test_params=${1}

    ./release.py \
      --dry-run \
      --no-sanity-checks \
    gz-foo 1.2.3 token ${test_params}""
}

expect_job_run()
{
  output="${1}" job="${2}"

  if ! grep -q "job/${job}/buildWith" <<< "${output}"; then
    echo "${job} not found in test output"
    exit 1
  fi
}

expect_job_not_run()
{
  output="${1}" job="${2}"

  if grep -q "job/${job}/buildWith" <<< "${output}"; then
    echo "${job} found in test output. Should not appear."
    exit 1
  fi
}

expect_number_of_jobs()
{
  output=${1} njobs=${2}

  if [[ $(grep  -c "job/.*/buildWithParameters" <<< "${output}") != "${njobs}" ]]; then
    echo "Number of jobs caled is not the expected ${njobs}"
    exit 1
  fi
}

expect_param()
{
  output="${1}" param="${2}"

  if ! grep -q "[&\?]${param}" <<< "${output}"; then
    echo "${param} not found in test output"
    exit 1
  fi

}

source_repo_uri_test=$(exec_releasepy_test "--source-repo-uri https://github.org/gazebosim/gz-foo")
expect_job_run "${source_repo_uri_test}" "gz-foo-source"
expect_job_not_run "${source_repo_uri_test}" "gz-foo-debbuilder"
expect_number_of_jobs "${source_repo_uri_test}" "1"

source_tarball_uri_test=$(exec_releasepy_test "--source-tarball-uri https://gazebosim/gz-foo-1.2.3.tar.gz")
expect_job_run "${source_tarball_uri_test}" "gz-foo-debbuilder"
expect_job_run "${source_tarball_uri_test}" "generic-release-homebrew_pull_request_updater"
expect_job_not_run "${source_tarball_uri_test}" "gz-foo-source"
expect_number_of_jobs "${source_tarball_uri_test}" "7"
expect_param "${source_tarball_uri_test}" "SOURCE_TARBALL_URI=https%3A%2F%2Fgazebosim%2Fgz-foo-1.2.3.tar.gz"

nightly_test=$(exec_releasepy_test "--nightly-src-branch my-nightly-branch3 --upload-to-repo nightly")
expect_job_run "${nightly_test}" "gz-foo-debbuilder"
expect_job_not_run "${nightly_test}" "generic-release-homebrew_pull_request_updater"
expect_job_not_run "${nightly_test}" "gz-foo-source"
expect_number_of_jobs "${nightly_test}" "2"
expect_param "${nightly_test}" "SOURCE_TARBALL_URI=my-nightly-branch3"

