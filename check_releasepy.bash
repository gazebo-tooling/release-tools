#!/bin/bash -e

export _RELEASEPY_DEBUG=1
test_dir=$(mktemp -d)
export _RELEASEPY_TEST_RELEASE_REPO="${test_dir}/test-release"
mkdir -p ${_RELEASEPY_TEST_RELEASE_REPO}/{focal,jammy,ubuntu}/debian
export _RELEASEPY_TEST_SOURCE_REPO="${test_dir}/src"
mkdir -p ${_RELEASEPY_TEST_SOURCE_REPO}
# Fake packages.xml to make the vendor package script happy
cat > "${_RELEASEPY_TEST_SOURCE_REPO}/package.xml" <<-EOF
<?xml version="1.0"?>
<package format="2">
  <name>gz-foo</name>
  <version>0.0.0</version>
  <description>test</description>
  <maintainer email="test@test.foo">Testing maintainer</maintainer>
  <license>Foo License</license>
</package>
EOF

exec_releasepy_test()
{
  test_params=${1}

    ./release.py \
      --dry-run \
      --no-sanity-checks \
    gz-foo 1.2.3 token ${test_params}
}

exec_ignition_releasepy_test()
{
  test_params=${1}

    ./release.py \
      --dry-run \
      --no-sanity-checks \
    ign-foo 1.2.3 token ${test_params}
}

exec_ignition_gazebo_releasepy_test()
{
  test_params=${1}

    ./release.py \
      --dry-run \
      --no-sanity-checks \
    ign-gazebo 1.2.3 token ${test_params}
}

exec_releasepy_with_real_gz()
{
  gz_pkg=${1} major_version=${2}
    ./release.py \
      --dry-run \
      --no-sanity-checks \
      --source-repo-uri http://github.com/gazebosim/gz-common \
      --source-repo-existing-ref http://github.com/gazebosim/gz-common/foo-tag \
    "${gz_pkg}" "${major_version}.x.y" token
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
    echo "Number of jobs called is not the expected ${njobs}"
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

expect_vendor_repo()
{
  output="${1}" repo="${2}"

  if ! grep -q "Github ${repo}" <<< "${output}"; then
    echo "${repo} not found in test output"
    exit 1
  fi
}

expect_no_vendor()
{
  output="${1}"

  if grep -q 'in ROS 2' <<< "${output}"; then
    echo "ROS 2 string found in output"
    exit 1
  fi
}

source_repo_uri_test=$(exec_releasepy_test "--source-repo-uri https://github.com/gazebosim/gz-foo.git")
expect_job_run "${source_repo_uri_test}" "gz-foo-source"
expect_job_not_run "${source_repo_uri_test}" "gz-foo-debbuilder"
expect_number_of_jobs "${source_repo_uri_test}" "1"
expect_param "${source_repo_uri_test}" "SOURCE_REPO_URI=https%3A%2F%2Fgithub.com%2Fgazebosim%2Fgz-foo.git"
expect_no_vendor "${source_repo_uri_test}"  # non existing package

source_tarball_uri_test=$(exec_releasepy_test "--source-tarball-uri https://gazebosim/gz-foo-1.2.3.tar.gz")
expect_job_run "${source_tarball_uri_test}" "gz-foo-debbuilder"
expect_job_run "${source_tarball_uri_test}" "generic-release-homebrew_pull_request_updater"
expect_job_not_run "${source_tarball_uri_test}" "gz-foo-source"
expect_number_of_jobs "${source_tarball_uri_test}" "7"
expect_param "${source_tarball_uri_test}" "SOURCE_TARBALL_URI=https%3A%2F%2Fgazebosim%2Fgz-foo-1.2.3.tar.gz"
expect_no_vendor "${source_tarball_uri_test}"

nightly_test=$(exec_releasepy_test "--nightly-src-branch my-nightly-branch3 --upload-to-repo nightly")
expect_job_run "${nightly_test}" "gz-foo-debbuilder"
expect_job_not_run "${nightly_test}" "generic-release-homebrew_pull_request_updater"
expect_job_not_run "${nightly_test}" "gz-foo-source"
expect_number_of_jobs "${nightly_test}" "2"
expect_param "${nightly_test}" "SOURCE_TARBALL_URI=my-nightly-branch3"
expect_no_vendor "${nightly_test}"

bump_linux_test=$(exec_releasepy_test "--source-tarball-uri https://gazebosim/gz-foo-1.2.3.tar.gz --only-bump-revision-linux -r 2")
expect_job_run "${bump_linux_test}" "gz-foo-debbuilder"
expect_job_not_run "${bump_linux_test}" "generic-release-homebrew_pull_request_updater"
expect_job_not_run "${bump_linux_test}" "gz-foo-source"
expect_number_of_jobs "${bump_linux_test}" "6"
expect_param "${bump_linux_test}" "RELEASE_VERSION=2"
expect_no_vendor "${bump_linux_test}"

ignition_test=$(exec_ignition_releasepy_test "--source-repo-uri https://github.com/gazebosim/gz-foo.git")
expect_job_run "${ignition_test}" "gz-foo-source"
expect_job_not_run "${ignition_test}" "ignition-foo-source"
expect_number_of_jobs "${ignition_test}" "1"
expect_param "${ignition_test}" "PACKAGE=ign-foo"
expect_param "${ignition_test}" "PACKAGE_ALIAS=ignition-foo"
expect_param "${ignition_test}" "SOURCE_REPO_URI=https%3A%2F%2Fgithub.com%2Fgazebosim%2Fgz-foo.git"

ignition_source_tarball_uri_test=$(exec_ignition_releasepy_test "--source-tarball-uri https://gazebosim/gz-foo-1.2.3.tar.gz")
expect_job_run "${ignition_source_tarball_uri_test}" "gz-foo-debbuilder"
expect_job_run "${ignition_source_tarball_uri_test}" "generic-release-homebrew_pull_request_updater"
expect_job_not_run "${ignition_source_tarball_uri_test}" "gz-foo-source"
expect_number_of_jobs "${ignition_source_tarball_uri_test}" "7"
expect_param "${ignition_source_tarball_uri_test}" "SOURCE_TARBALL_URI=https%3A%2F%2Fgazebosim%2Fgz-foo-1.2.3.tar.gz"
expect_param "${ignition_source_tarball_uri_test}" "PACKAGE=ign-foo"
expect_param "${ignition_source_tarball_uri_test}" "PACKAGE_ALIAS=ignition-foo"

ign_gazebo_source_tarball_uri_test=$(exec_ignition_gazebo_releasepy_test "--source-tarball-uri https://gazebosim/ign-gazebo-1.2.3.tar.gz")
expect_job_run "${ign_gazebo_source_tarball_uri_test}" "gz-sim-debbuilder"
expect_job_run "${ign_gazebo_source_tarball_uri_test}" "generic-release-homebrew_pull_request_updater"
expect_job_not_run "${ign_gazebo_source_tarball_uri_test}" "gz-sim-source"
expect_number_of_jobs "${ign_gazebo_source_tarball_uri_test}" "7"
expect_param "${ign_gazebo_source_tarball_uri_test}" "SOURCE_TARBALL_URI=https%3A%2F%2Fgazebosim%2Fign-gazebo-1.2.3.tar.gz"
expect_param "${ign_gazebo_source_tarball_uri_test}" "PACKAGE=ign-gazebo"
expect_param "${ign_gazebo_source_tarball_uri_test}" "PACKAGE_ALIAS=ignition-gazebo"

ros_vendor_test=$(exec_releasepy_with_real_gz gz-fuel-tools 9)
expect_vendor_repo "${ros_vendor_test}" gazebo-release/gz_fuel_tools_vendor

ros_vendor_test=$(exec_releasepy_with_real_gz gz-cmake 2)
expect_no_vendor "${ros_vendor_test}"

ros_vendor_test=$(exec_releasepy_with_real_gz gz-ionic 3)
expect_no_vendor "${ros_vendor_test}"
