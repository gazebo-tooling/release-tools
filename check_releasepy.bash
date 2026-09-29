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
      --auth user:fake \
    gz-foo 1.2.3 ${test_params}
}

exec_ignition_releasepy_test()
{
  test_params=${1}

    ./release.py \
      --dry-run \
      --no-sanity-checks \
      --auth user:fake \
    ign-foo 1.2.3 ${test_params}
}

exec_ignition_gazebo_releasepy_test()
{
  test_params=${1}

    ./release.py \
      --dry-run \
      --no-sanity-checks \
      --auth user:fake \
    ign-gazebo 1.2.3 ${test_params}
}

exec_releasepy_with_real_gz()
{
  gz_pkg=${1} major_version=${2} extra_params=${3}
    ./release.py \
      --dry-run \
      --no-sanity-checks \
      --auth user:fake \
      --source-repo-uri http://github.com/gazebosim/gz-common \
      --source-repo-existing-ref http://github.com/gazebosim/gz-common/foo-tag \
    "${gz_pkg}" "${major_version}.x.y" ${extra_params}
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
expect_number_of_jobs "${source_tarball_uri_test}" "5"
expect_param "${source_tarball_uri_test}" "SOURCE_TARBALL_URI=https%3A%2F%2Fgazebosim%2Fgz-foo-1.2.3.tar.gz"
expect_no_vendor "${source_tarball_uri_test}"

source_tarball_uri_with_sha256_test=$(exec_releasepy_test "--source-tarball-uri https://gazebosim/gz-foo-1.2.3.tar.gz --source-tarball-sha256 abc123def456")
expect_job_run "${source_tarball_uri_with_sha256_test}" "gz-foo-debbuilder"
expect_job_run "${source_tarball_uri_with_sha256_test}" "generic-release-homebrew_pull_request_updater"
expect_job_not_run "${source_tarball_uri_with_sha256_test}" "gz-foo-source"
expect_number_of_jobs "${source_tarball_uri_with_sha256_test}" "5"
expect_param "${source_tarball_uri_with_sha256_test}" "SOURCE_TARBALL_URI=https%3A%2F%2Fgazebosim%2Fgz-foo-1.2.3.tar.gz"
expect_param "${source_tarball_uri_with_sha256_test}" "SOURCE_TARBALL_SHA256=abc123def456"
expect_no_vendor "${source_tarball_uri_with_sha256_test}"

nightly_test=$(exec_releasepy_test "--nightly-src-branch my-nightly-branch3 --upload-to-repo nightly")
expect_job_run "${nightly_test}" "gz-foo-debbuilder"
expect_job_not_run "${nightly_test}" "generic-release-homebrew_pull_request_updater"
expect_job_not_run "${nightly_test}" "gz-foo-source"
expect_number_of_jobs "${nightly_test}" "2"
expect_param "${nightly_test}" "SOURCE_TARBALL_URI=my-nightly-branch3"
expect_no_vendor "${nightly_test}"

# Rotary nightly test: package name includes "rotary" prefix
exec_rotary_releasepy_test()
{
  test_params=${1}
    ./release.py \
      --dry-run \
      --no-sanity-checks \
      --auth user:fake \
    gz-rotary-cmake 1.2.3 ${test_params}
}

rotary_nightly_test=$(exec_rotary_releasepy_test "--nightly-src-branch main --upload-to-repo nightly")
expect_job_run "${rotary_nightly_test}" "gz-rotary-cmake-debbuilder"
expect_job_not_run "${rotary_nightly_test}" "gz-cmake-debbuilder"
expect_job_not_run "${rotary_nightly_test}" "generic-release-homebrew_pull_request_updater"
expect_number_of_jobs "${rotary_nightly_test}" "2"
expect_param "${rotary_nightly_test}" "PACKAGE=gz-rotary-cmake"
expect_param "${rotary_nightly_test}" "SOURCE_TARBALL_URI=main"
expect_no_vendor "${rotary_nightly_test}"

bump_linux_test=$(exec_releasepy_test "--source-tarball-uri https://gazebosim/gz-foo-1.2.3.tar.gz --only-bump-revision-linux -r 2")
expect_job_run "${bump_linux_test}" "gz-foo-debbuilder"
expect_job_not_run "${bump_linux_test}" "generic-release-homebrew_pull_request_updater"
expect_job_not_run "${bump_linux_test}" "gz-foo-source"
expect_number_of_jobs "${bump_linux_test}" "4"
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
expect_number_of_jobs "${ignition_source_tarball_uri_test}" "5"
expect_param "${ignition_source_tarball_uri_test}" "SOURCE_TARBALL_URI=https%3A%2F%2Fgazebosim%2Fgz-foo-1.2.3.tar.gz"
expect_param "${ignition_source_tarball_uri_test}" "PACKAGE=ign-foo"
expect_param "${ignition_source_tarball_uri_test}" "PACKAGE_ALIAS=ignition-foo"

ign_gazebo_source_tarball_uri_test=$(exec_ignition_gazebo_releasepy_test "--source-tarball-uri https://gazebosim/ign-gazebo-1.2.3.tar.gz")
expect_job_run "${ign_gazebo_source_tarball_uri_test}" "gz-sim-debbuilder"
expect_job_run "${ign_gazebo_source_tarball_uri_test}" "generic-release-homebrew_pull_request_updater"
expect_job_not_run "${ign_gazebo_source_tarball_uri_test}" "gz-sim-source"
expect_number_of_jobs "${ign_gazebo_source_tarball_uri_test}" "5"
expect_param "${ign_gazebo_source_tarball_uri_test}" "SOURCE_TARBALL_URI=https%3A%2F%2Fgazebosim%2Fign-gazebo-1.2.3.tar.gz"
expect_param "${ign_gazebo_source_tarball_uri_test}" "PACKAGE=ign-gazebo"
expect_param "${ign_gazebo_source_tarball_uri_test}" "PACKAGE_ALIAS=ignition-gazebo"

ros_vendor_test=$(exec_releasepy_with_real_gz gz-fuel-tools 9)
expect_vendor_repo "${ros_vendor_test}" gazebo-release/gz_fuel_tools_vendor

ros_vendor_test=$(exec_releasepy_with_real_gz gz-fuel-tools 9 "--upload-to-repo prerelease")
expect_no_vendor "${ros_vendor_test}"

ros_vendor_test=$(exec_releasepy_with_real_gz gz-cmake 2)
expect_no_vendor "${ros_vendor_test}"

ros_vendor_test=$(exec_releasepy_with_real_gz gz-ionic 3)
expect_no_vendor "${ros_vendor_test}"

# Infer package and version from the source checkout
releasepy="${PWD}/release.py"
create_checkout()
{
  checkout_dir=$(mktemp -d -p "${test_dir}")
  git -C "${checkout_dir}" init -q -b "${1}"
  printf '%s\n' "${2}" > "${checkout_dir}/CMakeLists.txt"
  git -C "${checkout_dir}" add CMakeLists.txt
  git -C "${checkout_dir}" -c user.name=test -c user.email=test@test.foo \
    commit -q -m "Prepare release"
  echo "${checkout_dir}"
}

exec_inferred_releasepy_test()
{
  checkout_dir=${1} test_params=${2}
  (cd "${checkout_dir}" && "${releasepy}" \
      --no-sanity-checks \
      --auth user:fake \
      --source-repo-uri https://github.com/gazebosim/gz-math.git \
    ${test_params})
}

expect_output()
{
  output="${1}" text="${2}"

  if ! grep -qF -- "${text}" <<< "${output}"; then
    echo "'${text}' not found in test output"
    exit 1
  fi
}

stable_checkout=$(create_checkout gz-math8 'project(gz-math8 VERSION 8.4.0)')
inferred_test=$(exec_inferred_releasepy_test "${stable_checkout}" "--dry-run")
expect_job_run "${inferred_test}" "gz-math8-source"
expect_number_of_jobs "${inferred_test}" "1"
expect_param "${inferred_test}" "PACKAGE=gz-math8"
expect_param "${inferred_test}" "VERSION=8.4.0"
expect_param "${inferred_test}" "UPLOAD_TO_REPO=stable"
expect_output "${inferred_test}" "Release plan for gz-math8 8.4.0-1 (stable)  [branch gz-math8 @"
expect_output "${inferred_test}" "inferred    package, version, upload repo"
expect_output "${inferred_test}" "tag         gz-math8_8.4.0 (local HEAD)"

inferred_version_test=$(exec_inferred_releasepy_test "${stable_checkout}" "--dry-run gz-math8")
expect_param "${inferred_version_test}" "VERSION=8.4.0"
expect_output "${inferred_version_test}" "inferred    version, upload repo"

main_checkout=$(create_checkout main "project(gz-math VERSION 10.0.0)
gz_configure_project(VERSION_SUFFIX pre1)")
inferred_pre_test=$(exec_inferred_releasepy_test "${main_checkout}" "--dry-run")
expect_job_run "${inferred_pre_test}" "gz-math10-source"
expect_param "${inferred_pre_test}" "PACKAGE=gz-math10"
expect_param "${inferred_pre_test}" "VERSION=10.0.0~pre1"
expect_param "${inferred_pre_test}" "UPLOAD_TO_REPO=prerelease"
expect_output "${inferred_pre_test}" "tag         gz-math10_10.0.0-pre1 (local HEAD)"

if mismatch_test=$(exec_inferred_releasepy_test "${stable_checkout}" "--dry-run gz-math7"); then
  echo "package mismatch with CMakeLists.txt should fail"
  exit 1
fi
expect_output "${mismatch_test}" "CMakeLists.txt says package gz-math8, command line says gz-math7"

# Without --dry-run, an inferred release needs a confirmation: no terminal and
# no --yes must abort before tagging or calling any job
if no_confirm_test=$(exec_inferred_releasepy_test "${stable_checkout}" "" < /dev/null); then
  echo "inferred release without terminal or --yes should fail"
  exit 1
fi
expect_output "${no_confirm_test}" "no terminal to confirm"
expect_number_of_jobs "${no_confirm_test}" "0"
if git -C "${stable_checkout}" tag | grep -q .; then
  echo "inferred release without confirmation created a tag"
  exit 1
fi
