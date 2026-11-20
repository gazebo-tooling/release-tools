#!/bin/bash
# macOS (arm64) build of a Gazebo library with the pixi_ci driver
# (jenkins-scripts/tools/pixi_ci/README.md lists the job variables)
#
# Parameters:
#   - arg1 : gz-collections.yaml library name, e.g. gz-math
#
# Actions
#   - Download pixi, check its SHA-256 and create the bootstrap environment,
#     unless REUSE_PIXI_INSTALLATION is set
#   - Run the driver with the bootstrap environment python; its exit code is
#     the job result
#
# Paths: pixi in $WORKSPACE/.pixi_ci/pixi (PIXI_TMP), the bootstrap
# environment in $WORKSPACE/.pixi_ci/bootstrap (PIXI_BOOTSTRAP_PROJECT_PATH)

set -euo pipefail
# bash 3.2, the /bin/bash of macOS, ignores set -e for a failing ( ... )
# subshell: check the status of each one explicitly

if [ $# -ne 1 ]; then
  echo "usage: $0 <gz-collections.yaml library name>" >&2
  exit 2
fi
if [ -z "${WORKSPACE:-}" ]; then
  echo "ERROR: WORKSPACE is not set"
  exit 1
fi

# the jenkins-scripts directory, whatever the caller set
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
CONFIG_DETECTOR="${SCRIPT_DIR}/../conda/config-detector"

PIXI_VERSION=0.44.0
PIXI_URL="https://github.com/prefix-dev/pixi/releases/download/v${PIXI_VERSION}/pixi-aarch64-apple-darwin.tar.gz"
# pixi-aarch64-apple-darwin.tar.gz.sha256 of the release
PIXI_SHA256=56faaad85973c5739f46192ad8b62b64d6af391d4e3ded452f43dd5b603ea888

# The driver reads PIXI_TMP; PIXI_PROJECT_PATH has its default there too
export PIXI_TMP="${PIXI_TMP:-${WORKSPACE}/.pixi_ci/pixi}"
PIXI_BOOTSTRAP_PROJECT_PATH="${PIXI_BOOTSTRAP_PROJECT_PATH:-${WORKSPACE}/.pixi_ci/bootstrap}"
BOOTSTRAP_PYTHON="${PIXI_BOOTSTRAP_PROJECT_PATH}/.pixi/envs/default/bin/python"

if [ -z "${REUSE_PIXI_INSTALLATION:-}" ]; then
  echo "# BEGIN SECTION: pixi: installation"
  echo "Downloading pixi ${PIXI_VERSION} to ${PIXI_TMP}"
  # A run killed halfway leaves *.download files behind, never a broken pixi
  archive="${PIXI_TMP}.tar.gz.download"
  binary="${PIXI_TMP}.download"
  mkdir -p "$(dirname "${PIXI_TMP}")"
  curl -fsSL --retry 5 --retry-all-errors -o "${archive}" "${PIXI_URL}"
  if ! (cd "$(dirname "${archive}")" &&
      echo "${PIXI_SHA256}  $(basename "${archive}")" | shasum -a 256 -c -); then
    echo "ERROR: the pixi ${PIXI_VERSION} download does not match PIXI_SHA256"
    exit 1
  fi
  tar -xzOf "${archive}" pixi > "${binary}"
  chmod +x "${binary}"
  mv -f "${binary}" "${PIXI_TMP}"
  rm -f "${archive}"
  "${PIXI_TMP}" --version
  echo "# END SECTION"

  echo "# BEGIN SECTION: pixi: create bootstrap environment"
  mkdir -p "${PIXI_BOOTSTRAP_PROJECT_PATH}"
  cp "${CONFIG_DETECTOR}/pixi.toml" "${CONFIG_DETECTOR}/pixi.lock" \
    "${PIXI_BOOTSTRAP_PROJECT_PATH}/"
  if ! (cd "${PIXI_BOOTSTRAP_PROJECT_PATH}" && "${PIXI_TMP}" install --locked); then
    echo "ERROR: pixi install --locked failed in ${PIXI_BOOTSTRAP_PROJECT_PATH}"
    exit 1
  fi
  echo "# END SECTION"
elif [ ! -x "${BOOTSTRAP_PYTHON}" ]; then
  echo "ERROR: REUSE_PIXI_INSTALLATION is set but ${PIXI_BOOTSTRAP_PROJECT_PATH} has no bootstrap environment: run once without it"
  exit 1
fi

# -I: the PYTHON* variables of the job do not reach the driver's interpreter
exec "${BOOTSTRAP_PYTHON}" -u -I "${SCRIPT_DIR}/tools/run_pixi_ci.py" "$1"
