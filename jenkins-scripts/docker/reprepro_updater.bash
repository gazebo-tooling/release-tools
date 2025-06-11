#!/bin/bash -x

set -e

# Knowing Script dir beware of symlink
[[ -L "${0}" ]] && SCRIPT_DIR=$(readlink "${0}") || SCRIPT_DIR="${0}"
SCRIPT_DIR="${SCRIPT_DIR%/*}"

PYTHON_VENV="${WORKSPACE}/venv"
REPREPRO_GIT_REPO="https://github.com/ros-infrastructure/reprepro-updater"
REPREPRO_GIT_BRANCH="${REPREPRO_GIT_BRANCH:-refactor_osrfbuild}"
REPREPRO_REPO_PATH="$WORKSPACE/repo"

REPREPRO_PARAMS=''
if ${COMMIT}; then
    REPREPRO_PARAMS='--commit'
fi

if [[ ! -f "${HOME}/.buildfarm/reprepro-updater.ini" ]]; then
    echo "The reprepro configuration file was not found"
    exit -1
fi

echo '#BEGIN: prepare and join python-venv'
if [[ ! -d ${PYTHON_VENV} ]]; then
  python3 -m venv "${PYTHON_VENV}"
  source "${PYTHON_VENV}/bin/activate"
  pip3 install configparser
  # for some reason the build is broken on Xenial but seems to install
  # the necessary support
  pip3 install pyyaml || true
else
  source "${PYTHON_VENV}/bin/activate"
fi
echo '# END SECTION'

echo '# BEGIN SECTION: clone the git repo'
rm -fr ${REPREPRO_REPO_PATH}
git clone "${REPREPRO_GIT_REPO}" -b "${REPREPRO_GIT_BRANCH}" "${REPREPRO_REPO_PATH}"
echo '# END SECTION'

export PYTHONPATH="${REPREPRO_REPO_PATH}/src"

echo '# BEGIN SECTION: run reprepro'
cd "${REPREPRO_REPO_PATH}/scripts"
PYTHONPATH=${REPREPRO_REPO_PATH}/src python3 import_upstream.py ${REPREPRO_PARAMS} \
  "${UPLOAD_TO_REPO:-:_}" \
  "${REPREPRO_REPO_PATH}/config/citest.packages.osrfoundation.org/${REPREPRO_IMPORT_YAML_FILE}"
echo '# END SECTION'

echo '#BEGIN: exit the venv'
deactivate
echo '# END SECTION'
