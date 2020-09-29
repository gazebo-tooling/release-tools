#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L "${0}" ]] && SCRIPT_DIR=$(readlink "${0}") || SCRIPT_DIR="${0}"
SCRIPT_DIR="${SCRIPT_DIR%/*}"

PYTHON_VENV="${WORKSPACE}/venv"
REPREPRO_GIT_REPO="https://github.com/j-rivero/reprepro-updater"
REPREPRO_GIT_BRANCH="${REPREPRO_GIT_BRANCH:-refactor}"
REPREPRO_REPO_PATH="$WORKSPACE/repo"

REPREPRO_PARAMS=''
if [ -z "${COMMIT+x}" ]; then
    REPREPRO_PARAMS='--commit'
fi

if [[ ! -f "${HOME}/.buildfarm/reprepro-updater.ini" ]]; then
    echo "The reprepro configuration file was not found"
    exit -1
fi

echo '#BEGIN: prepare and join python-venv'
if [[ ! -d ${PYTHON_VENV} ]]; then
  python3 -m venv "${PYTHON_VENV}"
  pip3 install configparser
fi
source "${PYTHON_VENV}/bin/activate"
echo '# END SECTION'

echo '# BEGIN SECTION: clone the git repo'
rm -fr ${REPREPRO_REPO_PATH}
git clone "${REPREPRO_GIT_REPO}" -b "${REPREPRO_GIT_BRANCH}" "${REPREPRO_REPO_PATH}"
echo '# END SECTION'

export PYTHONPATH="${REPREPRO_REPO_PATH}/src"
export GNUPGHOME=/var/lib/jenkins/.gnupg 

echo '# BEGIN SECTION: run reprepro'
cd "${REPREPRO_REPO_PATH}/scripts"
python import_upstream.py ${REPREPRO_PARAMS} \
  "${UPLOAD_TO_REPO}" \
  "${REPREPRO_IMPORT_YAML_FILE}"
echo '# END SECTION'

echo '#BEGIN: exit the venv'
deactivate
echo '# END SECTION'
