#!/usr/bin/env bash

# Be sure to run this script from outside a git repository. Otherwise, the 
# first run will fail with `fatal: remote origin already exists.` In case
# this happens, running the script one more time goes through.
# See https://github.com/cli/cli/issues/2166

set -e

SCRIPT_DIR="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

if [[ $# -lt 1 ]]; then
    echo "$0 <list_of_new_ignition_names_space_separated>"
    echo " example: $0 ign-cmake3 ign-common22"
    exit 1
fi

NEW_REPOS=${1}

repo_exists()
{
    local repo_github=${1}

    if gh repo view $repo_github 2>1 > /dev/null; then
        echo true
    else
        echo false
    fi
}

empty_directory()
{
    local dir=${1}

    if [[ -d "${dir}/ubuntu" ]]; then
       echo false
    else
       echo true
    fi
}

for repo_name in ${NEW_REPOS}; do
    echo "Procesing ${repo_name}"
    repo_fname="${repo_name}-release"
    repo_github="gazebo-release/${repo_fname}"

    echo " + creating the repository "
    # check destination directory
    if [[ -d ${repo_fname} ]]; then
       echo "  ? local dir ${repo_fname} exist in this directory. Is it emptY?"
       if ! $(empty_directory ${repo_fname}); then
           echo "   ! directory is not empty. Abort"
        continue
       fi
       echo "   + ${repo_fname} is empty, assume github repo"
    else
        # check repo in github
        if $(repo_exists ${repo_github}); then
            echo "  ? repository ${repo_github} already exists!. Is it empty?"
            gh repo clone ${repo_github} ${repo_fname} 2> /dev/null
            if ! $(empty_directory ${repo_fname}); then
                echo "   ! directory is not empty. Abort"
                continue
            fi
            echo "   + ${repo_fname} is empty"
        else
            gh repo create \
                --confirm \
                --public \
                --enable-issues \
                --description "Debian/Ubuntu metadata for ${repo_name}" \
                ${repo_github}
        fi
    fi
    # repository checkout in place
    cd ${repo_fname}
    version="$(sed  's/.*[^0-9]\([0-9]\+\)[^0-9]*$/\1/' <<< ${repo_name})"
    previous_version=$(expr ${version} - 1)
    previous_repo_github="${repo_github/[0-9]*}${previous_version}-release"
    echo " + pull from previous version ${previous_repo_github}"
    git remote add previous "https://github.com/${previous_repo_github}"
    git fetch previous
    git pull -q previous master >/dev/null 2>&1 || git pull -q previous main >/dev/null 2>&1
    git remote remove previous
    echo " + run bump_major_version script ${previous_version} -> ${version}"
    echo
    echo " -------------------------------"
    "${SCRIPT_DIR}/bump_major_version.bash" ${previous_version} ${version}
    echo " -------------------------------"
    echo
    echo " ? check output for possible FIXME messages"
    read -n 1 -s -r -p "  press any key to continue"
    git checkout -b main
    git status
    echo " ? ready to commit --all and push ?"
    read -n 1 -s -r -p "  press any key to continue"
    git commit -m "Change metadata from ${previous_version} version to ${version}" --all
    git push origin main
done
