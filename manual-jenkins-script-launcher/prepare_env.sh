#!/bin/bash

die()
{
   local msg=${1}

   echo " ! ${msg} " && exit 1
}

set_up_release_tools()
{
    [[ -z ${WORKSPACE} ]] && die "No WORKSPACE variable defined"

    if [[ -z ${RELEASE_TOOLS_DIR} ]]; then
       # Download release-tools from bitbucket
       echo " + Downloading release-repo tools"
       hg clone http://bitbucket.org/osrf/release-tools ${WORKSPACE}/scripts 
    else
       # Use directory as release-repo tools
       echo " + Copy directory for use as release-repo tools"
       [[ ! -d ${RELEASE_TOOLS_DIR} ]] && echo "Fail to detect directory for repo-tools" && exit 1
       cp -a ${RELEASE_TOOLS_DIR} ${WORKSPACE}/scripts
    fi
}

set_up_software_code()
{
    # Sanity checks
    [[ -z ${PACKAGE} ]] && die "No PACKAGE variable defined"
    [[ -z ${WORKSPACE} ]] && die "No WORKSPACE variable defined"
    
    if [[ -z ${SOFTWARE_CODE_DIR} ]]; then
       # Download software from bitbucket
       echo " + Download code from bitbucket"
       hg clone http://bitbucket.org/osrf/${PACKAGE} ${WORKSPACE}/${PACKAGE}
    else
       # Use directory as release-repo tools
       echo " + Using a system directory as package: ${SOFTWARE_CODE_DIR}"
       [[ ! -d ${SOFTWARE_CODE_DIR} ]] && echo "Fail to detect directory" && exit 1
       cp -a ${SOFTWARE_CODE_DIR} ${WORKSPACE}/${PACKAGE}
    fi
}

set_up_workspace()
{
    local clean_workspace=${1-true} 

    [[ -z ${WORKSPACE} ]] && die "No WORKSPACE variable defined"

    # Prepare workspace
    if $clean_workspace; then
      [[ -d ${WORKSPACE} ]] && sudo chown -R ${USER}:${USER} ${WORKSPACE}
      rm -fr ${WORKSPACE} && mkdir -p ${WORKSPACE}
    fi

    # script_dir variable
    export SCRIPT_DIR=${WORKSPACE}/scripts/jenkins-scripts
  
    # Home related variables
    FAKE_HOME="${WORKSPACE}/home"
    mkdir -p ${FAKE_HOME}
    touch "${FAKE_HOME}/.gnupg"
    touch "${FAKE_HOME}/bullseye-jenkins-license"
    mkdir -p "${FAKE_HOME}/.ssh/" && touch "${FAKE_HOME}/.ssh/id_rsa"
}
