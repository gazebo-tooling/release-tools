#!/bin/bash 
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

JOB_DSL_VER="1.92"

echo " * Download job-dsl-core jar"
curl -sSL https://repo.jenkins-ci.org/public/org/jenkins-ci/plugins/job-dsl-core/${JOB_DSL_VER}/job-dsl-core-${JOB_DSL_VER}-jar-with-dependencies.jar -o jobdsl.jar
echo " >> Run 'java -jar ${PWD}/jobdsl.jar <file.dsl>' to generate locally the Jenkins XML config"
popd >/dev/null
