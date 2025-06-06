#!/bin/bash 
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

SNAKEYAML_PATH="lib/snakeyaml.jar"
# JOB_DSL_VER="1.77"

pushd "${SCRIPT_DIR}" >/dev/null
echo " * Download job-dsl-core jar"
# curl -sSL https://repo.jenkins-ci.org/public/org/jenkins-ci/plugins/job-dsl-core/${JOB_DSL_VER}/job-dsl-core-${JOB_DSL_VER}-standalone.jar -o jobdsl.jar
curl -sSL https://github.com/j-rivero/job-dsl-plugin/releases/download/job-dsl-1.85-pr-1577/job-dsl-core-1.85-pr-1577-jar-with-dependencies.jar -o jobdsl.jar
echo " * Download snakeyaml jar"
# need to inject the yaml module into the jar file. lib subdir is needed to ineject into
# tar in the right place.
mkdir -p lib
curl -sSL https://repo1.maven.org/maven2/org/yaml/snakeyaml/1.32/snakeyaml-1.32.jar -o ${SNAKEYAML_PATH}
echo " * Inject snakeyaml in job-dsl-core jar"
jar uf jobdsl.jar ${SNAKEYAML_PATH}
rm -fr ${SNAKEYAML_PATH}
rmdir lib
echo " >> Run 'java -jar ${PWD}/jobdsl.jar <file.dsl>' to generate locally the Jenkins XML config"
popd >/dev/null
