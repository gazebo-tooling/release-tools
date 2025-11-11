#!/bin/bash 
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# JOB_DSL_VER="1.77"
pushd "${SCRIPT_DIR}" >/dev/null

echo " * Download job-dsl-core jar"
# Restore with fixed in master branch
# curl -sSL https://repo.jenkins-ci.org/public/org/jenkins-ci/plugins/job-dsl-core/${JOB_DSL_VER}/job-dsl-core-${JOB_DSL_VER}-standalone.jar -o jobdsl.jar
curl -sSL https://github.com/j-rivero/job-dsl-plugin/releases/download/job-dsl-1.85-pr-1577/job-dsl-core-1.85-pr-1577-jar-with-dependencies.jar \
  -o job-dsl-core-jar-with-dependencies.jar
echo " * Download snakeyaml jar"
curl -sSL https://repo1.maven.org/maven2/org/yaml/snakeyaml/2.0/snakeyaml-2.0.jar -o snakeyaml-2.0.jar
echo " * Inject snakeyaml in job-dsl-core jar and rebuild"

rm -fr dsl-core-jar && mkdir dsl-core-jar
pushd dsl-core-jar > /dev/null
jar -xf ../job-dsl-core-jar-with-dependencies.jar
popd > /dev/null

rm -fr snake-jar && mkdir snake-jar
pushd snake-jar > /dev/null
jar -xf ../snakeyaml-2.0.jar
popd > /dev/null

cp -a snake-jar/org dsl-core-jar/

pushd dsl-core-jar > /dev/null
jar -cfm ../jobdsl.jar META-INF/MANIFEST.MF *
popd > /dev/null

rm -fr snakeyaml-*.jar
rm -fr job-dsl-core-jar-with-dependencies*.jar

echo " >> Run 'java -jar ${PWD}/jobdsl.jar <file.dsl>' to generate locally the Jenkins XML config"
popd >/dev/null
