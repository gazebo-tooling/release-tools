echo '# BEGIN SECTION: setup the testing enviroment'
export DOCKER_JOB_NAME="gzdev"
. "${SCRIPT_DIR}/lib/boilerplate_prepare.sh"
. "${SCRIPT_DIR}/lib/_common_scripts.bash"
echo '# END SECTION'

cat > build.sh << DELIM
$(generate_buildsh_header)

echo '# BEGIN SECTION: check the gzdev checkout'
# No gzdev tests run here: the job only checks that the image builds and
# that the checkout is in place.
cd ${WORKSPACE}/gzdev
echo '# END SECTION'
DELIM

export OSRF_REPOS_TO_USE="stable"

. "${SCRIPT_DIR}/lib/docker_generate_dockerfile.bash"
. "${SCRIPT_DIR}/lib/docker_run.bash"
