#!/bin/bash -x

# Knowing Script dir beware of symlink
[[ -L ${0} ]] && SCRIPT_DIR=$(readlink ${0}) || SCRIPT_DIR=${0}
SCRIPT_DIR="${SCRIPT_DIR%/*}"

# Need for priviledge mode (docker in docker)
export GPU_SUPPORT_NEEDED=true

if [[ -z ${ROS_DISTRO} ]]; then
    echo "Need to define a ROS_DISTRO value"
    exit
fi

# Avoid caching all intermediate containers
export DOCKER_DO_NOT_CACHE=true

echo '# BEGIN SECTION: clean docker cache to get disk space'
wget https://raw.githubusercontent.com/spotify/docker-gc/master/docker-gc
sudo bash -c "GRACE_PERIOD_SECONDS=86400 bash docker-gc"
df -h
echo '# END SECTION'

export INSTALL_JOB_PREINSTALL_HOOK="""
echo '# BEGIN SECTION: workaround on docker ip overlaping'
find ${WORKSPACE}/ariac-docker -name '*.bash' -exec sed -i -e 's:172\.18:172.19:g' {} \\;
grep 172.19 ${WORKSPACE}/ariac-docker/ariac-server/run_container.bash 
echo '# END SECTION'

echo '# BEGIN SECTION: install docker inside docker'

# Needed policy to run docker daemon
echo \"exit 0\" > /usr/sbin/policy-rc.d
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
add-apt-repository \"deb https://download.docker.com/linux/ubuntu ${DISTRO} stable\"
apt-get update
# Workaround for trusty
if [[ ${DISTRO} == 'trusty' ]]; then
 touch /etc/init/cgroup-lite.conf 
 apt-get install -y o Dpkg::Options::='--force-confdef' cgroup-lite 
 rm /etc/init/cgroup-lite.conf
fi
apt-get install -y docker-ce
docker network ls
echo '# END SECTION'

echo '# BEGIN SECTION: run the ariac_system script for ${ROS_DISTRO}'
cd ${WORKSPACE}/ariac-docker
bash -x ./prepare_ariac_system.bash ${ROS_DISTRO}
df -h
echo '# END SECTION'

echo '# BEGIN SECTION: run the team_system script for ${ROS_DISTRO} team'
cd ${WORKSPACE}/ariac-docker
echo ${ROS_DISTRO} > team_config/example_team/ros_distro.txt
bash -x ./prepare_team_system.bash example_team
df -h
echo '# END SECTION'

echo '# BEGIN SECTION: run all trials for example_team runtime tests'
cd ${WORKSPACE}/ariac-docker/ariac-competitor
bash -x ./ariac_network.bash
cd ${WORKSPACE}/ariac-docker
bash -x ./run_all_trials.bash example_team
df -h
echo '# END SECTION'
"""

# Need bc to proper testing and parsing the time
export DEPENDENCY_PKGS DEPENDENCY_PKGS="apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common"

. "${SCRIPT_DIR}/lib/generic-install-base.bash"
