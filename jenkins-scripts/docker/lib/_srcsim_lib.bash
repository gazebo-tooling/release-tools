# Whole SRCSIM setup
SRCSIM_SETUP_REPOSITORIES="""
apt-get install -y wget
wget http://srcsim.gazebosim.org/src/src.key -O - | sudo apt-key add -
echo \"deb http://srcsim.gazebosim.org/src ${DISTRO} main\" > /etc/apt/sources.list.d/src-latest.list
apt-get update
# workaround by the lack of state pub
apt-get install ros-trusty-joint-state-publisher

mkdir -p /etc/ros/rosdep/sources.list.d/
wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gazebo7/00-gazebo7.list -O /etc/ros/rosdep/sources.list.d/00-gazebo7.list
"""

SRCSIM_INIT_SETUP="""
echo '@ros - rtprio 99' > /etc/security/limits.d/ros-rtprio.conf
groupadd ros
usermod -a -G ros root
chown -R root:root /opt/ros/indigo/share/ihmc_ros_java_adapter

wget -P /tmp/ http://gazebosim.org/distributions/srcsim/valkyrie_controller.tar.gz
tar -xvf /tmp/valkyrie_controller.tar.gz -C ~/
rm /tmp/valkyrie_controller.tar.gz

wget -P /tmp/ https://bitbucket.org/osrf/gazebo_models/get/default.tar.gz
mkdir -p ~/.gazebo/models
tar -xvf /tmp/default.tar.gz -C ~/.gazebo/models --strip 1
rm /tmp/default.tar.gz
"""

SRCSIM_ENV_SETUP="""
update-alternatives --set java /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java
#update-alternatives --set javac /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/javac
rm /usr/lib/jvm/default-java
sudo ln -s /usr/lib/jvm/java-8-openjdk-amd64 /usr/lib/jvm/default-java
export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
export IS_GAZEBO=true
export ROS_IP=127.0.0.1

mkdir -p ~/.ihmc; curl https://raw.githubusercontent.com/ihmcrobotics/ihmc_ros_core/0.8.0/ihmc_ros_common/configurations/IHMCNetworkParametersTemplate.ini > ~/.ihmc/IHMCNetworkParameters.ini

# pre-built cache
source  /opt/nasa/indigo/setup.bash
roslaunch ihmc_valkyrie_ros valkyrie_warmup_gradle_cache.launch
"""
