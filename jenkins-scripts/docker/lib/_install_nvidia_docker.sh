INSTALL_NVIDIA_DOCKER1="""
echo '# BEGIN SECTION: install docker (in docker)'
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
add-apt-repository \"deb [arch=amd64] https://download.docker.com/linux/ubuntu \$(lsb_release -cs) stable\"
apt-get update
apt-get install -y docker-ce
echo '# END SECTION'

echo '# BEGIN SECTION: install nvidia-docker1 (in docker)'
apt-get install -y wget nvidia-340 nvidia-modprobe
wget -P /tmp https://github.com/NVIDIA/nvidia-docker/releases/download/v1.0.1/nvidia-docker_1.0.1-1_amd64.deb
dpkg -i /tmp/nvidia-docker*.deb && rm /tmp/nvidia-docker*.deb
echo '# END SECTION'
"""

INSTALL_NVIDIA_DOCKER2="""
echo '# BEGIN SECTION: install docker (in docker)'
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
add-apt-repository \"deb [arch=amd64] https://download.docker.com/linux/ubuntu \$(lsb_release -cs) stable\"
apt-get update
# apt-get install -y docker-ce
echo '# END SECTION'

echo '# BEGIN SECTION: install nvidia-docker2 (in docker)'
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  apt-key add -
distribution=\$(. /etc/os-release;echo \$ID\$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/\$distribution/nvidia-docker.list | \
  tee /etc/apt/sources.list.d/nvidia-docker.list
apt-get update
apt-get install -y nvidia-docker2
# systemctl daemon-reload
# systemctl restart docker
echo '# END SECTION'
"""

DOCKER2_CMD="docker run --privileged -e DISPLAY=unix:0 -v /sys:/sys:ro -v /var/run/docker.sock:/var/run/docker.sock -v /tmp/.X11-unix:/tmp/.X11-unix:rw --runtime=nvidia -e DOCKER_FIX= -v /dev/log:/dev/log:ro -v /run/log:/run/log:ro -v /sys/fs/cgroup:/sys/fs/cgroup:ro --device /dev/snd --tty --rm"

DEPENDENCY_PKGS="${DEPENDENCY_PKGS} curl"
