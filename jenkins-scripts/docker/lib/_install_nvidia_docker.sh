INSTALL_NVIDIA_DOCKER2="""
echo '# BEGIN SECTION: install docker (in docker)'
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository -y \"deb [arch=amd64] https://download.docker.com/linux/ubuntu \$(lsb_release -cs) stable\"
sudo apt-get update
# sudo apt-get install -y docker-ce
echo '# END SECTION'

echo '# BEGIN SECTION: install nvidia-docker2 (in docker)'
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -
distribution=\$(. /etc/os-release;echo \$ID\$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/\$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo docker info
echo '# END SECTION'
"""

DEPENDENCY_PKGS="${DEPENDENCY_PKGS} curl software-properties-common"
