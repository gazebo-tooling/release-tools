#
# Script to generate the dockerfile needed for running the build.sh script
#
# Inputs used:
# - DISTRO            : base distribution (ex: vivid)
# - LINUX_DISTRO      : [default ubuntu] base linux distribution (ex: debian)
# - ARCH              : [default amd64] base arquitecture (ex: amd64)
# - OSRF_REPOS_TO_USE : [default empty] space separated list of osrf repos to add to sourcess.list
# - USE_ROS_REPO      : [default false] true|false if add the packages.ros.org to the sources.list
# - DEPENDENCY_PKGS   : (optional) packages to be installed in the image
# - SOFTWARE_DIR      : (optional) directory to copy inside the image
# - DOCKER_DO_NOT_CACHE : (optional) do not cache docker intermediate images
# - DOCKER_PREINSTALL_HOOK : (optional) bash code to run before installing  DEPENDENCY_PKGS.
#                       It can be used for installing extra repositories needed for DEPENDENCY_PKGS
# - DOCKER_POSTINSTALL_HOOK : (optional) bash code to run after installing  DEPENDENCY_PKGS.
#                       It can be used for gem ruby installations or pip python

#   - USE_OSRF_REPO     : deprecated! [default false] true|false if true, add the stable osrf repo to sources.list

if [[ -z ${ARCH} ]]; then
  echo "Arch undefined, default to amd64"
  export ARCH="amd64"
fi

if [[ -z ${LINUX_DISTRO} ]]; then
  echo "Linux distro undefined, default to ubuntu"
  export LINUX_DISTRO="ubuntu"
fi

[[ -z ${USE_GCC8} ]] && USE_GCC8=false

GZDEV_DIR=${WORKSPACE}/gzdev
GZDEV_BRANCH=${GZDEV_BRANCH:-master}

dockerfile_install_gzdev_repos()
{
cat >> Dockerfile << DELIM_OSRF_REPO_GIT
RUN rm -fr ${GZDEV_DIR}
RUN git clone --depth 1 https://github.com/osrf/gzdev -b ${GZDEV_BRANCH} ${GZDEV_DIR}
DELIM_OSRF_REPO_GIT
if [[ -n ${GZDEV_PROJECT_NAME} ]]; then
# debian sid docker images does not return correct name so we need to use
# force-linux-distro
cat >> Dockerfile << DELIM_OSRF_REPO_GZDEV
RUN ${GZDEV_DIR}/gzdev.py repository enable --project=${GZDEV_PROJECT_NAME} --force-linux-distro=${DISTRO} || ( git -C ${GZDEV_DIR} pull origin ${GZDEV_BRANCH} && \
    ${GZDEV_DIR}/gzdev.py repository enable --project=${GZDEV_PROJECT_NAME} --force-linux-distro=${DISTRO} )
DELIM_OSRF_REPO_GZDEV
else
for repo in ${OSRF_REPOS_TO_USE}; do
cat >> Dockerfile << DELIM_OSRF_REPO
RUN ${GZDEV_DIR}/gzdev.py repository enable osrf ${repo} --force-linux-distro=${DISTRO}  || ( git -C ${GZDEV_DIR} pull origin ${GZDEV_BRANCH} && \
    ${GZDEV_DIR}/gzdev.py repository enable osrf ${repo} --force-linux-distro=${DISTRO} )
DELIM_OSRF_REPO
done
fi
}

case ${LINUX_DISTRO} in
  'ubuntu')
    SOURCE_LIST_URL="http://archive.ubuntu.com/ubuntu"
    # zesty does not ship locales by default
    export DEPENDENCY_PKGS="locales ${DEPENDENCY_PKGS}"
    ;;
  'debian')
    # debian does not ship locales by default
    export DEPENDENCY_PKGS="locales ${DEPENDENCY_PKGS}"
    ;;
  *)
    echo "Unknow linux distribution: ${LINUX_DISTRO}"
    exit 1
esac

# Select the docker container depenending on the ARCH
case ${ARCH} in
  'amd64')
     FROM_VALUE=${LINUX_DISTRO}:${DISTRO}
     ;;
  'i386')
     if [[ ${LINUX_DISTRO} == 'ubuntu' ]]; then
       FROM_VALUE=osrf/${LINUX_DISTRO}_${ARCH}:${DISTRO}
     else
       # debian i386
       FROM_VALUE=${ARCH}/${LINUX_DISTRO}:${DISTRO}
     fi
     ;;
   'armhf' | 'arm64' )
       FROM_VALUE=osrf/${LINUX_DISTRO}_${ARCH}:${DISTRO}
     ;;
  *)
     echo "Arch unknown"
     exit 1
esac

[[ -z ${USE_OSRF_REPO} ]] && USE_OSRF_REPO=false
[[ -z ${OSRF_REPOS_TO_USE} ]] && OSRF_REPOS_TO_USE=""
[[ -z ${USE_ROS_REPO} ]] && USE_ROS_REPO=false
# Default ros-testing to internal test it and get quick fixes
[[ -z ${ROS_REPO_NAME} ]] && ROS_REPO_NAME="ros-testing"

# depracted variable, do migration here
if [[ -z ${OSRF_REPOS_TO_USE} ]]; then
  if ${USE_OSRF_REPO}; then
     OSRF_REPOS_TO_USE="stable"
  fi
fi

echo '# BEGIN SECTION: create the Dockerfile'
cat > Dockerfile << DELIM_DOCKER
#######################################################
# Docker file to run build.sh

FROM ${FROM_VALUE}
MAINTAINER Jose Luis Rivero <jrivero@osrfoundation.org>

# setup environment
ENV LANG C
ENV LC_ALL C
ENV DEBIAN_FRONTEND noninteractive
ENV DEBFULLNAME "OSRF Jenkins"
ENV DEBEMAIL "build@osrfoundation.org"
DELIM_DOCKER

# Handle special INVALIDATE_DOCKER_CACHE keyword by set a random
if [[ -n ${INVALIDATE_DOCKER_CACHE} ]]; then
cat >> Dockerfile << DELIM_DOCKER_INVALIDATE
RUN echo 'BEGIN SECTION: invalidate full docker cache'
RUN echo "Detecting content in INVALIDATE_DOCKER_CACHE. Invalidating it"
RUN echo "Invalidate cache enabled. ${DOCKER_RND_ID}"
RUN echo 'END SECTION'
DELIM_DOCKER_INVALIDATE
fi

# The redirection fails too many times using us ftp
if [[ ${LINUX_DISTRO} == 'debian' ]]; then
cat >> Dockerfile << DELIM_DEBIAN_APT
  RUN sed -i -e 's:httpredir:ftp.us:g' /etc/apt/sources.list
  RUN echo "deb-src http://ftp.us.debian.org/debian ${DISTRO} main" \\
                                                         >> /etc/apt/sources.list
DELIM_DEBIAN_APT
fi

if [[ ${LINUX_DISTRO} == 'ubuntu' ]]; then
  if [[ ${ARCH} != 'armhf' && ${ARCH} != 'arm64' ]]; then
cat >> Dockerfile << DELIM_DOCKER_ARCH
  # Note that main,restricted and universe are not here, only multiverse
  # main, restricted and unvierse are already setup in the original image
  RUN echo "deb ${SOURCE_LIST_URL} ${DISTRO} multiverse" \\
                                                         >> /etc/apt/sources.list && \\
      echo "deb ${SOURCE_LIST_URL} ${DISTRO}-updates main restricted universe multiverse" \\
                                                         >> /etc/apt/sources.list && \\
      echo "deb ${SOURCE_LIST_URL} ${DISTRO}-security main restricted universe multiverse" && \\
                                                         >> /etc/apt/sources.list
DELIM_DOCKER_ARCH
  fi
fi

# i386 image only have main by default
if [[ ${LINUX_DISTRO} == 'ubuntu' && ${ARCH} == 'i386' ]]; then
cat >> Dockerfile << DELIM_DOCKER_I386_APT
RUN echo "deb ${SOURCE_LIST_URL} ${DISTRO} restricted universe" \\
                                                       >> /etc/apt/sources.list
DELIM_DOCKER_I386_APT
fi

# Workaround for: https://bugs.launchpad.net/ubuntu/+source/systemd/+bug/1325142
if [[ ${ARCH} == 'i386' ]]; then
cat >> Dockerfile << DELIM_DOCKER_PAM_BUG
RUN echo "Workaround on i386 to bug in libpam. Needs first apt-get update"
RUN dpkg-divert --rename --add /usr/sbin/invoke-rc.d \\
        && ln -s /bin/true /usr/sbin/invoke-rc.d \\
	&& apt-get update \\
        && apt-get install -y libpam-systemd \\
	&& rm /usr/sbin/invoke-rc.d \\
        && dpkg-divert --rename --remove /usr/sbin/invoke-rc.d
DELIM_DOCKER_PAM_BUG
fi

# dirmngr from Yaketty on needed by apt-key
# git and python-* for gzdev
if [[ $DISTRO != 'xenial' ]]; then
    # not in xenial, available from Bionic on and all debians
    extra_python_mod="python3-distro"
fi
cat >> Dockerfile << DELIM_DOCKER_DIRMNGR
RUN apt-get update && \\
    apt-get install -y dirmngr git python3 python3-docopt python3-yaml ${extra_python_mod}
DELIM_DOCKER_DIRMNGR

# Install necessary repositories using gzdev
dockerfile_install_gzdev_repos

if ${USE_ROS_REPO}; then
  if ${ROS2}; then
cat >> Dockerfile << DELIM_ROS_REPO
# Note that ROS uses ubuntu hardcoded in the paths of repositories
RUN apt-get update \\
    && apt-get install -y curl \\
    && rm -rf /var/lib/apt/lists/*
RUN echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main ${DISTRO} main" > \\
                                                 /etc/apt/sources.list.d/ros2-latest.list
RUN echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/testing ${DISTRO} main" > \\
                                                 /etc/apt/sources.list.d/ros2-testing.list
RUN curl http://repo.ros2.org/repos.key | apt-key add -
DELIM_ROS_REPO
  else
cat >> Dockerfile << DELIM_ROS_REPO
# Note that ROS uses ubuntu hardcoded in the paths of repositories
RUN echo "deb http://packages.ros.org/${ROS_REPO_NAME}/ubuntu ${DISTRO} main" > \\
                                                /etc/apt/sources.list.d/ros.list
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
DELIM_ROS_REPO
# Need ros stable for the cases of ros-testing
if [[ ${ROS_REPO_NAME} != "ros" ]]; then
cat >> Dockerfile << DELIM_ROS_REPO_STABLE
# Note that ROS uses ubuntu hardcoded in the paths of repositories
RUN echo "deb http://packages.ros.org/ros/ubuntu ${DISTRO} main" > \\
                                         /etc/apt/sources.list.d/ros-stable.list
DELIM_ROS_REPO_STABLE
fi
  fi
fi

if [[ $ARCH == 'arm64' ]]; then
cat >> Dockerfile << DELIM_SYSCAL_ARM64
# Workaround for problem with syscall 277 in man-db
ENV MAN_DISABLE_SECCOMP 1
RUN apt-get update \\
    && apt-get install -y man-db \\
    && rm -rf /var/lib/apt/lists/*
DELIM_SYSCAL_ARM64
fi

if [ `expr length "${DOCKER_PREINSTALL_HOOK}"` -gt 1 ]; then
cat >> Dockerfile << DELIM_WORKAROUND_PRE_HOOK
RUN ${DOCKER_PREINSTALL_HOOK}
DELIM_WORKAROUND_PRE_HOOK
fi

# Dart repositories
if ${DART_FROM_PKGS} || ${DART_COMPILE_FROM_SOURCE}; then
if [[ $DISTRO == 'xenial' ]]; then
cat >> Dockerfile << DELIM_DOCKER_DART_PKGS
# Install dart from pkgs
RUN apt-get update \\
 && apt-get install -y apt-utils software-properties-common \\
 && rm -rf /var/lib/apt/lists/*
RUN apt-add-repository -y ppa:dartsim
DELIM_DOCKER_DART_PKGS
fi
fi

# Workaround a problem in simbody on artful bad paths
if [[ $DISTRO == "artful" ]]; then
cat >> Dockerfile << DELIM_DOCKER_WORKAROUND_SIMBODY
RUN apt-get update \\
 && apt-get install -y apt-utils software-properties-common \\
 && rm -rf /var/lib/apt/lists/*
RUN add-apt-repository ppa:j-rivero/simbody-artful
DELIM_DOCKER_WORKAROUND_SIMBODY
fi

# Packages that will be installed and cached by docker. In a non-cache
# run below, the docker script will check for the latest updates
PACKAGES_CACHE_AND_CHECK_UPDATES="${BASE_DEPENDENCIES} ${DEPENDENCY_PKGS}"

if $USE_GPU_DOCKER; then
  PACKAGES_CACHE_AND_CHECK_UPDATES="${PACKAGES_CACHE_AND_CHECK_UPDATES} ${GRAPHIC_CARD_PKG}"
fi

cat >> Dockerfile << DELIM_DOCKER3
# Invalidate cache monthly
# This is the firt big installation of packages on top of the raw image.
# The expection of updates is low and anyway it is cathed by the next
# update command below
# The rm after the fail of apt-get update is a workaround to deal with the error:
# Could not open file *_Packages.diff_Index - open (2: No such file or directory)
RUN echo "${MONTH_YEAR_STR}" \
 && (apt-get update || (rm -rf /var/lib/apt/lists/* && apt-get update)) \
 && apt-get install -y ${PACKAGES_CACHE_AND_CHECK_UPDATES} \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# This is killing the cache so we get the most recent packages if there was any
# update.
RUN echo "Invalidating cache $(( ( RANDOM % 100000 )  + 1 ))"
DELIM_DOCKER3

# A new install of gzdev is needed to update to possible recent changes in
# configuratin and/or code and not being used since the docker cache did
# not get them.
dockerfile_install_gzdev_repos

cat >> Dockerfile << DELIM_DOCKER31
# Note that we don't remove the apt/lists file here since it will make
# to run apt-get update again
RUN (apt-get update || (rm -rf /var/lib/apt/lists/* && apt-get update)) \
 && apt-get dist-upgrade -y \
 && apt-get clean

# Map the workspace into the container
RUN mkdir -p ${WORKSPACE}
DELIM_DOCKER31

# Beware of moving this code since it needs to run update-alternative after
# installing the default compiler in PACKAGES_CACHE_AND_CHECK_UPDATES
if ${USE_GCC8}; then
cat >> Dockerfile << DELIM_GCC8
   RUN apt-get update \\
   && apt-get install -y g++-8 \\
   && rm -rf /var/lib/apt/lists/* \\
   && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8 --slave /usr/bin/gcov gcov /usr/bin/gcov-8
DELIM_GCC8
fi

if ${USE_SQUID}; then
  cat >> Dockerfile << DELIM_DOCKER_SQUID
# If host is running squid-deb-proxy on port 8000, populate /etc/apt/apt.conf.d/30proxy
# By default, squid-deb-proxy 403s unknown sources, so apt shouldn't proxy ppa.launchpad.net
RUN route -n | awk '/^0.0.0.0/ {print \$2}' > /tmp/host_ip.txt
RUN echo "HEAD /" | nc \$(cat /tmp/host_ip.txt) 8000 | grep squid-deb-proxy \
  && (echo "Acquire::http::Proxy \"http://\$(cat /tmp/host_ip.txt):8000\";" > /etc/apt/apt.conf.d/30proxy) \
  && (echo "Acquire::http::Proxy::ppa.launchpad.net DIRECT;" >> /etc/apt/apt.conf.d/30proxy) \
  || echo "No squid-deb-proxy detected on docker host"
DELIM_DOCKER_SQUID
fi

if [[ -n ${SOFTWARE_DIR} ]]; then
cat >> Dockerfile << DELIM_DOCKER4
COPY ${SOFTWARE_DIR} ${WORKSPACE}/${SOFTWARE_DIR}
DELIM_DOCKER4
fi

if $USE_GPU_DOCKER; then
 if [[ $GRAPHIC_CARD_NAME == "Nvidia" ]]; then
   if $NVIDIA_DOCKER2_NODE; then
   # NVIDIA is using nvidia_docker2 integration
   cat >> Dockerfile << DELIM_NVIDIA2_GPU
# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
DELIM_NVIDIA2_GPU
   else
   # NVIDIA-DOCKER1
   cat >> Dockerfile << DELIM_NVIDIA_GPU
# nvidia-container-runtime
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:\${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:\${LD_LIBRARY_PATH}
DELIM_NVIDIA_GPU
   fi
 else
  # No NVIDIA cards needs to have the same X stack than the host
  cat >> Dockerfile << DELIM_DISPLAY
# Check to be sure version of kernel graphic card support is the same.
# It will kill DRI otherwise
RUN CHROOT_GRAPHIC_CARD_PKG_VERSION=\$(dpkg -l | grep "^ii.*${GRAPHIC_CARD_PKG}\ " | awk '{ print \$3 }' | sed 's:-.*::') \\
    if [ "\${CHROOT_GRAPHIC_CARD_PKG_VERSION}" != "${GRAPHIC_CARD_PKG_VERSION}" ]; then \\
       echo "Package ${GRAPHIC_CARD_PKG} has different version in chroot and host system" \\
       echo "Maybe you need to update your host" \\
       exit 1 \\
   fi
DELIM_DISPLAY
  fi
fi

if [ `expr length "${DOCKER_POSTINSTALL_HOOK}"` -gt 1 ]; then
cat >> Dockerfile << DELIM_WORKAROUND_POST_HOOK
RUN ${DOCKER_POSTINSTALL_HOOK}
DELIM_WORKAROUND_POST_HOOK
fi

cat >> Dockerfile << DELIM_WORKAROUND_91
# Workaround to issue:
# https://bitbucket.org/osrf/handsim/issue/91
RUN echo "en_GB.utf8 UTF-8" >> /etc/locale.gen
RUN locale-gen en_GB.utf8
ENV LC_ALL en_GB.utf8
ENV LANG en_GB.utf8
ENV LANGUAGE en_GB
# Docker has problems with Qt X11 MIT-SHM extension
ENV QT_X11_NO_MITSHM 1
DELIM_WORKAROUND_91

if $ENABLE_CCACHE; then
cat >> Dockerfile << DELIM_CCACHE
ENV PATH /usr/lib/ccache:\$PATH
ENV CCACHE_DIR ${CCACHE_DIR}
ENV CCACHE_MAXSIZE ${CCACHE_MAXSIZE}
DELIM_CCACHE

# Add the statistics about ccache at the beggining of the build
# first 3 lines: bash, set and space
sed -i '4iecho "# BEGIN SECTION: starting ccache statistics"' build.sh
sed -i "5iccache -M ${CCACHE_MAXSIZE}" build.sh
sed -i '6iccache -s' build.sh
sed -i '7iecho # "END SECTION"' build.sh
sed -i '8iecho ""' build.sh

# Add the statistics about ccache at the end
cat >> build.sh << BUILDSH_CCACHE
echo '# BEGIN SECTION: starting ccache statistics'
ccache -s
echo '# END SECTION'
BUILDSH_CCACHE
fi

echo '# BEGIN SECTION: see build.sh script'
cat build.sh
echo '# END SECTION'

cat >> Dockerfile << DELIM_DOCKER4
COPY build.sh build.sh
RUN chmod +x build.sh
DELIM_DOCKER4
echo '# END SECTION'

echo '# BEGIN SECTION: see Dockerfile'
cat Dockerfile
echo '# END SECTION'
