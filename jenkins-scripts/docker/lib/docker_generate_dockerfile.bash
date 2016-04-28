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

case ${LINUX_DISTRO} in
  'ubuntu')
    SOURCE_LIST_URL="http://archive.ubuntu.com/ubuntu"
    ;;
    
  'debian')
    # Currently not needed
    # SOURCE_LIST_URL="http://ftp.us.debian.org/debian"

    # debian does not ship locales by default
    export DEPENDENCY_PKGS="locales ${DEPENDENCY_PKGS}"

    if [[ -n ${OSRF_REPOS_TO_USE} ]]; then
      echo "WARN!! OSRF has no debian repositories yet!"
      OSRF_REPOS_TO_USE=""
    fi
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
  'i386' | 'armhf' | 'arm64' )
     FROM_VALUE=osrf/${LINUX_DISTRO}_${ARCH}:${DISTRO}
     ;;
  *)
     echo "Arch unknown"
     exit 1
esac

[[ -z ${USE_OSRF_REPO} ]] && USE_OSRF_REPO=false
[[ -z ${OSRF_REPOS_TO_USE} ]] && OSRF_REPOS_TO_USE=""
[[ -z ${USE_ROS_REPO} ]] && USE_ROS_REPO=false

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

# The redirection fails too many times using us ftp
if [[ ${LINUX_DISTRO} == 'debian' ]]; then
cat >> Dockerfile << DELIM_DEBIAN_APT
  RUN sed -i -e 's:httpredir:ftp.us:g' /etc/apt/sources.list
DELIM_DEBIAN_APT
fi

if [[ ${LINUX_DISTRO} == 'ubuntu' ]]; then
  if [[ ${ARCH} != 'armhf' ]]; then
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
if [[ ${ARCH} == 'i386' ]]; then
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

for repo in ${OSRF_REPOS_TO_USE}; do
cat >> Dockerfile << DELIM_OSRF_REPO
RUN echo "deb http://packages.osrfoundation.org/gazebo/${LINUX_DISTRO}-${repo} ${DISTRO} main" >\\
                                                /etc/apt/sources.list.d/osrf.${repo}.list
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
DELIM_OSRF_REPO
done

if ${USE_ROS_REPO}; then
cat >> Dockerfile << DELIM_ROS_REPO
RUN echo "deb http://packages.ros.org/ros/ubuntu ${DISTRO} main" > \\
                                                /etc/apt/sources.list.d/ros.list
RUN apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
DELIM_ROS_REPO
fi

# Dart repositories
if ${DART_FROM_PKGS} || ${DART_COMPILE_FROM_SOURCE}; then
cat >> Dockerfile << DELIM_DOCKER_DART_PKGS
# Install dart from pkgs
RUN apt-get install -y apt-utils software-properties-common
RUN apt-add-repository -y ppa:libccd-debs
RUN apt-add-repository -y ppa:fcl-debs
RUN apt-add-repository -y ppa:dartsim
DELIM_DOCKER_DART_PKGS
fi

# Handle special INVALIDATE_DOCKER_CACHE keyword by set a random
if [[ -n ${INVALIDATE_DOCKER_CACHE} ]]; then
cat >> Dockerfile << DELIM_DOCKER_INVALIDATE
RUN echo 'BEGIN SECTION: invalidate full docker cache'
RUN echo "Detecting content in INVALIDATE_DOCKER_CACHE. Invalidating it"
RUN echo "Invalidate cache enabled. ${DOCKER_RND_ID}"
RUN echo 'END SECTION'
DELIM_DOCKER_INVALIDATE
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
RUN echo "${MONTH_YEAR_STR}"
# The rm command will minimize the layer size
RUN apt-get update && \
    apt-get install -y ${PACKAGES_CACHE_AND_CHECK_UPDATES} && \
    rm -rf /var/lib/apt/lists/*

# This is killing the cache so we get the most recent packages if there
# was any update
RUN echo "Invalidating cache $(( ( RANDOM % 100000 )  + 1 ))"
RUN apt-get update && \
    apt-get install -y ${PACKAGES_CACHE_AND_CHECK_UPDATES}
# Map the workspace into the container
RUN mkdir -p ${WORKSPACE}
DELIM_DOCKER3

cat >> Dockerfile << DELIM_DOCKER_SQUID
# If host is running squid-deb-proxy on port 8000, populate /etc/apt/apt.conf.d/30proxy
# By default, squid-deb-proxy 403s unknown sources, so apt shouldn't proxy ppa.launchpad.net
RUN route -n | awk '/^0.0.0.0/ {print \$2}' > /tmp/host_ip.txt
RUN echo "HEAD /" | nc \$(cat /tmp/host_ip.txt) 8000 | grep squid-deb-proxy \
  && (echo "Acquire::http::Proxy \"http://\$(cat /tmp/host_ip.txt):8000\";" > /etc/apt/apt.conf.d/30proxy) \
  && (echo "Acquire::http::Proxy::ppa.launchpad.net DIRECT;" >> /etc/apt/apt.conf.d/30proxy) \
  || echo "No squid-deb-proxy detected on docker host"
DELIM_DOCKER_SQUID

if [[ -n ${SOFTWARE_DIR} ]]; then
cat >> Dockerfile << DELIM_DOCKER4
COPY ${SOFTWARE_DIR} ${WORKSPACE}/${SOFTWARE_DIR}
DELIM_DOCKER4
fi

if $USE_GPU_DOCKER; then
cat >> Dockerfile << DELIM_DISPLAY
ENV DISPLAY ${DISPLAY}

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

if [ `expr length "${DOCKER_POSTINSTALL_HOOK}"` -gt 1 ]; then
cat >> Dockerfile << DELIM_WORKAROUND_POST_HOOK
RUN ${DOCKER_POSTINSTALL_HOOK}
DELIM_WORKAROUND_POST_HOOK
fi

cat >> Dockerfile << DELIM_WORKAROUND_91
# Workaround to issue:
# https://bitbucket.org/osrf/handsim/issue/91
RUN locale-gen en_GB.utf8
ENV LC_ALL en_GB.utf8
ENV LANG en_GB.utf8
ENV LANGUAGE en_GB
# Docker has problems with Qt X11 MIT-SHM extension
ENV QT_X11_NO_MITSHM 1
DELIM_WORKAROUND_91

cat >> Dockerfile << DELIM_DOCKER4
COPY build.sh build.sh
RUN chmod +x build.sh
DELIM_DOCKER4
echo '# END SECTION'

echo '# BEGIN SECTION: see Dockerfile'
cat Dockerfile
echo '# END SECTION'
