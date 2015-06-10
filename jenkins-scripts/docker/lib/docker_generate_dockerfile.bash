# 
# Script to generate the dockerfile needed for running the build.sh script
#
# Inputs used:
#   - DISTRO          : base distribution (ex: vivid)
#   - ARCH            : [default amd64] base arquitecture (ex: amd64)
#   - USE_OSRF_REPO   : [default false] true|false if add the packages.osrfoundation.org to the sources.list
#   - DEPENDENCY_PKGS : (optional) packages to be installed in the image
#   - SOFTWARE_DIR    : (optional) directory to copy inside the image

if [[ -z ${ARCH} ]]; then
  echo "Arch undefined, default to amd64"
  export ARCH="amd64"
fi

# Select the docker container depenending on the ARCH
case ${ARCH} in
  'amd64')
     FROM_VALUE=${LINUX_DISTRO}:${DISTRO}
     ;;
  'i386')
     if [[ ${LINUX_DISTRO} != 'ubuntu' ]]; then
	 echo "There is no support for i386 non-ubuntu images yet"
	 exit 1
     fi

     # There are no i386 official images. Only 14.04 (trusty) is available
     # https://registry.hub.docker.com/u/32bit/ubuntu/tags/manage/
     if [[ $DISTRO != 'trusty' ]]; then
	 echo "Only trusty images are avilable for i386"
	 exit 1
     fi

     FROM_VALUE=32bit/ubuntu:14.04
     ;;
 'armhf')
     if [[ ${LINUX_DISTRO} != 'ubuntu' ]]; then
	 echo "There is no support for armhf non-ubuntu images yet"
	 exit 1
     fi

     FROM_VALUE=osrf/ubuntu_armhf:${DISTRO}
     ;;
  *)
     echo "Arch unknown"
     exit 1
esac

[[ -z ${USE_OSRF_REPO} ]] && USE_OSRF_REPO=false

echo '# BEGIN SECTION: create the Dockerfile'
cat > Dockerfile << DELIM_DOCKER
#######################################################
# Docker file to run build.sh

FROM ${FROM_VALUE}
MAINTAINER Jose Luis Rivero <jrivero@osrfoundation.org>
DELIM_DOCKER

# check and run the full cache invalidation
if ${INVALIDATE_DOCKER_CACHE}; then
cat >> Dockerfile << DELIM_ALL_CACHE_INVALIDATION
# Request by the user, invalidating all cache
RUN echo "**************************************************"
RUN echo " WARNING: FULL CACHE INVALIDATION ENABLED         "
RUN echo " $(( ( RANDOM % 100000 )  + 1 ))                  "
RUN echo "**************************************************"
DELIM_ALL_CACHE_INVALIDATION
fi

# Squid-deb-proxy configuration
cat >> Dockerfile << DELIM_DOCKER_SQUID
# If host is running squid-deb-proxy on port 8000, populate /etc/apt/apt.conf.d/30proxy
# By default, squid-deb-proxy 403s unknown sources, so apt shouldn't proxy ppa.launchpad.net
RUN route -n | awk '/^0.0.0.0/ {print \$2}' > /tmp/host_ip.txt
RUN echo "HEAD /" | nc \$(cat /tmp/host_ip.txt) 8000 | grep squid-deb-proxy \
  && (echo "Acquire::http::Proxy \"http://\$(cat /tmp/host_ip.txt):8000\";" > /etc/apt/apt.conf.d/30proxy) \
  && (echo "Acquire::http::Proxy::ppa.launchpad.net DIRECT;" >> /etc/apt/apt.conf.d/30proxy) \
  || echo "No squid-deb-proxy detected on docker host"
DELIM_DOCKER_SQUID

case ${LINUX_DISTRO} in
  ubuntu)
      if [[ ${ARCH} != 'armhf' ]]; then
cat >> Dockerfile << DELIM_DOCKER_ARCH
RUN echo "deb http://archive.ubuntu.com/ubuntu ${DISTRO} main restricted universe multiverse" \\
						       >> /etc/apt/sources.list && \\
    echo "deb http://archive.ubuntu.com/ubuntu ${DISTRO}-updates main restricted universe multiverse" \\
						       >> /etc/apt/sources.list && \\
    echo "deb http://archive.ubuntu.com/ubuntu ${DISTRO}-security main restricted universe multiverse" && \\
						       >> /etc/apt/sources.list
DELIM_DOCKER_ARCH
      fi
      ;;
  debian)
cat >> Dockerfile << DELIM_DOCKER_DEBIAN
RUN echo "deb http://ftp.us.debian.org/debian ${DISTRO} main" \\
						       >> /etc/apt/sources.list
DELIM_DOCKER_DEBIAN
      ;;
esac

# OSRF repository
if ${USE_OSRF_REPO}; then
  if [[ ${LINUX_DISTRO} != 'ubuntu' ]]; then
    echo "No OSRF repo non-ubuntu"
    exit 1
  fi

cat >> Dockerfile << DELIM_DOCKER2
RUN apt-get update && apt-get install -y wget
RUN echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > \\
                                                           /etc/apt/sources.list.d/drc-latest.list && \\
    wget http://packages.osrfoundation.org/drc.key -O - | apt-key add - 
DELIM_DOCKER2
fi

# Dart repositories
if ${DART_FROM_PKGS} || ${DART_COMPILE_FROM_SOURCE}; then
cat >> Dockerfile << DELIM_DOCKER_DART_PKGS
# Install dart from pkgs 
RUN apt-get install -y python-software-properties apt-utils software-properties-common
RUN apt-add-repository -y ppa:libccd-debs
RUN apt-add-repository -y ppa:fcl-debs
RUN apt-add-repository -y ppa:dartsim
DELIM_DOCKER_DART_PKGS
fi

cat >> Dockerfile << DELIM_DOCKER3
# Invalidate cache monthly
# This is the firt big installation of packages on top of the raw image. 
# The expection of updates is low and anyway it is cathed by the next
# update command below
RUN echo "${MONTH_YEAR_STR}"
RUN apt-get update && \
    apt-get install -y ${BASE_DEPENDENCIES} ${DEPENDENCY_PKGS}

# This is killing the cache so we should be getting the most recent packages
RUN echo "Invalidating cache $(( ( RANDOM % 100000 )  + 1 ))"
RUN apt-get update
RUN apt-get install -y ${BASE_DEPENDENCIES} ${DEPENDENCY_PKGS}

# Map the workspace into the container
RUN mkdir -p ${WORKSPACE}
DELIM_DOCKER3

if [[ -n ${SOFTWARE_DIR} ]]; then
cat >> Dockerfile << DELIM_DOCKER4
COPY ${SOFTWARE_DIR} ${WORKSPACE}/${SOFTWARE_DIR}
DELIM_DOCKER4
fi

cat >> Dockerfile << DELIM_DOCKER4
COPY build.sh build.sh
RUN chmod +x build.sh
DELIM_DOCKER4
echo '# END SECTION'

echo '# BEGIN SECTION: see Dockerfile'
cat Dockerfile
echo '# END SECTION'
