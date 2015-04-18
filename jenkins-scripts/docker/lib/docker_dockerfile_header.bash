# Select the docker container depenending on the ARCH
if [[ ${ARCH} == 'amd64' ]]; then
   FROM_VALUE=ubuntu:${DISTRO}
elif [[ ${ARCH} == 'armhf' ]]; then
   FROM_VALUE=osrf/ubuntu_armhf:${DISTRO}
elif [[ ${ARCH} == 'i386' ]]; then
   FROM_VALUE=32bit/ubuntu:${DISTRO}
else
  echo "Arch unsupported"
  exit 1
fi

[[ -z ${USE_OSRF_REPO} ]] && USE_OSRF_REPO=false

echo '# BEGIN SECTION: create the Dockerfile'
cat > Dockerfile << DELIM_DOCKER
#######################################################
# Docker file to run build.sh

FROM ${FROM_VALUE}
MAINTAINER Jose Luis Rivero <jrivero@osrfoundation.org>

# If host is running squid-deb-proxy on port 8000, populate /etc/apt/apt.conf.d/30proxy
# By default, squid-deb-proxy 403s unknown sources, so apt shouldn't proxy ppa.launchpad.net
RUN route -n | awk '/^0.0.0.0/ {print \$2}' > /tmp/host_ip.txt
RUN echo "HEAD /" | nc \$(cat /tmp/host_ip.txt) 8000 | grep squid-deb-proxy \
  && (echo "Acquire::http::Proxy \"http://\$(cat /tmp/host_ip.txt):8000\";" > /etc/apt/apt.conf.d/30proxy) \
  && (echo "Acquire::http::Proxy::ppa.launchpad.net DIRECT;" >> /etc/apt/apt.conf.d/30proxy) \
  || echo "No squid-deb-proxy detected on docker host"

RUN \
  echo "deb http://archive.ubuntu.com/ubuntu ${DISTRO} main restricted universe multiverse" >> /etc/apt/sources.list && \\
  echo "deb http://archive.ubuntu.com/ubuntu ${DISTRO}-updates main restricted universe multiverse" >> /etc/apt/sources.list && \\
  echo "deb http://archive.ubuntu.com/ubuntu ${DISTRO}-security main restricted universe multiverse" >> /etc/apt/sources.list

# Invalidate cache daily
RUN echo "${TODAY_STR}"
RUN apt-get update && \
    apt-get install -y ${BASE_DEPENDENCIES} ${DEPENDENCY_PKGS}

# Map the workspace into the container
RUN mkdir -p ${WORKSPACE}
COPY sdformat ${WORKSPACE}/sdformat
COPY build.sh build.sh
RUN chmod +x build.sh
DELIM_DOCKER

if ${USE_OSRF_REPO}; then
cat >> Dockerfile << DELIM_DOCKER2
RUN apt-get install -y wget
# OSRF needed for ignition math
RUN echo "deb http://packages.osrfoundation.org/drc/ubuntu ${DISTRO} main" > \\
                                                           /etc/apt/sources.list.d/drc-latest.list && \\
    wget http://packages.osrfoundation.org/drc.key -O - | apt-key add - 
DELIM_DOCKER2
fi
echo '# END SECTION'

echo '# BEGIN SECTION: see Dockerfile'
cat Dockerfile
echo '# END SECTION'
