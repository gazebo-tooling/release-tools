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

# Map the workspace into the container
RUN mkdir -p ${WORKSPACE}
# automatic invalidation of the cache if day is different
RUN echo "${TODAY_STR}"
DELIM_DOCKER
