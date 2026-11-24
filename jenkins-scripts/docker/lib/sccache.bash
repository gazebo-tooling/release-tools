# Compiler cache (sccache) of the Linux CI builds.
#
# Off unless the job sets SCCACHE_MODE. The DSL sets it per kind of job:
#   rw   read and write the cache that the builds on the agent share. Only for
#        builds of code that is already in the upstream repositories (branch
#        CI): whatever a build stores there ends up in the binaries of the
#        next builds on the agent.
#   ro   read the shared cache and store nothing (pull requests).
#   abi  read the shared cache and store the new objects in a layer of the
#        job alone, so that the second build of the ABI checker reuses the
#        first one.
#
# The agent provisioning (osrf_jenkins_agent cookbook) installs
# /usr/local/bin/sccache and mounts the instance store at /srv/sccache. When
# either is missing, the build runs without a cache.
#
# ro and abi see the shared cache through an overlay volume: the kernel sends
# every write to directories of the job, deleted after the build. A read-only
# bind mount would give no hits: sccache updates the times of every file it
# reads from the cache, and fails the read when it cannot.
#
# The local backend of sccache supports one server per cache directory, and
# each build container runs its own server: this needs agents with a single
# executor.

SCCACHE_HOST_ROOT=${SCCACHE_HOST_ROOT:-/srv/sccache}
SCCACHE_HOST_BIN=${SCCACHE_HOST_BIN:-/usr/local/bin/sccache}
SCCACHE_SHARED_DIR=${SCCACHE_HOST_ROOT}/cache
SCCACHE_OVERLAY_ROOT=${SCCACHE_HOST_ROOT}/overlay
SCCACHE_VOLUME_PREFIX=sccache-overlay-

# Validate SCCACHE_MODE and empty it when the agent has no cache
sccache_check_host()
{
  [[ -z ${SCCACHE_MODE} ]] && return 0

  case ${SCCACHE_MODE} in
    rw | ro | abi) ;;
    *)
      echo "Unknown SCCACHE_MODE: ${SCCACHE_MODE}"
      exit 1
      ;;
  esac

  if ! mountpoint -q "${SCCACHE_HOST_ROOT}"; then
    echo "No volume mounted at ${SCCACHE_HOST_ROOT}: building without sccache"
    SCCACHE_MODE=
  elif [[ ! -x ${SCCACHE_HOST_BIN} ]]; then
    echo "No ${SCCACHE_HOST_BIN}: building without sccache"
    SCCACHE_MODE=
  fi
}

# Remove the overlay volumes and their directories. With one executor per
# agent, anything there belongs to a build that is over.
sccache_remove_overlays()
{
  local volume
  for volume in $(sudo docker volume ls -q | grep "^${SCCACHE_VOLUME_PREFIX}"); do
    sudo docker volume rm "${volume}" > /dev/null || true
  done
  # sudo: the kernel creates the work/work directory of an overlay as root
  if [[ -d ${SCCACHE_OVERLAY_ROOT} ]]; then
    sudo rm -fr "${SCCACHE_OVERLAY_ROOT}"
  fi
}

# Most of the volume, leaving room for the overlays of the pull requests
sccache_cache_size()
{
  local size_gb
  size_gb=$(df --output=size -BG "${SCCACHE_HOST_ROOT}" | tail -n 1 | tr -dc '0-9')
  echo "$(( size_gb * 3 / 4 ))G"
}

# Add the cache to EXTRA_PARAMS_STR, the arguments of docker run
sccache_setup_docker_run()
{
  [[ -z ${SCCACHE_MODE} ]] && return 0

  sccache_remove_overlays
  mkdir -p "${SCCACHE_SHARED_DIR}"

  local volume rw_mode basedirs name overlay_dir
  case ${SCCACHE_MODE} in
    rw)
      volume=${SCCACHE_SHARED_DIR}
      rw_mode=READ_WRITE
      ;;
    ro | abi)
      # No commas or colons: they would break the mount options
      name=$(printf '%s' "${BUILD_TAG:-build-$$}" | tr -c 'A-Za-z0-9_.-' '_')
      overlay_dir=${SCCACHE_OVERLAY_ROOT}/${name}
      mkdir -p "${overlay_dir}/upper" "${overlay_dir}/work"
      volume=${SCCACHE_VOLUME_PREFIX}${name}
      sudo docker volume create --driver local \
        --opt type=overlay \
        --opt device=overlay \
        --opt "o=lowerdir=${SCCACHE_SHARED_DIR},upperdir=${overlay_dir}/upper,workdir=${overlay_dir}/work" \
        "${volume}" > /dev/null
      if [[ ${SCCACHE_MODE} == ro ]]; then
        rw_mode=READ_ONLY
      else
        rw_mode=READ_WRITE
      fi
      ;;
  esac

  # Paths are hashed relative to these directories, so that jobs with
  # different workspaces reuse each other's objects. The ABI checker builds a
  # copy of the sources in /tmp.
  basedirs=${WORKSPACE}
  if [[ ${SCCACHE_MODE} == abi ]]; then
    basedirs=/tmp:${WORKSPACE}
  fi

  echo "sccache: mode ${SCCACHE_MODE} (${rw_mode}), shared cache ${SCCACHE_SHARED_DIR}"

  # SCCACHE_DIRECT=false in every mode: objects stored with the preprocessor
  #   cache mode on never hit with it off, and the reverse.
  # SCCACHE_CONF=/dev/null: no configuration file from the cache directory or
  #   the home of the user.
  # SCCACHE_IDLE_TIMEOUT=0: the server lives as long as the container, so the
  #   statistics at the end cover the whole build.
  # CMAKE_<LANG>_COMPILER_LAUNCHER: CMake 3.17 or newer.
  EXTRA_PARAMS_STR="${EXTRA_PARAMS_STR} \
                    -v ${SCCACHE_HOST_BIN}:/usr/local/bin/sccache:ro \
                    -v ${volume}:/srv/sccache \
                    -e SCCACHE_DIR=/srv/sccache \
                    -e SCCACHE_LOCAL_RW_MODE=${rw_mode} \
                    -e SCCACHE_CACHE_SIZE=$(sccache_cache_size) \
                    -e SCCACHE_BASEDIRS=${basedirs} \
                    -e SCCACHE_DIRECT=false \
                    -e SCCACHE_CONF=/dev/null \
                    -e SCCACHE_IDLE_TIMEOUT=0 \
                    -e CMAKE_C_COMPILER_LAUNCHER=sccache \
                    -e CMAKE_CXX_COMPILER_LAUNCHER=sccache"
}

# Append the statistics of the build to build.sh
sccache_add_statistics_to_buildsh()
{
  [[ -z ${SCCACHE_MODE} ]] && return 0

  cat >> build.sh << BUILDSH_SCCACHE
echo '# BEGIN SECTION: sccache statistics of this build'
sccache --show-stats || true
echo '# END SECTION'
BUILDSH_SCCACHE
}
