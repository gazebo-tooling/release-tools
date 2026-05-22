export APT_INSTALL="sudo DEBIAN_FRONTEND=noninteractive apt-get install -y"

generate_buildsh_header()
{
  SHELL_ON_ERRORS=${SHELL_ON_ERRORS:-false}
  echo "#!/bin/bash"
  echo "set -ex"
  if ${SHELL_ON_ERRORS}; then
    echo 'trap "/bin/bash" 0 INT QUIT ABRT PIPE TERM'
  fi
  if $GENERIC_ENABLE_TIMING; then
    echo "source ${TIMING_DIR}/_time_lib.sh ${WORKSPACE}"
  fi
  # Fail-fast precheck: if the host forwarded a DISPLAY into the container
  # (i.e. USE_GPU_DOCKER=true at docker_run.bash), confirm the X server is
  # actually reachable BEFORE we burn build time on apt installs, source
  # builds, compilation, etc. Without this, a misconfigured X session is
  # only detected mid-`make test`, hours into the job. The check is gated
  # on `xdpyinfo` being present so specialised images that do not pull
  # x11-utils are left untouched. See release-tools#1499.
  cat <<'DISPLAY_PRECHECK'
if [ -n "$DISPLAY" ] && command -v xdpyinfo > /dev/null 2>&1; then
  echo '# BEGIN SECTION: display readiness check'
  echo "DISPLAY=$DISPLAY XAUTHORITY=${XAUTHORITY:-<unset>}"
  ls -la /tmp/.X11-unix/ 2>&1 || true
  if command -v xauth > /dev/null 2>&1; then
    echo "xauth -n list entries: $(xauth -n list 2>/dev/null | wc -l)"
  fi
  if ! xdpyinfo -display "$DISPLAY" > /dev/null 2>&1; then
    echo "ERROR: cannot open X display '$DISPLAY' from inside the container."
    echo "       Aborting now to avoid wasting build time on a job that"
    echo "       will fail in the test phase. Failing diagnostics follow:"
    xdpyinfo -display "$DISPLAY" 2>&1 | head -5 || true
    command -v glxinfo > /dev/null 2>&1 && glxinfo -B 2>&1 | head -10 || true
    echo '# END SECTION'
    exit 1
  fi
  echo "X display $DISPLAY reachable from the container"
  echo '# END SECTION'
fi
DISPLAY_PRECHECK
}
