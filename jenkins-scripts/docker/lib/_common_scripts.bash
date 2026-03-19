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
}
