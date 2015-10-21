# Need to take as argument the WORKSPACE.
# Needed to run inside the generated script

[[ -z ${1} ]] && echo "WORKSPACE argument missing" && exit -1
[[ ! -d ${1} ]] && echo "WORKSPACE not found at ${1}" && exit -1

export TIMING_STORE_PATH=${1}/timing
[[ -f ${TIMING_STORE_PATH}/_enable_timing ]] && ENABLE_TIMING=true || ENABLE_TIMING=false

[[ ! -d ${TIMING_STORE_PATH} ]] && mkdir -p ${TIMING_STORE_PATH}

get_time()
{
   echo `date +%s`
}

init_stopwatch()
{
   ${ENABLE_TIMING} || return 0

   local filename=${TIMING_STORE_PATH}/_time_${1}

   [[ -z ${filename} ]] && echo "No name" && exit -1
   [[ -f ${filename} ]] && echo "[clock] '${1}' label already exists! file: ${filename}" \
                        && exit -1

   get_time > ${filename}
}

stop_stopwatch()
{
  ${ENABLE_TIMING} || return 0

  local clock_filepath=${TIMING_STORE_PATH}/_time_${1}
  local csv_filepath=${TIMING_STORE_PATH}/_time_csv_${1}
  local csv_aggregate_filepath=${TIMING_STORE_PATH}/_time_csv_aggregate

  [[ ! -f ${clock_filepath} ]] && echo "[clock] Not clock file found in tag '${1}'" \
                         && ls -las ${TIMING_STORE_PATH} && exit -1
  START_CLOCK=`cat ${clock_filepath}`
  END_CLOCK=`get_time`
  TIME=`expr $END_CLOCK - $START_CLOCK`

  echo "Time for '${1}': ${TIME}"
  # Create a csv file for every label
  echo -e "\"${1}\"\n\"${TIME}\"" > ${csv_filepath}

  # Aggregated csv file
  # Output to file: first line the label, second line time (both quoted)
  if [[ ! -f ${csv_aggregate_filepath} ]]; then
    cp ${csv_filepath} ${csv_aggregate_filepath}
  else
    sed -i -e "1s:\$:,\"${1}\":" ${csv_aggregate_filepath}
    sed -i -e "2s:\$:,\"${TIME}\":" ${csv_aggregate_filepath}
  fi

  rm ${clock_filepath}
}
