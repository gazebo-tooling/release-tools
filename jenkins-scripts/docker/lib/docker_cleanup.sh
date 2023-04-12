#function that cleans docker 
# Receives parameters  levels

docker_version="$(sudo docker version --format '{{.Server.APIVersion}}')"
 if [[ $(echo "$docker_version > 1.25" | bc -l) ]]; then
    echo "Running version of docker $(sudo docker version --format '{{.Server.APIVersion}}') with prune" 
        case $1 in
        # preferred utilizing each object prune to not affect the build cache in low and medium
        low)
        echo "Cleaning dangling images created more than 5 days ago ..."
        sudo bash -c "docker image prune -f --filter "until=35h""
        echo "Cleaning stopped containers created more than 5 days ago ..."
        sudo bash -c "docker container prune -f --filter "until=35h" "
        ;;
        medium)
        echo "Cleaning dangling images created more than 1 day ago ..."
        sudo bash -c "docker image prune -f --filter "until=24h""
        echo "Cleaning stopped containers created more than 1 day ago ..."
        sudo bash -c "docker container prune -f --filter "until=24h""
        ;;
        high)
        echo "Cleaning all unusued images, containers, networks and cache ..."
        sudo docker system prune --all -f
        ;;
        extreme)
        MAX_SIZE_FOR_BUILD_DIRS="5G"
        echo "Removing all cache "
        sudo docker builder prune -f --keep-storage $MAX_SIZE_FOR_BUILD_DIRS
        ;;
        *)
        echo "Did not run docker cleaner, level $1 not recognized (accepted values low | medium | high | extreme!! "
        ;;
        esac
    else
    
        echo "WARNING - Running version of docker $docker_version without prune, utilizing deprecated docker_gc as cleanup method!" 
        case $1 in
        low)
            wget https://raw.githubusercontent.com/spotify/docker-gc/master/docker-gc
            sudo bash -c "GRACE_PERIOD_SECONDS=432000 bash docker-gc"
        ;;
        medium)
            wget https://raw.githubusercontent.com/spotify/docker-gc/master/docker-gc
            sudo bash -c "GRACE_PERIOD_SECONDS=86400 bash docker-gc"
        ;;

        high)
          [[ -n $(sudo docker ps -q) ]] && sudo docker kill $(sudo docker ps -q) || true
          [[ -n $(sudo docker images -a -q) ]] && sudo docker rmi $(sudo docker images -a -q) || true
        ;;
        extreme)
            MAX_SIZE_FOR_BUILD_DIRS="5G"
            CMD_FIND_BIG_DIRS=(find ${HOME}/workspace -name build -exec du -h -d 0 -t ${MAX_SIZE_FOR_BUILD_DIRS} {} \;)
            echo "Clean up the whole cache was not enough. Look for build/ directories bigger than ${MAX_SIZE_FOR_BUILD_DIRS}:"
            # run command twice to avoid parsing. First for information proposes
            echo ${CMD_FIND_BIG_DIRS[@]}
            for d in $("${CMD_FIND_BIG_DIRS[@]}" | awk '{ print $2 }'); do
            # safe checks on paths
            if [[ $d == ${d/$HOME} ]] || [[ $d == ${d/build} ]]; then
                echo "System is trying to delete a path $d outside Jenkins home: $HOME with subdir build/"
                exit -1
            fi
            # avoid to rm -fr without a path. The ugly trick should leave d as it
            # is, in case of a bug in code it will not remove anything at random
            sudo rm -fr ${d/build}build
            done
        ;;
        *)
        echo "Did not run docker cleaner, level $1 not recognized (accepted values low | medium | high | extreme!! "
        ;;
        esac
    fi

