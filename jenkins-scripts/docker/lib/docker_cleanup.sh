# DOCKER CLEANUP FUNCTION
# Provides different increasing levels of cleanup (low, medium, high, extreme) for cleaning docker files. 
# (PARAMS) 
#           cleanup_level :  
#                          low = cleanups dangling images and stopped containers created > 5 days ago
#                          medium = cleanups dangling images and stopped containers created > 1 days ago
#                          high = docker prune -a
#                          extreme= cleans docker builder cache

cleanup_level=${1}
docker_version="$(sudo docker version --format '{{.Server.APIVersion}}')"
 if [[ $(echo "$docker_version > 1.25" | bc -l) ]]; then
        case $1 in
        # preferred utilizing each object prune to not affect the build cache in low and medium
        low)
            echo "Cleaning dangling images created more than 5 days ago ..."
            sudo docker image prune -f --filter "until=35h"
            echo "Cleaning stopped containers created more than 5 days ago ..."
            sudo docker container prune -f --filter "until=35h" 
            ;;
        medium)
            echo "Cleaning dangling images created more than 1 day ago ..."
            sudo docker image prune -f --filter "until=24h"
            echo "Cleaning stopped containers created more than 1 day ago ..."
            sudo docker container prune -f --filter "until=24h"
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
            echo "Did not run docker cleaner, level $cleanup_level not recognized (accepted values low | medium | high | extreme!! "
            exit 1
            ;;
        esac
 else
    echo ERROR: Please update docker to a newer version with prune!
    exit 1
 fi

