## Scripts:

### Create a new gazeboX_rosY_pkgs repository

For a new official wrappers the notation used below correspond to:
`gazeboX_rosY_pkgs` (`X` is major version in Gazebo, `Y` is 1 or 2 for ROS)

 1. Create new repo under github.com/osrf/ organization named:
    - ROS1 https://github.com/osrf/gazeboX_ros_pkgs-release
    - ROS2 https://github.com/osrf/gazeboX_ros2_pkgs-release

 1. Run initial_info_for_new_release_repo.bash
    It will import all branches and tags from official gbp into the new repo
    - Usage: *$ initial_info_for_new_release_repo X rosY*

 1. Clone the new repo, go to the directory and run rename-gazebo-ros-pkgs.bash
    - Usage: *$ rename-gazebo-ros-pkgs.bash X <space separted list of rosdistros to release>*

### Release new versiones for the gazeboX_rosY_pkgs repository    

 * release-bloom.py: generic python script that call -bloom osrf jenkins
   jobs.
 
 * ros_gazebo_pkgs-release.py.bash: bash script that will use release-bloom.py
   passing all the names of ros_gazebo_pkgs (gazebo-msgs, gazebo-ros, etc).
