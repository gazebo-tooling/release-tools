## Scripts

### Release new version of Gazebo unofficial wrappers

1. bloom in the osrf/ repository
```bash
   $ BLOOM_RELEASE_REPO_BASE=https://github.com/osrf/ bloom-release --no-pull-request --rosdistro <ros_distro> --track <ros_distro> gazeboX_ros[2]_pkgs

   Example
   $ BLOOM_RELEASE_REPO_BASE=https://github.com/osrf/ bloom-release --no-pull-request --rosdistro foxy --track foxy gazebo11_ros2_pkgs
```

1. Trigger Jenkins jobs with ros_gazebo_pkgs-release.py.bash
```bash
   $ ros_gazebo_pkgs-release. <version <release_repo> <ros_distro> <token> 'other arguments used in release.py'*

   Example:
   $ ros_gazebo_pkgs-release.py.bash 3.4.4 https://github.com/osrf/gazebo11_ros2_pkgs-release foxy xxx -r 1 --dry-run
```

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

