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
