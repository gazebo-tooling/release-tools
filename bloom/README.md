## Scripts

### Release new version of Gazebo unofficial wrappers

1. bloom in the osrf/ repository
   *$ BLOOM_RELEASE_REPO_BASE=https://github.com/osrf/ bloom-release --no-pull-request --rosdistro <ros_distro> --track <ros_distro> gazeboX_ros[2]_pkgs*

1. Trigger Jenkins jobs with ros_gazebo_pkgs-release.py.bash
   - Usage *$ ros_gazebo_pkgs-release. <version <release_repo> <ros_distro> <token> 'other arguments used in release.py'*
     (i.e ros_gazebo_pkgs-release.py.bash 3.4.4 https://github.com/osrf/gazebo11_ros2_pkgs-release dashing xxx -r 1 --dry-run)
