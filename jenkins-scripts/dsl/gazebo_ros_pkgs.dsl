import _configs_.*
import javaposse.jobdsl.dsl.Job

def ros_distros    = Globals.get_ros_suported_distros()
def ci_arch        = 'amd64'
def ci_gpu         = 'nvidia'

ros_distros.each { ros_distro ->
  ubuntu_distros = Globals.ros_ci[ros_distro]

  ubuntu_distros.each { ubuntu_distro ->
    // --------------------------------------------------------------
    // 1. Create the default ci jobs
    def default_ci_job = job("ros_gazebo_pkgs-ci-default_${ros_distro}-${ubuntu_distro}-${ci_arch}")
    OSRFLinuxCompilation.create(default_ci_job)
    default_ci_job.with
    {
      scm {
        git("https://github.com/ros-simulation/gazebo_ros_pkgs") {
          branch("${ros_distro}-devel")
          subdirectory("gazebo_ros_pkgs")
        }
      }

      label "gpu-${ci_gpu}"

      triggers {
        scm('*/5 * * * *')
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              export ROS_DISTRO=${ros_distro}
              export DISTRO=${ubuntu_distro}
              export ARCH=${ci_arch}
              /bin/bash -xe ./scripts/jenkins-scripts/docker/lib/gazebo_ros_pkgs-base.bash
              """.stripIndent())
      }
    }
  } // end of ubuntu_distros
} // end of ros_distros
