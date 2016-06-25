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
    // Enable testing but not cppcheck
    OSRFLinuxCompilation.create(default_ci_job, true, false)
    default_ci_job.with
    {
      scm {
        git {
          remote {
            github("ros-simulation/gazebo_ros_pkgs")
          }
          branch("${ros_distro}-devel")
          relativeTargetDir("gazebo_ros_pkgs")
        }
      }

      label "gpu-reliable-${ubuntu_distro}"

      triggers {
        scm('*/5 * * * *')
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              export ROS_DISTRO=${ros_distro}
              export DISTRO=${ubuntu_distro}
              export ARCH=${ci_arch}
              /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo_ros_pkgs-compilation.bash
              """.stripIndent())
      }
    }

    // --------------------------------------------------------------
    // 2. Create the default ci pr-any jobs
    def any_job_name = "ros_gazebo_pkgs-ci-pr_any_${ros_distro}-${ubuntu_distro}-${ci_arch}"
    def any_job = job(any_job_name)
    // Use generic any but remove hg
    OSRFLinuxCompilationAny.create(any_job, 'fake')
    any_job.with
    {
      // remove the scm (mercurial)
      configure { project ->
        project.remove(project / scm)
      }

      scm {
        git {
          remote {
            github("ros-simulation/gazebo_ros_pkgs")
          }
          branch("${ros_distro}-devel")
          relativeTargetDir("gazebo_ros_pkgs")
        }
      }

      label "gpu-reliable-${ubuntu_distro}"

      steps {
        shell("""\
              #!/bin/bash -xe

              export ROS_DISTRO=${ros_distro}
              export DISTRO=${ubuntu_distro}
              export ARCH=${ci_arch}
              /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo_ros_pkgs-compilation.bash
              """.stripIndent())
      }
    }
  } // end of ubuntu_distros
} // end of ros_distros
