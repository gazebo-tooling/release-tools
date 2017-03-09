import _configs_.*
import javaposse.jobdsl.dsl.Job
import groovy.transform.Field

@Field
ArrayList ros_distros        = Globals.get_ros_suported_distros()
@Field
String ci_arch               = 'amd64'
// version to test more than the official one in each ROS distro
ArrayList extra_gazebo_versions = ['7']

Job create_common_compilation(String job_name, 
                              String ubuntu_distro, 
                              String gz_version,
                              String script_name)
{
   def comp_job = job(job_name)

   OSRFLinuxCompilationAnyGitHub.create(comp_job,
              Globals.get_ros_distros_by_ubuntu_distro(ubuntu_distro))

   comp_job.with 
   {
      String use_non_official_gazebo_package = ""
      if (gz_version != "default") {
        use_non_official_gazebo_package = """\
                                          export GAZEBO_VERSION_FOR_ROS="${gz_version}"
                                          export OSRF_REPOS_TO_USE="stable"
                                          """.stripIndent()
      }

      label "gpu-reliable-${ubuntu_distro}"

      steps {
        shell("""\
              #!/bin/bash -xe

              ${use_non_official_gazebo_package}

              export DISTRO=${ubuntu_distro}
              export ARCH=${ci_arch}
              /bin/bash -xe ./scripts/jenkins-scripts/docker/${script_name}.bash
              """.stripIndent())
      }
   }
}

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
          extensions {
            relativeTargetDirectory("gazebo_ros_pkgs")
          }
          branch("${ros_distro}-devel")
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
    // Note: this package are independent of the ROS distro used. It is guess
    // from target branch at runtime, handled by gazebo_ros_pkgs-compilation.bash

    // TODO: these set of jobs will be created probably several times if
    // different ROS versions support same ubuntu platforms.

    // Assume that gazebo means official version chose by ROS on every distribution
    gazebo_versions = [ 'default' ] + extra_gazebo_versions

    gazebo_versions.each { gz_version ->
      // Do not generate special jobs for official supported package. They will
      // be created using plain 'gazebo' name.
      if (Globals.gz_version_by_rosdistro[ros_distro] != gz_version)
      {
        if (gz_version == "default")
          gz_version=""

        def any_job_name = "ros_gazebo${gz_version}_pkgs-ci-pr_any-${ubuntu_distro}-${ci_arch}"
        Job any_job = create_common_compilation(any_job_name,
                                                ubuntu_distro,
                                                gz_version,
                                                "gazebo_ros_pkgs-compilation")
      }
    } // end of gazebo_versions


    // --------------------------------------------------------------
    // 2. Create the regressions ci pr-any jobs
    def regression_job_name = "ros_gazebo_pkgs-ci-pr_regression_any-${ubuntu_distro}-${ci_arch}"
    Job regression_job = create_common_compilation(regression_job_name,
                                                   ubuntu_distro,
                                                   "default",
                                                   "gazebo_ros_pkgs-compilation_regression")
  } // end of ubuntu_distros
} // end of ros_distros
