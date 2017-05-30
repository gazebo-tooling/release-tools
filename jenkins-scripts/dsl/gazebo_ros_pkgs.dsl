import _configs_.*
import javaposse.jobdsl.dsl.Job
import groovy.transform.Field

@Field
ArrayList ros_distros        = Globals.get_ros_suported_distros()
@Field
String ci_arch               = 'amd64'
// version to test more than the official one in each ROS distro
extra_gazebo_versions = [ 'indigo'  :  ['7'],
                          'jade'    :  ['7'],
                          'kinetic' :  ['8']]

Job create_common_compilation(String job_name,
                              String ubuntu_distro,
                              String ros_distro,
                              String gz_version,
                              String script_name)
{
   def comp_job = job(job_name)

   OSRFLinuxCompilationAnyGitHub.create(comp_job, [ "${ros_distro}" ])

   include_common_params(comp_job,
                         ubuntu_distro,
                         ros_distro,
                         gz_version,
                         script_name)
   return comp_job
}

void include_common_params(Job gazebo_ros_pkgs_job,
                           String ubuntu_distro,
                           String ros_distro,
                           String gz_version,
                           String script_name)
{
   gazebo_ros_pkgs_job.with
   {
      String use_non_official_gazebo_package = ""
      if (gz_version != "default") {
        use_non_official_gazebo_package = """\
                                          export GAZEBO_VERSION_FOR_ROS="${gz_version}"
                                          export USE_GZ_VERSION_ROSDEP=true
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
              export ROS_DISTRO=${ros_distro}
              /bin/bash -xe ./scripts/jenkins-scripts/docker/${script_name}.bash
              """.stripIndent())
      }
   }
}

ros_distros.each { ros_distro ->
  ubuntu_distros = Globals.ros_ci[ros_distro]

  ubuntu_distros.each { ubuntu_distro ->
    suffix_triplet="${ros_distro}-${ubuntu_distro}-${ci_arch}"

    // --------------------------------------------------------------
    // 1. Create the default ci jobs
    def default_ci_job = job("ros_gazebo_pkgs-ci-default_$suffix_triplet")
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
    def any_job_name = "ros_gazebo_pkgs-ci-pr_any_${suffix_triplet}"
    Job any_job = create_common_compilation(any_job_name,
                                            ubuntu_distro,
                                            ros_distro,
                                            "default",
                                            "gazebo_ros_pkgs-compilation")

    // --------------------------------------------------------------
    // 3. Create the default install
    def install_default_job = job("ros_gazebo_pkgs-install_pkg_${suffix_triplet}")
    OSRFLinuxInstall.create(install_default_job)
    include_common_params(install_default_job,
                          ubuntu_distro,
                          ros_distro,
                          "default",
                          "gazebo_ros_pkgs-release-testing")
    install_default_job.with
    {
      triggers {
        cron('@daily')
      }
    } 


    // Assume that gazebo means official version chose by ROS on every distribution
    gazebo_unofficial_versions = extra_gazebo_versions[ros_distro]
    gazebo_unofficial_versions.each { gz_version ->
      // Do not generate special jobs for official supported package. They will
      // be created using plain 'gazebo' name.
      if (! (gz_version in Globals.gz_version_by_rosdistro[ros_distro]))
      {
        // --------------------------------------------------------------
        // 1.2 Testing packages jobs install_pkg
        def install_default_job = job("ros_gazebo${gz_version}_pkgs-install_pkg_${suffix_triplet}")
        OSRFLinuxInstall.create(install_default_job)
        include_common_params(install_default_job,
                              ubuntu_distro,
                              ros_distro,
                              gz_version,
                              "gazebo_ros_pkgs-release-testing")
        install_default_job.with
        {
          triggers {
            cron('@daily')
          }
        } 

        // --------------------------------------------------------------
        // 2.2 Extra ci pr-any jobs
        def ci_pr_job_name = "ros_gazebo${gz_version}_pkgs-ci-pr_any_${suffix_triplet}"
        Job ci_pr_job = create_common_compilation(ci_pr_job_name,
                                            ubuntu_distro,
                                            ros_distro,
                                            gz_version,
                                            "gazebo_ros_pkgs-compilation")
      }
    } // end of gazebo_versions


    // --------------------------------------------------------------
    // 2. Create the regressions ci pr-any jobs
    def regression_job_name = "ros_gazebo_pkgs-ci-pr_regression_any_${suffix_triplet}"
    Job regression_job = create_common_compilation(regression_job_name,
                                                   ubuntu_distro,
                                                   ros_distro,
                                                   "default",
                                                   "gazebo_ros_pkgs-compilation_regression")
  } // end of ubuntu_distros
} // end of ros_distros
