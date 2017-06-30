import _configs_.*
import javaposse.jobdsl.dsl.Job
import groovy.transform.Field

@Field
ArrayList ros_distros        = Globals.get_ros_suported_distros()
@Field
String ci_arch               = 'amd64'
// version to test more than the official one in each ROS distro
extra_gazebo_versions = [ 'indigo'  :  ['7'],
                          'kinetic' :  ['8'],
                          'lunar'   :  ['8']]

bloom_debbuild_jobs = [ 'gazebo-dev', 'gazebo-msgs', 'gazebo-plugins', 'gazebo-ros', 'gazebo-ros-control', 'gazebo-ros-pkgs' ]

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
      if (gz_version != "default") {
        gz_package_version_str = """\
                                export GAZEBO_VERSION_FOR_ROS="${gz_version}"
                                export USE_GZ_VERSION_ROSDEP=true
                                export OSRF_REPOS_TO_USE="stable"
                                """.stripIndent()
      } else {
        gz_package_version_str = """\
                                export USE_DEFAULT_GAZEBO_VERSION_FOR_ROS=true
                                export OSRF_REPOS_TO_USE="stable"
                                """.stripIndent()
      }

      label "gpu-reliable-${ubuntu_distro}"

      steps {
        shell("""\
              #!/bin/bash -xe

              ${gz_package_version_str}

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
        def install_alternative_job = job("ros_gazebo${gz_version}_pkgs-install_pkg_${suffix_triplet}")
        OSRFLinuxInstall.create(install_alternative_job)
        include_common_params(install_alternative_job,
                              ubuntu_distro,
                              ros_distro,
                              gz_version,
                              "gazebo_ros_pkgs-release-testing")
        install_alternative_job.with
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


bloom_debbuild_jobs.each { bloom_pkg ->

  postfix_job_str = "bloom-debbuilder"
  def build_pkg_job = job("${bloom_pkg}-${postfix_job_str}")

  // Use the linux install as base
  OSRFLinuxBuildPkgBase.create(build_pkg_job)
  GenericRemoteToken.create(build_pkg_job)

  build_pkg_job.with
  {
      properties {
        priority 100
      }

      parameters {
        stringParam("PACKAGE","${bloom_pkg}","Package name to be built")
        stringParam("VERSION",null,"Packages version to be built")
        stringParam("RELEASE_VERSION", null, "Packages release version")
        stringParam("LINUX_DISTRO", 'ubuntu', "Linux distribution to build packages for")
        stringParam("DISTRO", null, "Linux release inside LINUX_DISTRO to build packages for")
        stringParam("ARCH", null, "Architecture to build packages for")
        stringParam('ROS_DISTRO', '','ROS DISTRO to build pakcages for')
        stringParam("UPLOAD_TO_REPO", null, "OSRF repo name to upload the package to")
        stringParam('UPSTREAM_RELEASE_REPO', '', 'https://github.com/ros-gbp/gazebo_ros_pkgs-release')
      }

      // Blocks to control dependencies
      if ("${bloom_pkg}" == 'gazebo-ros')
        blockOn(["gazebo-dev-${postfix_job_str}","gazebo-msgs-${postfix_job_str}"])
      else if ("${bloom_pkg}" == 'gazebo-plugins')
        blockOn(["gazebo-dev-${postfix_job_str}","gazebo-msgs-${postfix_job_str}"])
      else if ("${bloom_pkg}" == 'gazebo-ros-control')
        blockOn(["gazebo-dev-${postfix_job_str}","gazebo-msgs-${postfix_job_str}","gazebo-ros-${postfix_job_str}"])
      else if ("${bloom_pkg}" == 'gazebo-ros-pkgs')
        blockOn(["gazebo-dev-${postfix_job_str}","gazebo-msgs-${postfix_job_str}","gazebo-ros-${postfix_job_str}"])

      steps {
        systemGroovyCommand("""\
          build.setDescription(
          '<b>' + build.buildVariableResolver.resolve('ROS_DISTRO') + '-'
                + build.buildVariableResolver.resolve('VERSION') + '-'
                + build.buildVariableResolver.resolve('RELEASE_VERSION') + '</b>' +
          '(' + build.buildVariableResolver.resolve('LINUX_DISTRO') + '/' +
                build.buildVariableResolver.resolve('DISTRO') + '::' +
                build.buildVariableResolver.resolve('ARCH') + ')' +
          '<br />' +
          'upload to: ' + build.buildVariableResolver.resolve('UPLOAD_TO_REPO') +
          '<br />' +
          'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent()
        )
      }

      publishers {
        downstreamParameterized {
	  trigger('repository_uploader_ng') {
	    condition('SUCCESS')
	    parameters {
	      currentBuild()
	      predefinedProp("PROJECT_NAME_TO_COPY_ARTIFACTS", "\${JOB_NAME}")
	      predefinedProp("PACKAGE_ALIAS", "\${JOB_NAME}")
	    }
	  }
        }
      }


      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -x ./scripts/jenkins-scripts/docker/bloom-debbuild.bash
              """.stripIndent())
      }
  }
}
