import _configs_.*
import javaposse.jobdsl.dsl.Job

def bridge_packages = [
  'ros_gz',
  'ros_gz_bridge',
  'ros_gz_image',
  'ros_gz_interfaces',
  'ros_gz_sim',
  'ros_gz_sim_demos'
]

// BLOOM PACKAGE BUILDER JOBS
bridge_packages.each { pkg ->
  pkg_dashed = pkg.replaceAll("_", "-")

  postfix_job_str = "bloom-debbuilder"
  def build_pkg_job = job("${pkg_dashed}-${postfix_job_str}")

  // Use the linux install as base
  OSRFLinuxBuildPkgBase.create(build_pkg_job)
  GenericRemoteToken.create(build_pkg_job)

  build_pkg_job.with
  {
    properties {
      priority 100
    }

    configure { project ->
      project / 'properties' / 'hudson.plugins.copyartifact.CopyArtifactPermissionProperty' / 'projectNameList' {
        'string' 'repository_uploader_*'
      }
    }

    parameters {
        stringParam("PACKAGE","$pkg_dashed","Package name to be built")
        stringParam("VERSION",null,"Packages version to be built")
        stringParam("RELEASE_VERSION", null, "Packages release version")
        stringParam("LINUX_DISTRO", 'ubuntu', "Linux distribution to build packages for")
        stringParam("DISTRO", "jammy", "Linux release inside LINUX_DISTRO to build packages for")
        stringParam("ARCH", "amd64", "Architecture to build packages for")
        stringParam('ROS_DISTRO', 'humble','ROS DISTRO to build pakcages for')
        stringParam("UPLOAD_TO_REPO", 'stable', "OSRF repo name to upload the package to")
        stringParam('UPSTREAM_RELEASE_REPO', 'https://github.com/j-rivero/ros_ign-release', 'Release repository url')
    }

    // Blocks to control dependencies
    projects_to_blockon = []
    if ("${pkg}" == 'ros_gz_sim_demos')
      projects_to_blockon = ["ros-gz-sim-${postfix_job_str}",
                            "ros-gz-bridge-${postfix_job_str}",
                            "ros-gz-image-${postfix_job_str}"]
    else if ("${pkg}" == 'ros_gz_image')
      projects_to_blockon = ["ros-gz-bridge-${postfix_job_str}"]
    else if ("${pkg}" == 'ros_gz_bridge')
      projects_to_blockon = ["ros-gz-interfaces-${postfix_job_str}"]
    else if ("${pkg}" == 'ros_gz')
      projects_to_blockon = ["ros-gz-sim-demos-${postfix_job_str}",
                            "ros-gz-sim-${postfix_job_str}",
                            "ros-gz-bridge-${postfix_job_str}",
                            "ros-gz-image-${postfix_job_str}"]

    if (projects_to_blockon) {
      blockOn(projects_to_blockon) {
        blockLevel('GLOBAL')
        scanQueueFor('ALL')
      }
    }


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
        trigger('repository_uploader_packages') {
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

          export OSRF_REPOS_TO_USE='stable'
          export ROS2=true

          /bin/bash -x ./scripts/jenkins-scripts/docker/bloom-debbuild.bash
          """.stripIndent())
    }
  }
}

def install_test_job =
job("ros_gz_bridge-install-pkg_unofficial-any-manual")
OSRFLinuxInstall.create(install_test_job)
install_test_job.with
{
  parameters {
    stringParam("LINUX_DISTRO", 'ubuntu', "Linux distribution to build packages for")
    stringParam("DISTRO", '', "Linux release inside LINUX_DISTRO to build packages for")
    stringParam("ARCH", '', "Architecture to build packages for")
    stringParam("ROS_DISTRO", 'humble', "ROS distribution")
    stringParam("GZ_VERSION", 'garden', "Gazebo version")
    stringParam("OSRF_REPOS_TO_USE", 'stable prerelease', "Repositories to add to the testing install")
    labelParam('JENKINS_NODE_TAG') {
      description('Jenkins node or group to run the build on')
      defaultValue('gpu-reliable')
    }
  }

  // Designed to be run manually. No triggers.
  label "gpu-reliable"

  steps {
    systemGroovyCommand("""\
      build.setDescription(
      '<b>' + build.buildVariableResolver.resolve('LINUX_DISTRO') + '/' +
              build.buildVariableResolver.resolve('DISTRO') + '::' +
              build.buildVariableResolver.resolve('ROS_DISTRO') + '::' +
              build.buildVariableResolver.resolve('GZ_VERSION') + '</b>' +
      '<br />' +
      'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
      """.stripIndent())

    shell("""\
         #!/bin/bash -xe

         export INSTALL_JOB_PKG=ros-\${ROS_DISTRO}-ros-gz\${GZ_VERSION}-sim
         export ROS2=true
         /bin/bash -x ./scripts/jenkins-scripts/docker/ign_launch-install-test-job.bash
         """.stripIndent())
  }
}
