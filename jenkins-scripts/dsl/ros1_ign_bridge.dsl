import _configs_.*
import javaposse.jobdsl.dsl.Job

def bridge_packages = [
  'ros_ign_bridge',
  'ros_ign_gazebo_demos',
  'ros_ign_image',
  'ros_ign',
  'ros_ign_gazebo',
  'ros_ign_point_cloud'
]

// BLOOM PACKAGE BUILDER JOBS
bridge_packages.each { pkg ->
  pkg_dashed = pkg.replaceAll("_", "-")

  def build_pkg_job = job("$pkg_dashed-bloom-debbuilder")

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
        stringParam("IGNITION_VERSION", '', 'Ignition release supported in the binaries')
        stringParam("VERSION",null,"Packages version to be built")
        stringParam("RELEASE_VERSION", null, "Packages release version")
        stringParam("LINUX_DISTRO", 'ubuntu', "Linux distribution to build packages for")
        stringParam("DISTRO", "bionic", "Linux release inside LINUX_DISTRO to build packages for")
        stringParam("ARCH", "amd64", "Architecture to build packages for")
        stringParam('ROS_DISTRO', 'noetic','ROS DISTRO to build pakcages for')
        stringParam("UPLOAD_TO_REPO", 'stable', "OSRF repo name to upload the package to")
        stringParam('UPSTREAM_RELEASE_REPO', 'https://github.com/ignition-release/ros1_ign_bridge-release', 'Release repository url')
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

          /bin/bash -x ./scripts/jenkins-scripts/docker/bloom-debbuild.bash
          """.stripIndent())
    }
  }
}
