import _configs_.*
import javaposse.jobdsl.dsl.Job

def WRITE_JOB_LOG = System.getenv('WRITE_JOB_LOG') ?: false

def bridge_packages = [
  'ros_gz',
  'ros_gz_bridge',
  'ros_gz_image',
  'ros_gz_interfaces',
  'ros_gz_sim',
  'ros_gz_sim_demos'
]

def gzgarden_ros_distros_ci = [
  'humble',
  'iron'
]

def unofficial_combinations = [
  'garden' : ['humble', 'iron'],
  'harmonic' : ['iron']
]

logging_list = [:]
logging_list['unofficial_wrappers_install_pkg_ci'] = []

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
        stringParam('ROS_DISTRO', null,'ROS DISTRO to build pakcages for')
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

void generate_install_test_job(Job install_test_job)
{
  OSRFLinuxInstall.create(install_test_job)
  install_test_job.with
  {
    parameters {
      stringParam("LINUX_DISTRO", 'ubuntu', "Linux distribution to build packages for")
      stringParam("OSRF_REPOS_TO_USE", 'stable', "Repositories to add to the testing install")
      labelParam('JENKINS_NODE_TAG') {
        description('Jenkins node or group to run the build on')
        defaultValue('gpu-reliable')
      }
    }

    // Designed to be run manually. No triggers.
    label Globals.nontest_label("gpu-reliable")

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

           export INSTALL_JOB_PKG=ros-\${ROS_DISTRO}-ros-gz\${GZ_VERSION}
           export INSTALL_JOB_REPOS=\${OSRF_REPOS_TO_USE}
           export USE_ROS_REPO=true
           export ROS2=true
           /bin/bash -x ./scripts/jenkins-scripts/docker/ros_gz-install-test-job.bash
           """.stripIndent())
    }
  }
}

def manual_install_test_job = job("ros_gz_bridge-install-pkg_unofficial-any-manual")
generate_install_test_job(manual_install_test_job)

manual_install_test_job.with
{
  parameters {
    stringParam("DISTRO", '', "Linux release inside LINUX_DISTRO to build packages for")
    stringParam("ARCH", '', "Architecture to build packages for")
    stringParam("ROS_DISTRO", '', "ROS distribution")
    stringParam("GZ_VERSION", '', "Gazebo version")
  }
}

unofficial_combinations.each { gz_release, ros_distros ->
  ros_distros.each { ros_distro ->
    def periodic_install_test_job = job("ros_gz${gz_release}_bridge-install-pkg_${ros_distro}-ci-jammy-amd64")
    generate_install_test_job(periodic_install_test_job)
    periodic_install_test_job.with
    {
      parameters {
        choiceParam("DISTRO", ['jammy'], "Linux release inside LINUX_DISTRO to build packages for")
        choiceParam("ARCH", ['amd64'], "Architecture to build packages for")
        choiceParam("ROS_DISTRO", [ros_distro], "ROS distribution")
        choiceParam("GZ_VERSION", [gz_release], "Gazebo version")
      }

      triggers {
        scm('@daily')
      }
    }

    logging_list['unofficial_wrappers_install_pkg_ci'].add(
      [collection: gz_release,
       job_name: periodic_install_test_job.name])
  }
}


if (WRITE_JOB_LOG) {
  File log_file = new File("jobs.txt")
  log_file.withWriter{ file_writer ->
    logging_list.each { log_type, items ->
      items.each {file_writer.println "${log_type} ${it.collection} ${it.job_name}"}
    }
  }
}
