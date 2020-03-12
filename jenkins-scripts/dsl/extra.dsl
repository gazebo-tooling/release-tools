import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org"

// List of repositories that have a counter part -release repo
// under Open Robotics control that host metadata for debian builds
release_repo_debbuilds = [ 'opensplice' ]

// List of repositories that host branches compatible with gbp (git build
// package) method used by debian
gbp_repo_debbuilds = [ 'ogre-2.1', 'lark-parser' ]

release_repo_debbuilds.each { software ->
  // --------------------------------------------------------------
  // 1. Create the deb build job
  def build_pkg_job = job("${software}-debbuilder")
  OSRFLinuxBuildPkg.create(build_pkg_job)

  build_pkg_job.with
  {
    // use only the most powerful nodes
    label "large-memory"

    steps {
      shell("""\
            #!/bin/bash -xe

            export USE_ROS_REPO=true
            /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-debbuild.bash
            """.stripIndent())
    }
  }
}

gbp_repo_debbuilds.each { software ->
  def build_pkg_job = job("${software}-debbuilder")
  OSRFLinuxBase.create(build_pkg_job)

  build_pkg_job.with
  {
    // use only the most powerful nodes
    label "large-memory"

    parameters
    {
       stringParam('BRANCH','master',
                   "${software}-release branch to test")
       stringParam('LINUX_DISTRO','ubuntu',
                   'Linux distribution to build package for')
       stringParam('DISTRO','bionic',
                   'Ubuntu DISTRO to build package for')
       stringParam('ARCH','amd64',
                   'Architecture to be used in the built of the package')
    }

    scm {
      git {
        remote {
          github("osrf/${software}-release", 'https')
          branch('${BRANCH}')
        }

        extensions {
          cleanBeforeCheckout()
          relativeTargetDirectory('repo')
        }
      }
    }

    logRotator {
      artifactNumToKeep(10)
    }

    concurrentBuild(true)

    throttleConcurrentBuilds {
      maxPerNode(1)
      maxTotal(5)
    }

    wrappers {
      preBuildCleanup {
          includePattern('pkgs/*')
          deleteCommand('sudo rm -rf %s')
      }
    }

    str_extra_bash=""
    // autopkgtest is broken for lark-parser
    if (software == "lark-parser")
       str_extra_bash = 'export RUN_AUTOPKGTEST=false'

    steps {
      shell("""\
            #!/bin/bash -xe

            ${str_extra_bash}
            /bin/bash -xe ./scripts/jenkins-scripts/docker/debian-gbp-generic.bash
            """.stripIndent())
    }

    publishers
    {
      publishers {
        archiveArtifacts('pkgs/*')
      }

      downstreamParameterized {
        trigger('repository_uploader_ng') {
          condition('SUCCESS')
          parameters {
            currentBuild()
            predefinedProp("PROJECT_NAME_TO_COPY_ARTIFACTS", "\${JOB_NAME}")
            predefinedProp("UPLOAD_TO_REPO", "stable")
            predefinedProp("PACKAGE_ALIAS", "${software}")
          }
        }
      }

      postBuildScripts {
        steps {
          shell("""\
                #!/bin/bash -xe

                sudo chown -R jenkins \${WORKSPACE}/repo
                sudo chown -R jenkins \${WORKSPACE}/pkgs
                """.stripIndent())
        }

        onlyIfBuildSucceeds(false)
        onlyIfBuildFails(false)
      }
    }
  }
}


def bridge_job = job("ros1_ign_bridge-debbuilder")
default_params = [ PACKAGE             : "ros1_ign_bridge",
                   PACKAGE_ALIAS       : "ros1-ign-bridge",
                   DISTRO              : "bionic",
                   ARCH                : "amd64",
                   RELEASE_REPO_BRANCH : "default",
                   RELEASE_VERSION     : "1",
                   UPLOAD_TO_REPO      : "stable",
                   OSRF_REPOS_TO_USE   : "stable" ]

OSRFLinuxBuildPkg.create(bridge_job, default_params)

bridge_job.with
{
    steps {
      shell("""\
            #!/bin/bash -xe

            export ENABLE_ROS=true
            /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-debbuild.bash
            """.stripIndent())
    }
}
