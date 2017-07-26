import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'xenial' ]
def supported_arches = [ 'amd64' ]

// LINUX
supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the deb build job
    def build_pkg_job = job("drake-debbuilder")
    OSRFLinuxBase.create(build_pkg_job)

    build_pkg_job.with
    {
      scm {
        git {
          remote {
            github('osrf/drake-release', 'https')
            branch('refs/heads/master')
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

      steps {
        shell("""\
              #!/bin/bash -xe

              export LINUX_DISTRO=ubuntu
              export ARCH=${arch}
              export DISTRO=${distro}
              export USE_ROS_REPO=true
              export ENABLE_CCACHE=false

              /bin/bash -xe ./scripts/jenkins-scripts/docker/debian-git-debbuild.bash
              """.stripIndent())
      }

      publishers
      {
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

    // --------------------------------------------------------------
    // 2. Create the compilation job
    def drake_ci_job = job("drake-ci-default-${distro}-${arch}")
    OSRFLinuxCompilation.create(drake_ci_job, false, false)

    drake_ci_job.with
    {
      scm {
        git {
          remote {
            github('osrf/drake-release', 'https')
            branch('refs/heads/master')

            extensions {
              relativeTargetDirectory('repo')
            }
          }
        }
      }

      triggers {
        scm('*/5 * * * *')
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}

              /bin/bash -xe ./scripts/jenkins-scripts/docker/drake-compilation.bash
              """.stripIndent())
      }
    }
  }
}
