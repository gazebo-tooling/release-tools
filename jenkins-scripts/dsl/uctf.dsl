import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'xenial' ]
def supported_arches = [ 'amd64' ]

// LINUX
supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the deb build job
    def build_pkg_job = job("uctf-debbuilder")
    OSRFLinuxBase.create(build_pkg_job)

    build_pkg_job.with
    {
      scm {
        git {
          remote {
            github('osrf/uctf-debian', 'https')
            credentials('923bb60c-535f-4fb9-8768-44d4d0cd11dd')
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

              /bin/bash -xe ./scripts/jenkins-scripts/docker/debian-git-debbuild.bash
              """.stripIndent())
      }

      publishers
      {
        publishers {
          archiveArtifacts('pkgs/*')

          downstreamParameterized {
            trigger('repository_uploader_ng') {
              condition('SUCCESS')
              parameters {
                currentBuild()
                predefinedProp("PROJECT_NAME_TO_COPY_ARTIFACTS", "\${JOB_NAME}")
                predefinedProp("UPLOAD_TO_REPO", "stable")
                predefinedProp("PACKAGE_ALIAS" , "uctf")
                predefinedProp("DISTRO",         "${distro}")
                predefinedProp("ARCH",           "${arch}")
              }
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
    // --------------------------------------------------------------
    // 2. Create the install test job
    def install_default_job = job("uctf-install-pkg-${distro}-${arch}")

    // Use the linux install as base
    OSRFLinuxInstall.create(install_default_job)

    install_default_job.with
    {
        triggers {
          cron('@weekly')
        }

        steps {
          shell("""#!/bin/bash -xe

                export LINUX_DISTRO=ubuntu
                export ARCH=${arch}
                export DISTRO=${distro}

                /bin/bash -x ./scripts/jenkins-scripts/docker/uctf-install-test-job.bash
                """.stripIndent())
       }
    }
  }
}
