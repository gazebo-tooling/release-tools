import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'xenial' ]
def supported_arches = [ 'amd64' ]


Globals.default_emails = "jrivero@osrfoundation.org, steven@osrfoundation.org"

void include_bazel_parselog(Job job)
{
  job.with
  {
    publishers {
        consoleParsing {
            projectRules('bazel.parser')
            unstableOnWarning()
        }
    }
  }
}

// LINUX
supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the deb build job
    def build_pkg_job = job("drake-debbuilder")
    OSRFLinuxBase.create(build_pkg_job)

    build_pkg_job.with
    {
      parameters
      {
         stringParam('BRANCH','master',
                     'Drake-release branch to test')
      }

      scm {
        git {
          remote {
            github('osrf/drake-release', 'https')
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

              /bin/bash -xe ./scripts/jenkins-scripts/docker/drake-debbuild.bash
              """.stripIndent())
      }

      publishers
      {
        publishers {
          archiveArtifacts('pkgs/*')
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
    // 2. Create the compilation job
    def drake_ci_job = job("drake-ci-default-${distro}-${arch}")
    OSRFLinuxCompilation.create(drake_ci_job, false, false)
    include_bazel_parselog(drake_ci_job)

    drake_ci_job.with
    {
      scm {
        git {
          remote {
            github('osrf/drake', 'https')
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

    // --------------------------------------------------------------
    // 3. Create the testing any job
    def drake_any_job = job("drake-ci-pr_any-${distro}-${arch}")
    // Use stub to supply a fake bitbucket repository. It is overwritten by the
    // git scm section below. False to disable testing.
    OSRFLinuxCompilationAny.create(drake_any_job, "repo_stub", false, false)
    include_bazel_parselog(drake_any_job)

    drake_any_job.with
    {
      // use only the most powerful nodes
      label "large-memory"

      scm {
        git {
          remote {
            github('osrf/drake', 'https')
            branch('${SRC_BRANCH}')

            extensions {
              relativeTargetDirectory('repo')
            }
          }
        }
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

    // --------------------------------------------------------------
    // 4. Create the testing job of drake + ROS
    def drake_ros_ci_job = job("drake-ci-default_ROS+MoveIt+Navstak-kinetic-${distro}-${arch}")
    OSRFLinuxCompilation.create(drake_ros_ci_job, false, false)

    drake_ros_ci_job.with
    {
      scm {
        git {
          remote {
            github('osrf/drake', 'https')
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
              export ROS_DISTRO=kinetic

              /bin/bash -xe ./scripts/jenkins-scripts/docker/drake-ROS-install.bash
              """.stripIndent())
      }
    }

    // --------------------------------------------------------------
    // 5. Install ROS Kinetic + MoveIt + NavStack
    def drake_ros_install_job = job("drake-install-ROS+MoveIt+Navstak-kinetic-${distro}-${arch}")
    OSRFLinuxCompilation.create(drake_ros_install_job, false, false)

    drake_ros_install_job.with
    {
      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}
              export ROS_DISTRO=kinetic
              export INSTALL_JOB_PKG="ros-kinetic-moveit ros-kinetic-navigation ros-kinetic-desktop"
              export INSTALL_JOB_REPOS=stable
              export USE_ROS_REPO=true

              /bin/bash -x ./scripts/jenkins-scripts/docker/generic-install-test-job.bash
              """.stripIndent())
      }
    }

    // --------------------------------------------------------------
    // 5. Eigen ABI checker
    abi_job_name = "eigen3-abichecker-333_to_33beta1-${distro}-${arch}"
    def abi_job = job(abi_job_name)
    OSRFLinuxABI.create(abi_job)

    abi_job.with
    {
      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}
              /bin/bash -xe ./scripts/jenkins-scripts/docker/eigen-abichecker.bash
	      """.stripIndent())
      } // end of steps
    }

    // --------------------------------------------------------------
    // 6. PCL/Eigen ABI checker
    abi_job_pcl_name = "pcl_eigen3-abichecker-333_to_33beta1-${distro}-${arch}"
    def abi_job_pcl = job(abi_job_pcl_name)
    OSRFLinuxABI.create(abi_job_pcl)

    abi_job_pcl.with
    {
      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}
              /bin/bash -xe ./scripts/jenkins-scripts/docker/eigen_pcl-abichecker.bash
	      """.stripIndent())
      } // end of steps
    }
  }
}
