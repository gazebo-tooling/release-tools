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
      // use only the most powerful nodes
      label "large-memory"

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

        downstreamParameterized {
	  trigger('repository_uploader_ng') {
	    condition('SUCCESS')
	    parameters {
	      currentBuild()
	      predefinedProp("PROJECT_NAME_TO_COPY_ARTIFACTS", "\${JOB_NAME}")
	      predefinedProp("DISTRO", "${distro}")
	      predefinedProp("ARCH", "${arch}")
	      predefinedProp("UPLOAD_TO_REPO", "drake")
	      predefinedProp("PACKAGE_ALIAS", "drake")
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

      parameters {
        booleanParam('CHECK_BINARY_SYMBOLS', true, 'check for bin symbols inside libdrake')
      }

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
    // 6. Install ROS Kinetic + MoveIt + NavStack
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

    // 7. Create the install ros_drake
    def install_ros_drake_job = job("ros_drake-install-${distro}-${arch}")
    OSRFLinuxInstall.create(install_ros_drake_job)

    install_ros_drake_job.with
    {
      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}
              export ROS_DISTRO=kinetic
              /bin/bash -xe ./scripts/jenkins-scripts/docker/ros_drake-release-testing.bash
	      """.stripIndent())
      }
    }

    // --------------------------------------------------------------
    // 8. Eigen ABI checker
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
    // 9. PCL/Eigen ABI checker
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

    // --------------------------------------------------------------
    // 10. Autobuilder
    standalone_job_name = "drake-standalone-debbuilder-${distro}-${arch}"
    def standalone_job = job(standalone_job_name)
    OSRFLinuxBase.create(standalone_job)

    standalone_job.with
    {
      // use only the most powerful nodes
      label "large-memory"

      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}
              /bin/bash -xe ./scripts/jenkins-scripts/docker/drake-standalone-release.bash
	      """.stripIndent())
      } // end of steps
    }
  }
}

// Bloom for ros_drake
def build_pkg_job = job("ros-drake-bloom-debbuilder")

// Use the linux install as base
OSRFLinuxBuildPkgBase.create(build_pkg_job)
GenericRemoteToken.create(build_pkg_job)

build_pkg_job.with
{
    properties {
      priority 100
    }

    parameters {
      stringParam("PACKAGE","ros-drake","Package name to be built")
      stringParam("VERSION",null,"Packages version to be built")
      stringParam("RELEASE_VERSION", null, "Packages release version")
      stringParam("LINUX_DISTRO", 'ubuntu', "Linux distribution to build packages for")
      stringParam("DISTRO", "xenial", "Linux release inside LINUX_DISTRO to build packages for")
      stringParam("ARCH", "amd64", "Architecture to build packages for")
      stringParam('ROS_DISTRO', 'kinetic','ROS DISTRO to build pakcages for')
      stringParam("UPLOAD_TO_REPO", 'drake', "OSRF repo name to upload the package to")
      stringParam('UPSTREAM_RELEASE_REPO', '', 'https://github.com/ros-gbp/gazebo_ros_pkgs-release')
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

// Bloom for catkin
def catkin_build_pkg_job = job("catkin-bloom-debbuilder")

// Use the linux install as base
OSRFLinuxBuildPkgBase.create(catkin_build_pkg_job)
GenericRemoteToken.create(catkin_build_pkg_job)

catkin_build_pkg_job.with
{
    properties {
      priority 100
    }

    parameters {
      stringParam("PACKAGE","catkin","Package name to be built")
      stringParam("VERSION",null,"Packages version to be built")
      stringParam("RELEASE_VERSION", null, "Packages release version")
      stringParam("LINUX_DISTRO", 'ubuntu', "Linux distribution to build packages for")
      stringParam("DISTRO", "xenial", "Linux release inside LINUX_DISTRO to build packages for")
      stringParam("ARCH", "amd64", "Architecture to build packages for")
      stringParam('ROS_DISTRO', 'kinetic','ROS DISTRO to build pakcages for')
      stringParam('UPSTREAM_RELEASE_REPO', 'https://github.com/ros-gbp/catkin-release', 'release repo to use')
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
        'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
        """.stripIndent()
      )
    }

    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -x ./scripts/jenkins-scripts/docker/bloom-debbuild.bash
            """.stripIndent())
    }
}
