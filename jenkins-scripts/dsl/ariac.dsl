import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'xenial' ]
def supported_arches = [ 'amd64' ]

// LINUX
// --------------------------------------------------------------
// 1. Create the deb build job
def build_pkg_job = job("ariac-debbuilder")
OSRFLinuxBuildPkg.create(build_pkg_job)

build_pkg_job.with
{

  scm {
    git {
      remote {
        url('git@bitbucket.org:osrf/ariac.git')
        credentials('65cd22d1-f3d5-4ff4-b18f-1d88efa13a02')
      }

      branch('master')

      extensions {
        cleanBeforeCheckout()
        relativeTargetDirectory('repo')
      }
    }
  }

  steps {
    steps {
      shell("""\
            #!/bin/bash -xe

            export NIGHTLY_MODE=true
            export USE_REPO_DIRECTORY_FOR_NIGHTLY=true
            /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-debbuild.bash
            """.stripIndent())
    }
  }
}

supported_distros.each { distro ->
  supported_arches.each { arch ->

    if (distro == 'trusty')
       ros_distro = 'indigo'
    else if (distro == 'xenial')
       ros_distro = 'kinetic'

    // --------------------------------------------------------------
    // 2. Create the install test job
    def install_default_job = job("ariac-install-pkg-${distro}-${arch}")

    // Use the linux install as base
    OSRFLinuxInstall.create(install_default_job)

    install_default_job.with
    {
        triggers {
          cron('@weekly')
        }

        label "gpu-reliable-${distro}"

        steps {
          shell("""#!/bin/bash -xe

                export LINUX_DISTRO=ubuntu
                export ARCH=${arch}
                export DISTRO=${distro}


                /bin/bash -x ./scripts/jenkins-scripts/docker/ariac-install-test-job.bash
                """.stripIndent())
       }
    }

    // --------------------------------------------------------------
    // 3. Docker install test job
    def install_docker_default_job = job("ariac-install-docker_${ros_distro}-${distro}-${arch}")

    // Use the linux install_docker as base
    OSRFLinuxInstall.create(install_docker_default_job)

    install_docker_default_job.with
    {
      label "large-memory"

      scm {
        git {
          remote {
            github("osrf/ariac-docker")
          }
          extensions {
            relativeTargetDirectory("ariac-docker")
          }
          branch("refs/heads/master")
        }
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              export LINUX_DISTRO=ubuntu
              export ARCH=${arch}
              export DISTRO=${distro}
              export ROS_DISTRO=${ros_distro}

              /bin/bash -xe ./scripts/jenkins-scripts/docker/ariac-docker-installation.bash
              """.stripIndent())
      }
    }

    // --------------------------------------------------------------
    // 4. -any- PR integration
    def install_docker_any_job = job("ariac-ci-pr_any-docker_${ros_distro}-${distro}-${arch}")

    // Use the linux install_docker as base
    OSRFLinuxCompilationAnyGitHub.create(install_docker_any_job,
                                         "osrf/ariac-docker",
                                         [ "${ros_distro}" ])

    install_docker_any_job.with
    {
      label "large-memory"

      steps {
        shell("""\
              #!/bin/bash -xe

              export LINUX_DISTRO=ubuntu
              export ARCH=${arch}
              export DISTRO=${distro}
              export ROS_DISTRO=${ros_distro}

              /bin/bash -xe ./scripts/jenkins-scripts/docker/ariac-docker-installation.bash
              """.stripIndent())
      }
    }
  }
}
