import _configs_.*
import javaposse.jobdsl.dsl.Job

def ci_distro = [ 'trusty' ]
def other_supported_distros = [ ]
def all_supported_distros = ci_distro + other_supported_distros
def supported_arches = [ 'amd64' ]

Globals.extra_emails = "caguero@osrfoundation.org"

// Early testing shows that xenial jobs can be run on a
// trusty host with good results.
void include_gpu_label(Job job, String distro)
{
  job.with
  {
    if (distro == 'xenial')
      label "gpu-reliable-${distro} || gpu-reliable-trusty"
    else
      label "gpu-reliable-${distro}"
  }
}

void include_parselog(Job job)
{
  job.with
  {
    publishers {
        consoleParsing {
            globalRules('/var/lib/jenkins/logparser_error_on_roslaunch_failed')
            failBuildOnError()
        }
    }
  }
}

// MAIN CI JOBS
ci_distro.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the default ci jobs
    def srcsim_ci_job = job("srcsim-ci-default-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(srcsim_ci_job)
    // GPU label and parselog
    include_gpu_label(srcsim_ci_job, distro)
    include_parselog(srcsim_ci_job)

    srcsim_ci_job.with
    {
        scm {
          hg('https://bitbucket.org/osrf/srcsim') {
            branch('default')
            subdirectory('srcsim')
          }
        }

        triggers {
          scm('*/5 * * * *')
        }

        steps {
          shell("""#!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}

                /bin/bash -xe ./scripts/jenkins-scripts/docker/srcsim-compilation.bash
                """.stripIndent())
        }
     }
    // --------------------------------------------------------------
    // 2. Create the any job
    def srcsim_ci_any_job = job("srcsim-ci-pr_any-${distro}-${arch}")
    OSRFLinuxCompilationAny.create(srcsim_ci_any_job,
                                  'https://bitbucket.org/osrf/srcsim')
    // GPU label and parselog
    include_gpu_label(srcsim_ci_any_job, distro)
    include_parselog(srcsim_ci_any_job)

    srcsim_ci_any_job.with
    {
        steps {
          shell("""\
                export DISTRO=${distro}
                export ARCH=${arch}

                /bin/bash -xe ./scripts/jenkins-scripts/docker/srcsim-compilation.bash
                """.stripIndent())
        }
    }
  }
}

// OTHER DAILY CI JOBS
other_supported_distros.each { distro ->
  supported_arches.each { arch ->

    // --------------------------------------------------------------
    // 1. Create the other daily CI jobs
    def srcsim_ci_job = job("srcsim-ci-default-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(srcsim_ci_job)
    // GPU label and parselog
    include_gpu_label(srcsim_ci_job, distro)
    include_parselog(srcsim_ci_job)

    srcsim_ci_job.with
    {
        scm {
          hg('https://bitbucket.org/osrf/srcsim') {
            branch('default')
            subdirectory('srcsim')
          }
        }

        triggers {
          scm('@daily')
        }

        steps {
          shell("""#!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}

                /bin/bash -xe ./scripts/jenkins-scripts/docker/srcsim-compilation.bash
                """.stripIndent())
        }
     }
  } // end of arch
} // end of distro

// DAILY INSTALL TESTS
all_supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Install srcsim testing pkg testing
    def install_default_job = job("srcsim-install_pkg-${distro}-${arch}")
    OSRFLinuxInstall.create(install_default_job)
    // GPU label and parselog
    include_gpu_label(install_default_job, distro)
    include_parselog(install_default_job)

    install_default_job.with
    {
      triggers {
        cron('@daily')
      }

      steps {
        shell("""\
            #!/bin/bash -xe

            export DISTRO=${distro}
            export ARCH=${arch}
            export INSTALL_JOB_PKG=srcsim
            export INSTALL_JOB_REPOS=stable
            /bin/bash -xe ./scripts/jenkins-scripts/docker/srcsim-install-test-job.bash
            """.stripIndent())
      }
    } // end of with

    // 2. Install ihmc-ros-valkyrie testing pkg testing
    def ihmc_install_default_job = job("ihmc_valkyrie_ros-install_pkg-${distro}-${arch}")
    OSRFLinuxInstall.create(ihmc_install_default_job)
    // GPU label
    include_gpu_label(ihmc_install_default_job, distro)
    include_parselog(ihmc_install_default_job)

    ihmc_install_default_job.with
    {
      triggers {
        cron('@daily')
      }

      disabled()

      steps {
        shell("""\
            #!/bin/bash -xe

            export DISTRO=${distro}
            export ARCH=${arch}
            export INSTALL_JOB_PKG=ros-indigo-ihmc-valkyrie-ros
            export INSTALL_JOB_REPOS=stable
            /bin/bash -xe ./scripts/jenkins-scripts/docker/ihmc_valkyrie_ros-install-test-job.bash
            """.stripIndent())
        }
      } // end of with

    // 3. Run one-liner installation
    def install_one_liner_job = job("srcsim-install-one_liner-${distro}-${arch}")
    OSRFLinuxInstall.create(install_one_liner_job)
    // GPU label
    include_gpu_label(install_one_liner_job, distro)
    include_parselog(install_one_liner_job)

    install_one_liner_job.with
    {
       triggers {
         cron('@daily')
       }

       steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}

              /bin/bash -x ./scripts/jenkins-scripts/docker/srcsim-install_one_liner-test-job.bash
              """.stripIndent())
      }
    }
  }
}


// --------------------------------------------------------------
// srcsim package builder
def build_pkg_job = job("srcsim-debbuilder")

// Use the linux install as base
OSRFLinuxBuildPkg.create(build_pkg_job)

build_pkg_job.with
{
    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -x ./scripts/jenkins-scripts/docker/srcsim-debbuild.bash
            """.stripIndent())
    }
}

// --------------------------------------------------------------
// ihmc-valyrie-ros package builder
def ihmc_build_pkg = job("ihmc_valkyrie_ros-debbuilder")
OSRFLinuxBuildPkgBase.create(ihmc_build_pkg)

ihmc_build_pkg.with
{
  // TODO: convert into parameteres. Maybe in the OSRFLinuxBuildPkgBase class?
  def ihmc_distro = 'trusty'
  def ihmc_arch   = 'amd64'

  scm {
    git {
      remote {
        github('j-rivero/ihmc_valkyrie_ros-debian', 'https')
        branch('refs/heads/master')
      }

      extensions {
        cleanBeforeCheckout()
        relativeTargetDirectory('repo')
      }
    }
  }

  concurrentBuild(true)

  throttleConcurrentBuilds {
    maxPerNode(1)
    maxTotal(5)
  }

  steps {
    shell("""\
          #!/bin/bash -xe

          export ARCH=${ihmc_arch}
          export DISTRO=${ihmc_distro}
          export USE_ROS_REPO=true

          /bin/bash -xe ./scripts/jenkins-scripts/docker/ihmc_valkyrie_ros-debbuild.bash
          """.stripIndent())
  }

  publishers
  {
    publishers {
      archiveArtifacts('pkgs/*')

      /* temporary hosted in external server
      downstreamParameterized {
        trigger('repository_uploader_ng') {
          condition('SUCCESS')
          parameters {
            currentBuild()
            predefinedProp("PROJECT_NAME_TO_COPY_ARTIFACTS", "\${JOB_NAME}")
            predefinedProp("UPLOAD_TO_REPO", "stable")
            predefinedProp("PACKAGE_ALIAS" , "ihmc_valkyrie_ros")
            predefinedProp("DISTRO",         "${ihmc_distro}")
            predefinedProp("ARCH",           "${ihmc_arch}")
          }
        }
      } */
    }

    postBuildScripts {
      steps {
        shell("""\
          #!/bin/bash -xe

          [[ -d \${WORKSPACE}/repo ]] && sudo chown -R jenkins \${WORKSPACE}/repo
          """.stripIndent())
      }

      onlyIfBuildSucceeds(false)
      onlyIfBuildFails(false)
    }
  }
}

