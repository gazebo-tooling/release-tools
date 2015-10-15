import _configs_.OSRFLinuxCompilation
import _configs_.OSRFLinuxCompilationAny
import _configs_.OSRFBrewCompilation
import _configs_.OSRFBrewCompilationAny
import _configs_.OSRFWinCompilation
import _configs_.OSRFWinCompilationAny
import _configs_.OSRFLinuxInstall
import _configs_.OSRFLinuxBuildPkg
import javaposse.jobdsl.dsl.Job

// Main platform using for quick CI
def ci_distro = [ 'trusty' ]
// Other supported platform to be checked but no for quick
// CI integration.
def other_supported_distros = [ 'trusty','vivid' ]
def supported_arches = [ 'amd64' ]

def all_supported_distros = ci_distro + other_supported_distros

// MAIN CI JOBS (check every 5 minutes)
ci_distro.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the default ci jobs
    def ignition_ci_job = job("ignition_transport-ci-default-${distro}-${arch}")
    OSRFLinuxCompilation.create(ignition_ci_job)
    ignition_ci_job.with
    {
        scm {
          hg('http://bitbucket.org/ignitionrobotics/ign-transport') {
            branch('default')
            subdirectory('ignition-transport')
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

                /bin/bash -xe ./scripts/jenkins-scripts/docker/ign_transport-compilation.bash
                """.stripIndent())
        }
    }
    // --------------------------------------------------------------
    // 2. Create the any job
    def ignition_ci_any_job = job("ignition_transport-ci-pr_any-${distro}-${arch}")
    OSRFLinuxCompilationAny.create(ignition_ci_any_job,
                                  'http://bitbucket.org/ignitionrobotics/ign-transport')
    ignition_ci_any_job.with
    {
        steps {
          shell("""\
                export DISTRO=${distro}
                export ARCH=${arch}

                /bin/bash -xe ./scripts/jenkins-scripts/docker/ign_transport-compilation.bash
                """.stripIndent())
        }
    }
  }
}

// INSTALL PACKAGE ALL PLATFORMS / DAILY
all_supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    def install_default_job = job("ignition_transport-install-pkg-${distro}-${arch}")
    OSRFLinuxInstall.create(install_default_job)
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
              export INSTALL_JOB_PKG=libignition-transport0-dev
              export INSTALL_JOB_REPOS=stable
              /bin/bash -x ./scripts/jenkins-scripts/docker/generic-install-test-job.bash
              """.stripIndent())
      }
    }
  }
}

// OTHER CI SUPPORTED JOBS / DAILY
other_supported_distros.each { distro ->
   supported_arches.each { arch ->
    // --------------------------------------------------------------
    // ci_default job for the rest of arches / scm@daily
    def ignition_ci_job = job("ignition_transport-ci-default-${distro}-${arch}")
    OSRFLinuxCompilation.create(ignition_ci_job)
    ignition_ci_job.with
    {
        scm {
          hg('http://bitbucket.org/ignitionrobotics/ign-transport') {
            branch('default')
            subdirectory('ignition-transport')
          }
        }

        triggers {
          scm('@daily')
        }

        steps {
          shell("""\
                #!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}
                /bin/bash -xe ./scripts/jenkins-scripts/docker/ign_transport-compilation.bash
                """.stripIndent())
        }
    }
  }
}

// --------------------------------------------------------------
// DEBBUILD: ignition-transport package builder
def build_pkg_job = job("ign-transport-debbuilder")
OSRFLinuxBuildPkg.create(build_pkg_job)
build_pkg_job.with
{
    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-ignition-debbuild.bash
            """.stripIndent())
    }
}

// --------------------------------------------------------------
// BREW: CI jobs

// 1. any job
def ignition_brew_ci_any_job = job("ignition_transport-ci-pr_any-homebrew-amd64")
OSRFBrewCompilationAny.create(ignition_brew_ci_any_job,
                              'http://bitbucket.org/ignitionrobotics/ign-transport')
ignition_brew_ci_any_job.with
{
    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -xe ./scripts/jenkins-scripts/ign_transport-default-devel-homebrew-amd64.bash
            """.stripIndent())
    }
}

// 2. default
def ignition_brew_ci_job = job("ignition_transport-ci-default-homebrew-amd64")
OSRFBrewCompilation.create(ignition_brew_ci_job)
ignition_brew_ci_job.with
{
    scm {
      hg('http://bitbucket.org/ignitionrobotics/ign-transport') {
        branch('default')
        subdirectory('ignition-transport')
      }
    }

    triggers {
      scm('@daily')
    }

    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -xe ./scripts/jenkins-scripts/ign_transport-default-devel-homebrew-amd64.bash
            """.stripIndent())
    }
}

// --------------------------------------------------------------
// WINDOWS: CI job

// 1. any
def ignition_win_ci_any_job = job("ignition_transport-ci-pr_any-windows7-amd64")
OSRFWinCompilationAny.create(ignition_win_ci_any_job,
                              'http://bitbucket.org/ignitionrobotics/ign-transport')
ignition_win_ci_any_job.with
{
    steps {
      batchFile("""\
            call "./scripts/jenkins-scripts/ign_transport-default-devel-windows-amd64.bat"
            """.stripIndent())
    }
}

// 2. default
def ignition_win_ci_job = job("ignition_transport-ci-default-windows7-amd64")
OSRFWinCompilation.create(ignition_win_ci_job)
ignition_win_ci_job.with
{
    scm {
      hg('http://bitbucket.org/ignitionrobotics/ign-transport') {
        branch('default')
        subdirectory('ign-transport')
      }
    }

    triggers {
      scm('@daily')
    }

    steps {
      batchFile("""\
            call "./scripts/jenkins-scripts/ign_transport-default-devel-windows-amd64.bat"
            """.stripIndent())
    }
}
