import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION PACKAGES
def ignition_software = [ 'transport', 'math' ]

// Main platform using for quick CI
def ci_distro = [ 'trusty' ]
// Other supported platform to be checked but no for quick
// CI integration.
def other_supported_distros = [ 'vivid' ]
def supported_arches = [ 'amd64' ]

def all_supported_distros = ci_distro + other_supported_distros

Globals.extra_emails = "caguero@osrfoundation.org"

// MAIN CI JOBS (check every 5 minutes)
ignition_software.each { ign_sw ->
  ci_distro.each { distro ->
    supported_arches.each { arch ->
      // --------------------------------------------------------------
      // 1. Create the default ci jobs
      def ignition_ci_job = job("ignition_${ign_sw}-ci-default-${distro}-${arch}")
      OSRFLinuxCompilation.create(ignition_ci_job)
      ignition_ci_job.with
      {
          scm {
            hg("http://bitbucket.org/ignitionrobotics/ign-${ign_sw}") {
              branch('default')
              subdirectory("ignition-${ign_sw}")
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

                  /bin/bash -xe ./scripts/jenkins-scripts/docker/ign_${ign_sw}-compilation.bash
                  """.stripIndent())
          }
      }
      // --------------------------------------------------------------
      // 2. Create the any job
      def ignition_ci_any_job = job("ignition_${ign_sw}-ci-pr_any-${distro}-${arch}")
      OSRFLinuxCompilationAny.create(ignition_ci_any_job,
                                    "http://bitbucket.org/ignitionrobotics/ign-${ign_sw}")
      ignition_ci_any_job.with
      {
          steps {
            shell("""\
                  export DISTRO=${distro}
                  export ARCH=${arch}

                  /bin/bash -xe ./scripts/jenkins-scripts/docker/ign_${ign_sw}-compilation.bash
                  """.stripIndent())
          }
      }
    }
  }
}

// INSTALL PACKAGE ALL PLATFORMS / DAILY
ignition_software.each { ign_sw ->
  all_supported_distros.each { distro ->
    supported_arches.each { arch ->
      // --------------------------------------------------------------
      def install_default_job = job("ignition_${ign_sw}-install-pkg-${distro}-${arch}")
      OSRFLinuxInstall.create(install_default_job)
      install_default_job.with
      {
         triggers {
           cron('@daily')
         }
 
         def dev_package = "libignition-${ign_sw}0-dev"

         if ("${ign_sw}" == "math")
          dev_package = "libignition-${ign_sw}-dev"

         steps {
          shell("""\
                #!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}
                export INSTALL_JOB_PKG=${dev_package}
                export INSTALL_JOB_REPOS=stable
                /bin/bash -x ./scripts/jenkins-scripts/docker/generic-install-test-job.bash
                """.stripIndent())
        }
      }
    }
  }
}

// OTHER CI SUPPORTED JOBS / DAILY
ignition_software.each { ign_sw ->
  other_supported_distros.each { distro ->
     supported_arches.each { arch ->
      // --------------------------------------------------------------
      // ci_default job for the rest of arches / scm@daily
      def ignition_ci_job = job("ignition_${ign_sw}-ci-default-${distro}-${arch}")
      OSRFLinuxCompilation.create(ignition_ci_job)
      ignition_ci_job.with
      {
          scm {
            hg("http://bitbucket.org/ignitionrobotics/ign-${ign_sw}") {
              branch('default')
              subdirectory("ignition-${ign_sw}")
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
                  /bin/bash -xe ./scripts/jenkins-scripts/docker/ign_${ign_sw}-compilation.bash
                  """.stripIndent())
          }
      }
    }
  }
}

// --------------------------------------------------------------
// DEBBUILD: linux package builder
ignition_software.each { ign_sw ->
  def build_pkg_job = job("ign-${ign_sw}-debbuilder")
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
}

// --------------------------------------------------------------
// BREW: CI jobs

// 1. any job
ignition_software.each { ign_sw ->
  def ignition_brew_ci_any_job = job("ignition_${ign_sw}-ci-pr_any-homebrew-amd64")
  OSRFBrewCompilationAny.create(ignition_brew_ci_any_job,
                                "http://bitbucket.org/ignitionrobotics/ign-${ign_sw}")
  ignition_brew_ci_any_job.with
  {
      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -xe ./scripts/jenkins-scripts/ign_${ign_sw}-default-devel-homebrew-amd64.bash
              """.stripIndent())
      }
  }
}

// 2. default
ignition_software.each { ign_sw ->
  def ignition_brew_ci_job = job("ignition_${ign_sw}-ci-default-homebrew-amd64")
  OSRFBrewCompilation.create(ignition_brew_ci_job)

  ignition_brew_ci_job.with
  {
      scm {
        hg("http://bitbucket.org/ignitionrobotics/ign-${ign_sw}") {
          branch('default')
          subdirectory("ignition-${ign_sw}")
        }
      }

      triggers {
        scm('@daily')
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -xe ./scripts/jenkins-scripts/ign_${ign_sw}-default-devel-homebrew-amd64.bash
              """.stripIndent())
      }
  }
}

// --------------------------------------------------------------
// WINDOWS: CI job

// 1. any
ignition_software.each { ign_sw ->
  def ignition_win_ci_any_job = job("ignition_${ign_sw}-ci-pr_any-windows7-amd64")
  OSRFWinCompilationAny.create(ignition_win_ci_any_job,
                                "http://bitbucket.org/ignitionrobotics/ign-${ign_sw}")
  ignition_win_ci_any_job.with
  {
      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/ign_${ign_sw}-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }
}

// 2. default
ignition_software.each { ign_sw ->
  def ignition_win_ci_job = job("ignition_${ign_sw}-ci-default-windows7-amd64")
  OSRFWinCompilation.create(ignition_win_ci_job)

  ignition_win_ci_job.with
  {
      scm {
        hg("http://bitbucket.org/ignitionrobotics/ign-${ign_sw}") {
          branch('default')
          subdirectory("ign-${ign_sw}")
        }
      }

      triggers {
        scm('@daily')
      }

      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/ign_${ign_sw}-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }
}
