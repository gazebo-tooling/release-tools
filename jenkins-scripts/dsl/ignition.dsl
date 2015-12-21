import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION PACKAGES
def ignition_software         = [ 'transport', 'math' ]
def ignition_transport_series = '0'
def ignition_math_series      = '2'

// Main platform using for quick CI
def ci_distro               = Globals.get_ci_distro()
def abi_distro              = Globals.get_abi_distro()
// Other supported platform to be checked but no for quick
// CI integration.
def other_supported_distros = Globals.get_other_supported_distros()
def supported_arches        = Globals.get_supported_arches()

def all_supported_distros = ci_distro + other_supported_distros

// Need to be used in ci_pr
String abi_job_name = ''

Globals.extra_emails = "caguero@osrfoundation.org"

// ABI Checker job
// Need to be the before ci-pr_any so the abi job name is defined
ignition_software.each { ign_sw ->
  abi_distro.each { distro ->
    supported_arches.each { arch ->
      abi_job_name = "ignition_${ign_sw}-abichecker-any_to_any-${distro}-${arch}"
      def abi_job = job(abi_job_name)
      OSRFLinuxABI.create(abi_job)
      abi_job.with
      {
        checkout_subdir = "ign-${ign_sw}"

        scm {
          hg("http://bitbucket.org/ignitionrobotics/ign-${ign_sw}") {
            branch('default')
            subdirectory(checkout_subdir)
          }
        }

        steps {
          shell("""\
                #!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}
                export ABI_JOB_SOFTWARE_NAME=${checkout_subdir}
                /bin/bash -xe ./scripts/jenkins-scripts/docker/ignition-abichecker.bash
                """.stripIndent())
        } // end of steps
      }  // end of with
    } // end of arch
  } // end of distro
} // end of ignition

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
              subdirectory("ign-${ign_sw}")
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
        steps 
        {
           conditionalSteps
           {
             condition
             {
               not {
                 expression('${ENV, var="DEST_BRANCH"}', 'default')
               }

               steps {
                 downstreamParameterized {
                   trigger("${abi_job_name}") {
                     parameters {
                       predefinedProp("ORIGIN_BRANCH", '$DEST_BRANCH')
                       predefinedProp("TARGET_BRANCH", '$SRC_BRANCH')
                     }
                   }
                 }
               }
             }
           }

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
 
         def dev_package = "libignition-${ign_sw}${ignition_transport_series}-dev"

         if ("${ign_sw}" == "math")
          dev_package = "libignition-${ign_sw}${ignition_math_series}-dev"

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
              subdirectory("ign-${ign_sw}")
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
          // in brew use ign-math to match OSRFBrewCompilationAny mechanism
          subdirectory("ign-${ign_sw}")
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
          // in win use ign-math to match OSRFWinCompilationAny mechanism
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
