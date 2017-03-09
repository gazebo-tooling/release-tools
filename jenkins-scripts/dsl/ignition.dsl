import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION PACKAGES
ignition_software           = [ 'transport', 'math', 'msgs', 'common' ]
ignition_debbuild           = ignition_software + [ 'transport2', 'transport3', 'math3' ]
// no registered branches in ignition_branches means only series 0 or 1
ignition_branches           = [ transport : [ '2', '3' ],
                                math      : [ '2', '3' ]]
// Main platform using for quick CI
def ci_distro               = Globals.get_ci_distro()
def abi_distro              = Globals.get_abi_distro()
// Other supported platform to be checked but no for quick
// CI integration.
def other_supported_distros = Globals.get_other_supported_distros()
def supported_arches        = Globals.get_supported_arches()

def all_supported_distros = ci_distro + other_supported_distros

// Map needed to be used in ci_pr
abi_job_names = [:]

Globals.extra_emails = "caguero@osrfoundation.org"

String ci_distro_str = ci_distro[0]

// Map of lists to use in CIWorkflow
ci_pr_any_list = [:]
ignition_software.each { ign_sw ->
  def list_empty = []
  ci_pr_any_list[ign_sw] = list_empty
}

// return major versions supported or empty if just 0,1 series under
// -dev package.
ArrayList supported_branches(String ign_software)
{
   major_versions_registered = ignition_branches["${ign_software}"]

   if (major_versions_registered == null)
     return [ '' ]

   return major_versions_registered
}

// ABI Checker job
// Need to be the before ci-pr_any so the abi job name is defined
ignition_software.each { ign_sw ->
  abi_distro.each { distro ->
    supported_arches.each { arch ->
      abi_job_names[ign_sw] = "ignition_${ign_sw}-abichecker-any_to_any-${distro}-${arch}"
      def abi_job = job(abi_job_names[ign_sw])
      checkout_subdir = "ign-${ign_sw}"

      OSRFLinuxABI.create(abi_job)
      OSRFBitbucketHg.create(abi_job,
                            "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}/", 
                            "default", checkout_subdir)
      abi_job.with
      {
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
      // 1. Create the any job
      def ignition_ci_job_name = "ignition_${ign_sw}-ci-pr_any-${distro}-${arch}"
      def ignition_ci_any_job = job(ignition_ci_job_name)
      OSRFLinuxCompilationAny.create(ignition_ci_any_job,
                                    "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}")
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
                   trigger(abi_job_names[ign_sw]) {
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
                #!/bin/bash -xe
                export DISTRO=${distro}
                export ARCH=${arch}

                /bin/bash -xe ./scripts/jenkins-scripts/docker/ign_${ign_sw}-compilation.bash
                """.stripIndent())
        } // end of steps
      } // end of ci_any_job

      // add ci-pr_any to the list for CIWorkflow
      ci_pr_any_list[ign_sw] << ignition_ci_job_name
    }
  }
}

// INSTALL PACKAGE ALL PLATFORMS / DAILY
ignition_software.each { ign_sw ->
  all_supported_distros.each { distro ->
    supported_arches.each { arch ->
      supported_branches(ign_sw).each { major_version ->
        // --------------------------------------------------------------
        def install_default_job = job("ignition_${ign_sw}${major_version}-install-pkg-${distro}-${arch}")
        OSRFLinuxInstall.create(install_default_job)
        install_default_job.with
        {
           triggers {
             cron('@daily')
           }

           def dev_package = "libignition-${ign_sw}${major_version}-dev"
           
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
}

// OTHER CI SUPPORTED JOBS / DAILY
ignition_software.each { ign_sw ->
  other_supported_distros.each { distro ->
     supported_arches.each { arch ->
      // --------------------------------------------------------------
      // ci_default job for the rest of arches / scm@daily
      def ignition_ci_job = job("ignition_${ign_sw}-ci-default-${distro}-${arch}")
      OSRFLinuxCompilation.create(ignition_ci_job)
      OSRFBitbucketHg.create(ignition_ci_job,
                            "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}/", 
                            "default", "ign-${ign_sw}")

      ignition_ci_job.with
      {
          triggers {
            scm('@daily')
          }

          // msgs does not work on trusty 
          // https://bitbucket.org/ignitionrobotics/ign-msgs/issues/8
          if (("${ign_sw}" == "msgs") && ("${distro}" == "trusty"))
            disabled()

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
  supported_branches("${ign_sw}").each { major_version ->
    def build_pkg_job = job("ign-${ign_sw}${major_version}-debbuilder")
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
}

// --------------------------------------------------------------
// BREW: CI jobs

// 1. any job
ignition_software.each { ign_sw ->
  String ignition_brew_ci_any_job_name = "ignition_${ign_sw}-ci-pr_any-homebrew-amd64"
  def ignition_brew_ci_any_job = job(ignition_brew_ci_any_job_name)
  OSRFBrewCompilationAny.create(ignition_brew_ci_any_job,
                                "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}")
  ignition_brew_ci_any_job.with
  {
      steps {
        shell("""\
              #!/bin/bash -xe

              export HOMEBREW_SCRIPT="./scripts/jenkins-scripts/ign_${ign_sw}-default-devel-homebrew-amd64.bash"
              if [ -s "\$HOMEBREW_SCRIPT" ]
              then
                /bin/bash -xe "\$HOMEBREW_SCRIPT"
              else
                /bin/bash -xe "./scripts/jenkins-scripts/lib/project-default-devel-homebrew-amd64.bash" "ignition-${ign_sw}"
              fi
              """.stripIndent())
      }
  }

  // add ci-pr_any to the list for CIWorkflow
  ci_pr_any_list[ign_sw] << ignition_brew_ci_any_job_name

  // 2. default
  def ignition_brew_ci_job = job("ignition_${ign_sw}-ci-default-homebrew-amd64")
  OSRFBrewCompilation.create(ignition_brew_ci_job)
  OSRFBitbucketHg.create(ignition_brew_ci_job,
                            "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}/", 
                            "default", "ign-${ign_sw}")
  ignition_brew_ci_job.with
  {
      triggers {
        scm('@daily')
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              export HOMEBREW_SCRIPT="./scripts/jenkins-scripts/ign_${ign_sw}-default-devel-homebrew-amd64.bash"
              if [ -s "\$HOMEBREW_SCRIPT" ]
              then
                /bin/bash -xe "\$HOMEBREW_SCRIPT"
              else
                /bin/bash -xe "./scripts/jenkins-scripts/lib/project-default-devel-homebrew-amd64.bash" "ignition-${ign_sw}"
              fi
              """.stripIndent())
      }
  }
}

// --------------------------------------------------------------
// WINDOWS: CI job

// 1. any
ignition_software.each { ign_sw ->
  String ignition_win_ci_any_job_name = "ignition_${ign_sw}-ci-pr_any-windows7-amd64"
  def ignition_win_ci_any_job = job(ignition_win_ci_any_job_name)
  OSRFWinCompilationAny.create(ignition_win_ci_any_job,
                                "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}")
  ignition_win_ci_any_job.with
  {
      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/ign_${ign_sw}-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }

  // add ci-pr_any to the list for CIWorkflow
  ci_pr_any_list[ign_sw] << ignition_win_ci_any_job_name

  // 2. default
  def ignition_win_ci_job = job("ignition_${ign_sw}-ci-default-windows7-amd64")
  OSRFWinCompilation.create(ignition_win_ci_job)
  OSRFBitbucketHg.create(ignition_win_ci_job,
                            "https://bitbucket.org/ignitionrobotics/ign-${ign_sw}/", 
                            "default", "ign-${ign_sw}")
 
  ignition_win_ci_job.with
  {
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

// Main CI workflow
ignition_software.each { ign_sw ->
  def String ci_main_name = "ignition_${ign_sw}-ci-pr_any"
  def ign_ci_main = pipelineJob(ci_main_name)
  OSRFCIWorkFlowMultiAny.create(ign_ci_main, ci_pr_any_list[ign_sw])
}
