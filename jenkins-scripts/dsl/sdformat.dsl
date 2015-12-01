import _configs_.*
import javaposse.jobdsl.dsl.Job

def sdformat_supported_branches = [ 'sdformat2', 'sdformat3' ]
def nightly_sdformat_branch = [ 'sdformat4' ]

// Main platform using for quick CI
def ci_distro               = Globals.get_ci_distro()
def abi_distro              = Globals.get_abi_distro()
def performance_box         = Globals.get_performance_box()
// Other supported platform to be checked but no for quick
// CI integration.
def other_supported_distros = Globals.get_other_supported_distros()
def all_supported_distros   = Globals.get_all_supported_distros()
def supported_arches        = Globals.get_supported_arches()
def experimental_arches     = Globals.get_experimental_arches()

// Need to be used in ci_pr
String abi_job_name = ''

// Helper function
String get_sdformat_branch_name(String full_branch_name)
{
  String sdf_branch = full_branch_name.replace("ormat",'')

  if ("${full_branch_name}" == 'sdformat2')
     sdf_branch = 'sdf_2.3'

  return sdf_branch
}

// ABI Checker job
// Need to be the before ci-pr_any so the abi job name is defined
abi_distro.each { distro ->
  supported_arches.each { arch ->
    abi_job_name = "sdformat-abichecker-any_to_any-${distro}-${arch}"
    def abi_job = job(abi_job_name)
    OSRFLinuxABI.create(abi_job)
    abi_job.with
    {
      scm
      {
        hg("http://bitbucket.org/osrf/sdformat") {
          branch('default')
          subdirectory("sdformat")
        }
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}
              /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-abichecker.bash
	      """.stripIndent())
      } // end of steps
    }  // end of with
  } // end of arch
} // end of distro

// MAIN CI JOBS @ SCM/5 min
ci_distro.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the default ci jobs
    def sdformat_ci_job = job("sdformat-ci-default-${distro}-${arch}")
    OSRFLinuxCompilation.create(sdformat_ci_job)
    sdformat_ci_job.with
    {
      scm {
        hg("http://bitbucket.org/osrf/sdformat") {
          branch('default')
          subdirectory("sdformat")
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
	      /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-compilation.bash
	      """.stripIndent())
      }
    }

    // --------------------------------------------------------------
    // 2. Create the any job
    def sdformat_ci_any_job = job("sdformat-ci-pr_any-${distro}-${arch}")
    OSRFLinuxCompilationAny.create(sdformat_ci_any_job,
				  "http://bitbucket.org/osrf/sdformat")
    sdformat_ci_any_job.with
    {
      parameters
      {
        stringParam('DEST_BRANCH','default',
                    'Destination branch where the pull request will be merged')
      }

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
         #!/bin/bash -xe

         export DISTRO=${distro}
         export ARCH=${arch}
         /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-compilation.bash
         """.stripIndent())
       }
     }
  } // end of arch
} // end of distro

// OTHER CI SUPPORTED JOBS (default branch) @ SCM/DAILY
other_supported_distros.each { distro ->
  supported_arches.each { arch ->
    // ci_default job for the rest of arches / scm@daily
    def sdformat_ci_job = job("sdformat-ci-default-${distro}-${arch}")
    OSRFLinuxCompilation.create(sdformat_ci_job)
    sdformat_ci_job.with
    {
      scm {
        hg("http://bitbucket.org/osrf/sdformat") {
          branch('default')
          subdirectory("sdformat")
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
        /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-compilation.bash
        """.stripIndent())
      }
    }
  } // end of arch
} // end of distro

// BRANCHES CI JOB @ SCM/DAILY
sdformat_supported_branches.each { branch ->
  ci_distro.each { distro ->
    supported_arches.each { arch ->
      // ci_default job for the rest of arches / scm@daily
      def sdformat_ci_job = job("sdformat-ci-${branch}-${distro}-${arch}")
      OSRFLinuxCompilation.create(sdformat_ci_job)
      sdformat_ci_job.with
      {
        scm
        {
          // The usual form using branch in the clousure does not work
          hg("http://bitbucket.org/osrf/sdformat",
             get_sdformat_branch_name(branch),
             { node -> node / subdir << "sdformat" })
        }

        triggers {
          scm('@daily')
        }

        steps {
          shell("""\
          #!/bin/bash -xe

  	  export DISTRO=${distro}
          export ARCH=${arch}
          /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-compilation.bash
          """.stripIndent())
        }
      }
    } // end of arch
  } // end of distro
} // end of distro
//

// EXPERIMENTAL ARCHES @ SCM/WEEKLY
ci_distro.each { distro ->
  experimental_arches.each { arch ->
    def sdformat_ci_job = job("sdformat-ci-default-${distro}-${arch}")
    OSRFLinuxCompilation.create(sdformat_ci_job)
    sdformat_ci_job.with
    {
      scm
      {
        hg("http://bitbucket.org/osrf/sdformat") {
          branch('default')
          subdirectory("sdformat")
        }
      }

      triggers {
        scm('@weekly')
      }

      steps {
        shell("""\
        #!/bin/bash -xe

        export DISTRO=${distro}
        export ARCH=${arch}
        /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-compilation.bash
        """.stripIndent())
      }
    }
  }
}

// INSTALL LINUX -DEV PACKAGES ALL PLATFORMS @ CRON/DAILY
sdformat_supported_branches.each { branch ->
  ci_distro.each { distro ->
    supported_arches.each { arch ->
      // --------------------------------------------------------------
      def install_default_job = job("sdformat-install-${branch}_pkg-${distro}-${arch}")
      OSRFLinuxInstall.create(install_default_job)
      install_default_job.with
      {
         triggers {
           cron('@daily')
         }

         def dev_package = "lib${branch}-dev"

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
      } // end of with
    } // end of arch
  } // end of distro
} // end of branch

// --------------------------------------------------------------
// PERFORMANCE: linux performance test
ci_distro.each { distro ->
  supported_arches.each { arch ->
    def performance_job = job("sdformat-performance-default-${distro}-${arch}")
    OSRFLinuxPerformance.create(performance_job)
    performance_job.with
    {
      label "${performance_box}"

      scm
      {
        hg("http://bitbucket.org/osrf/sdformat") {
          branch('default')
          subdirectory("sdformat")
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
              /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-compilation.bash
              """.stripIndent())
      } // end of steps
    } // end of with
  } // end of arch
} // end of distro

// --------------------------------------------------------------
// DEBBUILD: linux package builder

all_debbuild_branches = sdformat_supported_branches + nightly_sdformat_branch
all_debbuild_branches.each { branch ->
  def build_pkg_job = job("${branch}-debbuilder")
  OSRFLinuxBuildPkg.create(build_pkg_job)

  build_pkg_job.with
  {
      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-debbuild.bash
              """.stripIndent())
      }
  }
}

// --------------------------------------------------------------
// BREW: CI jobs

// 1. ANY job @ SCM/5min
def sdformat_brew_ci_any_job = job("sdformat-ci-pr_any-homebrew-amd64")
OSRFBrewCompilationAny.create(sdformat_brew_ci_any_job,
                              "http://bitbucket.org/osrf/sdformat")
sdformat_brew_ci_any_job.with
{
    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -xe ./scripts/jenkins-scripts/sdformat-default-devel-homebrew-amd64.bash
            """.stripIndent())
    }
}

// 2. default in all branches @SCM/daily
// No sdformat2 for brew
all_branches = sdformat_supported_branches + 'default' - 'sdformat2'
all_branches.each { branch ->
  def sdformat_brew_ci_job = job("sdformat-ci-${branch}-homebrew-amd64")
  OSRFBrewCompilation.create(sdformat_brew_ci_job)

  sdformat_brew_ci_job.with
  {
      scm {
        hg("http://bitbucket.org/osrf/sdformat",
           get_sdformat_branch_name(branch),
           { node -> node / subdir << "sdformat" })
      }

      triggers {
        scm('@daily')
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -xe ./scripts/jenkins-scripts/sdformat-default-devel-homebrew-amd64.bash
              """.stripIndent())
      }
  }
}

// --------------------------------------------------------------
// WINDOWS: CI job

// 1. any
  def sdformat_win_ci_any_job = job("sdformat-ci-pr_any-windows7-amd64")
  OSRFWinCompilationAny.create(sdformat_win_ci_any_job,
                                "http://bitbucket.org/osrf/sdformat")
  sdformat_win_ci_any_job.with
  {
      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/sdformat-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }

// 2. default / @ SCM/Daily
all_branches = sdformat_supported_branches + 'default'
all_branches.each { branch ->
  def sdformat_win_ci_job = job("sdformat-ci-${branch}-windows7-amd64")
  OSRFWinCompilation.create(sdformat_win_ci_job)

  sdformat_win_ci_job.with
  {
      scm
      {
        hg("http://bitbucket.org/osrf/sdformat",
           'default',
           { node -> node / subdir << "sdformat" })
      }

      triggers {
        scm('@daily')
      }

      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/sdformat-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }
}
