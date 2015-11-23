import _configs_.*
import javaposse.jobdsl.dsl.Job

def sdformat_supported_branches = [ 'sdformat2', 'sdformat3', 'sdformat4' ]

// Main platform using for quick CI
def ci_distro = Globals.get_ci_distro()
// Other supported platform to be checked but no for quick
// CI integration.
def other_supported_distros = Globals.get_other_supported_distros()
def supported_arches = [ 'amd64' ]

def all_supported_distros = Globals.get_all_supported_distros()

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
                 trigger('sdformat-any_to_any-abichecker-vivid-amd64') {
                   parameters {
                     predefinedProp("SDFORMAT_ORIGIN_BRANCH", '${ENV, var="DEST_BRANCH"}')
                     predefinedProp("SDFORMAT_TARGET_BRANCH", '${ENV, var="SRC_BRANCH"}')
                   }
                 }
               }
             }
           }
         }

         shell("""\
         #!/bin/bash -xe

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
        String sdf_branch = branch.replace("ormat",'')

        if ("${branch}" == 'sdformat2')
           sdf_branch = 'sdf_2.3'

        scm {
          // The usual form using branch in the clousure does not work
          hg("http://bitbucket.org/osrf/sdformat", sdf_branch)
          {
            subdirectory("sdformat")
          }
        }

        triggers {
          scm('@daily')
        }

        steps {
          shell("""\
          #!/bin/bash -xe

          /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-compilation.bash
          """.stripIndent())
        }
      }
    } // end of arch
  } // end of distro
} // end of distro

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
      }
    } // end of arch
  } // end of distro
} // end of branch

// --------------------------------------------------------------
// DEBBUILD: linux package builder
sdformat_supported_branches.each { branch ->
  def build_pkg_job = job("${branch}-debbuilder")
  OSRFLinuxBuildPkg.create(build_pkg_job)

  build_pkg_job.with
  {
      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-sdformat-debbuild.bash
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
all_branches = sdformat_supported_branches + 'default'
all_branches.each { branch ->
  def sdformat_brew_ci_job = job("sdformat-ci-${branch}-homebrew-amd64")
  OSRFBrewCompilation.create(sdformat_brew_ci_job)

  sdformat_brew_ci_job.with
  {
      scm {
        hg("http://bitbucket.org/osrf/sdformat", branch)
        {
          subdirectory("sdformat")
        }
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
        batchFile("""\
              call "./scripts/jenkins-scripts/sdformat-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }
}
