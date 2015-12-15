import _configs_.*
import javaposse.jobdsl.dsl.Job

def gazebo_supported_branches = [ 'gazebo_2.2', 'gazebo_4.1', 'gazebo5', 'gazebo6' ]
def nightly_gazebo_branch = [ 'gazebo7' ]

// Main platform using for quick CI
def ci_distro               = Globals.get_ci_distro()
def ci_gpu                  = Globals.get_ci_gpu()
def abi_distro              = Globals.get_abi_distro()
def performance_box         = Globals.get_performance_box()
// Other supported platform to be checked but no for quick
// CI integration.
def other_supported_distros = Globals.get_other_supported_distros()
def all_supported_distros   = Globals.get_all_supported_distros()
def supported_arches        = Globals.get_supported_arches()
def experimental_arches     = Globals.get_experimental_arches()
def all_supported_gpus      = Globals.get_all_supported_gpus()

// Need to be used in ci_pr
String abi_job_name = ''

// ABI Checker job
// Need to be the before ci-pr_any so the abi job name is defined
abi_distro.each { distro ->
  supported_arches.each { arch ->
    abi_job_name = "gazebo-abichecker-any_to_any-${distro}-${arch}"
    def abi_job = job(abi_job_name)
    OSRFLinuxABI.create(abi_job)
    abi_job.with
    {
      scm
      {
        hg("http://bitbucket.org/osrf/gazebo") {
          branch('default')
          subdirectory("gazebo")
        }
      }

      label ci_gpu

      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}
              export GPU_SUPPORT_NEEDED=true
              /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-abichecker.bash
	      """.stripIndent())
      } // end of steps
    }  // end of with
  } // end of arch
} // end of distro

// MAIN CI JOBS @ SCM/5 min
ci_gpu_include_gpu_none = ci_gpu + [ 'none' ]
ci_distro.each { distro ->
  ci_gpu_include_gpu_none.each { gpu ->
    supported_arches.each { arch ->     
      // --------------------------------------------------------------
      // 1. Create the any job
      def gazebo_ci_any_job = job("gazebo-ci-pr_any-${distro}-${arch}-gpu-${gpu}")
      OSRFLinuxCompilationAny.create(gazebo_ci_any_job,
                                    "http://bitbucket.org/osrf/gazebo")
      gazebo_ci_any_job.with
      {
        parameters
        {
          stringParam('DEST_BRANCH','default',
                      'Destination branch where the pull request will be merged')
        }
        
        label ci_gpu

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

           String gpu_needed = 'true'
           if (gpu == 'none') {
              gpu_needed = 'false'
           } 

           shell("""\
           #!/bin/bash -xe

           export DISTRO=${distro}
           export ARCH=${arch}
           export GPU_SUPPORT_NEEDED=${gpu_needed}
           /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-compilation.bash
           """.stripIndent())
         }
      }

      // --------------------------------------------------------------
      // 2. Create the default ci jobs
      def gazebo_ci_job = job("gazebo-ci-default-${distro}-${arch}-gpu-${gpu}")
      OSRFLinuxCompilation.create(gazebo_ci_job)
      gazebo_ci_job.with
      {
        scm {
          hg("http://bitbucket.org/osrf/gazebo") {
            branch('default')
            subdirectory("gazebo")
          }
        }
      
        label ci_gpu

        triggers {
          scm('*/5 * * * *')
        }

        steps {
          shell("""\
                #!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}
                export GPU_SUPPORT_NEEDED=true
                /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-compilation.bash
                """.stripIndent())
        }
      }
    } // end of gpu
  } // end of arch
} // end of distro


// OTHER CI SUPPORTED JOBS (default branch) @ SCM/DAILY
other_supported_distros.each { distro ->
  supported_arches.each { arch ->

    // get the supported gpus by distro
    gpus = Globals.gpu_by_distro[distro]
    if (gpus == null)
      gpus = [ 'none' ]

    gpus.each { gpu ->
      // ci_default job for the rest of arches / scm@daily
      def gazebo_ci_job = job("gazebo-ci-default-${distro}-${arch}-gpu-${gpu}")
      OSRFLinuxCompilation.create(gazebo_ci_job)
      gazebo_ci_job.with
      {
        scm {
          hg("http://bitbucket.org/osrf/gazebo") {
            branch('default')
            subdirectory("gazebo")
          }
        }

        triggers {
          scm('@daily')
        }

        String gpu_needed = 'true'
        if (gpu == 'none') {
          gpu_needed = 'false'
        } 

        steps {
          shell("""\
          #!/bin/bash -xe

          export DISTRO=${distro}
          export ARCH=${arch}
          export GPU_SUPPORT_NEEDED=${gpu_needed}
          /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-compilation.bash
          """.stripIndent())
        }
      }
    } // end of gpus
  } // end of arch
} // end of distro

// BRANCHES CI JOB @ SCM/DAILY
gazebo_supported_branches.each { branch ->
  ci_distro.each { distro ->
    supported_arches.each { arch ->
      ci_gpu.each { gpu ->
        // ci_default job for the rest of arches / scm@daily
        def gazebo_ci_job = job("gazebo-ci-${branch}-${distro}-${arch}-gpu-${gpu}")
        OSRFLinuxCompilation.create(gazebo_ci_job)
        gazebo_ci_job.with
        {
          scm
          {
            // The usual form using branch in the clousure does not work
            hg("http://bitbucket.org/osrf/gazebo",
               get_gazebo_branch_name(branch),
               { node -> node / subdir << "gazebo" })
          }

          triggers {
            scm('@daily')
          }

          steps {
            shell("""\
            #!/bin/bash -xe

            export DISTRO=${distro}
            export ARCH=${arch}
            export GPU_SUPPORT_NEEDED=true
            /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-compilation.bash
            """.stripIndent())
          }
        }
      } // end of gpu
    } // end of arch
  } // end of distro
} // end of branch

// EXPERIMENTAL ARCHES @ SCM/WEEKLY
ci_distro.each { distro ->
  experimental_arches.each { arch ->
    def gazebo_ci_job = job("gazebo-ci-default-${distro}-${arch}-gpu-none")
    OSRFLinuxCompilation.create(gazebo_ci_job)
    gazebo_ci_job.with
    {
      scm
      {
        hg("http://bitbucket.org/osrf/gazebo") {
          branch('default')
          subdirectory("gazebo")
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
        export GPU_SUPPORT_NEEDED=false
        /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-compilation.bash
        """.stripIndent())
      }
    }
  }
}

// INSTALL LINUX -DEV PACKAGES ALL PLATFORMS @ CRON/DAILY
gazebo_supported_branches.each { branch ->
  ci_distro.each { distro ->
    supported_arches.each { arch ->
      // --------------------------------------------------------------
      def install_default_job = job("gazebo-install-${branch}_pkg-${distro}-${arch}")
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
    def performance_job = job("gazebo-performance-default-${distro}-${arch}")
    OSRFLinuxPerformance.create(performance_job)
    performance_job.with
    {
      label "${performance_box}"

      scm
      {
        hg("http://bitbucket.org/osrf/gazebo") {
          branch('default')
          subdirectory("gazebo")
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
              export GPU_SUPPORT_NEEDED=true
              /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-compilation.bash
              """.stripIndent())
      } // end of steps
    } // end of with
  } // end of arch
} // end of distro

// --------------------------------------------------------------
// DEBBUILD: linux package builder

all_debbuild_branches = gazebo_supported_branches + nightly_gazebo_branch
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
def gazebo_brew_ci_any_job = job("gazebo-ci-pr_any-homebrew-amd64")
OSRFBrewCompilationAny.create(gazebo_brew_ci_any_job,
                              "http://bitbucket.org/osrf/gazebo")
gazebo_brew_ci_any_job.with
{
    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -xe ./scripts/jenkins-scripts/gazebo-default-devel-homebrew-amd64.bash
            """.stripIndent())
    }
}

// 2. default in all branches @SCM/daily
// No gazebo2 for brew
all_branches = gazebo_supported_branches + 'default' - 'gazebo2'
all_branches.each { branch ->
  def gazebo_brew_ci_job = job("gazebo-ci-${branch}-homebrew-amd64")
  OSRFBrewCompilation.create(gazebo_brew_ci_job)

  gazebo_brew_ci_job.with
  {
      scm {
        hg("http://bitbucket.org/osrf/gazebo",
           get_gazebo_branch_name(branch),
           { node -> node / subdir << "gazebo" })
      }

      triggers {
        scm('@daily')
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -xe ./scripts/jenkins-scripts/gazebo-default-devel-homebrew-amd64.bash
              """.stripIndent())
      }
  }
}

// --------------------------------------------------------------
// WINDOWS: CI job

// 1. any
  def gazebo_win_ci_any_job = job("gazebo-ci-pr_any-windows7-amd64")
  OSRFWinCompilationAny.create(gazebo_win_ci_any_job,
                                "http://bitbucket.org/osrf/gazebo")
  gazebo_win_ci_any_job.with
  {
      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/gazebo-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }

// 2. default / @ SCM/Daily
all_branches = gazebo_supported_branches + 'default'
all_branches.each { branch ->
  def gazebo_win_ci_job = job("gazebo-ci-${branch}-windows7-amd64")
  OSRFWinCompilation.create(gazebo_win_ci_job)

  gazebo_win_ci_job.with
  {
      scm
      {
        hg("http://bitbucket.org/osrf/gazebo",
           'default',
           { node -> node / subdir << "gazebo" })
      }

      triggers {
        scm('@daily')
      }

      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/gazebo-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }
}
