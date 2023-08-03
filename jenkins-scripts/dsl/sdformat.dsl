import _configs_.*
import javaposse.jobdsl.dsl.Job

def sdformat_supported_versions = [ 'sdformat6' , 'sdformat9', 'sdformat12', 'sdformat13' ]
def sdformat_gz11_versions = [ 'sdformat9', 'sdformat12', 'sdformat13', 'main' ]
// nightly and prereleases
def extra_sdformat_debbuilder = ['sdformat14']

// Main platform using for quick CI
def ci_distro               = Globals.get_ci_distro()
def abi_distro              = Globals.get_abi_distro()
// Other supported platform to be checked but no for quick
// CI integration.
def other_supported_distros = Globals.get_other_supported_distros()
def all_supported_distros   = Globals.get_all_supported_distros()
def supported_arches        = Globals.get_supported_arches()
def experimental_arches     = Globals.get_experimental_arches()

String ci_distro_str = ci_distro[0]
String ci_build_any_job_name_linux = "sdformat-ci-pr_any-ubuntu_auto-amd64"

// Need to be used in ci_pr
String abi_job_name = ''

// Helper function
String get_sdformat_branch_name(String full_branch_name)
{
  String sdf_branch = full_branch_name.replace("ormat",'')

  return sdf_branch
}

// ABI Checker job
// Need to be the before ci-pr_any so the abi job name is defined
abi_branches = sdformat_supported_versions.collect { it -> get_sdformat_branch_name(it) }
abi_distro.each { distro ->
  supported_arches.each { arch ->
    abi_job_name = "sdformat-abichecker-any_to_any-ubuntu_auto-${arch}"
    def abi_job = job(abi_job_name)
    OSRFLinuxABIGitHub.create(abi_job)
    GenericAnyJobGitHub.create(abi_job, 'gazebosim/sdformat', abi_branches)
    abi_job.with
    {
      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}

              export ARCH=${arch}
              export DEST_BRANCH=\${DEST_BRANCH:-\$ghprbTargetBranch}
              export SRC_BRANCH=\${SRC_BRANCH:-\$ghprbSourceBranch}
              export SRC_REPO=\${SRC_REPO:-\$ghprbAuthorRepoGitUrl}

              /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-abichecker.bash
	      """.stripIndent())
      } // end of steps
    }  // end of with
  } // end of arch
} // end of distro

// MAIN CI job
// CI JOBS @ SCM/5 min
ci_distro.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the main ci jobs
    def sdformat_ci_job = job("sdformat-ci-main-${distro}-${arch}")
    OSRFLinuxCompilation.create(sdformat_ci_job)
    OSRFGitHub.create(sdformat_ci_job, "gazebosim/sdformat", "main")

    sdformat_ci_job.with
    {
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
    String sdf_repo = "gazebosim/sdformat"

    def sdformat_ci_any_job = job(ci_build_any_job_name_linux)
    OSRFLinuxCompilationAnyGitHub.create(sdformat_ci_any_job, sdf_repo)
    sdformat_ci_any_job.with
    {
      steps
      {
         shell("""\
         #!/bin/bash -xe

         export DISTRO=${ci_distro_str}

         export ARCH=${arch}
         /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-compilation.bash
         """.stripIndent())
       }
     }
  } // end of arch
} // end of distro

// OTHER CI SUPPORTED JOBS (main branch) @ SCM/DAILY
other_supported_distros.each { distro ->
  supported_arches.each { arch ->
    // ci_main job for the rest of arches / scm@daily
    def sdformat_ci_job = job("sdformat-ci-main-${distro}-${arch}")
    OSRFLinuxCompilation.create(sdformat_ci_job)
    OSRFGitHub.create(sdformat_ci_job, "gazebosim/sdformat", "main")

    sdformat_ci_job.with
    {
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

void generate_asan_ci_job(sdformat_ci_job, version, distro, arch)
{
  generate_ci_job(sdformat_ci_job, version, distro, arch,
                  '-DGZ_SANITIZER=Address',
                  Globals.MAKETEST_SKIP_GZ,
                  'export ASAN_OPTIONS=check_initialization_order=true:strict_init_order=true')
}


void generate_ci_job(sdformat_ci_job, version, distro, arch,
                     extra_cmake = '', extra_test = '', extra_cmd = '')
{
  OSRFLinuxCompilation.create(sdformat_ci_job)
  OSRFGitHub.create(sdformat_ci_job, "gazebosim/sdformat",
                    get_sdformat_branch_name(version))
  sdformat_ci_job.with
  {
    steps {
      shell("""\
      #!/bin/bash -xe

      ${extra_cmd}
      export BUILDING_EXTRA_CMAKE_PARAMS="${extra_cmake}"
      export BUILDING_EXTRA_MAKETEST_PARAMS="${extra_test}"
      export DISTRO=${distro}
      export ARCH=${arch}
      /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-compilation.bash
      """.stripIndent())
    }
  }
}

// BRANCHES CI JOB @ SCM
sdformat_supported_versions.each { version ->
  ci_distro.each { distro ->
    supported_arches.each { arch ->
      // ci job for the rest of arches / scm@daily
      def sdformat_ci_job = job("sdformat-ci-${version}-${distro}-${arch}")
      generate_ci_job(sdformat_ci_job, version, distro, arch)
      sdformat_ci_job.with
      {
        triggers {
          scm('@daily')
        }
      }
      // ci_asan job for the rest of arches / scm@weekend
      def sdformat_ci_asan_job = job("sdformat-ci_asan-${version}-${distro}-${arch}")
      generate_asan_ci_job(sdformat_ci_asan_job, version, distro, arch)
      sdformat_ci_asan_job.with
      {
        triggers {
          scm(Globals.CRON_ON_WEEKEND)
        }
      }
    }
  }
}

// EXPERIMENTAL ARCHES @ SCM/WEEKLY
ci_distro.each { distro ->
  experimental_arches.each { arch ->
    def sdformat_ci_job = job("sdformat-ci-main-${distro}-${arch}")
    OSRFLinuxCompilation.create(sdformat_ci_job)
    OSRFGitHub.create(sdformat_ci_job, "gazebosim/sdformat", "main")

    sdformat_ci_job.with
    {
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
sdformat_supported_versions.each { version ->
  ci_distro.each { distro ->
    supported_arches.each { arch ->
      // --------------------------------------------------------------
      def install_default_job = job("sdformat-install-${version}_pkg-${distro}-${arch}")
      OSRFLinuxInstall.create(install_default_job)
      install_default_job.with
      {
         triggers {
           cron(Globals.CRON_EVERY_THREE_DAYS)
         }

         def dev_package = "lib${version}-dev"

         steps {
          shell("""\
                #!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}
                export INSTALL_JOB_PKG=${dev_package}
                export GZDEV_PROJECT_NAME="${version}"
                /bin/bash -x ./scripts/jenkins-scripts/docker/generic-install-test-job.bash
                """.stripIndent())
          }
      } // end of with
    } // end of arch
  } // end of distro
} // end of version

// --------------------------------------------------------------
// DEBBUILD: linux package builder

all_debbuild_versions = sdformat_supported_versions + extra_sdformat_debbuilder
all_debbuild_versions.each { version ->
  def build_pkg_job = job("${version}-debbuilder")
  OSRFLinuxBuildPkg.create(build_pkg_job)

  extra_cmd_str = ""
  if (version in sdformat_gz11_versions)
     extra_cmd_str = "export NEED_C17_COMPILER=true"

  build_pkg_job.with
  {
      steps {
        shell("""\
              #!/bin/bash -xe

              ${extra_cmd_str}
              export ENABLE_ROS=false
              /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-debbuild.bash
              """.stripIndent())
      }
  }
}

// --------------------------------------------------------------
// BREW: CI jobs

// 1. ANY job @ SCM/5min
String ci_build_any_job_name_brew = "sdformat-ci-pr_any-homebrew-amd64"
def sdformat_brew_ci_any_job = job(ci_build_any_job_name_brew)
OSRFBrewCompilationAnyGitHub.create(sdformat_brew_ci_any_job,
                                    "gazebosim/sdformat")
sdformat_brew_ci_any_job.with
{
    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -xe ./scripts/jenkins-scripts/sdformat-default-devel-homebrew-amd64.bash
            """.stripIndent())
    }
}

// 2. main in all branches @SCM/daily
all_versions = sdformat_supported_versions + 'main'
all_versions.each { version ->
  def sdformat_brew_ci_job = job("sdformat-ci-${version}-homebrew-amd64")
  OSRFBrewCompilation.create(sdformat_brew_ci_job)
  OSRFGitHub.create(sdformat_brew_ci_job, "gazebosim/sdformat",
                         get_sdformat_branch_name(version))

  sdformat_brew_ci_job.with
  {
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

// 3. install jobs to test bottles
sdformat_supported_versions.each { version ->
  def install_default_job = job("${version}-install_bottle-homebrew-amd64")
  OSRFBrewInstall.create(install_default_job)

  install_default_job.with
  {
    triggers {
      cron('@daily')
    }

    steps {
     shell("""\
           #!/bin/bash -xe

           /bin/bash -x ./scripts/jenkins-scripts/lib/project-install-homebrew.bash ${version}
           """.stripIndent())
    }

    publishers
    {
       configure { project ->
         project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
            unstableOnWarning true
            failBuildOnError false
            parsingRulesPath('/var/lib/jenkins/logparser_warn_on_mark_unstable')
          }
       }
    }
  }
}

// --------------------------------------------------------------
// WINDOWS: CI job

// 1. any
  String ci_build_any_job_name_win7 = "sdformat-pr-win"
  def sdformat_win_ci_any_job = job(ci_build_any_job_name_win7)
  OSRFWinCompilationAnyGitHub.create(sdformat_win_ci_any_job,
                                "gazebosim/sdformat")
  sdformat_win_ci_any_job.with
  {
      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/sdformat-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }

// 2. main / @ SCM/Daily
all_versions = sdformat_supported_versions + 'main'
all_versions.each { version ->
  // Use replace to get branch names to sync with ignition packages and the
  // format of $branch-$version
  def sdformat_win_ci_job = job("sdformat-sdf-" + version.replace('sdformat','') + "-win")
  OSRFWinCompilation.create(sdformat_win_ci_job)
  OSRFGitHub.create(sdformat_win_ci_job, "gazebosim/sdformat",
                    get_sdformat_branch_name(version))
  sdformat_win_ci_job.with
  {
      triggers {
        scm('@daily')
      }

      steps {
        batchFile("""\
              set USE_GZ_ZIP=FALSE
              call "./scripts/jenkins-scripts/sdformat-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }
}

// Create the manual all-platforms jobs
def sdformat_ci_main = pipelineJob("sdformat-ci-manual_any")
OSRFCIWorkFlowMultiAnyGitHub.create(sdformat_ci_main,
                                    [ci_build_any_job_name_linux,
                                     ci_build_any_job_name_brew,
                                     ci_build_any_job_name_win7])
