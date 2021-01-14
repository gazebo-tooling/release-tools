import _configs_.*
import javaposse.jobdsl.dsl.Job

def sdformat_supported_branches = [ 'sdformat4', 'sdformat6' , 'sdformat9', 'sdformat10' ]
def sdformat_gz11_branches = [ 'sdformat9', 'sdformat10', 'master' ]
// nightly and prereleases
def extra_sdformat_debbuilder = [ 'sdformat11' ]

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
abi_branches = sdformat_supported_branches.collect { it -> get_sdformat_branch_name(it) }
abi_distro.each { distro ->
  supported_arches.each { arch ->
    abi_job_name = "sdformat-abichecker-any_to_any-ubuntu_auto-${arch}"
    def abi_job = job(abi_job_name)
    OSRFLinuxABIGitHub.create(abi_job)
    GenericAnyJobGitHub.create(abi_job, 'osrf/sdformat', abi_branches)
    abi_job.with
    {
      steps {
        shell("""\
              #!/bin/bash -xe
              wget https://raw.githubusercontent.com/osrf/bash-yaml/master/yaml.sh -O yaml.sh
              source yaml.sh

              create_variables \${WORKSPACE}/sdformat/bitbucket-pipelines.yml

              export DISTRO=${distro}

              if [[ -n \${image} ]]; then
                echo "Bitbucket pipeline.yml detected. Default DISTRO is ${distro}"
                export DISTRO=\$(echo \${image} | sed  's/ubuntu://')
              fi

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
[ Globals.get_gz11_ubuntu_distro() ].each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the master ci jobs
    def sdformat_ci_job = job("sdformat-ci-master-${distro}-${arch}")
    OSRFLinuxCompilation.create(sdformat_ci_job)
    OSRFGitHub.create(sdformat_ci_job, "osrf/sdformat")

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
    String sdf_repo = "osrf/sdformat"

    def sdformat_ci_any_job = job(ci_build_any_job_name_linux)
    OSRFLinuxCompilationAnyGitHub.create(sdformat_ci_any_job, sdf_repo)
    sdformat_ci_any_job.with
    {
      steps
      {
         conditionalSteps
         {
           condition
           {
             not {
               expression('${ENV, var="ghprbTargetBranch"}', 'master')
             }
           }
         }

         shell("""\
         #!/bin/bash -xe
         wget https://raw.githubusercontent.com/osrf/bash-yaml/master/yaml.sh -O yaml.sh
         source yaml.sh

         create_variables \${WORKSPACE}/sdformat/bitbucket-pipelines.yml

         export DISTRO=${ci_distro_str}

         if [[ -n \${image} ]]; then
           echo "Bitbucket pipeline.yml detected. Default DISTRO is ${ci_distro_str}"
           export DISTRO=\$(echo \${image} | sed  's/ubuntu://')
         fi

         export ARCH=${arch}
         /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-compilation.bash
         """.stripIndent())
       }
     }
  } // end of arch
} // end of distro

// OTHER CI SUPPORTED JOBS (master branch) @ SCM/DAILY
other_supported_distros.each { distro ->
  // master doesn't support xenial anymore
  if ("${distro}" == "xenial")
    return

  supported_arches.each { arch ->
    // ci_master job for the rest of arches / scm@daily
    def sdformat_ci_job = job("sdformat-ci-master-${distro}-${arch}")
    OSRFLinuxCompilation.create(sdformat_ci_job)
    OSRFGitHub.create(sdformat_ci_job, "osrf/sdformat")

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

// BRANCHES CI JOB @ SCM/DAILY
sdformat_supported_branches.each { branch ->
  ci_distro.each { distro ->
    // special check to modify ci_distro if the branch is part of gz11
    if (branch in sdformat_gz11_branches)
      distro = Globals.get_gz11_ubuntu_distro()

    supported_arches.each { arch ->
      // ci_master job for the rest of arches / scm@daily
      def sdformat_ci_job = job("sdformat-ci-${branch}-${distro}-${arch}")
      OSRFLinuxCompilation.create(sdformat_ci_job)
      OSRFGitHub.create(sdformat_ci_job, "osrf/sdformat",
                        get_sdformat_branch_name(branch))
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
} // end of distro
//

// EXPERIMENTAL ARCHES @ SCM/WEEKLY
[ Globals.get_gz11_ubuntu_distro() ].each { distro ->
  experimental_arches.each { arch ->
    def sdformat_ci_job = job("sdformat-ci-master-${distro}-${arch}")
    OSRFLinuxCompilation.create(sdformat_ci_job)
    OSRFGitHub.create(sdformat_ci_job, "osrf/sdformat")

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
sdformat_supported_branches.each { branch ->
  // special check to modify ci_distro if the branch is part of gz11
  if (branch in sdformat_gz11_branches)
    ref_distro = [ Globals.get_gz11_ubuntu_distro() ]
  else
    ref_distro = ci_distro

  ref_distro.each { distro ->
    supported_arches.each { arch ->
      // --------------------------------------------------------------
      def install_default_job = job("sdformat-install-${branch}_pkg-${distro}-${arch}")
      OSRFLinuxInstall.create(install_default_job)
      install_default_job.with
      {
         triggers {
           cron(Globals.CRON_EVERY_THREE_DAYS)
         }

         def dev_package = "lib${branch}-dev"

         steps {
          shell("""\
                #!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}
                export INSTALL_JOB_PKG=${dev_package}
                export GZDEV_PROJECT_NAME="${branch}"
                /bin/bash -x ./scripts/jenkins-scripts/docker/generic-install-test-job.bash
                """.stripIndent())
          }
      } // end of with
    } // end of arch
  } // end of distro
} // end of branch

// --------------------------------------------------------------
// DEBBUILD: linux package builder

all_debbuild_branches = sdformat_supported_branches + extra_sdformat_debbuilder
all_debbuild_branches.each { branch ->
  def build_pkg_job = job("${branch}-debbuilder")
  OSRFLinuxBuildPkg.create(build_pkg_job)

  extra_cmd_str = ""
  if (branch in sdformat_gz11_branches)
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
                                    "osrf/sdformat")
sdformat_brew_ci_any_job.with
{
    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -xe ./scripts/jenkins-scripts/sdformat-default-devel-homebrew-amd64.bash
            """.stripIndent())
    }
}

// 2. master in all branches @SCM/daily
all_branches = sdformat_supported_branches + 'master'
all_branches.each { branch ->
  def sdformat_brew_ci_job = job("sdformat-ci-${branch}-homebrew-amd64")
  OSRFBrewCompilation.create(sdformat_brew_ci_job)
  OSRFGitHub.create(sdformat_brew_ci_job, "osrf/sdformat",
                         get_sdformat_branch_name(branch))

  sdformat_brew_ci_job.with
  {
      triggers {
        scm('@daily')
      }

      // special check to modify ci_distro if the branch is part of gz11
      if (branch in sdformat_gz11_branches)
        label "osx_" + Globals.get_gz11_mac_distro()

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
  String ci_build_any_job_name_win7 = "sdformat-ci-pr_any-windows7-amd64"
  def sdformat_win_ci_any_job = job(ci_build_any_job_name_win7)
  OSRFWinCompilationAnyGitHub.create(sdformat_win_ci_any_job,
                                "osrf/sdformat")
  sdformat_win_ci_any_job.with
  {
      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/sdformat-default-devel-windows-amd64.bat"
              """.stripIndent())
      }
  }

// 2. master / @ SCM/Daily
all_branches = sdformat_supported_branches + 'master'
all_branches.each { branch ->
  def sdformat_win_ci_job = job("sdformat-ci-${branch}-windows7-amd64")
  OSRFWinCompilation.create(sdformat_win_ci_job)
  OSRFGitHub.create(sdformat_win_ci_job, "osrf/sdformat",
                    get_sdformat_branch_name(branch))
  sdformat_win_ci_job.with
  {
      triggers {
        scm('@daily')
      }

      if (branch == 'sdformat4')
        disabled()

      steps {
        batchFile("""\
              set USE_IGNITION_ZIP=FALSE
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
