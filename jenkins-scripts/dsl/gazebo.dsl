import _configs_.*
import javaposse.jobdsl.dsl.Job

def gazebo_supported_branches = [ 'gazebo9', 'gazebo11' ]
def gazebo_supported_build_types = [ 'Release', 'Debug', 'Coverage' ]
// testing official packages without osrf repo
def ubuntu_official_packages_distros = [ 'bionic' : 'gazebo9',
                                         'focal'  : 'gazebo9',
                                         'jammy'  : 'gazebo11']
// Main platform using for quick CI
// Using bionic until https://github.com/osrf/gazebo/issues/3151 is fixed
def ci_distro               = [ 'bionic' ]
def ci_gpu                  = Globals.get_ci_gpu()
def abi_distro              = Globals.get_abi_distro()
// Other supported platform to be checked but no for quick
// CI integration.
def other_supported_distros = Globals.get_other_supported_distros()
def all_supported_distros   = Globals.get_all_supported_distros()
def supported_arches        = Globals.get_supported_arches()
def experimental_arches     = Globals.get_experimental_arches()
def all_supported_gpus      = Globals.get_all_supported_gpus()

def DISABLE_TESTS = false
def DISABLE_GITHUB_INTEGRATION = false

String ci_distro_str = ci_distro[0]
String ci_gpu_str = ci_gpu[0]
String ci_build_any_job_name_linux = "gazebo-ci-pr_any-ubuntu_auto-amd64-gpu-${ci_gpu_str}"

// Need to be used in ci_pr
String abi_job_name = ''

boolean is_watched_by_buildcop(branch, distro = 'bionic', gpu = 'nvidia')

{
  if (branch == 'master' || branch == 'gazebo9' || branch == 'gazebo11')
    return true

  return false
}

void generate_install_job(Job job, gz_branch, distro, arch, use_osrf_repos = false)
{
  job.with
  {
    triggers {
      cron(Globals.CRON_EVERY_THREE_DAYS)
    }

    // Branch is exactly in the form of gazeboN
    def dev_packages = "lib${gz_branch}-dev ${gz_branch}"
    // Ubuntu Jammy has no versioned gazebo package
    if (distro == 'jammy')
      dev_packages = "libgazebo-dev gazebo"

    def gzdev_str = ""
    if (use_osrf_repos)
        gzdev_str = "export GZDEV_PROJECT_NAME=${gz_branch}"

    // Need gpu for running the runtime test
    label "gpu-reliable"

    steps {
      shell("""\
          #!/bin/bash -xe

          export DISTRO=${distro}
          export ARCH=${arch}
          export INSTALL_JOB_PKG=\"${dev_packages}\"
          ${gzdev_str}

          /bin/bash -x ./scripts/jenkins-scripts/docker/gazebo-install-test-job.bash
          """.stripIndent())
    }

    publishers {
      consoleParsing {
        projectRules('scripts/jenkins-scripts/parser_rules/gazebo_err.parser')
        failBuildOnError(true)
      }
    }
  }
}

// ABI Checker job
// Need to be the before ci-pr_any so the abi job name is defined
abi_distro.each { distro ->
  supported_arches.each { arch ->
    abi_job_name = "gazebo-abichecker-any_to_any-ubuntu_auto-${arch}"
    def abi_job = job(abi_job_name)
    OSRFLinuxABIGitHub.create(abi_job)
    GenericAnyJobGitHub.create(abi_job, 'osrf/gazebo', gazebo_supported_branches)
    abi_job.with
    {
      label "large-memory"

      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}

              export ARCH=${arch}
              export DEST_BRANCH=\${DEST_BRANCH:-\$ghprbTargetBranch}
              export SRC_BRANCH=\${SRC_BRANCH:-\$ghprbSourceBranch}
              export SRC_REPO=\${SRC_REPO:-\$ghprbAuthorRepoGitUrl}

              /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-abichecker.bash
	      """.stripIndent())
      } // end of steps
    }  // end of with
  } // end of arch
} // end of distro

// CI JOBS @ SCM/5 min
ci_distro.each { distro ->
  ci_gpu.each { gpu ->
    supported_arches.each { arch ->
      // --------------------------------------------------------------
      // 1. Create the any job
      def gazebo_ci_any_job_name = "gazebo-ci-pr_any-ubuntu_auto-${arch}-gpu-${gpu}"
      def gazebo_ci_any_job      = job(gazebo_ci_any_job_name)
      OSRFLinuxCompilationAnyGitHub.create(gazebo_ci_any_job, "osrf/gazebo")
      gazebo_ci_any_job.with
      {
        label "gpu-reliable && large-memory"

        steps
        {
           shell("""\
           #!/bin/bash -xe

           export DISTRO=${distro}

           export ARCH=${arch}
           export GPU_SUPPORT_NEEDED=true
           /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-compilation.bash
           """.stripIndent())
         }
      }

      // reset build cop email in global list of mails
      Globals.extra_emails = ''
    } // end of gpu
  } // end of arch
} // end of distro

// OTHER CI SUPPORTED JOBS (gazebo11 branch) @ SCM/DAILY
other_supported_distros.each { distro ->
  supported_arches.each { arch ->

    // get the supported gpus by distro
    gpus = Globals.gpu_by_distro[distro]
    if (gpus == null)
      gpus = [ 'none' ]

    gpus.each { gpu ->
      // gazebo11 job for the rest of arches / scm@daily
      def gazebo_ci_job = job("gazebo-ci-gazebo11-${distro}-${arch}-gpu-${gpu}")
      OSRFLinuxCompilation.create(gazebo_ci_job)
      OSRFGitHub.create(gazebo_ci_job, "osrf/gazebo")

      gazebo_ci_job.with
      {

        // gazebo builds require a powerful node not to take too long and
        // block backup for hours
        label "gpu-reliable && large-memory"

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

// sdformat and ignition dependencies
ci_distro.each { distro ->
  supported_arches.each { arch ->
    ci_gpu.each { gpu ->
      def multi_any_job = job("gazebo-ci-pr_any+sdformat_any+ign_any-${distro}-${arch}-gpu-${gpu}")
      OSRFLinuxCompilationAnyGitHub.create(multi_any_job, "osrf/gazebo", true, true, [], DISABLE_GITHUB_INTEGRATION)
      multi_any_job.with
      {
        parameters
        {
          stringParam('SDFORMAT_BRANCH', 'main', 'sdformat branch to use')
          stringParam('IGN_MATH_BRANCH', 'main', 'ignition math branch to use')
          stringParam('IGN_TRANSPORT_BRANCH', 'main', 'ignition transport branch to use')
        }

        label "gpu-reliable"

        steps {
            shell("""\
            #!/bin/bash -xe

            export DISTRO=${distro}
            export ARCH=${arch}
            export GPU_SUPPORT_NEEDED=true
            export GAZEBO_BUILD_SDFORMAT=true
            export GAZEBO_BUILD_IGN_MATH=true
            export GAZEBO_BUILD_IGN_TRANSPORT=true
            /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-compilation.bash
            """.stripIndent())
        }
      }
    }
  }
}

// BRANCHES CI JOB @ SCM/DAILY
gazebo_supported_branches.each { branch ->
  distros = ci_distro
  distros.each { distro ->
    supported_arches.each { arch ->
      ci_gpu.each { gpu ->
        // ci_default job for the rest of arches / scm@daily
        def gazebo_ci_job = job("gazebo-ci-${branch}-${distro}-${arch}-gpu-${gpu}")
        // note that we are already using the CI reference GPU and distro, no
        // need to check for build_cop email
        if (is_watched_by_buildcop(branch, distro, gpu))
          Globals.extra_emails = Globals.build_cop_email
        OSRFLinuxCompilation.create(gazebo_ci_job)
        OSRFGitHub.create(gazebo_ci_job, "osrf/gazebo", branch)

        gazebo_ci_job.with
        {
          label "gpu-reliable && large-memory"

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

        // reset build cop email in global list of mails
        Globals.extra_emails = ""
      } // end of gpu
    } // end of arch
  } // end of distro
} // end of branch

// EXPERIMENTAL ARCHES @ SCM/WEEKLY
ci_distro.each { distro ->
  experimental_arches.each { arch ->
    def gazebo_ci_job = job("gazebo-ci-gazebo11-${distro}-${arch}-gpu-none")
    OSRFLinuxCompilation.create(gazebo_ci_job)
    OSRFGitHub.create(gazebo_ci_job, "osrf/gazebo", "gazebo11")

    gazebo_ci_job.with
    {
      label "gpu-reliable && large-memory"

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

// COVERAGE TYPE @ SCM/DAILY
ci_distro.each { distro ->
  supported_arches.each { arch ->
    ci_gpu.each { gpu ->
      def gazebo_ci_job = job("gazebo-ci-coverage-${distro}-${arch}-gpu-${gpu}")
      OSRFLinuxCompilation.create(gazebo_ci_job)
      OSRFGitHub.create(gazebo_ci_job, "osrf/gazebo", "gazebo11")
      gazebo_ci_job.with
      {
        triggers {
          scm('@daily')
        }

        label "gpu-reliable && large-memory"

        // Problem with the compilation of Gazebo under bullseyes
        // See: https://github.com/ignition-tooling/release-tools/issues/129
        disabled()

        steps {
          shell("""\
          #!/bin/bash -xe

          export DISTRO=${distro}
          export ARCH=${arch}
          export GPU_SUPPORT_NEEDED=false
          export COVERAGE_ENABLED=true
          /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-compilation.bash
          """.stripIndent())
        }
      }
    }
  }
}

// BUILD TYPES CI JOBS @ SCM/DAILY
ci_distro.each { distro ->
  supported_arches.each { arch ->
    gazebo_supported_build_types.each { build_type ->
      def gazebo_ci_job = job("gazebo-ci_BT${build_type}-gazebo11-${distro}-${arch}-gpu-none")
      OSRFLinuxCompilation.create(gazebo_ci_job)
      OSRFGitHub.create(gazebo_ci_job, "osrf/gazebo", "gazebo11")

      gazebo_ci_job.with
      {
        label "gpu-reliable && large-memory"

        triggers {
          scm('@daily')
        }

        steps {
          shell("""\
          #!/bin/bash -xe

          export DISTRO=${distro}
          export ARCH=${arch}
          export GPU_SUPPORT_NEEDED=false
          export GAZEBO_BASE_CMAKE_ARGS="-DCMAKE_BUILD_TYPE=${build_type}"
          /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-compilation.bash
          """.stripIndent())
        }
      }

    }
  }
}

// INSTALL ONELINER
all_supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    def install_default_job = job("gazebo-install-one_liner-${distro}-${arch}")
    OSRFLinuxInstall.create(install_default_job)
    install_default_job.with
    {
      triggers {
        cron(Globals.CRON_EVERY_THREE_DAYS)
      }

      label "gpu-reliable"

      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}
              /bin/bash -x ./scripts/jenkins-scripts/docker/gazebo-one-line-install-test-job.bash
              """.stripIndent())
      }
    } // end of with
  } // end of arch
} // end of distro

// INSTALL LINUX -DEV PACKAGES ALL PLATFORMS @ CRON/DAILY
gazebo_supported_branches.each { branch ->
  distros = ci_distro
  distros.each { distro ->
    supported_arches.each { arch ->
      // --------------------------------------------------------------
      def install_default_job = job("gazebo-install-${branch}_pkg-${distro}-${arch}")
      OSRFLinuxInstall.create(install_default_job)

      generate_install_job(install_default_job, branch, distro, arch, true)
    } // end of arch
  } // end of distro
} // end of branch

// INSTALL LINUX -DEV PACKAGES ALL PLATFORMS @ CRON/DAILY - ONLY UBUNTU OFFICIAL
// PACKAGES
ubuntu_official_packages_distros.each { distro, branch ->
    supported_arches.each { arch ->
    // --------------------------------------------------------------
    def install_default_job = job("gazebo-install_ubuntu_official-${branch}_pkg-${distro}-${arch}")
    OSRFLinuxInstall.create(install_default_job)

    generate_install_job(install_default_job, branch, distro, arch)
  } // end of arch
} // end of branch

// --------------------------------------------------------------
// DEBBUILD: linux package builder

all_debbuild_branches = gazebo_supported_branches
all_debbuild_branches.each { branch ->
  def build_pkg_job = job("${branch}-debbuilder")
  OSRFLinuxBuildPkg.create(build_pkg_job)

  build_pkg_job.with
  {
      steps {
        shell("""\
              #!/bin/bash -xe

              export ENABLE_ROS=false
              /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-debbuild.bash
              """.stripIndent())
      }
  }
}

// --------------------------------------------------------------
// BREW: CI jobs

// 1. ANY job @ SCM/5min
String ci_build_any_job_name_brew = "gazebo-ci-pr_any-homebrew-amd64"
def gazebo_brew_ci_any_job = job(ci_build_any_job_name_brew)
OSRFBrewCompilationAnyGitHub.create(gazebo_brew_ci_any_job, "osrf/gazebo")
gazebo_brew_ci_any_job.with
{
    label "osx_gazebo"

    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -xe ./scripts/jenkins-scripts/gazebo-default-devel-homebrew-amd64.bash
            """.stripIndent())
    }
}

def install_brew_job = job("gazebo-install-one_liner-homebrew-amd64")
OSRFOsXBase.create(install_brew_job)
install_brew_job.with
{
  triggers {
    cron('@daily')
  }

  steps {
    shell("""\
          #!/bin/bash -xe

          /bin/bash -xe ./scripts/jenkins-scripts/gazebo-one_liner-homebrew.bash
          """.stripIndent())
  }
}

// 2. default in all branches @SCM/daily
// No gazebo2 for brew
all_branches = gazebo_supported_branches
all_branches.each { branch ->
  if (is_watched_by_buildcop(branch))
    Globals.extra_emails = Globals.build_cop_email

  def osx_label = 'osx_gazebo'

  def gazebo_brew_ci_job = job("gazebo-ci-${branch}-homebrew-amd64")
  OSRFBrewCompilation.create(gazebo_brew_ci_job)
  OSRFGitHub.create(gazebo_brew_ci_job, "osrf/gazebo", branch)

  gazebo_brew_ci_job.with
  {
      label osx_label

      triggers {
        scm("TZ=US/Pacific \n H H(0-17) * * *")
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -xe ./scripts/jenkins-scripts/gazebo-default-devel-homebrew-amd64.bash
              """.stripIndent())
      }
  }

  // reset build cop email in global list of mails
  Globals.extra_emails = ""
}

// --------------------------------------------------------------
// WINDOWS: CI job

// 1. any
  String ci_build_any_job_name_win7 = "gazebo-ci-pr_any-windows7-amd64"
  def gazebo_win_ci_any_job = job(ci_build_any_job_name_win7)
  OSRFWinCompilationAnyGitHub.create(gazebo_win_ci_any_job, "osrf/gazebo")
  gazebo_win_ci_any_job.with
  {
      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/gazebo-default-devel-windows7-amd64.bat"
              """.stripIndent())
      }
  }

// 2. default / @ SCM/Daily
all_branches = gazebo_supported_branches
all_branches.each { branch ->
  def gazebo_win_ci_job = job("gazebo-ci-${branch}-windows7-amd64")
  OSRFWinCompilation.create(gazebo_win_ci_job)
  OSRFGitHub.create(gazebo_win_ci_job, "osrf/gazebo", branch)

  gazebo_win_ci_job.with
  {
      triggers {
        scm('@daily')
      }

      steps {
        batchFile("""\
              call "./scripts/jenkins-scripts/gazebo-default-devel-windows7-amd64.bat"
              """.stripIndent())
      }
  }
}

// --------------------------------------------------------------
// Create the main CI work flow job
def gazebo_ci_main = pipelineJob("gazebo-ci-manual_any")
OSRFCIWorkFlowMultiAnyGitHub.create(gazebo_ci_main,
                                   [ci_build_any_job_name_linux,
                                    ci_build_any_job_name_win7,
                                    ci_build_any_job_name_brew])
