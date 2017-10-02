import _configs_.*
import javaposse.jobdsl.dsl.Job

// Main platform using for quick CI
def ci_distro               = ['xenial']
def ci_gpu                  = Globals.get_ci_gpu()
// Other supported platform to be checked but no for quick
// CI integration.
def supported_arches        = Globals.get_supported_arches()

def DISABLE_TESTS = false

String ci_distro_str = ci_distro[0]
String ci_gpu_str = ci_gpu[0]
String ci_build_any_job_name_linux = "gazebo_experimental-ci-pr_any-${ci_distro_str}-amd64-gpu-${ci_gpu_str}"

// Need to be used in ci_pr
String abi_job_name = ''


// CI JOBS @ SCM/5 min
ci_gpu_include_gpu_none = ci_gpu + [ 'none' ]
ci_distro.each { distro ->
  ci_gpu_include_gpu_none.each { gpu ->
    supported_arches.each { arch ->
      // Temporary workaround to use Xenial as distro for gpu-none
      if (gpu == 'none')
      {
        distro = "xenial"
      }

      // --------------------------------------------------------------
      // 1. Create the any job
      def gazebo_ci_any_job_name = "gazebo_experimental-ci-pr_any-${distro}-${arch}-gpu-${gpu}"
      def gazebo_ci_any_job      = job(gazebo_ci_any_job_name)
      OSRFLinuxCompilationAny.create(gazebo_ci_any_job,
                                    "https://bitbucket.org/osrf/gazebo_experimental")
      gazebo_ci_any_job.with
      {
        if (gpu != 'none')
        {
           label "gpu-${gpu}-${distro}"
        }

        steps
        {
           String gpu_needed = 'true'
           if (gpu == 'none') {
              gpu_needed = 'false'
              // save the name to be used in he Workflow job
              ci_build_any_job_name_linux_no_gpu = gazebo_ci_any_job_name
           }

           shell("""\
           #!/bin/bash -xe

           export DISTRO=${distro}
           export ARCH=${arch}
           export GPU_SUPPORT_NEEDED=${gpu_needed}
           /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo_experimental-compilation.bash
           """.stripIndent())
         }
      }

      // --------------------------------------------------------------
      // 2. Create the default ci jobs
      def gazebo_ci_job = job("gazebo_experimental-ci-default-${distro}-${arch}-gpu-${gpu}")
      OSRFLinuxCompilation.create(gazebo_ci_job)
      OSRFBitbucketHg.create(gazebo_ci_job, "https://bitbucket.org/osrf/gazebo_experimental")

      gazebo_ci_job.with
      {
        if (gpu != 'none')
        {
          label "gpu-${gpu}-${distro}"
        }

        triggers {
          scm('*/5 * * * *')
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
                /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo_experimental-compilation.bash
                """.stripIndent())
        }
      }

      // reset build cop email in global list of mails
      Globals.extra_emails = ''
    } // end of gpu
  } // end of arch
} // end of distro

// sdformat and ignition dependencies
ci_distro.each { distro ->
  supported_arches.each { arch ->
    ci_gpu.each { gpu ->
      def multi_any_job = job("gazebo_experimental-ci-pr_any+sdformat_any+ign_any-${distro}-${arch}-gpu-${gpu}")
      OSRFLinuxCompilationAny.create(multi_any_job,
                                    "https://bitbucket.org/osrf/gazebo_experimental")
      multi_any_job.with
      {
        parameters
        {
          stringParam('SDFORMAT_BRANCH', 'default', 'sdformat branch to use')
          stringParam('IGN_MATH_BRANCH', 'default', 'ignition math branch to use')
          stringParam('IGN_TRANSPORT_BRANCH', 'default', 'ignition transport branch to use')
        }

        label "gpu-${gpu}-${distro}"

        steps {
            shell("""\
            #!/bin/bash -xe

            export DISTRO=${distro}
            export ARCH=${arch}
            export GPU_SUPPORT_NEEDED=true
            export GAZEBO_BUILD_SDFORMAT=true
            export GAZEBO_BUILD_IGN_MATH=true
            export GAZEBO_BUILD_IGN_TRANSPORT=true
            /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo_experimental-compilation.bash
            """.stripIndent())
        }
      }
    }
  }
}


// Other build types

// --------------------------------------------------------------
// BREW: CI jobs

// 1. ANY job @ SCM/5min
String ci_build_any_job_name_brew = "gazebo_experimental-ci-pr_any-homebrew-amd64"
def gazebo_brew_ci_any_job = job(ci_build_any_job_name_brew)
OSRFBrewCompilationAny.create(gazebo_brew_ci_any_job,
                              "https://bitbucket.org/osrf/gazebo_experimental")
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
all_branches = ['default']
all_branches.each { branch ->
  def gazebo_brew_ci_job = job("gazebo_experimental-ci-${branch}-homebrew-amd64")
  OSRFBrewCompilation.create(gazebo_brew_ci_job)
  OSRFBitbucketHg.create(gazebo_brew_ci_job, "https://bitbucket.org/osrf/gazebo_experimental", branch,
                                             "gazebo_experimental", "HomeBrew")

  gazebo_brew_ci_job.with
  {
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

  // reset build cop email in global list of mails
  Globals.extra_emails = ""
}

// --------------------------------------------------------------
// WINDOWS: CI job

// 1. any
String ci_build_any_job_name_win7 = "gazebo_experimental-ci-pr_any-windows7-amd64"
def gazebo_win_ci_any_job = job(ci_build_any_job_name_win7)
OSRFWinCompilationAny.create(gazebo_win_ci_any_job,
                             "https://bitbucket.org/osrf/gazebo_experimental",
                             DISABLE_TESTS)
gazebo_win_ci_any_job.with
{
    steps {
      batchFile("""\
            call "./scripts/jenkins-scripts/gazebo-default-devel-windows7-amd64.bat"
            """.stripIndent())
    }
}

// 2. default / @ SCM/Daily
all_branches = ['default']
all_branches.each { branch ->
  def gazebo_win_ci_job = job("gazebo_experimental-ci-${branch}-windows7-amd64")
  OSRFWinCompilation.create(gazebo_win_ci_job, DISABLE_TESTS)
  OSRFBitbucketHg.create(gazebo_win_ci_job, "https://bitbucket.org/osrf/gazebo_experimental", branch)

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
def gazebo_ci_main = pipelineJob("gazebo_experimental-ci-pr_any")
OSRFCIWorkFlowMultiAny.create(gazebo_ci_main,
                                   [ci_build_any_job_name_linux,
                                    ci_build_any_job_name_linux_no_gpu,
                                    ci_build_any_job_name_win7,
                                    ci_build_any_job_name_brew])
