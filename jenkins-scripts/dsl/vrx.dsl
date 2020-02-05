import _configs_.*
import javaposse.jobdsl.dsl.Job

def ci_distro = [ 'bionic' : 'melodic',
                  'xenial' : 'kinetic' ]
def supported_arches = [ 'amd64' ]

def ENABLE_TESTS = true
def DISABLE_CPPCHECK = false
// Globals.extra_emails = "caguero@osrfoundation.org"

void include_parselog(Job job)
{
  job.with
  {
    publishers {
        consoleParsing {
            globalRules('/var/lib/jenkins/logparser_error_on_roslaunch_failed')
            failBuildOnError()
        }
        // Needed to detect problems in test compilation since at that step the
        // return is always true (don't want to fail build on failing tests).
        consoleParsing {
            globalRules('/var/lib/jenkins/logparser_error_on_failed_compilation')
            failBuildOnError()
        }
        consoleParsing {
            projectRules('scripts/jenkins-scripts/parser_rules/gazebo_err.parser')
            failBuildOnError()
      }
    }
  }
}

// MAIN CI JOBS
ci_distro.each { distro, ros_distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the default ci jobs
    def vrx_ci_job = job("vrx-ci-default_${ros_distro}-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(vrx_ci_job, ENABLE_TESTS, DISABLE_CPPCHECK)
    // GPU label and parselog
    include_parselog(vrx_ci_job)

    vrx_ci_job.with
    {
        scm {
          hg('https://bitbucket.org/osrf/vrx') {
            branch('default')
            subdirectory('vrx')
          }
        }

        label "gpu-reliable"

        triggers {
          scm('*/5 * * * *')
        }

        steps {
          shell("""#!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}
                export ROS_DISTRO=${ros_distro}

                /bin/bash -xe ./scripts/jenkins-scripts/docker/vrx-compilation.bash
                """.stripIndent())
        }
    }
    // --------------------------------------------------------------
    // 2. Create the any job
    def vrx_ci_any_job = job("vrx-ci-pr_any_${ros_distro}-${distro}-${arch}")
    OSRFLinuxCompilationAny.create(vrx_ci_any_job,
                                  'https://bitbucket.org/osrf/vrx',
                                  ENABLE_TESTS, DISABLE_CPPCHECK)
    // GPU label and parselog
    include_parselog(vrx_ci_any_job)

    vrx_ci_any_job.with
    {
        label "gpu-reliable"

        steps {
          shell("""\
                export DISTRO=${distro}
                export ARCH=${arch}
                export ROS_DISTRO=${ros_distro}
                /bin/bash -xe ./scripts/jenkins-scripts/docker/vrx-compilation.bash
                """.stripIndent())
        }
    }
  }
}

// DAILY INSTALL TESTS
ci_distro.each { distro, ros_distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Install vrx testing pkg testing
    def install_default_job = job("vrx-install_pkg_${ros_distro}-${distro}-${arch}")
    OSRFLinuxInstall.create(install_default_job)
    // GPU label and parselog
    include_parselog(install_default_job)

    install_default_job.with
    {
      if ("${ros_distro}" == "kinetic")
        disabled()

      triggers {
        cron('@daily')
      }

      label "gpu-reliable"

      steps {
        shell("""\
            #!/bin/bash -xe

            export DISTRO=${distro}
            export ARCH=${arch}
            export ROS_DISTRO=${ros_distro}
            export INSTALL_JOB_PKG=ros-${ros_distro}-vrx-gazebo
            export INSTALL_JOB_REPOS="stable"
            /bin/bash -xe ./scripts/jenkins-scripts/docker/vrx-install-test-job.bash
            """.stripIndent())
      }
    } // end of with
  }
}
