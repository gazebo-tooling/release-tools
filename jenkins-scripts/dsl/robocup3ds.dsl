import _configs_.*
import javaposse.jobdsl.dsl.Job

def ci_distro = [ 'trusty' ]
def other_supported_distros = [ 'xenial' ]
def all_supported_distros = ci_distro + other_supported_distros
def supported_arches = [ 'amd64' ]

Globals.extra_emails = "caguero@osrfoundation.org"

// Early testing shows that xenial jobs can be run on a
// trusty host with good results.
void include_gpu_label(Job job, String distro)
{
  job.with
  {
    if (distro == 'xenial')
      label "gpu-reliable-${distro} || gpu-reliable-trusty"
    else
      label "gpu-reliable-${distro}"
  }
}

// MAIN CI JOBS
ci_distro.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the default ci jobs
    def robocup3ds_ci_job = job("robocup3ds-ci-default-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(robocup3ds_ci_job)
    // GPU label
    include_gpu_label(robocup3ds_ci_job, distro)

    robocup3ds_ci_job.with
    {
        scm {
          hg('http://bitbucket.org/osrf/robocup3ds') {
            branch('default')
            subdirectory('robocup3ds')
          }
        }

        triggers {
          scm('*/5 * * * *')
        }

        steps {
          shell("""#!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}

                /bin/bash -xe ./scripts/jenkins-scripts/docker/robocup3ds-compilation.bash
                """.stripIndent())
        }
     }
    // --------------------------------------------------------------
    // 2. Create the any job
    def ignition_ci_any_job = job("robocup3ds-ci-pr_any-${distro}-${arch}")
    OSRFLinuxCompilationAny.create(ignition_ci_any_job,
                                  'http://bitbucket.org/osrf/robocup3ds')
    // GPU label
    include_gpu_label(ignition_ci_any_job, distro)

    ignition_ci_any_job.with
    {
        steps {
          shell("""\
                export DISTRO=${distro}
                export ARCH=${arch}

                /bin/bash -xe ./scripts/jenkins-scripts/docker/robocup3ds-compilation.bash
                """.stripIndent())
        }
    }

  }
}

// OTHER DAILY CI JOBS
other_supported_distros.each { distro ->
  supported_arches.each { arch ->

    // --------------------------------------------------------------
    // 1. Create the other daily CI jobs
    def robocup3ds_ci_job = job("robocup3ds-ci-default-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(robocup3ds_ci_job)
    // GPU label
    include_gpu_label(robocup3ds_ci_job, distro)

    robocup3ds_ci_job.with
    {
        scm {
          hg('http://bitbucket.org/osrf/robocup3ds') {
            branch('default')
            subdirectory('robocup3ds')
          }
        }

        triggers {
          scm('@daily')
        }

        steps {
          shell("""#!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}

                /bin/bash -xe ./scripts/jenkins-scripts/docker/robocup3ds-compilation.bash
                """.stripIndent())
        }
     }
  }
}

// DAILY INSTALL TESTS
all_supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    def install_default_job = job("robocup3ds-install-one_liner-${distro}-${arch}")
    OSRFLinuxInstall.create(install_default_job)
    // GPU label
    include_gpu_label(install_default_job, distro)

    install_default_job.with
    {
       triggers {
         cron('@daily')
       }

       steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}

              /bin/bash -x ./scripts/jenkins-scripts/docker/robocup3ds-install-test-job.bash
              """.stripIndent())
      }
    }
  }
}


// --------------------------------------------------------------
// ignition-transport package builder
def build_pkg_job = job("robocup3ds-debbuilder")

// Use the linux install as base
OSRFLinuxBuildPkg.create(build_pkg_job)

build_pkg_job.with
{
    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-no-ros-debbuild.bash
            """.stripIndent())
    }
}
