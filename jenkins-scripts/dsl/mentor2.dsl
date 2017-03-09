import _configs_.*
import javaposse.jobdsl.dsl.Job

//
// Project currently disabled
//

def supported_distros = [ 'trusty' ]
def supported_arches = [ 'amd64' ]

Globals.extra_emails = "ichen@osrfoundation.org"

supported_distros.each { distro ->
  supported_arches.each { arch ->

    // --------------------------------------------------------------
    // 1. Create the default ci jobs
    def gazebo_ci_job = job("gazebo-ci-mentor2_v2-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(gazebo_ci_job)

    gazebo_ci_job.with
    {
        disabled()
        label "gpu-reliable-trusty"

        scm {
          hg('https://bitbucket.org/osrf/gazebo') {
            branch('mentor2_v2')
            subdirectory('gazebo')
          }
        }

        triggers {
          scm('*/5 * * * *')
        }

        steps {
          shell("""#!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}
                export GPU_SUPPORT_NEEDED=true
                /bin/bash -xe ./scripts/jenkins-scripts/docker/gazebo-compilation.bash
                """.stripIndent())
        }
     }

    def sdformat_ci_job = job("sdformat-ci-mentor2_v2-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(sdformat_ci_job)

    sdformat_ci_job.with
    {
        disabled()
        label "gpu-reliable-trusty"

        scm {
          hg('https://bitbucket.org/osrf/sdformat') {
            branch('mentor2_v2')
            subdirectory('sdformat')
          }
        }

        triggers {
          scm('*/5 * * * *')
        }

        steps {
          shell("""#!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}
                /bin/bash -xe ./scripts/jenkins-scripts/docker/sdformat-compilation.bash
                """.stripIndent())
        }
    }

    def mentor2_ci_job = job("mentor2-ci-default-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(mentor2_ci_job)

    mentor2_ci_job.with
    {
        disabled()
        label "gpu-reliable-trusty"

        scm {
          hg('https://bitbucket.org/osrf/mentor2') {
            branch('default')
            subdirectory('mentor2')
          }
        }

        triggers {
          scm('*/5 * * * *')
        }

        steps {
          shell("""#!/bin/bash -xe

                /bin/bash -x ./scripts/jenkins-scripts/docker/mentor2-default-devel-trusty-amd64.bash
                """.stripIndent())
        }
    }

    // --------------------------------------------------------------
    // 2. Create the install test job
    def install_default_job = job("mentor2-install-pkg-${distro}-${arch}")

    // Use the linux install as base
    OSRFLinuxInstall.create(install_default_job)

    install_default_job.with
    {
        disabled()
        triggers {
          cron('@weekly')
        }

        steps {
          shell("""#!/bin/bash -xe

                export INSTALL_JOB_PKG=mentor2
                export INSTALL_JOB_REPOS="mentor2 stable"
                /bin/bash -x ./scripts/jenkins-scripts/docker/generic-install-test-job.bash
                """.stripIndent())
       }
    }
  }
}


// --------------------------------------------------------------
// 3. mentor package builder
def build_pkg_job = job("mentor2-debbuilder")

// Use the linux install as base
OSRFLinuxBuildPkg.create(build_pkg_job)

build_pkg_job.with
{
    disabled()
    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-no-ros-debbuild.bash
            """.stripIndent())
    }
}
