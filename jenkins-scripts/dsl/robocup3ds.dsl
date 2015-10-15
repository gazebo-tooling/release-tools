import _configs_.OSRFLinuxCompilation
import _configs_.OSRFLinuxInstall
import _configs_.OSRFLinuxBuildPkg
import javaposse.jobdsl.dsl.Job

def ci_distro = [ 'trusty' ]
def other_supported_distros = [ 'vivid' ]
def supported_arches = [ 'amd64' ]

// MAIN CI JOBS
ci_distro.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the default ci jobs
    def robocup3ds_ci_job = job("robocup3ds-ci-default-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(robocup3ds_ci_job)

    robocup3ds_ci_job.with
    {
        label "gpu-reliable-trusty"

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

    robocup3ds_ci_job.with
    {
        label "gpu-reliable-trusty"

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
