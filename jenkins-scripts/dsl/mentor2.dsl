import _configs_.OSRFLinuxCompilation
import _configs_.OSRFLinuxInstall
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'trusty' ]
def supported_arches = [ 'amd64' ]

supported_distros.each { distro ->
  supported_arches.each { arch ->    

    // --------------------------------------------------------------
    // 1. Create the default ci jobs
    def gazebo_ci_job = job("gazebo-ci-mentor2_v2-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(gazebo_ci_job)

    gazebo_ci_job.with
    {
        label "gpu-reliable-trusty"

        scm {
          hg('http://bitbucket.org/osrf/gazebo') {
            branch('mentor2_v2')
            subdirectory('gazebo')
          }
        }

        triggers {
          scm('*/5 * * * *') 
        }

        steps {
          shell("""
                #!/bin/bash

                /bin/bash -x ./scripts/jenkins-scripts/docker/gazebo-default-gui-test-devel-trusty-amd64.bash
                """.stripIndent())
        }
     }

    def sdformat_ci_job = job("sdformat-ci-mentor2_v2-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(sdformat_ci_job)

    sdformat_ci_job.with
    {
        label "gpu-reliable-trusty"

        scm {
          hg('http://bitbucket.org/osrf/sdformat') {
            branch('mentor2_v2')
            subdirectory('sdformat')
          }
        }

        triggers {
          scm('*/5 * * * *') 
        }

        steps {
          shell("""
                #!/bin/bash

                /bin/bash -x ./scripts/jenkins-scripts/docker/sdformat-default-devel-trusty-amd64.bash
                """.stripIndent())
        }
     }

    // --------------------------------------------------------------
    // 2. Create the install test job
    def install_default_job = job("install-gazebo_mentor2-pkg-${distro}-${arch}")

    // Use the linux install as base
    OSRFLinuxInstall.create(install_default_job)

    install_default_job.with
    {
        triggers {
          scm('@daily') 
        }

        steps {
          shell("""
                #!/bin/bash

                export INSTALL_JOB_PKG=gazebo
                export INSTALL_JOB_REPOS=mentor2
                /bin/bash -x ./scripts/jenkins-scripts/docker/generic-install-test-job.bash
                """.stripIndent())
        }
     }
  }
}
