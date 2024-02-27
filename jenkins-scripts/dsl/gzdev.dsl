import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'jammy' ]
def supported_arches = [ 'amd64' ]


Globals.default_emails = "jrivero@osrfoundation.org, steven@osrfoundation.org"

// LINUX
supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the compilation job
    def gzdev_ci_job = job("gzdev-ci-default-${distro}-${arch}")
    OSRFLinuxCompilation.create(gzdev_ci_job, false, false)

    gzdev_ci_job.with
    {
    
      label Globals.nontest_label("large-memory")

      scm {
        git {
          remote {
            github('gazebo-tooling/gzdev', 'https')
            branch('refs/heads/master')

            extensions {
              relativeTargetDirectory('gzdev')
            }
          }
        }
      }

      triggers {
        scm('*/5 * * * *')
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}

              /bin/bash -xe ./scripts/jenkins-scripts/docker/gzdev-compilation.bash
              """.stripIndent())
      }
    }

    // --------------------------------------------------------------
    // 3. Create the testing any job
    def gzdev_any_job = job("gzdev-ci-pr_any-${distro}-${arch}")
    OSRFLinuxCompilationAnyGitHub.create(gzdev_any_job, "gazebo-tooling/gzdev", false, false)

    gzdev_any_job.with
    {
      label Globals.nontest_label("large-memory")

      steps {
        shell("""\
              #!/bin/bash -xe

              export DISTRO=${distro}
              export ARCH=${arch}

              /bin/bash -xe ./scripts/jenkins-scripts/docker/gzdev-compilation.bash
              """.stripIndent())
      }
    }
  }
}
