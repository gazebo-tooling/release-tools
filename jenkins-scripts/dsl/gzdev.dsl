import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'xenial' ]
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

      scm {
        git {
          remote {
            github('osrf/gzdev', 'https')
            branch('refs/heads/master')

            extensions {
              relativeTargetDirectory('gzdev')
            }
          }
        }
      }

      // use only the most powerful nodes
      label "large-memory"

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
    // Use stub to supply a fake bitbucket repository. It is overwritten by the
    // git scm section below. False to disable testing.
    OSRFLinuxCompilationAny.create(gzdev_any_job, "repo_stub", false, false)

    gzdev_any_job.with
    {
      // use only the most powerful nodes
      label "large-memory"

      scm {
        git {
          remote {
            github('osrf/gzdev', 'https')
            branch('${SRC_BRANCH}')

            extensions {
              relativeTargetDirectory('gzdev')
            }
          }
        }
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
  }
}



