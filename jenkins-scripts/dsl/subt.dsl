import _configs_.*
import javaposse.jobdsl.dsl.Job

def ci_distro = [ 'bionic' ]
def other_supported_distros = [ ]
def all_supported_distros = ci_distro + other_supported_distros
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
    }
  }
}

// MAIN CI JOBS
ci_distro.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the default ci jobs
    def subt_ci_job = job("subt-ci-default-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(subt_ci_job, ENABLE_TESTS, DISABLE_CPPCHECK)
    // GPU label and parselog
    include_parselog(subt_ci_job)

    subt_ci_job.with
    {
        scm {
          hg('https://bitbucket.org/osrf/subt') {
            branch('default')
            subdirectory('subt')
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

                /bin/bash -xe ./scripts/jenkins-scripts/docker/subt-compilation.bash
                """.stripIndent())
        }
    }
    // --------------------------------------------------------------
    // 2. Create the any job
    def subt_ci_any_job = job("subt-ci-pr_any-${distro}-${arch}")
    OSRFLinuxCompilationAny.create(subt_ci_any_job,
                                  'https://bitbucket.org/osrf/subt',
                                  ENABLE_TESTS, DISABLE_CPPCHECK)
    // GPU label and parselog
    include_parselog(subt_ci_any_job)

    subt_ci_any_job.with
    {
        label "gpu-reliable"

        steps {
          shell("""\
                export DISTRO=${distro}
                export ARCH=${arch}

                /bin/bash -xe ./scripts/jenkins-scripts/docker/subt-compilation.bash
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
    def subt_ci_job = job("subt-ci-default-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(subt_ci_job, ENABLE_TEST, DISABLE_CPPCHECK)
    // GPU label and parselog
    include_parselog(subt_ci_job)

    subt_ci_job.with
    {
        scm {
          hg('https://bitbucket.org/osrf/subt') {
            branch('default')
            subdirectory('subt')
          }
        }

        label "gpu-reliable"

        triggers {
          scm('@daily')
        }

        steps {
          shell("""#!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}

                /bin/bash -xe ./scripts/jenkins-scripts/docker/subt-compilation.bash
                """.stripIndent())
        }
     }
  } // end of arch
} // end of distro

// DAILY INSTALL TESTS
all_supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Install subt testing pkg testing
    def install_default_job = job("subt-install_pkg-${distro}-${arch}")
    OSRFLinuxInstall.create(install_default_job)
    // GPU label and parselog
    include_parselog(install_default_job)

    install_default_job.with
    {
      triggers {
        cron('@daily')
      }

      label "gpu-reliable"

      steps {
        shell("""\
            #!/bin/bash -xe

            export DISTRO=${distro}
            export ARCH=${arch}
            export INSTALL_JOB_PKG=subt
            export INSTALL_JOB_REPOS="stable prerelease"
            /bin/bash -xe ./scripts/jenkins-scripts/docker/subt-install-test-job.bash
            """.stripIndent())
      }
    } // end of with
  }
}


// --------------------------------------------------------------
// subt package builder
def build_pkg_job = job("subt-debbuilder")
OSRFLinuxBuildPkg.create(build_pkg_job)

build_pkg_job.with
{
  scm {
    hg("https://bitbucket.org/osrf/subt") {
      branch('default')
      installation('Default')
      subdirectory('repo')
      configure { project ->
       project / browser(class: 'hudson.plugins.mercurial.browser.BitBucket') / "url" <<  "https://bitbucket.org/osrf/subt"
      }
    }
  }

  steps {
    shell("""\
          #!/bin/bash -xe

          # subt only uses a subdirectory as package
          rm -fr \$WORKSPACE/repo_backup
          rm -fr \$WORKSPACE/subt_gazebo
          cp -a \$WORKSPACE/repo/subt_gazebo \$WORKSPACE/subt_gazebo
          mv \$WORKSPACE/repo \$WORKSPACE/repo_backup
          mv \$WORKSPACE/subt_gazebo \$WORKSPACE/repo

          export NIGHTLY_MODE=true
          export USE_REPO_DIRECTORY_FOR_NIGHTLY=true
          /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-debbuild.bash

          rm -fr \$WORKSPACE/repo
          mv \$WORKSPACE/repo_backup \$WORKSPACE/repo
          """.stripIndent())
  }
}
