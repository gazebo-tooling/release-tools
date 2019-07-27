import _configs_.*
import javaposse.jobdsl.dsl.Job

def ci_distro = [ 'bionic' ]
def other_supported_distros = [ ]
def all_supported_distros = ci_distro + other_supported_distros
def supported_arches = [ 'amd64' ]

def ENABLE_TESTS = true
def DISABLE_TESTS = true
def DISABLE_CPPCHECK = false
// Globals.extra_emails = "caguero@osrfoundation.org"

String ci_build_any_job_name_linux = ''

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

void common_params_compilation_job(Job job, distro, arch, test_timeout = 180)
{
    // GPU label and parselog
    include_parselog(job)

    job.with
    {
        scm {
          hg('https://bitbucket.org/osrf/subt') {
            branch('default')
            subdirectory('subt')
          }
        }

        label "gpu-reliable"

        steps {
          shell("""#!/bin/bash -xe

                export DISTRO=${distro}
                export ARCH=${arch}
                export TEST_TIMEOUT=${test_timeout}

                /bin/bash -xe ./scripts/jenkins-scripts/docker/subt-compilation.bash
                """.stripIndent())
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
    common_params_compilation_job(subt_ci_job, distro, arch)
    subt_ci_job.with
    {
        triggers {
          scm('@daily')
        }
    }

    // --------------------------------------------------------------
    // 2. Create the any job
    ci_build_any_job_name_linux = "subt-ci-pr_any-${distro}-${arch}"
    def subt_ci_any_job = job(ci_build_any_job_name_linux)
    OSRFLinuxCompilationAny.create(subt_ci_any_job,
                                  'https://bitbucket.org/osrf/subt',
                                  ENABLE_TESTS, DISABLE_CPPCHECK)
    common_params_compilation_job(subt_ci_any_job, distro, arch)
  }
}

// OTHER DAILY CI JOBS
other_supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Create the other daily CI jobs
    def subt_ci_job = job("subt-ci-default-${distro}-${arch}")

    // Use the linux compilation as base
    OSRFLinuxCompilation.create(subt_ci_job, ENABLE_TESTS, DISABLE_CPPCHECK)
    // GPU label and parselog
    include_parselog(subt_ci_job)

    subt_ci_job.with
    {
        triggers {
          scm('@daily')
        }
     }
  } // end of arch
} // end of distro

// NIGHLT LONG RUNS
all_supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Install subt testing pkg testing
    def long_run_job = job("subt-ci_long-default-${distro}-${arch}")
    OSRFLinuxCompilation.create(long_run_job, DISABLE_TESTS, DISABLE_CPPCHECK)
    // use 3600 secs as timeout to test 1 hour
    common_params_compilation_job(long_run_job, distro, arch, 3600)

    long_run_job.with
    {
      triggers {
        cron('@daily')
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

// Create the main CI work flow job
def subt_ci_main = pipelineJob("subt-ci-pr_any")
OSRFCIWorkFlowMultiAny.create(subt_ci_main, ci_build_any_job_name_linux)
