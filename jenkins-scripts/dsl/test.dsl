import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org"

// Jobs
def ignition_ci_job = job("_test_job_from_dsl")
OSRFLinuxBase.create(ignition_ci_job)
def ignition_ci_job_mac = job("_test_job_from_dsl_mac")
OSRFOsXBase.create(ignition_ci_job_mac)
def ignition_ci_job_win = job("_test_job_from_dsl_win")
OSRFWinBase.create(ignition_ci_job_win)
def ignition_ci_pr_job = job("_test_pr_job_from_dsl")
OSRFLinuxCompilationAnyGitHub.create(ignition_ci_pr_job,
                                     'gazebosim/testing',
                                     false,
                                     false,
                                     ['main'])

// relative to WORKSPACE
def pkg_sources_dir="pkgs"

def releasepy_job = job("_test_releasepy")
OSRFReleasepy.create(releasepy_job, [DRY_RUN: true])
releasepy_job.with {
      blockOn("_test_repository_uploader") {
        blockLevel('GLOBAL')
        scanQueueFor('ALL')
      }
}

def repo_uploader = job("_test_repository_uploader")
OSRFBase.create(repo_uploader)
repo_uploader.with
{
  label Globals.nontest_label("docker")

  // wrappers {
  //    preBuildCleanup()
  // }

  parameters
  {
    stringParam('UPLOAD_TO_REPO','none','repo to upload')
  }

  steps
  {
    copyArtifacts('_test_gz_source')
    {
      includePatterns("${pkg_sources_dir}/*.tar.*")
      buildSelector {
        upstreamBuild()
      }
    }

    shell("""\
          #!/bin/bash -xe

          ls -R \${WORKSPACE}

          """.stripIndent())
  }
}

def gz_source_job = job("_test_gz_source")
OSRFLinuxSourceCreation.create(gz_source_job)
gz_source_job.with
{
  label Globals.nontest_label("docker")

  def PACKAGE_NAME="gz-cmake3"

  steps {
    shell("""\
          #!/bin/bash -xe

          # Use Jammy/amd64 as base image to generate sources
          export DISTRO=jammy
          export ARCH=amd64
          export PACKAGE_NAME=${PACKAGE_NAME}
          export SOURCE_REPO_URI=https://github.com/gazebosim/gz-cmake.git

          /bin/bash -x ./scripts/jenkins-scripts/docker/gz-source-generation.bash
          """.stripIndent())
  }

  publishers {
    postBuildScripts {
      steps {
        conditionalSteps {
             condition {
              not {
                expression('none|None|^$','${ENV,var="UPLOAD_TO_REPO"}')
                }
              }
              steps {
                // Invoke repository_uploader
                downstreamParameterized {
                  trigger('_test_repository_uploader') {
                    parameters {
                      predefinedProps([PROJECT_NAME_TO_COPY_ARTIFACTS: "\${JOB_NAME}",
                                       PACKAGE: '${PACKAGE_NAME}',
                                       UPLOAD_TO_REPO: '${UPLOAD_TO_REPO}'])
                    }
                  }
                }
                downstreamParameterized {
                  trigger('_test_releasepy') {
                    parameters {
                      currentBuild()
                      predefinedProps([PROJECT_NAME_TO_COPY_ARTIFACTS: "\${JOB_NAME}",
                                       PACKAGE: '${PACKAGE_NAME}',
                                       SOURCE_TARBALL_URI: 'S3_PATH_TO_IMPLEMENT'])
                    }
                  }
                }
              }
            }
      }
    }
  }
}

// -------------------------------------------------------------------
def outdated_job_runner = job("_test_outdated_job_runner")
OSRFBase.create(outdated_job_runner)
outdated_job_runner.with
{
  label Globals.nontest_label("master")

  steps
  {
    systemGroovyCommand(readFileFromWorkspace('scripts/jenkins-scripts/tools/outdated-job-runner.groovy'))
  }
}
