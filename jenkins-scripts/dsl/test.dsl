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
    stringParam('PACKAGE','','Package name')
    stringParam('TARBALL_NAME', '', 'Tarball name to upload')
    stringParam('S3_UPLOAD_PATH','', 'S3 path to upload')
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

          # check that the tarball name actually exist

          ls -R \${WORKSPACE}
          test -f \${WORKSPACE}/${pkg_sources_dir}/\${TARBALL_NAME}

          echo "Fake upload of \${TARBALL_NAME} to \${S3_UPLOAD_PATH}"
          """.stripIndent())
  }
}

def gz_source_job = job("_test_gz_source")
OSRFLinuxSourceCreation.create(gz_source_job, [
  PACKAGE: "gz-cmake3" ,
  SOURCE_REPO_URI: "https://github.com/gazebosim/gz-cmake.git"])

def properties_file="package_name.prop"
gz_source_job.with
{
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
                  predefinedProps([RTOOLS_BRANCH: "\${RTOOLS_BRANCH}",
                                   PROJECT_NAME_TO_COPY_ARTIFACTS: "\${JOB_NAME}",
                                   S3_UPLOAD_PATH: Globals.s3_upload_tarball_path("gz-cmake3"),
                                   UPLOAD_TO_REPO: '${UPLOAD_TO_REPO}'])
                  propertiesFile(properties_file)
                                  // PACKAGE
                                  // TARBALL_NAME
                }
              }
            }
            downstreamParameterized {
              trigger('_test_releasepy') {
                parameters {
                  currentBuild()
                  predefinedProps([PROJECT_NAME_TO_COPY_ARTIFACTS: "\${JOB_NAME}"])
                  propertiesFile(properties_file)
                                  // PACKAGE
                                  // TARBALL_NAME
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
