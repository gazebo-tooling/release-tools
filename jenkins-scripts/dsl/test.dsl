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
def pkg_sources_dir="pkg_sources"

def releasepy_job = job("_test_releasepy")
OSRFReleasepy.create(releasepy_job, [DRY_RUN: true])

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
  // dsl should know the
  //   - PACKAGE name (idem PACKAGE_ALIAS)

  // release.py PACKAGE VERSION TOKEN -r RELEASE_VERSION --upload-to UPLOAD_TO_REPO

  // IMPLEMENT: SOURCE_TARBALL_URI

  // release.py generates:
  //  - LINUX_DISTRO unconditionally
  //  - DISTRO
  //  - ARCH
  //  - JENKINS_NODE_TAG
  //  - OSRF_REPOS_TO_USE

  label Globals.nontest_label("docker")

  steps {
    shell("""\
          #!/bin/bash -xe

          # export PACKAGE="gz-cmake3"  # TODO(implement)
          # Use Jammy/amd64 as base image to generate sources
          # export DISTRO=jammy
          # export ARCH=amd64

          # Check out sources from gazebosim/\${PACKAGE}
          # git cmake
          # cd source
          # cmake .. -DPACKAGE_SOURCE_ONLY
          # find the tarball + archive
          cd \${WORKSPACE}
          mkdir ${pkg_sources_dir}
          touch ${pkg_sources_dir}/_test.tar.bz2
          """.stripIndent())
  }

  publishers {
    archiveArtifacts("${pkg_sources_dir}")

    flexiblePublish
    {
      conditionalAction {
        condition {
          and {
            status('SUCCESS','SUCCESS')
          } {
            not {
              expression('none|None|^$','${ENV,var="UPLOAD_TO_REPO"}')
            }
          }
        }

        publishers {
          downstreamParameterized {
            trigger('_test_repository_uploader') {
              parameters {
                predefinedProps([PROJECT_NAME_TO_COPY_ARTIFACTS: "\${JOB_NAME}",
                                 UPLOAD_TO_REPO: '${UPLOAD_TO_REPO}'])
              }
            }
            trigger('_test_releasepy') {
              parameters {
                currentBuild()
                predefinedProps([PROJECT_NAME_TO_COPY_ARTIFACTS: "\${JOB_NAME}",
                                 SOURCE_TARBALL_URI: 'S3_PATH_TO_IMPLEMENT'])
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
