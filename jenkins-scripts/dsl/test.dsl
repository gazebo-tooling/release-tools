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
    // Input parameters
  //  - VERSION
  //  - RELEASE_VERSION
  //  - RELEASE_REPO_BRANCH
  //  - UPLOAD_TO_REPO

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

  // relative to WORKSPACE
  def pkg_sources_dir="pkg_sources"

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

    conditionalSteps {
      condition {
        not {
          expression('none|None|^$','${ENV,var="UPLOAD_TO_REPO"}')
        }
      }
      steps {
        copyArtifacts('test_repository_uplaoder') {
          excludePatterns("${pkg_sources_dir}/*.tar.*")
          buildSelector {
            upstreamBuild()
          }
        }

        downstreamParameterized {
          // TODO(implement) in repository_uploader tarball push to S3
          trigger('_test_repository_uploader') {
            block {
              buildStepFailure('FAILURE')
              failure('FAILURE')
              unstable('UNSTABLE')
            }
            parameters {
              predefinedProps([UPLOAD_TO_REPO: '${UPLOAD_TO_REPO}'])
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
