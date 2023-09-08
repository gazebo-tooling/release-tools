import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org"

// Usual CI jobs
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

// releasing testing job
def releasepy_job = job("_test_releasepy")
OSRFReleasepy.create(releasepy_job, [
  DRY_RUN: true])
releasepy_job.with {
      blockOn("_test_repository_uploader") {
        blockLevel('GLOBAL')
        scanQueueFor('ALL')
      }
}
// gz source testing job
def gz_source_job = job("_test_gz_source")
OSRFSourceCreation.create(gz_source_job, [
  PACKAGE: "gz-cmake3" ,
  SOURCE_REPO_URI: "https://github.com/gazebosim/gz-cmake.git"])
OSRFSourceCreation.call_uploader_and_releasepy(gz_source_job,
  '_test_repository_uploader',
  '_test_releasepy')

def pkg_sources_dir = 'pkgs'

// repository_uploader fake test job
// TODO: implement the S3_FILES_TO_UPLOAD support in this
def repo_uploader = job("_test_repository_uploader")
OSRFBase.create(repo_uploader)
repo_uploader.with
{
  label Globals.nontest_label("docker")

  wrappers {
      preBuildCleanup()
  }

  parameters
  {
    stringParam('PACKAGE','','Package name')
    stringParam('S3_FILE_TO_UPLOAD', '', 'Tarball name to upload')
    stringParam('S3_UPLOAD_PATH','', 'S3 path to upload')
    stringParam('UPLOAD_TO_REPO','none','repo to upload')
  }

  steps
  {
    copyArtifacts('_test_gz_source')
    {
      includePatterns("${pkg_sources_dir}/\${S3_FILE_TO_UPLOAD}")
      buildSelector {
        upstreamBuild()
      }
    }

    shell("""\
          #!/bin/bash -xe

          # check that the tarball name actually exist

          ls -R \${WORKSPACE}
          test -f \${WORKSPACE}/${pkg_sources_dir}/\${S3_FILE_TO_UPLOAD}

          echo "Fake upload of \${S3_FILE_TO_UPLOAD} to \${S3_UPLOAD_PATH}"
          """.stripIndent())
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
