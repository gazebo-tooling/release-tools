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
OSRFReleasepy.create(releasepy_job, [DRY_RUN: true])
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
// repository_uploader fake test job

def pkg_sources_dir = 'pkgs'
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
    stringParam('S3_UPLOAD_PATH','', 'S3 path to upload')
    stringParam('S3_FILES_TO_UPLOAD','', 'S3 file names to upload')
    stringParam('UPLOAD_TO_REPO','none','repo to upload')
    stringParam("PROJECT_NAME_TO_COPY_ARTIFACTS",
                "",
                "Internal use: parent job name passed by the job to be used in copy artifacts")
  }

  steps
  {
    copyArtifacts('${PROJECT_NAME_TO_COPY_ARTIFACTS}')
    {
      includePatterns("${pkg_sources_dir}/*")
      buildSelector {
        upstreamBuild()
      }
    }

    shell("""\
          #!/bin/bash -xe

          # check that the tarball name actually exist

          ls -R \${WORKSPACE}

          for pkg in \$(ls ${pkg_sources_dir}/); do
            test -f \${WORKSPACE}/${pkg_sources_dir}/\${pkg}
          done

          echo "Fake upload of \${S3_FILES_TO_UPLOAD} to \${S3_UPLOAD_PATH}"
          # code copied from repository_uploader
          pkgs_path="\$WORKSPACE/pkgs"

          for pkg in \${S3_FILES_TO_UPLOAD}; do
            # S3_UPLOAD_PATH should be send by the upstream job
            if [[ -z \${S3_UPLOAD_PATH} ]]; then
              echo "S3_UPLOAD_PATH was not defined. Not uploading"
              exit 1
            fi

            # Seems important to upload the path with a final slash
            echo "WILL RUN: s3cmd put \${pkgs_path}/\${pkg} s3://osrf-distributions/\${S3_UPLOAD_PATH}"
          done


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
