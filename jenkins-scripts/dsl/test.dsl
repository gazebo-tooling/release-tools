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
// gz source testing job
def gz_source_job = job("_test_gz_source")
OSRFSourceCreation.create(gz_source_job, [
  PACKAGE_NAME: "gz-cmake3",
  SOURCE_REPO_URI: "https://github.com/gazebosim/gz-cmake.git"])

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
