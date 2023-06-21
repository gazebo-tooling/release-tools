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

// -------------------------------------------------------------------
def outdated_job_runner = job("_outdated_job_runner")
OSRFBase.create(outdated_job_runner)
outdated_job_runner.with
{
  label Globals.nontest_label("master")

  steps
  {
    systemGroovyCommand(readFileFromWorkspace('scripts/jenkins-scripts/tools/outdated-job-runner.groovy'))
  }

  publishers
  {
    // Added the checker result parser (UNSTABLE if not success)
    configure { project ->
      project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
        unstableOnWarning true
        failBuildOnError false
        parsingRulesPath('/var/lib/jenkins/logparser_warn_on_mark_unstable')
      }
    }
  }
}
