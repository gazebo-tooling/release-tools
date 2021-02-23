import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org"

def ignition_ci_job = job("_test_job_from_dsl")
OSRFLinuxBase.create(ignition_ci_job)
def ignition_ci_job_mac = job("_test_job_from_dsl_mac")
OSRFOsXBase.create(ignition_ci_job_mac)
def ignition_ci_job_win = job("_test_job_from_dsl_win")
OSRFWinBase.create(ignition_ci_job_win)
def ignition_ci_pr_job = job("_test_pr_job_from_dsl")
OSRFLinuxCompilationAnyGitHub.create(ignition_ci_pr_job,
                                     'ignitionrobotics/testing',
                                     false,
                                     false,
                                     ['main'])
def test_credentials_token_job = job("_test_job_osrfbuild-credentials-token_from_dsl")
OSRFBase.create(test_credentials_token_job)
GitHubCredentialOsrfbuild.createToken(test_credentials_token_job)
test_credentials_token_job.with
{
  steps {
    shell("""\
          #!/bin/bash -xe

          export ssh_log=`ssh -T git@github.com 2>&1`
          echo \$ssl_log
          grep osrfbuild <<< \$ssh_log || exit 1
          """.stripIndent())
    }
}

def test_credentials_key_job = job("_test_job_osrfbuild-credentials-key_from_dsl")
OSRFBase.create(test_credentials_key_job)
GitHubCredentialOsrfbuild.createKey(test_credentials_key_job)

test_credentials_key_job.with
{
  steps {
    shell("""\
          #!/bin/bash -xe

          export ssh_log=`ssh -T -i \${OSRFBUILD_KEY} git@github.com 2>&1`
          echo \$ssl_log
          grep osrfbuild <<< \$ssh_log || exit 1
          """.stripIndent())
    }
}
