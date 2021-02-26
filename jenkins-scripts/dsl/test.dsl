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
GitHubCredentials.createOsrfbuildToken(test_credentials_token_job)
test_credentials_token_job.with
{
  label "osx"

  steps {
    shell("""\
          #!/bin/bash -xe

          # Check push+commit permissions for osrfbuild by uploading/deleting a
          # branch. Note that call to the API for permissions require of admin
          # perms that osrfbuild user does not have. Personal tokens don't
          # support ssh but https only.

          git clone https://github.com/osrfbuild/homebrew-simulation.git
          cd homebrew-simulation
          git config user.name 'osrfbuild' --replace-all
          git config user.email 'osrfbuild@openrobotics.org' --replace-all
          set +x
          git config url."https://osrfbuild:\${GITHUB_TOKEN}@github.com/osrfbuild/homebrew-simulation.git".InsteadOf https://github.com/osrfbuild/homebrew-simulation.git
          set -x
          git checkout -b _test_job_osrfbuild_
          git commit --allow-empty -m "testing commit"
          # protect token from errors
          git push -u origin _test_job_osrfbuild_ > push_log
          git push origin --delete _test_job_osrfbuild_ >> push_log
          """.stripIndent())
    }

    publishers
    {
      postBuildScripts {
        steps {
          shell("""\
                #!/bin/bash -xe

                # remove token after the build ends unconditionally
                rm -fr \${WORKSPACE}/homebrew-simulation/.git/config
                """.stripIndent())
        }

        onlyIfBuildSucceeds(false)
        onlyIfBuildFails(false)
      }
    }

    wrappers {
      preBuildCleanup()
    }
}
