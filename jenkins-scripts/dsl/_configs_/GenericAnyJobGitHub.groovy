package _configs_

import javaposse.jobdsl.dsl.Job

/*
   Implements:
     - parameters: SRC_REPO, SRC_BRANCH, JOB_DESCRIPTION
     - job.Description
     - scm check with SRC_REPO + SRC_BRANCH
*/


class GenericAnyJobGitHub
{
   static void create(Job job,
                     String github_repo,
                     ArrayList supported_branches = [],
                     boolean enable_github_pr_integration = true)
   {
     // setup special mail subject
     GenericMail.update_field(job, 'defaultSubject',
                    '$PROJECT_NAME - Branch: $GIT_BRANCH (#$BUILD_NUMBER) - $BUILD_STATUS!')
     GenericMail.update_field(job, 'defaultContent',
                    '$JOB_DESCRIPTION \n' + GenericCompilation.get_compilation_mail_content())

    // Get repo name for relativeTargetDirectory
    String github_repo_name = github_repo.substring(github_repo.lastIndexOf("/") + 1)

    job.with
    {
      parameters
      {
        stringParam('sha1', '', 'commit or refname to build')
      }

      scm
      {
        git {
          remote {
            github(github_repo)
            refspec('+refs/pull/*:refs/remotes/origin/pr/*')
          }
          extensions {
            relativeTargetDirectory(github_repo_name)
          }

          branch('${sha1}')
        }
      }

      if (enable_github_pr_integration) {
        configure { project ->
          project  / triggers / 'org.jenkinsci.plugins.ghprb.GhprbTrigger' {
              adminlist 'osrf-jenkins j-rivero'
              orgslist 'osrf'
              useGitHubHooks(true)
              allowMembersOfWhitelistedOrgsAsAdmin(true)
              useGitHubHooks(true)
              onlyTriggerPhrase(false)
              permitAll(true)
              cron()
              whiteListTargetBranches {
                supported_branches.each { supported_branch ->
                  'org.jenkinsci.plugins.ghprb.GhprbBranch' {
                    branch supported_branch
                  }
                }
              }
              triggerPhrase '.*(re)?run test(s).*'
              extensions {
                  'org.jenkinsci.plugins.ghprb.extensions.status.GhprbSimpleStatus' {
                    commitStatusContext '${JOB_NAME}'
                    triggeredStatus 'starting deployment to build.osrfoundation.org'
                    startedStatus 'deploying to build.osrfoundation.org'
                    addTestResults(true)
                  }
              }
          }
        }
      }
    } // end of with
  } // end of create method
}
