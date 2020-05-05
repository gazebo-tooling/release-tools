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
                     ArrayList supported_branches = [])
   {
     // setup special mail subject
     GenericMail.update_field(job, 'defaultSubject',
                    '$PROJECT_NAME - Branch: $GIT_BRANCH (#$BUILD_NUMBER) - $BUILD_STATUS!')
     GenericMail.update_field(job, 'defaultContent',
                    '$JOB_DESCRIPTION \n' + GenericCompilation.get_compilation_mail_content())

    // Get repo name for relativeTargetDirectory
    String github_repo_name = github_repo.substring(github_repo.lastIndexOf("/") + 1)

    // Transform GStringImp into real string
    ArrayList supported_branches_str = []
    supported_branches.each { branch ->
      supported_branches_str.add(branch.toString())
    }

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

      configure { project ->
        project  / triggers / 'org.jenkinsci.plugins.ghprb.GhprbTrigger' {
            adminList 'osrf-jenkins j-rivero'
            useGitHubHooks(true)
            allowMembersOfWhitelistedOrgsAsAdmin(true)
            useGitHubHooks(true)
            onlyTriggerPhrase(false)
            permitAll(true)
            cron()
            whiteListTargetBranches {
              'org.jenkinsci.plugins.ghprb.GhprbBranch' {
                 branch supported_ros_branches.join(" ")
              }
            }
            triggerPhrase '.*(re)?run test(s).*'
            extensions {
                'org.jenkinsci.plugins.ghprb.extensions.status.GhprbSimpleStatus' {
                  commitStatusContext '${JOB_NAME}'
                  triggeredStatus 'starting deployment to build.osrfoundation.org'
                  startedStatus 'deploying to build.osrfoundation.org'
                }
              }
            }
          }
    } // end of with
  } // end of create method
}
