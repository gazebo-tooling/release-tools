package _configs_

import javaposse.jobdsl.dsl.Job

/*
   Implements:
     - parameters: SRC_REPO, SRC_BRANCH, JOB_DESCRIPTION
     - job.Description
     - scm check with SRC_REPO + SRC_BRANCH
*/


class GenericAnyJob
{
   static void create(Job job,
                     String github_repo,
                     ArrayList supported_ros_distros = [])
   {
     // setup special mail subject
     GenericMail.update_field(job, 'defaultSubject',
                    '$PROJECT_NAME - Branch: $SRC_BRANCH (#$BUILD_NUMBER) - $BUILD_STATUS!')
     GenericMail.update_field(job, 'defaultContent',
                    '$JOB_DESCRIPTION \n' + GenericCompilation.get_compilation_mail_content())

    // Get repo name for relativeTargetDirectory
    String github_repo_name = github_repo.substring(github_repo.lastIndexOf("/") + 1)

    ArrayList supported_ros_branches = []
    supported_ros_distros.each { ros_distro ->
      if (ros_distro == Globals.get_ros2_development_distro()) {
        // Latest unreleased distro points to ros2
        supported_ros_branches.add("ros2")
      } else if (Globals.get_ros2_suported_distros().contains(ros_distro)) {
        supported_ros_branches.add(ros_distro)
      } else {
        // Keep the toString method to be sure that String is used and not
        // GStringImp which will make the whole thing to fail.
        supported_ros_branches.add("${ros_distro}-devel".toString())
      }
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

      triggers {
        githubPullRequest {
            admins(['osrf-jenkins', 'j-rivero'])
            useGitHubHooks()
            cron('')
            triggerPhrase('.*(re)?run test(s).*')
            allowMembersOfWhitelistedOrgsAsAdmin()
            if (supported_ros_branches) {
              // Only will be triggered in supported_ros_branches
              whiteListTargetBranches(supported_ros_branches)
            }
            permitAll(true)
            extensions {
                commitStatus {
                    context('${JOB_NAME}')
                    triggeredStatus('starting deployment to build.osrfoundation.org')
                    startedStatus('deploying to build.osrfoundation.org')
                }
            }
        }
      } // end of triggers
    } // end of with
  } // end of create method
}
