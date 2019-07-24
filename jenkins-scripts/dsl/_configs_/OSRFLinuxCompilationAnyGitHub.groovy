package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxCompilation

  Implements:
   - github scm with pr integration
   - github pull request builder
   - remove groovy scripts for description. github plugin set its own
*/
class OSRFLinuxCompilationAnyGitHub
{
  static void create(Job job,
                     String github_repo,
                     ArrayList supported_ros_distros,
                     boolean enable_testing  = true,
                     boolean enable_cppcheck = false)
  {
    // Do not include description from LinuxBase since the github pull request
    // builder set its own
    Globals.rtools_description = false
    OSRFLinuxCompilation.create(job, enable_testing, enable_cppcheck)
    Globals.rtools_description = true

    // Get repo name for relativeTargetDirectory
    String github_repo_name = github_repo.substring(github_repo.lastIndexOf("/") + 1)

    ArrayList supported_ros_branches = []
    supported_ros_distros.each { ros_distro ->
      if (ros_distro == 'eloquent') {
        // Latest unreleased distro points to ros2
        supported_ros_branches.add("ros2")
      } else if (ros_distro == 'dashing') {
        supported_ros_branches.add("dashing")
      } else if (ros_distro == 'crystal') {
        supported_ros_branches.add("crystal")
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
            // Only will be triggered in supported_ros_branches
            whiteListTargetBranches(supported_ros_branches)
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
} // end of class
