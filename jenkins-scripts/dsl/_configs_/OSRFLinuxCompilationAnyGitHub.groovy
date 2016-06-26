package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxCompilation

  Implements:
   - github scm with pr integration
*/
class OSRFLinuxCompilationAnyGitHub
{
  static void create(Job job, ArrayList supported_ros_distros,
                              boolean enable_testing  = true,
                              boolean enable_cppcheck = false)
  {
    OSRFLinuxCompilation.create(job, enable_testing, enable_cppcheck)

    ArrayList supported_ros_branches = []
    supported_ros_distros.each { ros_distro ->
      supported_ros_branches.add("${ros_distro}-devel")
    }

    job.with
    {
      scm 
      {
        git {
          remote {
            github("ros-simulation/gazebo_ros_pkgs")
            refspec('+refs/pull/*:refs/remotes/origin/pr/*')
          }
          branch('${sha1}')
          relativeTargetDir("gazebo_ros_pkgs")
        }
      }

      triggers {
        githubPullRequest {
            admin('osrf-jenkins')
            cron('H/5 * * * *')
            triggerPhrase('OK to test')
            useGitHubHooks()
            autoCloseFailedPullRequests()
            allowMembersOfWhitelistedOrgsAsAdmin()
            whiteListTargetBranches(supported_ros_branches)
            extensions {
                commitStatus {
                    context('deploy to staging site')
                    triggeredStatus('starting deployment to staging site...')
                    startedStatus('deploying to staging site...')
                    statusUrl('http://mystatussite.com/prs')
                    completedStatus('SUCCESS', 'All is well')
                    completedStatus('FAILURE', 'Something went wrong. Investigate!')
                    completedStatus('PENDING', 'still in progress...')
                    completedStatus('ERROR', 'Something went really wrong. Investigate!')
                }
            }
        }
      } // end of triggers
    } // end of with
  } // end of create method
} // end of class
