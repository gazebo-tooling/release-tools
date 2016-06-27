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
  static void create(Job job, ArrayList supported_ros_distros,
                              boolean enable_testing  = true,
                              boolean enable_cppcheck = false)
  {
    OSRFLinuxCompilation.create(job, enable_testing, enable_cppcheck)

    ArrayList supported_ros_branches = []
    supported_ros_distros.each { ros_distro ->
      // Keep the toString method to be sure that String is used and not
      // GStringImp whihc will make the whole thing to fail.
      supported_ros_branches.add("${ros_distro}-devel".toString())
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
          extensions {
            relativeTargetDirectory("gazebo_ros_pkgs")
          }

          branch('${sha1}')
        }
      }

      triggers {
        githubPullRequest {
            admins(['osrf-jenkins', 'j-rivero'])
            useGitHubHooks()
            cron('')
            triggerPhrase('run test please')
            allowMembersOfWhitelistedOrgsAsAdmin()
            // Only will be triggered in supported_ros_branches
            whiteListTargetBranches(supported_ros_branches)
            permitAll(true)
            extensions {
                commitStatus {
                    context('${JOB_NAME}')
                    triggeredStatus('starting deployment to build.osrfoundation.org')
                    startedStatus('deploying to build.osrfoundation.org')
                    completedStatus('SUCCESS', 'All is well')
                    completedStatus('FAILURE', 'Something went wrong')
                    completedStatus('PENDING', 'still in progress...')
                    completedStatus('ERROR', 'Something went really wrong. Contact with the admin to solve the problem.')
                }
            }
        }
      } // end of triggers

      // remove the systemgroovy scripts (assuming that it sets the description)
      // since github pull request builder plugin set its own description
      configure { project ->
          project.remove(project / builders << "hudson.plugins.groovy.SystemGroovy")
      }
    } // end of with
  } // end of create method
} // end of class
