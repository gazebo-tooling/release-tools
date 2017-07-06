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
    // Do not include description from LinuxBase since the github pull request
    // builder set its own
    Globals.rtools_description = false
    OSRFLinuxCompilation.create(job, enable_testing, enable_cppcheck)
    Globals.rtools_description = true

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
