package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxCompilation
  -> GenericAnyJob

  Implements:
   - DEST_BRANCH parameter
*/
class OSRFLinuxCompilationAny
{
  static void create(Job job, String repo, boolean enable_testing = true)
  {
    OSRFLinuxCompilation.create(job)

    /* Properties from generic any */
    GenericAnyJob.create(job, repo)

    job.with
    {
      steps
      {
        shell("""\
        #!/bin/bash -xe

        /bin/bash -xe ./scripts/jenkins-scripts/_bitbucket_create_build_status_file.bash
        /bin/bash -xe ./scripts/jenkins-scripts/_bitbucket_set_status.bash inprogress
        """.stripIndent())
      }

      parameters
      {
        stringParam('DEST_BRANCH','default',
                    'Destination branch where the pull request will be merged.' +
                    'Mostly used to decide if calling to ABI checker')
      }

      /* To implement the set of build status this implementation uses
         post-build scripts. With current versions it was broken the option of
         including different Tasks or TaskProperties blocks in the Postbuildtask
         plugin. In the PostScript plugin there is no option to code the
         'Unstable' state. This approaches uses both in this way:
           - Postbuildtask to implement all failures options
             (configure block since DSL code available can not code
              different LogProperties)
           - PostBuildScript to implement the ok status
      */
      configure { project ->
        project / publishers << 'hudson.plugins.postbuildtask.PostbuildTask' {
          tasks
          {
            "hudson.plugins.postbuildtask.TaskProperties"
            {
              logTexts {
                "hudson.plugins.postbuildtask.LogProperties" {
                  logText('marked build as failure')
                  operator('OR')
                }
                "hudson.plugins.postbuildtask.LogProperties" {
                  logText('Build was aborted')
                  operator('OR')
                }
                "hudson.plugins.postbuildtask.LogProperties" {
                              logText('result to UNSTABLE')
                  operator('OR')
                }
                "hudson.plugins.postbuildtask.LogProperties" {
                  logText('result is FAILURE')
                  operator('OR')
                 }
               }
            EscalateStatus(false)
            RunIfJobSuccessful(false)
            script('/bin/bash -xe ./scripts/jenkins-scripts/_bitbucket_set_status.bash failed')
            } // end of TaskProperties
          } // end of tasks
        } // end of project
      } // end of configure

      publishers {
        postBuildScripts {
          steps {
            shell("""\
            #!/bin/bash -xe

            /bin/bash -xe ./scripts/jenkins-scripts/_bitbucket_set_status.bash ok
            """.stripIndent())
          }
          onlyIfBuildSucceeds(true)
        }
      } // end of publishers

    } // end of with
  } // end of create method
} // end of class
