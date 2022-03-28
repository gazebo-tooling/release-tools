package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFUNIXBase

  Implements:
  - run on docker
*/
class OSRFLinuxBase
{
  static void create(Job job)
  {
    // Base class for the job
    OSRFUNIXBase.create(job)

    job.with
    {
        label "docker"

        parameters {
          booleanParam('WORKSPACE_CLEANUP', true,
                       'Disable to keep build artifacts in Jenkins for debugging')
        }

        publishers {
          postBuildScripts {
            steps {
              shell("""\
                #!/bin/bash -xe
                # Clean up build directory no matter what have happened to the
                # build
                if \${WORKSPACE_CLEANUP} && [[ -d \${WORKSPACE}/build ]]; then
                  sudo rm -fr \${WORKSPACE}/build
                  touch \${WORKSPACE}/build_dir_was_cleaned_up
                fi
                """.stripIndent())
            }

            onlyIfBuildSucceeds(false)
            onlyIfBuildFails(false)
         }

          archiveArtifacts {
            pattern('Dockerfile')
            pattern('build.sh')
            // there are no guarantees that a job using OSRFLinuxBase generate
            // these files (i.e: testing job). Do not fail if they are not
            // present
            allowEmpty(true)
        }
      }
    }
  }
}
