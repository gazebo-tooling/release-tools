package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
    - cleanup pkgs directory
    - keep only 200 build logs and 25 build artifacts
    - publish artifacts
*/
class OSRFLinuxBuildPkgBase
{
   static void create(Job job)
   {
     OSRFLinuxBase.create(job)

     job.with
     {
       logRotator {
         artifactNumToKeep(25)
         numToKeep(125)
       }

       wrappers {
         preBuildCleanup {
           // the sudo does not seems to be able to remove root owned packaged
           deleteCommand('sudo rm -rf %s')
         }
      }

      publishers {
        postBuildScript {
          markBuildUnstable(false)
          buildSteps {
            postBuildStep {
              results(['SUCCESS', 'NOT_BUILT', 'ABORTED', 'FAILURE', 'UNSTABLE'])
              role('BOTH')
              stopOnFailure(false)
              buildSteps {
                shell {
                  command("""\
                    #!/bin/bash -xe

                    [[ -d \${WORKSPACE}/pkgs ]] && sudo chown -R jenkins \${WORKSPACE}/pkgs
                    """.stripIndent())
                }
              }
            }
          }
        }
        archiveArtifacts('pkgs/*')
      }
    }
  }
}
