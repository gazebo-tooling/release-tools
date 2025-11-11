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
        archiveArtifacts('pkgs/*')
      }

      // This creates a post-build script that changes ownership of pkgs directory to jenkins user
      // Runs regardless of build success or failure
      configure { project ->
        project / 'publishers' / 'org.jenkinsci.plugins.postbuildscript.PostBuildScript' << {
        config {
          scriptFiles()
          groovyScripts()
          buildSteps {
          'org.jenkinsci.plugins.postbuildscript.model.PostBuildStep' {
            results {
              string('SUCCESS')
              string('NOT_BUILT')
              string('ABORTED')
              string('FAILURE')
              string('UNSTABLE')
            }
            role('BOTH')
            executeOn('BOTH')
            buildSteps {
              'hudson.tasks.Shell' {
                command("[ -d \${WORKSPACE}/pkgs ] && sudo chown -R jenkins \${WORKSPACE}/pkgs")
                configuredLocalRules()
              }
            } // buildSteps
            stopOnFailure('false')
          } // org.jenkinsci.plugins.postbuildscript.model.PostBuildStep
          } // buildSteps
          markBuildUnstable('false')
        } // config
        } // PostBuildScript
      } // configure
    }
  }
}