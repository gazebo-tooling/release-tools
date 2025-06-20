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

      publishers
      {
        // This creates a post-build script that changes ownership of pkgs directory to jenkins user
        // Runs regardless of build success or failure
        configure { project ->
          project / 'publishers' / 'org.jenkinsci.plugins.postbuildscript.PostBuildScript' << {
            config {
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
                      command('''#!/bin/bash -xe

  [[ -d ${WORKSPACE}/pkgs ]] && sudo chown -R jenkins ${WORKSPACE}/pkgs
  ''')
                    }
                  } // buildSteps
                  stopOnFailure('false')
                } // buildSteps
              } // config
              markBuildUnstable('false')
            } // project
          } // project
        } // configure        
        archiveArtifacts('pkgs/*')
      } // publishers
    } //  with
  } //create 
} // class
