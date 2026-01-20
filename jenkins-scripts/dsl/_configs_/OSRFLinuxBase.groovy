package _configs_

import javaposse.jobdsl.dsl.Job
import _configs_.Globals

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
        label Globals.nontest_label("docker")

        parameters {
          booleanParam('WORKSPACE_CLEANUP', true,
                       'Disable to keep build artifacts in Jenkins for debugging')
        }

        // No DSL code to support new changes in postBuildScripts, injection DSL
        // code directly. https://issues.jenkins.io/browse/JENKINS-66189
        configure { project ->
          project / 'publishers' / 'org.jenkinsci.plugins.postbuildscript.PostBuildScript' / 'config' {
            scriptFiles()
            groovyScripts()
            buildSteps {
              'org.jenkinsci.plugins.postbuildscript.model.PostBuildStep'  {
                results {
                  string 'SUCCESS'
                  string 'NOT_BUILT'
                  string 'ABORTED'
                  string 'FAILURE'
                  string 'UNSTABLE'
                }
                role('SLAVE')
                buildSteps {
                  'hudson.tasks.Shell' {
                    command("""\
                    #!/bin/bash -xe
                    # Clean up build directory no matter what have happened to the
                    # build except to test_results
                    if \${WORKSPACE_CLEANUP} && [ -d \${WORKSPACE}/build ]; then
                      sudo find \${WORKSPACE}/build -maxdepth 1 -mindepth 1 ! -name test_results -exec rm -rv {} +
                      echo "Disable WORKSPACE_CLEANUP parameter to avoid cleanup" > \${WORKSPACE}/build_dir_was_cleaned_up
                    fi
                    """.stripIndent())
                    configuredLocalRules()
                  }
                }
              }
            }
            markBuildUnstable(false)
          }
        }

        publishers {
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
