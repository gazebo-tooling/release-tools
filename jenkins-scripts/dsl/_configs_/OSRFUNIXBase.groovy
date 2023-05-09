package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFBase

  Implements:
     - colorize ansi output
     - allow concurrent builds
     - bash: RTOOLS checkout
*/
class OSRFUNIXBase extends OSRFBase
{
  static void create(Job job)
  {
    OSRFBase.create(job)

    job.with
    {
      concurrentBuild(true)

      throttleConcurrentBuilds {
        maxPerNode(1)
        maxTotal(4)
      }

      wrappers {
        colorizeOutput()
      }

      steps
      {
        shell("""\
             #!/bin/bash -xe

             [[ -d ./scripts ]] &&  rm -fr ./scripts
             git clone https://github.com/gazebo-tooling/release-tools scripts -b \$RTOOLS_BRANCH
             """.stripIndent())
      }
          publishers {
            postBuildScripts {
              steps{
              conditionalSteps {
                  condition {
                      expression('(.)*gpu-nvidia(.)*','${NODE_LABELS}')
                  }
                  steps {
                     systemGroovyCommand('''\
                        import hudson.model.Cause.UpstreamCause;
                        import hudson.model.*;
                    
                        def node = build.getBuiltOn()
                        def old_labels = node.getLabelString()

                        println("# BEGIN SECTION: NVIDIA MISMATCH RECOVERY")
                        if (!(build.getLog(1000) =~ "nvml error: driver/library version mismatch")) {
                          println(" NVIDIA driver/library version mismatch not detected in the log - Not performing any recovery automatic recovery step")
                          return 0;
                        } else {
                          try {
                            println("Removing labels and adding 'recovery-process' label to node")
                            node.setLabelString("recovery-process")

                            println("Requeuing job :" + build.project.name)
                            def job = Hudson.instance.getJob(build.project.name)
                          
                            def params = build.getAllActions().find{ it instanceof ParametersAction }?.parameters
                            def cause = new UpstreamCause(build)
                            // wait 70s to build again so that the computer has time to reboot (e.g only one agent available)
                            def scheduled = job.scheduleBuild2(70, cause, new ParametersAction(params))
                            if(!scheduled) {
                              throw new Exception("Job could not be requeued!")
                            }
                            println("Job requeued!")

                          } catch (Exception ex) {
                            println("ERROR - CANNOT PERFORM RECOVERY ACTIONS FOR NVIDIA ERROR")
                            println("Restoring to previous state")
                            node.setLabelString(old_labels)
                            throw ex
                          }
                        }
                        println("# END SECTION: NVIDIA MISMATCH RECOVERY")
                        '''.stripIndent()
                      )
                      shell("""sudo shutdown -r +1""")
                  }
                }
              }
                onlyIfBuildSucceeds(false)
                onlyIfBuildFails(true)
              }
              
          }
        
    }
  }
}