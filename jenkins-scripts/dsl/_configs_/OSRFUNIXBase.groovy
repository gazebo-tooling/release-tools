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
  // NOTE: The starting whitespace is important to match exactly to a label gpu-nvidia and not no-gpu-nvidia\
  static regexNvidiaLabel = ' gpu-nvidia+'

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
        // Manual insertion of xml for Naginator plugin because of this issue https://issues.jenkins.io/browse/JENKINS-66458
        configure { project -> 
          project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' {
            regexpForRerun(this.regexNvidiaLabel)
            checkRegexp(true)
            maxSchedule(1)
            delay(class: 'com.chikli.hudson.plugin.naginator.FixedDelay') {
              delay(70)
            }
          }
        }
        postBuildScripts {
          steps{
            conditionalSteps {
              condition {
                expression(this.regexNvidiaLabel,'${NODE_LABELS}')
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
                      println(" PROBLEM: NVIDIA driver/library version mismatch was detected in the log. Try to automatically resolve it:")
                      println("Removing labels and adding 'recovery-process' label to node")
                      node.setLabelString("recovery-process")
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