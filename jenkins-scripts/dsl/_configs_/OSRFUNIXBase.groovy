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
        // Add the new regex to naginator tag
        // There is no need to specify checkRegexp and maxSchedule because they are the default values
        HelperRetryFailures.create(job, [
          regexpForRerun: "nvml error: driver/library version mismatch",
          delay: 70
        ])
      }

      configure { project ->
        project / 'publishers' / 'org.jenkinsci.plugins.postbuildscript.PostBuildScript' << {
        config {
          scriptFiles()
          groovyScripts()
          buildSteps {
          'org.jenkinsci.plugins.postbuildscript.model.PostBuildStep' {
            results {
              string('FAILURE')
            }
            role('BOTH')
            executeOn('BOTH')
            buildSteps {
            'org.jenkinsci.plugins.conditionalbuildstep.ConditionalBuilder' {
              runner(class: 'org.jenkins_ci.plugins.run_condition.BuildStepRunner$Fail')
              conditionalbuilders {
                'hudson.plugins.groovy.SystemGroovy' {
                  bindings()
                  source(class: 'hudson.plugins.groovy.StringSystemScriptSource') {
                    script {
                      script('''\
import hudson.model.Cause.UpstreamCause;
import hudson.model.*;

def node = build.getBuiltOn()
def old_labels = node.getLabelString()

println("Checking if nvidia mismatch error is present in log")
if (!(build.getLog(1000) =~ "nvml error: driver/library version mismatch")) {
println(" NVIDIA driver/library version mismatch not detected in the log - Not performing any automatic recovery steps")
return 1;
} else {
println("# BEGIN SECTION: NVIDIA MISMATCH RECOVERY")
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
println("# END SECTION: NVIDIA MISMATCH RECOVERY")
}
''')
                      sandbox('false')
                      classpath()
                    }
                  }
                }
                'hudson.tasks.Shell' {
                  command('sudo shutdown -r +1')
                }
              }
              runCondition(class: 'org.jenkins_ci.plugins.run_condition.core.ExpressionCondition') {
                expression('(.)* gpu-nvidia (.)*')
                delegate.label('${NODE_LABELS}')
              }
            } // org.
            } // buildSteps
            stopOnFailure('false')
          } // org.
          } // buildSteps
          markBuildUnstable('false')
        } // config
        } // project
      } // configure
    }
  }
}
