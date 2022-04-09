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
             # try to recover from corrupted git on macOS
             git --version || brew remove --force git \$(brew deps git)
             git clone https://github.com/ignition-tooling/release-tools scripts -b \$RTOOLS_BRANCH
             """.stripIndent())
      }
    }
  }
}
