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
