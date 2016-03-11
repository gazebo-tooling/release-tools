package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
    - priorioty 100
    - keep only 15 builds
*/
class OSRFLinuxInstall extends OSRFLinuxBase
{
  static void create(Job job)
  {
    OSRFLinuxBase.create(job)

    job.with
    {
      priority 100

      logRotator {
        numToKeep(15)
      }
    }
  }
}
