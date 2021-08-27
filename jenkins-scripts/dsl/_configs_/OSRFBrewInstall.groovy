package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
    - priority 400
    - keep only 75 builds
*/
class OSRFBrewInstall extends OSRFOsXBase
{
  static void create(Job job)
  {
    OSRFOsXBase.create(job)

    job.with
    {
      properties {
        priority 400
      }

      logRotator {
        numToKeep(75)
      }
    }
  }
}
