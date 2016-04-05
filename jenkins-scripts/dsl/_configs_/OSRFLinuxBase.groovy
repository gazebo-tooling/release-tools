package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFUNIXBase

  Implements:
  - run on docker
  - concurrent builds
  - colorize ansi output
*/
class OSRFLinuxBase
{
   static void create(Job job)
   {
     // Base class for the job
     OSRFUNIXBase.create(job)

     job.with
     {
         label "docker"

         concurrentBuild()

         throttleConcurrentBuilds {
            maxPerNode(1)
            maxTotal(4)
         }

         wrappers {
           colorizeOutput()
        }
     }
   }
}
