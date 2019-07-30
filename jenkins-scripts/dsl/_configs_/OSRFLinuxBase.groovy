package _configs_

import javaposse.jobdsl.dsl.Job

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
         label "docker"
     }
   }
}
