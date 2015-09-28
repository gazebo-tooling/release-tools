package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
  - run on docker
  - colorize ansi output
*/
class OSRFLinuxBase extends OSRFBase
{
   static void create(Job job)
   {
     OSRFBase.create(job)
     job.with 
     {
         label "docker"
         
         wrappers {
           colorizeOutput()
        }
     }
   }
}
