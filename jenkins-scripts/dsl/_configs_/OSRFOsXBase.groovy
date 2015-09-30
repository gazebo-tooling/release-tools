package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFUNIXBase

  Implements:
    - run on docker
    - colorize ansi output
*/
class OSRFOsXBase
{
   static void create(Job job)
   {
     // UNIX Base
     OSRFUNIXBase.create(job)

     job.with
     {
         label "osx"

         wrappers {
           colorizeOutput()
        }
     }
   }
}
