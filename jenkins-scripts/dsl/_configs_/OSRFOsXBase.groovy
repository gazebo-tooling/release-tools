package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFOsXBase

  Implements:
    - run on osx
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
     }
   }
}
