package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
  - run on win
*/
class OSRFWinBase extends OSRFBase
{
   static void create(Job job)
   {
     OSRFBase.create(job)
     job.with
     {
         label "win"

         wrappers {
           colorizeOutput()
        }
     }
   }
}
