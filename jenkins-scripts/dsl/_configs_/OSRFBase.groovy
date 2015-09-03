package _configs_

import javaposse.jobdsl.dsl.Job
  
/* 
  Implements:
     - description
*/
class OSRFBase
{
   static void create(Job job)
   {
     job.with {
     	description 'Automatic generated job by DSL jenkins. Please do not edit manually'
     }
   }
}
