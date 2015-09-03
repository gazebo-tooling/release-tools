package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
    - compiler warning
*/
class OSRFLinuxCompilation extends OSRFLinuxBase
{   
  static void create(Job job)
  {
    OSRFLinuxBase.create(job)
    job.with
    {
      priority 100

      publishers 
      {
         publishBuild {
           discardOldBuilds(daysToKeep = -1, numToKeep = 15)
	 }

         warnings(['GNU C Compiler 4 (gcc)'])

         archiveXUnit {
            jUnit {
                pattern 'build/test_results/*.xml'
            } 
         }

       }
    }
  }
}
