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
    super.create(job)
    job.with
    {
      publishers {
         warnings(['GNU C Compiler 4 (gcc)'])
      }
    }
  }
}
