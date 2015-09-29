package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxBase
  -> GenericCompilation

  Implements:
    - compiler warning
*/
class OSRFLinuxCompilation extends OSRFLinuxBase
{
  static void create(Job job)
  {
    OSRFLinuxBase.create(job)

    /* Properties from generic compilations */
    GenericCompilation.create(job)

    job.with
    {
      publishers
      {
         // compilers warnings
         warnings(['GNU C Compiler 4 (gcc)'])
      } // end of publishers
    } // end of job
  } // end of method createJob
} // end of class
