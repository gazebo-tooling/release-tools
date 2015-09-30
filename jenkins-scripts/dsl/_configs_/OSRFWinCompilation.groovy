package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFWinBase
  -> GenericCompilation

  Implements:
    - compiler warning
*/
class OSRFBrewCompilation extends OSRFWinBase
{
  static void create(Job job)
  {
    OSRFWinBase.create(job)

    /* Properties from generic compilations */
    GenericCompilation.create(job)

    job.with
    {
      publishers
      {
         // compilers warnings
         warnings(['MSBuild'])
      }
    } // end of job
  } // end of method createJob
} // end of class
