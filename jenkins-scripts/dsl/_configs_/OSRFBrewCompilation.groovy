package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFOsXBase
  -> GenericCompilation

  Implements:
    - compiler warning
*/
class OSRFBrewCompilation extends OSRFOsXBase
{
  static void create(Job job)
  {
    OSRFOsXBase.create(job)

    /* Properties from generic compilations */
    GenericCompilation.create(job)

    job.with
    {
      publishers
      {
         // compilers warnings
         warnings(['Clang (LLVM based)'])
      }
    } // end of job
  } // end of method createJob
} // end of class
