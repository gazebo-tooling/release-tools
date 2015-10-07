package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFOsXBase
  -> GenericUNIXCompilation

  Implements:
    - compiler warning
*/
class OSRFBrewCompilation extends OSRFOsXBase
{
  static void create(Job job)
  {
    OSRFOsXBase.create(job)

    /* Properties from generic compilations */
    GenericUNIXCompilation.create(job)

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
