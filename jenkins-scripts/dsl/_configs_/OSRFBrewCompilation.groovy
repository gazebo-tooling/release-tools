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
  static void create(Job job, enable_testing = true)
  {
    OSRFOsXBase.create(job)

    /* Properties from generic compilations */
    GenericCompilation.create(job, enable_testing)

    job.with
    {
      publishers
      {
         // compilers warnings
         warnings(['Clang (LLVM based)'], null) {
             thresholds(unstableTotal: [all: 0])
         }
      }
    } // end of job
  } // end of method createJob
} // end of class
