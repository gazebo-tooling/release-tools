package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
    - priorioty 100
    - keep only 15 builds
    - compiler warning
    - mail with test results
    - test results
    - cppcheck results
*/
class OSRFBrewCompilation extends OSRFOsXBase
{
  static void create(Job job)
  {
    OSRFLinuxBase.create(job)

    /* Properties from generic compilations */
    def generic_compilation = GenericCompilation()
    generic_compilation.create(job)

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
