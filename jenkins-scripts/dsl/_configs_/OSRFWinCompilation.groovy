package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFWinBase
  -> GenericCompilation

  Implements:
    - compiler warning
*/
class OSRFWinCompilation extends OSRFWinBase
{
  static void create(Job job)
  {
    OSRFWinBase.create(job)

    /* Properties from generic compilations */
    GenericCompilation.create(job)

    job.with
    {
      parameters {
	  stringParam('BUILD_TYPE', 'Release','Release|Debug compilation type for MSVC')
      }

      publishers
      {
         // compilers warnings
         warnings(['MSBuild'])
      }
    } // end of job
  } // end of method createJob
} // end of class
