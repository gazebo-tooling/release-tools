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
  static void create(Job job, enable_testing = true)
  {
    OSRFWinBase.create(job)

    /* Properties from generic compilations */
    GenericCompilation.create(job, enable_testing)

    job.with
    {
      parameters {
	  stringParam('BUILD_TYPE', 'Release','Release|Debug compilation type for MSVC')
      }

      publishers
      {
        warnings(['MSBuild'], null) {
          thresholds(unstableTotal: [all: 0])
        }
      }
    } // end of job
  } // end of method createJob
} // end of class
