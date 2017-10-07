package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxCompilation
  -> GenericAnyJob

  Implements:
   - DEST_BRANCH parameter
*/
class OSRFLinuxCompilationAny
{
  static void create(Job job,
                     String repo,
                     boolean enable_testing = true,
                     boolean enable_cppcheck = true)
  {
    OSRFLinuxCompilation.create(job, enable_testing, enable_cppcheck)

    /* Properties from generic any */
    GenericAnyJob.create(job, repo)

    job.with
    {
      parameters
      {
        stringParam('DEST_BRANCH','default',
                    'Destination branch where the pull request will be merged.' +
                    'Mostly used to decide if calling to ABI checker')
      }
    } // end of with
  } // end of create method
} // end of class
