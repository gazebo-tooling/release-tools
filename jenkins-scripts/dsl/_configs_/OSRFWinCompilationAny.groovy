package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFWinCompilation
  -> GenericAnyJob

  Implements:
    - compiler warning
*/
class OSRFWinCompilationAny
{
  static void create(Job job, String repo)
  {
    OSRFWinCompilation.create(job)

    /* Properties from generic any */
    GenericAnyJob.create(job, repo)
  }
}
