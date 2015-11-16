package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxCompilation
  -> GenericAnyJob

  Implements:
    - compiler warning
*/
class OSRFLinuxCompilationAny
{
  static void create(Job job, String repo)
  {
    OSRFLinuxCompilation.create(job)

    /* Properties from generic any */
    GenericAnyJob.create(job, repo)
  }
}
