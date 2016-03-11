package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFBrewCompilation
  -> GenericAnyJob

  Implements:
    - compiler warning
*/
class OSRFBrewCompilationAny
{
  static void create(Job job, String repo)
  {
    OSRFBrewCompilation.create(job)

    /* Properties from generic any */
    GenericAnyJob.create(job, repo)
  }
}
