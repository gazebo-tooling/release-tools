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
  static void create(Job job, String repo, boolean enable_testing = true)
  {
    OSRFBrewCompilation.create(job, enable_testing)

    /* Properties from generic any */
    GenericAnyJob.create(job, repo)
  }
}
