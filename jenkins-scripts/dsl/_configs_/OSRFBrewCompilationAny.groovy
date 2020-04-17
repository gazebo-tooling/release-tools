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
  static void create(Job job,
                     String github_repo,
                     boolean enable_testing  = true,
                     ArrayList supported_ros_distros = [])
  {
    OSRFBrewCompilation.create(job, enable_testing)

    GenericAnyJob.create(job, github_repo)
  }
}
