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
  static void create(Job job,
                     String github_repo,
                     boolean enable_testing  = true,
                     ArrayList supported_ros_distros = [])
  {
    OSRFWinCompilation.create(job, enable_testing)

    /* Properties from generic any */
    GenericAnyJob.create(job, github_repo)
  }
}
