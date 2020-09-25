package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFWinCompilation
  -> GenericAnyJobGitHub

  Implements:
    - compiler warning
*/
class OSRFWinCompilationAnyGitHub
{
  static void create(Job job,
                     String github_repo,
                     boolean enable_testing  = true,
                     ArrayList supported_ros_distros = [])
  {
    OSRFWinCompilation.create(job, enable_testing)

    /* Properties from generic any */
    GenericAnyJobGitHub.create(job, github_repo, supported_ros_distros)
  }
}
