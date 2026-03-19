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
                     ArrayList supported_branches = [],
                     boolean enable_github_pr_integration = true,
                     boolean enable_cmake_warnings  = false)
  {
    OSRFWinCompilation.create(job, enable_testing, enable_cmake_warnings)

    /* Properties from generic any */
    // TODO(jrivero): remove replace once the rest of the migraton code is
    // deployed in the ignition.dsl file
    GenericAnyJobGitHub.create(job,
                               github_repo.replace('gz-gazebo','gz-sim'),
                               supported_branches,
                               enable_github_pr_integration)
    job.with
    {
      parameters
      {
        stringParam('CONDA_ENV_NAME', '','Force build to use a given conda environment if not empty')
      }
    }
  }
}
