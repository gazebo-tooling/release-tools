package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFBrewCompilation
  -> GenericAnyJobGitHub

  Implements:
    - compiler warning
*/
class OSRFBrewCompilationAnyGitHub
{
  static void create(Job job,
                     String github_repo,
                     boolean enable_testing  = true,
                     ArrayList supported_branches = [],
                     boolean enable_github_pr_integration = true,
                     boolean enable_cmake_warnings = false,
                     String arch)
  {
    OSRFBrewCompilation.create(job, enable_testing, enable_cmake_warnings, arch)

    GenericAnyJobGitHub.create(job, github_repo,
                               supported_branches,
                               enable_github_pr_integration)
  }
}
