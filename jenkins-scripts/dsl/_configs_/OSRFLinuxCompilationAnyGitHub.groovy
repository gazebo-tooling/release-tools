package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxCompilation

  Implements:
   - github scm with pr integration
   - github pull request builder
   - remove groovy scripts for description. github plugin set its own
*/
class OSRFLinuxCompilationAnyGitHub
{
  static void create(Job job,
                     String github_repo,
                     boolean enable_testing  = true,
                     boolean enable_cppcheck = true,
                     ArrayList supported_branches = [],
                     boolean enable_github_pr_integration = true)
  {
    // Do not include description from LinuxBase since the github pull request
    // builder set its own
    Globals.rtools_description = false
    OSRFLinuxCompilation.create(job, enable_testing, enable_cppcheck)
    Globals.rtools_description = true

    GenericAnyJobGitHub.create(job, github_repo, supported_branches, enable_github_pr_integration)

    job.with
    {
      parameters
      {
        stringParam('GZDEV_BRANCH', 'master', 'gzdev repository branch name. Useful for modifying the repositories (stable|prerelease|...) used for testing the packages ')
      }
    }
  }
} // end of class
