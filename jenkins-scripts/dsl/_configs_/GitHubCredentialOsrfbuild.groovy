package _configs_

import javaposse.jobdsl.dsl.Job

class GitHubCredentials
{
  static void createOsrfbuildToken(Job job)
  {
    job.with
    {
      wrappers {
        // credential name needs to be in sync with provision code at infra/osrf-chef repo
        credentialsBinding {
          string('GITHUB_TOKEN', 'osrfbuild-token')
        }
      }
    }
  }
}
