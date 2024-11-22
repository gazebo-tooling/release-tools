package _configs_

import javaposse.jobdsl.dsl.Job

class GitHubCredentials
{
  static void createOsrfbuildToken(Job job)
  {
    job.with
    {
      wrappers {
        // Credential name needs to be in sync with provision code at infra/osrf-chef repo
        credentialsBinding {
          usernamePassword('OSRFBUILD_USER', 'OSRFBUILD_TOKEN', 'github-osrfbuild-credentials')
        }
      }
    }
  }
}
