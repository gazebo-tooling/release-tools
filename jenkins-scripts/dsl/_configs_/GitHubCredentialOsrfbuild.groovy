package _configs_

import javaposse.jobdsl.dsl.Job

class GitHubCredentialOsrfbuild
{
  static void create(Job job)
  {
    job.with
    {
      parameters {
        // credential name needs to be in sync with provision code at infra/osrf-chef repo
        credentialsBinding {
          string('GITHUB_TOKEN', 'osrfbuild-token')
        }
      }
    }
  }
}
