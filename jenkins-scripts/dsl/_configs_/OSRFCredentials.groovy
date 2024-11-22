package _configs_

import javaposse.jobdsl.dsl.Job

class OSRFCredentials
{
  static void setOSRFCrendentials(Job job, List crendentials_list)
  {
    job.with
    {
      wrappers {
        // Credential name needs to be in sync with provision code at infra/osrf-chef repo
        credentialsBinding {
          crendentials_list.each { credential_keyword ->
            if (credential_keyword == 'OSRFBUILD_GITHUB_TOKEN') {
                usernamePassword('OSRFBUILD_USER',
                                 'OSRFBUILD_TOKEN',
                                 'github-osrfbuild-apitoken')
            } else if (credential_keyword == 'OSRFBUILD_JENKINS_TOKEN') {
                usernamePassword('OSRFBUILD_JENKINS_USER',
                                 'OSRFBUILD_JENKINS_TOKEN',
                                 'jenkins-osrfbuild-apitoken')
            } else {
              print ("Credential not support: ${credential_keyword}")
              exit(1)
            }
          }
        }
      }
    }
  }

  static void allowOsrfbuildToRunTheBuild(Job job)
  {
    job.with {
      authorization {
        permission('hudson.model.Item.Read', 'osrfbuild')
        permission('hudson.model.Item.Build', 'osrfbuild')
      }
    }
  }
}
