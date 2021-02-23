package _configs_

import javaposse.jobdsl.dsl.Job

class GitHubCredentialOsrfbuild
{
  static void createToken(Job job)
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

  static void createKey(Job job)
  {
    job.with
    {
      wrappers {
        sshAgent('osrfbuild')
      }
    }
/*
       configure { project ->
          project  / wrappers /'org.jenkinsci.plugins.credentialsbinding.impl.SecretBuildWrapper' {
           'bindings' {
             'org.jenkinsci.plugins.credentialsbinding.impl.SSHUserPrivateKeyBinding' {
              // credentialId needs to be in sync with provision code at infra/osrf-chef repo
              credentialsId 'osrfbuild'
              keyFileVariable 'OSRFBUILD_KEY'
              usernameVariable()
              passphraseVariable()
            }
          }
        }
      }
    }
  }
*/
}
