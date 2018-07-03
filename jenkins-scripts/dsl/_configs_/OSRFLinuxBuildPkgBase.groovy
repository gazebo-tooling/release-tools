package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
    - cleanup pkgs directory
    - keep only 10 builds
    - priority 100
    - keep only 10 last artifacts
    - parameters:
        - PACKAGE
        - VERSION
        - RELEASE_VERSION
        - LINUX_DISTRO
        - DISTRO
        - ARCH
        - SOURCE_TARBALL_URI
        - RELEASE_REPO_BRANCH
        - PACKAGE_ALIAS
        - UPLOAD_TO_REPO
    - launch repository_ng
    - publish artifacts
*/
class OSRFLinuxBuildPkgBase
{
   static void create(Job job, boolean include_upload=true)
   {
     OSRFLinuxBase.create(job)

     job.with
     {
       logRotator {
         artifactNumToKeep(10)
       }

       properties {
         priority 100
       }

       parameters {
         stringParam("PACKAGE",null,"Package name to be built")
         stringParam("VERSION",null,"Packages version to be built")
         stringParam("RELEASE_VERSION", null, "Packages release version")
         stringParam("LINUX_DISTRO", 'ubuntu', "Linux distribution to build packages for")
         stringParam("DISTRO", null, "Linux release inside LINUX_DISTRO to build packages for")
         stringParam("ARCH", null, "Architecture to build packages for")
         stringParam("UPLOAD_TO_REPO", null, "OSRF repo name to upload the package to")
       }

       wrappers {
         preBuildCleanup {
           includePattern('pkgs/*')
           // the sudo does not seems to be able to remove root owned packaged
           deleteCommand('sudo rm -rf %s')
         }
      }

      steps {
        systemGroovyCommand("""\
          build.setDescription(
          '<b>' + build.buildVariableResolver.resolve('VERSION') + '-' +
          build.buildVariableResolver.resolve('RELEASE_VERSION') + '</b>' +
          '(' + build.buildVariableResolver.resolve('LINUX_DISTRO') + '/' +
                build.buildVariableResolver.resolve('DISTRO') + '::' +
                build.buildVariableResolver.resolve('ARCH') + ')' +
          '<br />' +
          'upload to: ' + build.buildVariableResolver.resolve('UPLOAD_TO_REPO') +
          '<br />' +
          'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent()
        )
      }


      publishers
      {
        postBuildScripts {
          steps {
            shell("""\
              #!/bin/bash -xe

              [[ -d \${WORKSPACE}/pkgs ]] && sudo chown -R jenkins \${WORKSPACE}/pkgs
              """.stripIndent())
          }

          onlyIfBuildSucceeds(false)
          onlyIfBuildFails(false)
        }

        archiveArtifacts('pkgs/*')

        if (include_upload) {
          downstreamParameterized {
            trigger('repository_uploader_ng') {
              condition('SUCCESS')
              parameters {
                currentBuild()
                predefinedProp("PROJECT_NAME_TO_COPY_ARTIFACTS", "\${JOB_NAME}")
              }
            }
          }
        }
      }
    }
  }
}
