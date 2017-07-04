package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxBase
  -> GenericRemoteToken

  Implements:
    - priorioty 300
    - keep only 10 last artifacts
    - parameters:
        - PACKAGE
        - ARCHES
        - SOURCE_UBUNTU_DISTRO
        - DEST_UBUNTU_DISTRO
        - UPLOAD_TO_REPO
    - publish artifacts
    - launch repository_ng
*/
class OSRFLinuxBackportPkg

{
  static void create(Job job)
  {
    OSRFLinuxBase.create(job)
    GenericRemoteToken.create(job)

    job.with
    {
      properties {
        priority 300
      }

      logRotator {
        artifactNumToKeep(20)
      }

      parameters {
        stringParam("PACKAGE",null,"Package name to be built")
        stringParam("ARCHES", "amd64", "Space separated list of architectures to build packages for")
        stringParam("SOURCE_UBUNTU_DISTRO", null, "Distribution to get theç package from")
        stringParam("DEST_UBUNTU_DISTRO", null, "Distribution to generate the new package")
        stringParam("UPLOAD_TO_REPO", "none", "(CAUTION!! be sure before uploading package to a repository) OSRF repo name to upload the package to. none will skip the upload and leave artifacts available in jenkins")
        stringParam("OSRF_REPOS_TO_USE", null, "OSRF repos name to use when building the package")
      }

      steps {
        systemGroovyCommand("""\
          build.setDescription(
          '<b>' + build.buildVariableResolver.resolve('PACKAGE') + ' ' +
          build.buildVariableResolver.resolve('SOURCE_UBUNTU_DISTRO') + ' -> ' +
          build.buildVariableResolver.resolve('DEST_UBUNTU_DISTRO') + ' -> ' +
          '(' + build.buildVariableResolver.resolve('ARCHES') + ')' +
          '<br />' +
          'upload to: ' + build.buildVariableResolver.resolve('UPLOAD_TO_REPO') +
          '<br />' +
          'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent()
        )
      }

      publishers {
        archiveArtifacts('pkgs/*')

        flexiblePublish
        {
          conditionalAction {
            /* Upload if SUCCESS and UPLOAD_TO_REPO is not empty or none  */
            condition {
              and {
                status('SUCCESS','SUCCESS')
              } {
                not {
                  expression('none|None|^$','${ENV,var="UPLOAD_TO_REPO"}')
                }
              }
            }

            publishers {
              downstreamParameterized {
                trigger('repository_uploader_ng') {
                  parameters {
                    currentBuild()
                    predefinedProp("PROJECT_NAME_TO_COPY_ARTIFACTS", "\${JOB_NAME}")
                  }
                }
              }
            }
          }
        }
      } // end of publishers
    } // end of job
  } // end of method createJob
} // end of class
