package _configs_

import javaposse.jobdsl.dsl.Job
import _configs_.Globals

/*
  -> OSRFLinuxBuildPkgBase
  -> GenericRemoteToken

  Implements:
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
*/
class OSRFLinuxBuildPkg
{
  static void create(Job job, Map default_params = [:])
  {
    OSRFLinuxBuildPkgBase.create(job)
    GenericRemoteToken.create(job)

    job.with
    {
      properties {
        priority 100
      }

      parameters {
        stringParam("PACKAGE",
                    default_params.find{ it.key == "PACKAGE"}?.value,
                    "Package name to be built")
        stringParam("VERSION",
                    default_params.find{ it.key == "VERSION"}?.value,
                    "Packages version to be built or nightly (enable nightly build mode)")
        stringParam("RELEASE_VERSION",
                    default_params.find{ it.key == "RELEASE_VERSION"}?.value,
                    "Packages release version")
        stringParam("LINUX_DISTRO",
                    'ubuntu',
                    "Linux distribution to build packages for")
        stringParam("DISTRO",
                    default_params.find{ it.key == "DISTRO"}?.value,
                    "Linux release inside LINUX_DISTRO to build packages for")
        stringParam("ARCH",
                    default_params.find{ it.key == "ARCH"}?.value,
                    "Architecture to build packages for")
        stringParam("SOURCE_TARBALL_URI",
                    default_params.find{ it.key == "SOURCE_TARBALL_URI"}?.value,
                    "URL to the tarball containing the package sources")
        stringParam("RELEASE_REPO_BRANCH",
                    default_params.find{ it.key == "RELEASE_REPO_BRANCH"}?.value,
                    "Branch from the -release repo to be used")
        stringParam("PACKAGE_ALIAS",
                    default_params.find{ it.key == "PACKAGE_ALIAS"}?.value,
                    "If not empty, package name to be used instead of PACKAGE")
        stringParam("UPLOAD_TO_REPO",
                    default_params.find{ it.key == "UPLOAD_TO_REPO"}?.value,
                    "OSRF repo name to upload the package to: stable | prerelease | nightly | none (for testing proposes)")
        stringParam("OSRF_REPOS_TO_USE",
                    default_params.find{ it.key == "OSRF_REPOS_TO_USE"}?.value,
                    "OSRF repos name to use when building the package")
        labelParam('JENKINS_NODE_TAG') {
          description('Jenkins node or group to run build')
          defaultValue(Globals.nontest_label('docker'))
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
          'branch: ' + build.buildVariableResolver.resolve('RELEASE_REPO_BRANCH') + ' | ' +
          'upload to: ' + build.buildVariableResolver.resolve('UPLOAD_TO_REPO') +
          '<br />' +
          'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent()
        )
      }

      publishers {
        consoleParsing {
            projectRules('scripts/jenkins-scripts/parser_rules/debbuild_missing.parser')
            unstableOnWarning()
            failBuildOnError(false)
        }

        flexiblePublish
        {
          conditionalAction {
            condition {
              not {
                expression('none|None|^$','${ENV,var="UPLOAD_TO_REPO"}')
              }
            }
            publishers {
              downstreamParameterized {
                trigger('repository_uploader_packages') {
                  condition('UNSTABLE_OR_BETTER')
                  parameters {
                    currentBuild()
                    predefinedProp("PROJECT_NAME_TO_COPY_ARTIFACTS", "\${JOB_NAME}")
                    // Workaround to avoid problems on repository uploader. Real
                    // issue: https://issues.jenkins-ci.org/browse/JENKINS-45005
                    predefinedProp("JENKINS_NODE_TAG", "master")
                  }
                }
              }
            }
          }
        }
        configure { project ->
          project / 'properties' / 'hudson.plugins.copyartifact.CopyArtifactPermissionProperty' / 'projectNameList' {
            'string' 'repository_uploader_*'
          }
        }
      }
    } // end of job
  } // end of method createJob
} // end of class
