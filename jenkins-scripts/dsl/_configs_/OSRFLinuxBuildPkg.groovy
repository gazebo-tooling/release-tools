package _configs_

import javaposse.jobdsl.dsl.Job

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
  static void create(Job job)
  {
    OSRFLinuxBuildPkgBase.create(job)
    GenericRemoteToken.create(job)

    job.with
    {
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
        stringParam("SOURCE_TARBALL_URI", null, "URL to the tarball containing the package sources")
        stringParam("RELEASE_REPO_BRANCH", null, "Branch from the -release repo to be used")
        stringParam("PACKAGE_ALIAS", null, "If not empty, package name to be used instead of PACKAGE")
        stringParam("UPLOAD_TO_REPO", null, "OSRF repo name to upload the package to")
        stringParam("OSRF_REPOS_TO_USE", null, "OSRF repos name to use when building the package")
        labelParam('JENKINS_NODE_TAG') {
          description('Jenkins node or group to run build')
          defaultValue('docker')
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
        downstreamParameterized {
	  trigger('repository_uploader_ng') {
	    condition('SUCCESS')
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
    } // end of job
  } // end of method createJob
} // end of class
