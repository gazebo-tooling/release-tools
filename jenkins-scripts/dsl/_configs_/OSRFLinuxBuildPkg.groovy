package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxBuildPkgBase
  -> GenericRemoteToken

  Implements:
  parameters:
        - SOURCE_TARBALL_URI
        - RELEASE_REPO_BRANCH
        - PACKAGE_ALIAS
        - UPLOAD_TO_REPO

*/
class OSRFLinuxBuildPkg
{
  static void create(Job job)
  {
    OSRFLinuxBuildPkgBase.create(job)
    GenericRemoteToken.create(job)

    job.with
    {
      parameters {
        stringParam("SOURCE_TARBALL_URI", null, "URL to the tarball containing the package sources")
        stringParam("RELEASE_REPO_BRANCH", null, "Branch from the -release repo to be used")
        stringParam("PACKAGE_ALIAS", null, "If not empty, package name to be used instead of PACKAGE")
        stringParam("OSRF_REPOS_TO_USE", null, "OSRF repos name to use when building the package")
      }

      // add RELEASE_REPO_BRANCH to the build status overriding the BuilPkgBase
      // groovy job (they both will be in the configuration)
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

    } // end of job
  } // end of method createJob
} // end of class
