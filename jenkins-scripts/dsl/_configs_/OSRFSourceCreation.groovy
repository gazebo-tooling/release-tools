package _configs_

import javaposse.jobdsl.dsl.Job
import _configs_.Globals

class OSRFSourceCreation
{
  static void addParameters(Job job, Map default_params = [:])
  {
    job.with
    {
      parameters {
        stringParam("PACKAGE_NAME",
                    default_params.find{ it.key == "PACKAGE_NAME"}?.value,
                    "Software name (i.e gz-cmake3)")
        stringParam("SOURCE_REPO_URI",
                    default_params.find{ it.key == "SOURCE_REPO_URI"}?.value,
                    "GitHub URI to release the sources from (i.e: https://github.com/gazebosim/gz-cmake.git)")
        stringParam("VERSION",
                    default_params.find{ it.key == "VERSION"}?.value,
                    "Packages version to be built or nightly (enable nightly build mode)")
        stringParam("RELEASE_VERSION",
                    default_params.find{ it.key == "RELEASE_VERSION"}?.value,
                    "Packages release version")
        stringParam("RELEASE_REPO_BRANCH",
                    default_params.find{ it.key == "RELEASE_REPO_BRANCH"}?.value,
                    "Branch from the -release repo to be used")
        stringParam("UPLOAD_TO_REPO",
                    default_params.find{ it.key == "UPLOAD_TO_REPO"}?.value,
                    "OSRF repo name to upload the package to: stable | prerelease | nightly | none (for testing proposes)")
      }
    }
  }

  static void create(Job job, Map default_params = [:])
  {
    OSRFLinuxBuildPkgBase.create(job)
    GenericRemoteToken.create(job)
    OSRFSourceCreation.addParameters(job, default_params)

    job.with
    {
      label Globals.nontest_label("docker")

      wrappers {
        preBuildCleanup()
      }

      properties {
        priority 100
      }

      steps {
        systemGroovyCommand("""\
          build.setDescription(
          '<b>' + build.buildVariableResolver.resolve('VERSION') + '-' +
          build.buildVariableResolver.resolve('RELEASE_VERSION') + '</b>' +
          '<br />' +
          'branch: ' + build.buildVariableResolver.resolve('RELEASE_REPO_BRANCH') + ' | ' +
          'upload to: ' + build.buildVariableResolver.resolve('UPLOAD_TO_REPO') +
          '<br />' +
          'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent()
        )

        shell("""\
            #!/bin/bash -xe
            export DISTRO=jammy
            export ARCH=amd64

            /bin/bash -x ./scripts/jenkins-scripts/docker/gz-source-generation.bash
            """.stripIndent())
      }
    }
  }
}
