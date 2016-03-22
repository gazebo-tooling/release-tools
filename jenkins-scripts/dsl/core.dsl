import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org, scpeters@osrfoundation.org"

def build_status_file_name  = Globals.bitbucket_build_status_file_name
def build_status_file_path  = Globals.bitbucket_build_status_file_path

// -------------------------------------------------------------------
// BREW pull request SHA updater
def release_job = job("generic-release-homebrew_pull_request_updater")
OSRFLinuxBase.create(release_job)
GenericRemoteToken.create(release_job)
release_job.with
{
   label "master"

   wrappers {
        preBuildCleanup()
   }

   parameters
   {
     stringParam("PACKAGE_ALIAS", '',
                 'Name used for the package which may differ of the original repo name')
     stringParam("SOURCE_TARBALL_URI", '',
                 'URI with the tarball of the latest release')
     stringParam("VERSION", '',
                 'Version of the package just released')
     stringParam('SOURCE_TARBALL_SHA','',
                 'SHA Hash of the tarball file')
   }

   steps {
        systemGroovyCommand("""\
          build.setDescription(
          '<b>' + build.buildVariableResolver.resolve('PACKAGE_ALIAS') + '-' +
          build.buildVariableResolver.resolve('VERSION') + '</b>' +
          '<br />' +
          'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent()
        )

        shell("""\
              #!/bin/bash -xe

              /bin/bash -xe ./scripts/jenkins-scripts/lib/homebrew_formula_pullrequest.bash
              """.stripIndent())
   }
}

// -------------------------------------------------------------------
def create_status_name = '_bitbucket-create_build_status_file'
def create_status = job(create_status_name)
OSRFLinuxBase.create(create_status)
create_status.with
{
  parameters
  {
     stringParam('JENKINS_BUILD_REPO','',
                 'Repo to test')
     stringParam('JENKINS_BUILD_HG_HASH','',
                 'Hash of commit to test')
     stringParam('JENKINS_BUILD_JOB_NAME','',
                 'Branch of SRC_REPO to test')
     stringParam('JENKINS_BUILD_URL','',
                 'Link to jenkins main ci job')
  }

  wrappers {
     preBuildCleanup()
  }

  steps
  {
    systemGroovyCommand("""\
          desc = 'repo: ' + build.buildVariableResolver.resolve('JENKINS_BUILD_REPO') + '<br />' +
                 'hash: ' + build.buildVariableResolver.resolve('JENKINS_BUILD_HG_HASH') + '<br />' +
                 'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH')
        build.setDescription(desc)
        """.stripIndent()
    )

    shell("""\
          #!/bin/bash -xe

          export BITBUCKET_BUILD_STATUS_FILE="${build_status_file_path}"
          /bin/bash -xe ./scripts/jenkins-scripts/_bitbucket_create_build_status_file.bash
          """.stripIndent())
  }

  publishers
  {
    archiveArtifacts
    {
      pattern("${build_status_file_name}")
      onlyIfSuccessful()
    }
  }
}

// -------------------------------------------------------------------
def set_status = job("_bitbucket-set_status")
OSRFLinuxBase.create(set_status)
set_status.with
{
  parameters
  {
     stringParam('CREATE_CONFIG_BUILD_NUM',
                 '',
                 'build number from parent job to get the metadata file')
     stringParam('BITBUCKET_STATUS',
                 '',
                 'inprogress | fail | ok')
  }

  wrappers {
     preBuildCleanup()
  }

  steps
  {
    systemGroovyCommand("""\
          desc = 'status: ' + build.buildVariableResolver.resolve('BITBUCKET_STATUS') + '<br />' +
                 'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH')
        build.setDescription(desc)
        """.stripIndent()
    )

    copyArtifacts("${create_status_name}") {
      includePatterns("${build_status_file_name}")
      flatten()
      buildSelector {
         buildNumber('${CREATE_CONFIG_BUILD_NUM}')
      }
    }


    shell("""\
          #!/bin/bash -xe

          export BITBUCKET_BUILD_STATUS_FILE="${build_status_file_path}"
          /bin/bash -xe ./scripts/jenkins-scripts/_bitbucket_set_status.bash
          """.stripIndent())
  }
}
