import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org, scpeters@osrfoundation.org"

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
        shell("""\
              #!/bin/bash -xe

              /bin/bash -xe ./scripts/jenkins-scripts/lib/homebrew_formula_pullrequest.bash
              """.stripIndent())
   }
}
