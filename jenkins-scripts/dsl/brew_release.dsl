import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org, scpeters@osrfoundation.org"

// first distro in list is used as touchstone
brew_supported_distros         = [ "bigsur", "monterey", "ventura" ]
bottle_hash_updater_job_name   = 'generic-release-homebrew_pr_bottle_hash_updater'
bottle_builder_job_name        = 'generic-release-homebrew_triggered_bottle_builder'
directory_for_bottles          = 'pkgs'

def DISABLE_TESTS = false
def NO_SUPPORTED_BRANCHES = []
def DISABLE_GITHUB_INTEGRATION = false

/*
  release.py
  -> update upstream source tarball hash in formula
     (1. generic-release-homebrew_pull_request_updater)
      -> build bottles for supported distros
        (2.generic-release-homebrew_bottle_builder-<distro>)
        -> upload bottle to S3
          (3. repository_uploader)
        -> update bottle hash in formula
          (4. generic-release-homebrew_pr_bottle_hash_updater)
*/

// parameters needed by different reasons in the scripts or in the downstream
// called jobs
void include_common_params(Job job)
{
  job.with
  {
    properties {
      priority 100
    }

    parameters
    {
     stringParam("PACKAGE", '',
                 'Package name. Needed in downstream jobs.')
     stringParam("PACKAGE_ALIAS", '',
                 'Name used for the package which may differ of the original repo name')
     stringParam("VERSION", '',
                 'Version of the package just released')
    }
  }
}

// -------------------------------------------------------------------
// 1. BREW pull request SHA updater
def release_job = job("generic-release-homebrew_pull_request_updater")
OSRFUNIXBase.create(release_job)
GenericRemoteToken.create(release_job)

include_common_params(release_job)
release_job.with
{
   String PR_URL_export_file_name = 'pull_request_created.properties'
   String PR_URL_export_file = '${WORKSPACE}/' + PR_URL_export_file_name

   label Globals.nontest_label("master")

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

   steps
   {
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

          export PR_URL_export_file=${PR_URL_export_file}
          /bin/bash -xe ./scripts/jenkins-scripts/lib/homebrew_formula_pullrequest.bash
          """.stripIndent())
   }

   // call to the bottle
   publishers
   {
     // Added the checker result parser (UNSTABLE if not compatible)
     // IMPORTANT: the order of the steps here is important. Leave the configure
     // block first.
     configure { project ->
       project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
          unstableOnWarning true
          failBuildOnError false
          parsingRulesPath('/var/lib/jenkins/logparser_warn_on_mark_unstable')
       }
     } // end of configure

     archiveArtifacts
     {
       pattern("${PR_URL_export_file_name}")
       allowEmpty()
     }
   }
}

// -------------------------------------------------------------------
// 2. BREW bottle creation MATRIX job from pullrequest
def bottle_job_builder = matrixJob(bottle_builder_job_name)
// set enable_github_pr_integration flag to false so we can customize trigger behavior
OSRFBrewCompilationAnyGitHub.create(bottle_job_builder,
                                    "osrf/homebrew-simulation",
                                    DISABLE_TESTS,
                                    NO_SUPPORTED_BRANCHES,
                                    DISABLE_GITHUB_INTEGRATION)
GenericRemoteToken.create(bottle_job_builder)

bottle_job_builder.with
{
   wrappers {
        preBuildCleanup()
        credentialsBinding {
          // crendetial name needs to be in sync with provision code at infra/osrf-chef repo
          string('GITHUB_TOKEN', 'osrf-migration-token')
        }
   }

   properties {
     priority 100
   }

   logRotator {
     artifactNumToKeep(10)
   }

   axes {
     // use labels osx_$osxdistro
     label('label', brew_supported_distros.collect { "osx_$it" })
   }

   // Arbitrary choose the first one for touchstone label
   String touchstone_label = '"osx_' + brew_supported_distros[0] + '"'

   touchStoneFilter("label == ${touchstone_label}")

   steps {
        systemGroovyCommand("""\
          build.setDescription(
          '<b><a href="https://github.com/osrf/homebrew-simulation/pull/' +
          build.buildVariableResolver.resolve('ghprbPullId') + '">PR ' +
          build.buildVariableResolver.resolve('ghprbPullId') + '</a></b>' +
          '<br />' +
          'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent()
        )

        shell("""\
              #!/bin/bash -xe
              /bin/bash -xe ./scripts/jenkins-scripts/lib/homebrew_bottle_creation.bash
              """.stripIndent())
   }

   configure { project ->
     project / 'properties' / 'hudson.plugins.copyartifact.CopyArtifactPermissionProperty' / 'projectNameList' {
      'string' "${bottle_hash_updater_job_name}"
     }
     project  / triggers / 'org.jenkinsci.plugins.ghprb.GhprbTrigger' {
         adminlist 'osrf-jenkins j-rivero scpeters'
         orgslist 'gazebosim'
         whitelist 'osrfbuild'
         useGitHubHooks(true)
         allowMembersOfWhitelistedOrgsAsAdmin(true)
         useGitHubHooks(true)
         onlyTriggerPhrase(true)
         permitAll(false)
         cron()
         triggerPhrase '.*build bottle.*'
         extensions {
             'org.jenkinsci.plugins.ghprb.extensions.status.GhprbSimpleStatus' {
               commitStatusContext '${JOB_NAME}'
               triggeredStatus 'starting deployment to build.osrfoundation.org'
               startedStatus 'deploying to build.osrfoundation.org'
               addTestResults(true)
             }
         }
     }
   }

   publishers {
     archiveArtifacts
     {
       pattern("${directory_for_bottles}/*")
       allowEmpty()
     }

     // call to the repository_uploader_packages to upload to S3 the binary
     downstreamParameterized
     {
        trigger('repository_uploader_packages') {
          condition('SUCCESS')
          parameters {
            currentBuild()
              predefinedProp("PROJECT_NAME_TO_COPY_ARTIFACTS", "\${JOB_NAME}")
              predefinedProp("UPLOAD_TO_REPO", "only_s3_upload")
              predefinedProp("ARCH",           "64bits")
          }
        }
     }

     // call to the bottle hash updater
     downstreamParameterized
     {
        trigger(bottle_hash_updater_job_name)
        {
          condition('SUCCESS')
          parameters {
            currentBuild()
              predefinedProp("PULL_REQUEST_URL", "https://github.com/osrf/homebrew-simulation/pull/\${ghprbPullId}")
              predefinedProp("ghprbCommentBody", "\${ghprbCommentBody}")
          }
        }
     }
  }
}

// -------------------------------------------------------------------
// 4. BREW bottle hash update
def bottle_job_hash_updater = job(bottle_hash_updater_job_name)
OSRFUNIXBase.create(bottle_job_hash_updater)
GenericRemoteToken.create(bottle_job_hash_updater)

include_common_params(bottle_job_hash_updater)
bottle_job_hash_updater.with
{
  label Globals.nontest_label("master")

  wrappers
  {
    preBuildCleanup()
  }

  parameters
  {
    // copy the github trigger comment for extra parameter parsing
    stringParam("ghprbCommentBody", '',
                'GitHub trigger comment, which can be parsed for extra parameters.')
    // reuse the pull request created by homebrew_pull_request_updater in step 1
    stringParam("PULL_REQUEST_URL", '',
                'Pull request URL (osrf/homebrew-simulation) pointing to a pull request.')
  }

  steps
  {
    systemGroovyCommand("""\
        build.setDescription(
        '<b>' + build.buildVariableResolver.resolve('PACKAGE_ALIAS') + '-' +
        build.buildVariableResolver.resolve('VERSION') + '</b>' +
        '<br />' +
        'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
        """.stripIndent()
    )

    copyArtifacts(bottle_builder_job_name) {
      includePatterns('pkgs/*.json')
      excludePatterns('pkgs/*.tar.gz')
      optional()
      targetDirectory(directory_for_bottles)
      flatten()
      buildSelector {
        upstreamBuild()
      }
    }

    shell("""\
          #!/bin/bash -xe

          /bin/bash -xe ./scripts/jenkins-scripts/lib/homebrew_bottle_pullrequest.bash
          """.stripIndent())
  }

  publishers
  {
    // Added the checker result parser (UNSTABLE if not compatible)
    // IMPORTANT: the order of the steps here is important. Leave the configure
    // block first.
    configure { project ->
      project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
        unstableOnWarning true
        failBuildOnError false
        parsingRulesPath('/var/lib/jenkins/logparser_warn_on_mark_unstable')
      }
    } // end of configure
  }
}

