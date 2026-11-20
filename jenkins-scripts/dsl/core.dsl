import _configs_.*
import javaposse.jobdsl.dsl.Job

def NO_TESTING = false
def NO_BRANCHES = []
def NO_GITHUB_PR_INTEGRATION = false

// -------------------------------------------------------------------
def reprepro = job("reprepro_importer")
OSRFUNIXBase.create(reprepro)
reprepro.with
{
  label Globals.nontest_label("packages")

  parameters
  {
    stringParam('UPLOAD_TO_REPO','ubuntu-testing',
                'Repository to host the new imported packages. Should match entry in reprepro.ini')
    stringParam('REPREPRO_GIT_BRANCH', '',
                'Branch of the repository to be used for code and REPREPRO_IMPORT_YAML_FILE (empty implies uses default)')
    stringParam('REPREPRO_IMPORT_YAML_FILE','',
                'Name of the reprepo import file')
    booleanParam('COMMIT',false,
                 'Real action to import packages')
  }

  wrappers {
     preBuildCleanup()
  }

  logRotator {
    numToKeep(15)
  }

  steps
  {
    systemGroovyCommand("""\
      desc = 'repo:     ' + build.buildVariableResolver.resolve('UPLOAD_TO_REPO') +'<br />'+
             'config: ' + build.buildVariableResolver.resolve('BITBUCKET_STATUS') + '<br />' +
             'commit:   ' + build.buildVariableResolver.resolve('REPREPRO_IMPORT_YAML_FILE') +'<br />'+
             'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH')
        build.setDescription(desc)
        """.stripIndent()
    )

    shell("""\
          #!/bin/bash -xe

          /bin/bash -xe ./scripts/jenkins-scripts/docker/reprepro_updater.bash
          """.stripIndent())
  }
}

// -------------------------------------------------------------------
// Upload packages built by upstream jobs (-debbuilder, -source, brew
// bottles, ...) to the reprepro repositories and/or S3. Upstream jobs
// trigger it passing their own parameters plus PROJECT_NAME_TO_COPY_ARTIFACTS.
// Any parameter not declared here is dropped by Jenkins (SECURITY-170)
// unless it is listed in hudson.model.ParametersAction.safeParameters.
def repository_uploader = job("repository_uploader_packages")
OSRFUNIXBase.create(repository_uploader)
OSRFCredentials.allowOsrfbuildToRunTheBuild(repository_uploader)
repository_uploader.with
{
  label Globals.nontest_label("packages")

  // reprepro and S3 uploads must not run in parallel
  concurrentBuild(false)

  parameters
  {
    stringParam('LINUX_DISTRO', 'ubuntu',
                'Linux distribution of the packages: ubuntu | debian')
    stringParam('DISTRO', '',
                'Distribution codename used as reprepro target (i.e: noble)')
    stringParam('ARCH', '',
                'Only needed to display job information in jenkins')
    stringParam('PACKAGE_ALIAS', '',
                'Package name used for the S3 path of the .deb files and the job description')
    stringParam('VERSION', '',
                'Only needed to display job information in jenkins')
    stringParam('RELEASE_VERSION', '',
                'Only needed to display job information in jenkins')
    stringParam('S3_UPLOAD_PATH', '',
                'S3 path inside osrf-distributions to upload S3_FILES_TO_UPLOAD')
    booleanParam('S3_UPLOAD_CANONICAL_PATH', true,
                 'Correct S3_UPLOAD_PATH to get rid of versioned paths (i.e gazebo6/releases/ -> gazebo/releases/)')
    stringParam('UPLOAD_TO_REPO', '',
                'Repository to upload packages: stable | prerelease | nightly | only_s3_upload | none')
    stringParam('PROJECT_NAME_TO_COPY_ARTIFACTS', '',
                'Job name to copy artifacts from')
    stringParam('S3_FILES_TO_UPLOAD', '',
                'File names inside pkgs/ to upload to S3_UPLOAD_PATH')
  }

  wrappers {
    preBuildCleanup()
  }

  // ~120 builds per week in normal weeks, the count cap covers release bursts
  logRotator {
    daysToKeep(7)
    numToKeep(500)
  }

  steps
  {
    // release.py prints a search URL that relies on this description
    // format ("PACKAGE_ALIAS VERSION"). Keep both in sync.
    //
    // The text is byte-identical (trailing spaces included) to the script
    // approved for the hand-made job. Changing it requires a new approval
    // in "In-process Script Approval" or every upload fails.
    systemGroovyCommand([
      "build.setDescription(",
      "'<b>' + build.buildVariableResolver.resolve('PACKAGE_ALIAS') + '</b>' + ",
      "'  <b>' + build.buildVariableResolver.resolve('VERSION') + '-' + ",
      "build.buildVariableResolver.resolve('RELEASE_VERSION') + '</b>' +",
      "'(' + build.buildVariableResolver.resolve('LINUX_DISTRO') + '/' + ",
      "      build.buildVariableResolver.resolve('DISTRO') + '::' + ",
      "      build.buildVariableResolver.resolve('ARCH') + ')' +",
      "'<br />' +",
      "'upload to: ' + build.buildVariableResolver.resolve('UPLOAD_TO_REPO') +",
      "'<br />' +",
      "'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));",
      ""
    ].join('\n'))

    copyArtifacts('${PROJECT_NAME_TO_COPY_ARTIFACTS}')
    {
      includePatterns('pkgs/*.deb', 'pkgs/*.ddeb', 'pkgs/*.dsc',
                      'pkgs/*debian*', 'pkgs/*orig*', 'pkgs/*.zip',
                      'pkgs/*.tar.*', 'pkgs/*.json')
      targetDirectory('pkgs')
      flatten()
      buildSelector {
        upstreamBuild()
      }
    }

    shell("""\
          #!/bin/bash -xe

          /bin/bash -xe ./scripts/jenkins-scripts/lib/repository_uploader.bash
          """.stripIndent())
  }

  publishers
  {
    // repository_uploader.bash prints MARK_AS_UNSTABLE when reprepro
    // can not import a .dsc
    configure { project ->
      project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
        unstableOnWarning true
        failBuildOnError false
        parsingRulesPath('/var/lib/jenkins/logparser_warn_on_mark_unstable')
      }
    }
  }
}

// -------------------------------------------------------------------
def agent_usage_statistics = job("_agent_usage_statistics")
OSRFBase.create(agent_usage_statistics)
agent_usage_statistics.with
{
  label Globals.nontest_label("built-in")

  triggers {
    // TODO: running only in March to see how much disk space it needs
    cron('H/5 * * 3 *')
  }

  steps
  {
    systemGroovyCommand(readFileFromWorkspace('scripts/jenkins-scripts/tools/agent-usage-statistics.groovy'))
  }
}

// -------------------------------------------------------------------
def nightly_labeler = job("_nightly_node_labeler")
OSRFBase.create(nightly_labeler)
nightly_labeler.with
{
  label Globals.nontest_label("built-in")

  triggers {
    cron(Globals.CRON_NIGHTLY_NODES)
  }

  steps
  {
    // path root changes from standalone to Jenkins. Be careful
    systemGroovyCommand(readFileFromWorkspace('scripts/jenkins-scripts/tools/label-assignment-backstop.groovy'))
  }

  publishers
  {
    // Added the checker result parser (UNSTABLE if not success)
    configure { project ->
      project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
        unstableOnWarning true
        failBuildOnError false
        parsingRulesPath('/var/lib/jenkins/logparser_warn_on_mark_unstable')
      }
    }
  }
}

// -------------------------------------------------------------------
def outdated_job_runner = job("_outdated_job_runner")
OSRFBase.create(outdated_job_runner)
outdated_job_runner.with
{
  label Globals.nontest_label("built-in")

  triggers {
    cron(Globals.CRON_HOURLY)
  }

  steps
  {
    systemGroovyCommand(readFileFromWorkspace('scripts/jenkins-scripts/tools/outdated-job-runner.groovy'))
  }
}

// -------------------------------------------------------------------
def releasepy_job = job("_releasepy")
OSRFReleasepy.create(releasepy_job, [DRY_RUN: false, ONLY_BUMP_REVISION_LINUX: false])
releasepy_job.with {
      blockOn("repository_uploader_packages") {
        blockLevel('GLOBAL')
        scanQueueFor('ALL')
      }
}
