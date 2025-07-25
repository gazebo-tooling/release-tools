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
OSRFReleasepy.create(releasepy_job, [DRY_RUN: false])
releasepy_job.with {
      blockOn("repository_uploader_packages") {
        blockLevel('GLOBAL')
        scanQueueFor('ALL')
      }
}
