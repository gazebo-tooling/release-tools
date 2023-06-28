import _configs_.*
import javaposse.jobdsl.dsl.Job

def NO_TESTING = false
def NO_BRANCHES = []
def NO_GITHUB_PR_INTEGRATION = false

def update_vcpkg_snapshot_job = job("_vcpkg_update_snapshot")
OSRFWinBase.create(update_vcpkg_snapshot_job)
update_vcpkg_snapshot_job.with
{
    parameters {
        nodeParam('TARGET_NODE') {
            description('Node to be updated')
        }
    }

    steps
    {
      systemGroovyCommand("""\
        job_description = 'RTOOLS_BRANCH: ' +
            build.buildVariableResolver.resolve('RTOOLS_BRANCH') + '<br />' +
            'TARGET_NODE: ' +
            '<b>' + build.buildVariableResolver.resolve('TARGET_NODE') + '</b>'
        build.setDescription(job_description)
      """.stripIndent())

      batchFile("""\
            call "%WORKSPACE%/scripts/jenkins-scripts/vcpkg-bootstrap.bat
            """.stripIndent())
    }
}

def ignition_testing_software = 'gazebo'
def testing_vcpkg_job = job("_vcpkg_testing_snapshot")
OSRFWinCompilationAnyGitHub.create(testing_vcpkg_job,
                                  "gazebosim/ign-${ignition_testing_software}",
                                  NO_TESTING, NO_BRANCHES, NO_GITHUB_PR_INTEGRATION)
testing_vcpkg_job.with
{
    parameters {
        stringParam('VCPKG_SNAPSHOT', '','vcpkg tag/release to test')
    }

    steps
    {
      label Globals.nontest_label("win_testing")

      batchFile("""\
            call "%WORKSPACE%/scripts/jenkins-scripts/vcpkg-bootstrap.bat" || exit /B %errorlevel%
            call "%WORKSPACE%/scripts/jenkins-scripts/ign_${ignition_testing_software}-default-devel-windows-amd64.bat"
            """.stripIndent())
    }
}

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
def nightly_labeler = job("_nightly_node_labeler")
OSRFBase.create(nightly_labeler)
nightly_labeler.with
{
  label Globals.nontest_label("master")

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
  label Globals.nontest_label("master")

  triggers {
    cron(Globals.CRON_HOURLY)
  }

  steps
  {
    systemGroovyCommand(readFileFromWorkspace('scripts/jenkins-scripts/tools/outdated-job-runner.groovy'))
  }
}
