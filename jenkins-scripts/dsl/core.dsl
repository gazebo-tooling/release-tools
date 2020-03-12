import _configs_.*
import javaposse.jobdsl.dsl.Job

// -------------------------------------------------------------------
def set_status = job("_bitbucket-set_status")
OSRFLinuxBase.create(set_status)
set_status.with
{
  // TODO: workaround measure to avoid the collapse of lightweight-linux
  // when many jobs are triggered at the same time
  label "ash.intel.xenial || kitt.intel.bionic"

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
     stringParam('JENKINS_BUILD_DESC','',
                 'Description about building events')
     stringParam('BITBUCKET_STATUS',
                 '',
                 'inprogress | failed | ok')
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
      desc = 'repo:   ' + build.buildVariableResolver.resolve('JENKINS_BUILD_REPO') +'<br />'+
             'hash:   ' + build.buildVariableResolver.resolve('JENKINS_BUILD_HG_HASH') +'<br />'+
             'status: ' + build.buildVariableResolver.resolve('BITBUCKET_STATUS') + '<br />' +
             'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH')
        build.setDescription(desc)
        """.stripIndent()
    )

    shell("""\
          #!/bin/bash -xe

          /bin/bash -xe ./scripts/jenkins-scripts/_bitbucket_set_status.bash
          """.stripIndent())
  }

  configure { project ->
    project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
      unstableOnWarning true
      failBuildOnError false
      parsingRulesPath('/var/lib/jenkins/logparser_warn_on_mark_unstable')
    }
  } // end of configure
}
