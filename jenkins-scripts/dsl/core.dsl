import _configs_.*
import javaposse.jobdsl.dsl.Job

// -------------------------------------------------------------------
def reprepro = job("reprepro_importer")
OSRFUNIXBase.create(reprepro)
reprepro.with
{
  label "packages"

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
