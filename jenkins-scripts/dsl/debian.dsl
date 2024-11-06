import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org"
// ratt package to help during transition
def ratt_pkg_job = job("debian-ratt-builder")
OSRFLinuxBase.create(ratt_pkg_job)
ratt_pkg_job.with
{
  // use only the most powerful nodes
  label Globals.nontest_label("large-memory")

  parameters
  {
     stringParam('DEB_PACKAGE','master',
                 'package to run ratt against (check lib transition)')
     booleanParam('USE_UNSTABLE', true,
                 'use unstable instead of experimental to test packages')
     stringParam('RATT_INCLUDE','',
                 'Regexp for package inclusion: ^(hwloc|fltk1.3|starpu)$. See https://github.com/j-rivero/ratt/blob/master/README.md')
     stringParam('RATT_EXCLUDE','',
                 'Regexp for package exclusion: ^(gcc-9|gcc-8)$. See https://github.com/j-rivero/ratt/blob/master/README.md')
  }

  logRotator {
    artifactNumToKeep(10)
    numToKeep(75)
  }

  concurrentBuild(true)

  throttleConcurrentBuilds {
    maxPerNode(1)
    maxTotal(5)
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

    archiveArtifacts('logs/buildlogs/*')
  }


  steps {
    shell("""\
          #!/bin/bash -xe

          /bin/bash -xe ./scripts/jenkins-scripts/docker/debian-ratt-builder.bash
          """.stripIndent())
  }
}
