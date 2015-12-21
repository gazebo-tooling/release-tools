package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxBase

  Implements:
    - pritority 100
    - logrotator
    - concurrent builds
    - parameter: ORIGIN_BRANCH, TARGET_BRANCH
    - set description
    - parse log for result
    - publish report in HTML
*/

class OSRFLinuxABI
{
  static void create(Job job)
  {
    OSRFLinuxBase.create(job)

    job.with
    {
      priority 100

      logRotator {
        artifactNumToKeep(10)
      }

      concurrentBuild(true)

      throttleConcurrentBuilds {
	maxPerNode(1)
	maxTotal(5)
      }

      parameters {
        stringParam("ORIGIN_BRANCH", null,
                    'Branch to use as base for the comparison')
        stringParam("TARGET_BRANCH", null,
                    'Branch to use to compare against ORIGIN_BRANCH')
      }

      steps {
        systemGroovyCommand("""\
          build.setDescription(
            'origin branch: ' +
            '<b>' + build.buildVariableResolver.resolve('ORIGIN_BRANCH') + '</b><br />' +
            'target branch: ' + 
            '<b>' + build.buildVariableResolver.resolve('TARGET_BRANCH') + '</b><br />' +
            '<br />' +
            'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent()
        )
      }

      publishers {
          publishHtml {
              report('reports/') {
                  reportFiles('compat_report.html')
                  reportName('API_ABI report')
                  keepAll()
                  allowMissing(false)
                  alwaysLinkToLastBuild()
              }
          }
      }

      // Added the checker result parser (UNSTABLE if not compatible)
      configure { project ->
           project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
              unstableOnWarning true
              failBuildOnError true
              parsingRulesPath('/var/lib/jenkins/logparser_warn_on_abichecker_error')
           }
      } // end of configure
    } // end of with
  } // end of create
} // end of class
