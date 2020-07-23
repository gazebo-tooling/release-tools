package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxBase

  Implements:
    - priority 300
    - logrotator
    - concurrent builds
    - parameter: DEST_BRANCH, SRC_REPO, DESCRIPTION
    - set description
    - parse log for result
    - publish report in HTML
*/

class OSRFLinuxABIGitHub
{
  static void create(Job job)
  {
    OSRFLinuxBase.create(job)

    GenericMail.update_field(job, 'defaultSubject',
                    '$PROJECT_NAME - Branches: $DEST_BRANCH, $sha1 (#$BUILD_NUMBER) - $BUILD_STATUS!')
    GenericMail.update_field(job, 'defaultContent',
                    '$JOB_DESCRIPTION \n' +
                    'destination branch: $DEST_BRANCH \n' +
                    'source ref:         $sha1 \n' +
                    'source repository:  $SRC_REPO \n' +
                    'RTOOLS branch:      $RTOOLS_BRANCH \n' +
                    GenericMail.get_default_content() + '\n' +
                    'ABI report   : $BUILD_URL/API_ABI_report/\n')

    job.with
    {
      properties {
        priority 300
      }

      logRotator {
        artifactNumToKeep(10)
      }

      concurrentBuild(true)

      throttleConcurrentBuilds {
        maxPerNode(1)
        maxTotal(5)
      }

      parameters {
        stringParam("DEST_BRANCH", null,
                    'Branch to use as base for the comparison')
        stringParam('SRC_REPO', null,
                    'URL pointing to repository containing sha1 (refspec to git branch/tag/...)')
        stringParam("JOB_DESCRIPTION", "",
                    'Description of the job for informational purposes.')
      }

      steps {
        systemGroovyCommand("""\
          job_description = build.buildVariableResolver.resolve("JOB_DESCRIPTION")
          dest_branch = build.buildVariableResolver.resolve('DEST_BRANCH')
          if (dest_branch == "")
            dest_branch = build.buildVariableResolver.resolve('ghprbTargetBranch')
          src_repo = build.buildVariableResolver.resolve('SRC_REPO')
          if (src_repo == "")
             src_repo = build.buildVariableResolver.resolve('ghprbAuthorRepoGitUrl')

          if (job_description == "")
          {
            job_description = 'destination branch: ' +
              '<b>' + dest_branch + '</b><br />' +
              'source ref: ' +
              '<b>' + build.buildVariableResolver.resolve('sha1') + '</b><br />' +
              'source repository: ' +
              '<b>' + src_repo + '</b><br />' +
              '<br />' +
              'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH');
          }

          build.setDescription(job_description)
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
