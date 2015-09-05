package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
    - priorioty 100
    - keep only 15 builds
    - compiler warning
    - mail with test results
    - test results
    - cppcheck results
*/
class OSRFLinuxCompilation extends OSRFLinuxBase
{
  static void create(Job job)
  {
    OSRFLinuxBase.create(job)

    def mail_content =
'''
$DEFAULT_CONTENT

Test summary:
-------------
 * Total of ${TEST_COUNTS, var="total"} tests : ${TEST_COUNTS, var="fail"} failed and ${TEST_COUNTS, var="skip"

Data log:
${FAILED_TESTS}
'''
  job.with
    {
      priority 100

      logRotator {
        numToKeep(15)
      }

      publishers
      {
         // compilers warnings
         warnings(['GNU C Compiler 4 (gcc)'])

         // special content with testing failures
         extendedEmail('$DEFAULT_RECIPIENTS, scpeters@osrfoundation.org',
                       '$DEFAULT_SUBJECT',
                        mail_content)
         {
            trigger(triggerName: 'Failure',
                    subject: null, body: null, recipientList: null,
                    sendToDevelopers: true,
                    sendToRequester: true,
                    includeCulprits: false,
                    sendToRecipientList: true)
            trigger(triggerName: 'Unstable',
                    subject: null, body: null, recipientList: null,
                    sendToDevelopers: true,
                    sendToRequester: true,
                    includeCulprits: false,
                    sendToRecipientList: true)
            trigger(triggerName: 'Fixed',
                    subject: null, body: null, recipientList: null,
                    sendToDevelopers: true,
                    sendToRequester: true,
                    includeCulprits: false,
                    sendToRecipientList: true)
         }

         // junit plugin is not implemented. Use configure for it
         configure { project ->
            project / publishers << 'hudson.tasks.junit.JUnitResultArchiver' {
                 testResults('build/test_results/*.xml')
                 keepLongStdio false
                 testDataPublishers()
            }
         }

         // cppcheck is not implemented. Use configure for it
         configure { project ->
           project / publishers / 'org.jenkinsci.plugins.cppcheck.CppcheckPublisher' / cppcheckConfig {
             pattern('build/cppcheck_results/*.xml')
             ignoreBlankFiles true
             allowNoReport false

             configSeverityEvaluation {
           	   threshold 0
           	   newThreshold()
           	   failureThreshold()
               newFailureThreshold()
               healthy()
               unHealthy()
               severityError true
               severityWarning true
               severityStyle true
               severityPerformance true
               severityInformation true
               severityNoCategory false
               severityPortability false
             }

         	   configGraph {
               xSize 500
               ySize 200
               numBuildsInGraph 0
               displayAllErrors true
               displayErrorSeverity false
               displayWarningSeverity false
               displayStyleSeverity false
               displayPerformanceSeverity false
               displayInformationSeverity false
               displayNoCategorySeverity false
               displayPortabilitySeverity false
             } // end of configGraph
           } // end of cppcheckconfig
         } // end of configure
      } // end of publishers
    } // end of job
  } // end of method createJob
} // end of class
