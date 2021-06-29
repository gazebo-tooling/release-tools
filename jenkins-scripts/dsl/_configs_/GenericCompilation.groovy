package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
    - priority 300
    - keep only 75 builds
    - mail with test results
*/
class GenericCompilation
{

   static String get_compilation_mail_content()
   {
      return GenericMail.get_default_content() + '''\

     ${BUILD_LOG_REGEX, regex="^.*: (fatal ){0,1}error.*$",  linesBefore="5", linesAfter="5", maxMatches=0, showTruncatedLines=false}

     Compiler warnings:
     ------------------
      * Total of ${WARNINGS_COUNT} warnings : ${WARNINGS_NEW} new and ${WARNINGS_FIXED} fixed
      * $BUILD_URL/warningsResult


     Test summary:
     -------------
      * Total of ${TEST_COUNTS, var="total"} tests : ${TEST_COUNTS, var="fail"} failed and ${TEST_COUNTS, var="skip"} skipped
      * $BUILD_URL/testReport

     Data log:
     ${FAILED_TESTS}
     '''.stripIndent()
   }

   static void create(Job job, boolean enable_testing = true)
   {

     GenericMail.update_field(job, 'defaultContent',
                              GenericCompilation.get_compilation_mail_content())

     job.with
     {
        properties {
          priority 300
        }

        logRotator {
          numToKeep(75)
        }

        if (enable_testing)
        {
          publishers
          {
            archiveJunit('build/test_results/*.xml') {
              testDataPublishers {
                publishFlakyTestsReport()
              }
            }
          } // end of publishers
        } // end of enable_testing
      } // end of job
   } // end of create
} // end of class
