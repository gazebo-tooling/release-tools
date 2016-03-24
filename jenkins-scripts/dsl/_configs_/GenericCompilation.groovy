package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
    - priorioty 100
    - keep only 15 builds
    - mail with test results
*/
class GenericCompilation
{

   static String get_compilation_mail_content()
   {
      return '''\
     $DEFAULT_CONTENT

     ${BUILD_LOG_REGEX, regex="^.*: (fatal ){0,1}error.*$",  linesBefore="5", linesAfter="5", maxMatches=0, showTruncatedLines=false}

     Test summary:
     -------------
      * Total of ${TEST_COUNTS, var="total"} tests : ${TEST_COUNTS, var="fail"} failed and ${TEST_COUNTS, var="skip"}

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
        priority 100

        logRotator {
          numToKeep(15)
        }

        if (enable_testing)
        {
          publishers
          {
             // junit plugin is not implemented. Use configure for it
             configure { project ->
                project / publishers << 'hudson.tasks.junit.JUnitResultArchiver' {
                     testResults('build/test_results/*.xml')
                     keepLongStdio false
                     testDataPublishers()
                }
            }
          } // end of publishers
        } // end of enable_testing
      } // end of job
   } // end of create
} // end of class
