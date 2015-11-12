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
   static void create(Job job)
   {
     def mail_content ='''\
     $DEFAULT_CONTENT

     Test summary:
     -------------
      * Total of ${TEST_COUNTS, var="total"} tests : ${TEST_COUNTS, var="fail"} failed and ${TEST_COUNTS, var="skip"}

     Data log:
     ${FAILED_TESTS}
     '''.stripIndent()

     GenericMail.include_mail(job, '$DEFAULT_SUBJECT' , mail_content)

     job.with
     {
        priority 100

        logRotator {
          numToKeep(15)
        }

        publishers
        {
           // remove the existing 'extendedEmail' element
           configure { project ->
                project.remove(project / publishers << 'hudson.plugins.emailext.ExtendedEmailPublisher')
           }

           // junit plugin is not implemented. Use configure for it
           configure { project ->
              project / publishers << 'hudson.tasks.junit.JUnitResultArchiver' {
                   testResults('build/test_results/*.xml')
                   keepLongStdio false
                   testDataPublishers()
              }
          }
        } // end of publishers
      } // end of job
   } // end of create
} // end of class
