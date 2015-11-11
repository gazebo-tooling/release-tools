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
            
              // TODO: this is copied from OSRFBase. Find a better way of doing
              // this without duplicate the code here.
              configure { node ->
                node / presendScript << """                
                  boolean no_mail = build.getEnvVars()['NO_MAILS'].toBoolean()

                  if (no_mail)
                  {
                    logger.println("NO_MAILS parameter enable. Not sending mails! ")
                    cancel = true;
                  }
                  """.stripIndent()
               }
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
