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
      * Total of ${TEST_COUNTS, var="total"} tests : ${TEST_COUNTS, var="fail"} failed and ${TEST_COUNTS, var="skip"

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
           configure { publisher ->
                // remove the existing 'extendedEmail' element
                publisher.remove("hudson.plugins.emailext.ExtendedEmailPublisher")
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
           }
        } // end of publishers
      } // end of job
   } // end of create
} // end of class
