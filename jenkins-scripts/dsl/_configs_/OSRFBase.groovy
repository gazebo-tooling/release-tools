package _configs_

import javaposse.jobdsl.dsl.Job
  
/* 
  Implements:
     - description
*/
class OSRFBase
{
   static void create(Job job)
   {
     def content = 
'''
$DEFAULT_CONTENT

Test summary:
-------------
 * Total of ${TEST_COUNTS, var="total"} tests : ${TEST_COUNTS, var="fail"} failed and ${TEST_COUNTS, var="skip"

Data log:
${FAILED_TESTS}
'''
     job.with {
     	description 'Automatic generated job by DSL jenkins. Please do not edit manually'

        publishers { 
          extendedEmail('$DEFAULT_RECIPIENTS, scpeters@osrfoundation.org',
                        '$DEFAULT_SUBJECT',
                         content)  
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
       }
     }
   }
}
