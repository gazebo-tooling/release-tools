package _configs_

import javaposse.jobdsl.dsl.Job
  
/* 
  Implements:
     - description
     - base mail for Failures and Unstables
*/
class OSRFBase
{
   static void create(Job job)
   {
     job.with {
     	description 'Automatic generated job by DSL jenkins. Please do not edit manually'

        publishers { 
          extendedEmail('$DEFAULT_RECIPIENTS, scpeters@osrfoundation.org',
                        '$DEFAULT_SUBJECT',
                        '$DEFAULT_CONTENT')
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
