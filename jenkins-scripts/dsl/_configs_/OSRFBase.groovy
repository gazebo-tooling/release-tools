package _configs_

import javaposse.jobdsl.dsl.Job
import groovy.transform.Field

/*
  Implements:
     - description
     - RTOOLS parame + groovy to set jobDescription
     - base mail for Failures and Unstables
*/

print PROJECT_MAILS

class OSRFBase
{
   def project_mails = PROJECT_MAILS

   static void create(Job job)
   {
     job.with 
     {
     	description 'Automatic generated job by DSL jenkins. Please do not edit manually'

        parameters { stringParam('RTOOLS_BRANCH','default','release-tool branch to use') }

        steps
        {
           systemGroovyCommand("build.setDescription('RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));")
        }

        publishers 
        {
          extendedEmail("\$DEFAULT_RECIPIENTS, scpeters@osrfoundation.org " + project_mails,
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
