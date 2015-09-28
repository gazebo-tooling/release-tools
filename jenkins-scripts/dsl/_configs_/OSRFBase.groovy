package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
     - description
     - RTOOLS parameter + description tag in job
     - base mail for Failures and Unstables
*/
class OSRFBase
{
   static void create(Job job)
   {
     job.with {
     	description 'Automatic generated job by DSL jenkins. Please do not edit manually'

        parameters { stringParam('RTOOLS_BRANCH','default','release-tool branch to use') }

        steps
        {
           systemGroovyCommand("build.setDescription('RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));")

           shell("""\
                 #!/bin/bash -xe

                 [[ -d ./scripts ]] &&  rm -fr ./scripts
                 hg clone http://bitbucket.org/osrf/release-tools scripts -b \${RTOOLS_BRANCH}
                 """.stripIndent())
        }

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
