package _configs_

import javaposse.jobdsl.dsl.Job

class GenericMail
{
    static void include_mail(Job job)
    {
      include_mail(job,'$DEFAULT_SUBJECT','$DEFAULT_CONTENT')
    }

    static void include_mail(Job job, String subject, String content)
    {
      job.with
      {
        publishers
        {
           // remove the existing 'extendedEmail' element
           configure { project ->
                project.remove(project / publishers << 'hudson.plugins.emailext.ExtendedEmailPublisher')
           }

          extendedEmail(Globals.get_emails(),
                        subject,
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
            configure { node ->
              node / presendScript << """                
                boolean no_mail = build.getEnvVars()['NO_MAILS'].toBoolean()

                if (no_mail)
                {
                  logger.println("NO_MAILS parameter enable. Not sending mails! ")
                  cancel = true;
                }
                """.stripIndent()
    	    } // end of configure
          }
        }
      }
    }
}
