package _configs_

import javaposse.jobdsl.dsl.Job

class GenericMail
{
  static void include_mail(Job job)
  {
    include_mail(job,'$DEFAULT_SUBJECT','$DEFAULT_CONTENT')
  }

  static void update_field(Job job, String field, String new_value)
  {
    job.with
    {
      configure { project ->
        (project / publishers / 'hudson.plugins.emailext.ExtendedEmailPublisher' / field ).value = new_value
      }
    }
  }

  static void include_mail(Job job, String subject, String content)
  {
    job.with
    {
      publishers
      {
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
              boolean final_cancel_answer = false
              String logFilePath = build.getLogFile().getPath();
              String logContent = new File(logFilePath).text;

              // 1. Check if NO_MAILS is enabled
              boolean no_mail = build.getEnvVars()['NO_MAILS'].toBoolean()
              if (no_mail)
              {
                logger.println("NO_MAILS parameter enable. Not sending mails! ")
                final_cancel_answer = true;
              }

              // 2. MSVC internal compiler error
              // Let's assume does only very rarely several
              // Naginator plugin does not provide a way to know if MAX got
              // reach withput unsuccessful build
              // https://issues.jenkins-ci.org/browse/JENKINS-21241
              try {
                  if (logContent.find(/INTERNAL COMPILER ERROR/))
                  {
                    logger.println("INTERNAL COMPILER ERROR detected. Not sending mails!")
                    final_cancel_answer = true;
                  }
              }
              catch (all)
              {
              }

              cancel = final_cancel_answer
              """.stripIndent()
          } // end of configure
        }
      }
    }
  } // end of include_mail
} // end of class
