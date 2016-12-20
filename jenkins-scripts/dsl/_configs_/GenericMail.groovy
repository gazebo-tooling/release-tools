package _configs_

import javaposse.jobdsl.dsl.Job

class GenericMail
{
  static String get_default_content()
  {
     return '''\
     $PROJECT_NAME - Build # $BUILD_NUMBER - $BUILD_STATUS:

     ${BUILD_FAILURE_ANALYZER, includeTitle=true, includeIndications=true, useHtmlFormat=false}

     Build summary: $BUILD_URL
     Full log     : $BUILD_URL/consoleFull
     Retry        : $BUILD_URL/retry
     '''.stripIndent()
  }

  static void include_external_contributors_mail(Job job, String subject, String content)
  {
    GenericMail._include_mail_recipients(job, subject, content)
    GenericMail.update_field(job, "presendScript",
                             GenericMail.get_base_presend_script())
  }

  static void include_mail(Job job, String subject, String content)
  {
    GenericMail._include_mail_recipients(job, subject, content)
    GenericMail.update_field(job, "presendScript",
                             GenericMail.get_base_presend_script() +
                             GenericMail.get_OSRF_filter_presend_script())
  }

  static void include_mail(Job job)
  {
    include_mail(job,'$DEFAULT_SUBJECT', GenericMail.get_default_content())
  }

  static void include_external_contributors_mail(Job job)
  {
    include_external_contributors_mail(job,
                                       '$DEFAULT_SUBJECT',
                                       GenericMail.get_default_content())
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

  static void _include_mail_recipients(Job job, String subject, String content)
  {
    job.with
    {
      publishers
      {
        extendedEmail
        {
          recipientList(Globals.get_emails())
          defaultSubject(subject)
          defaultContent(content)

          triggers
          {
            failure {
              sendTo {
                developers()
                requester()
                recipientList()
              }
            }

            unstable {
              sendTo {
                developers()
                requester()
                recipientList()
              }
            }

            fixed {
              sendTo {
                developers()
                requester()
                recipientList()
              }
            }
          }
        }
      }
    }
  }

  static String get_base_presend_script()
  {
     return("""
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
            """.stripIndent())
  }

  static String get_OSRF_filter_presend_script()
  {
    return("""
           // 3. Filter mail to get only OSRF
           recipients =
              msg.getAllRecipients()
           filtered =
              recipients.findAll { addr -> addr.toString().contains('@osrfoundation.org') }
           msg.setRecipients(javax.mail.Message.RecipientType.TO,
                             filtered as javax.mail.Address[])
           logger.println("Final list of recipients after filtering:");
           logger.println(recipients);

           // always need to be the last line to cancel email when needed
           cancel = final_cancel_answer
        """.stripIndent())
  }
} // end of class
