class Globals
{
   // Notifications for email ext plugin
   static default_emails = '$DEFAULT_RECIPIENTS, scpeters@osrfoundation.org'
   static extra_emails   = ''

   static String get_emails()
   {
      static String all_mails = default_emails

      if (extra_emails != '')
      {
        all_mails = all_mails + ', ' + extra_emails
      }

      return default_emails
   }
}
