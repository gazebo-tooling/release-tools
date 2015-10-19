package _configs_

class Globals
{
   // Notifications for email ext plugin
   static default_emails = '$DEFAULT_RECIPIENTS, scpeters@osrfoundation.org'
   static extra_emails   = ''

   static String get_emails()
   {
      if (extra_emails != '')
      {
        return default_emails + ', ' + extra_emails
      }

      return default_emails
   }
}
