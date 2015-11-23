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

   // -- Officially support distributions for ign, sdformat and gazebo --
   // Main CI platform
   static ArrayList get_ci_distro()
   {
    return [ 'trusty' ]
   }
  
   static ArrayList get_other_supported_distros()
   {
     return [ 'vivid', 'wily' ]
   }

   static ArrayList get_all_supported_distros()
   {
     return get_ci_distro() + get_other_supported_distros()
   }
}
