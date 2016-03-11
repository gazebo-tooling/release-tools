package _configs_

class Globals
{
   // Notifications for email ext plugin
   static default_emails = '$DEFAULT_RECIPIENTS, scpeters@osrfoundation.org'
   static extra_emails   = ''

   static gpu_by_distro  = [ trusty : [ 'nvidia', 'intel' ],
                             vivid  : [ 'intel' ] ]

   static String get_emails()
   {
      if (extra_emails != '')
      {
        return default_emails + ', ' + extra_emails
      }

      return default_emails
   }

   static String get_performance_box()
   {
      return 'maria.intel.trusty'
   }

   // -- Officially support distributions for ign, sdformat and gazebo --
   // Main CI platform
   static ArrayList get_ci_distro()
   {
    return [ 'trusty' ]
   }

   static ArrayList get_abi_distro()
   {
     return [ 'vivid' ]
   }

   static ArrayList get_ci_gpu()
   {
     return [ 'nvidia' ]
   }

   static ArrayList get_other_supported_distros()
   {
     return [ 'vivid', 'wily' ]
   }

   static ArrayList get_supported_arches()
   {
     return [ 'amd64' ]
   }

   static ArrayList get_experimental_arches()
   {
     return [ 'i386', 'armhf' ]
   }

   static ArrayList get_all_supported_distros()
   {
     return get_ci_distro() + get_other_supported_distros()
   }

   static ArrayList get_all_supported_gpus()
   {
    return get_ci_gpu() + [ 'intel' ]
   }
}
