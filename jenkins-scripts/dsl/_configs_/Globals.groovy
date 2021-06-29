package _configs_

class Globals
{
   // Notifications for email ext plugin
   static default_emails = '$DEFAULT_RECIPIENTS, scpeters@osrfoundation.org'
   static build_cop_email = 'buildcop@osrfoundation.org'
   static extra_emails   = ''

   static rtools_description = true
   static gazebodistro_branch = false


   static CRON_EVERY_THREE_DAYS = 'H H * * H/3'

   static gpu_by_distro  = [ bionic  : [ 'nvidia' ]]

   static ros_ci = [ 'melodic'  : ['bionic'] ,
                     'noetic'   : ['focal'] ,
                     'foxy'     : ['focal'] ,
                     'galactic' : ['focal'] ,
                     'rolling'  : ['focal']]

   // This should be in sync with archive_library
   static gz_version_by_rosdistro = [ 'melodic'  : ['9'] ,
                                      'noetic'   : ['11'] ,
                                      'foxy'     : ['11'] ,
                                      'galactic' : ['11'] ,
                                      'rolling'  : ['11']]

   static ArrayList get_ros_distros_by_ubuntu_distro(String ubuntu_distro)
   {
      ArrayList result = []

      ros_ci.each { ros_distro, ubuntu_distros_in_ci ->
        ubuntu_distros_in_ci.each { ub_distro_in_ci ->
          if ("${ub_distro_in_ci}" == "${ubuntu_distro}") {
            result.add(ros_distro)
          }
        }
      }

      return result
   }

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
    return [ 'bionic' ]
   }

   static ArrayList get_abi_distro()
   {
     return [ 'bionic' ]
   }

   static ArrayList get_ci_gpu()
   {
     return [ 'nvidia' ]
   }

   static ArrayList get_other_supported_distros()
   {
     return [ 'bionic' ]
   }

   static ArrayList get_supported_arches()
   {
     return [ 'amd64' ]
   }

   static ArrayList get_experimental_arches()
   {
     return [ 'i386' ]
   }

   static ArrayList get_all_supported_distros()
   {
     return get_ci_distro() + get_other_supported_distros()
   }

   static ArrayList get_all_supported_gpus()
   {
    return get_ci_gpu() + [ 'intel' ]
   }

   static ArrayList get_ros_suported_distros()
   {
     return [ 'melodic', 'noetic' ]
   }

   static ArrayList get_ros2_suported_distros()
   {
     return [ 'foxy', 'galactic', 'rolling' ]
   }

   static String get_ros2_development_distro() {
     return 'rolling'
   }

   static String get_gz11_ubuntu_distro()
   {
     return 'bionic'
   }

   static String get_gz11_mac_distro()
   {
     return 'mojave'
   }
}
