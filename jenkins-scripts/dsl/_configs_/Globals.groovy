package _configs_

class Globals
{
   // Notifications for email ext plugin
   static default_emails = '$DEFAULT_RECIPIENTS'
   static build_cop_email = 'buildcop@osrfoundation.org'
   static extra_emails   = ''

   static rtools_description = true
   static gazebodistro_branch = false

   static CRON_EVERY_THREE_DAYS = 'H H * * H/3'
   static CRON_HOURLY = 'H * * * *'
   static CRON_ON_WEEKEND = 'H H * * 6-7'
   // Run nightly scheduler during the nightly creation to be sure
   // that any possible node killed is replaced. Starting -15min
   // before CRON_NIGHTLY_NODES and evert 20min for 3 hours
   static CRON_NIGHTLY_NODES = '*/20 9-11 * * *'
   // Start the nightly generation 10 minutes after the nigthly node
   // initial generation
   static CRON_START_NIGHTLY = '10 9 * * *'

   // Only one -E regex can be passed, so make a regex that matches both
   // _ign_TEST and _gz_TEST
   static MAKETEST_SKIP_GZ = "-E _i?g[nz]_TEST"

   static gpu_by_distro  = [ bionic  : [ 'nvidia' ]]

   static ros_ci = [ 'melodic'  : ['bionic'] ,
                     'noetic'   : ['focal'] ,
                     'foxy'     : ['focal'] ,
                     'rolling'  : ['jammy']]

   // This should be in sync with archive_library
   static gz_version_by_rosdistro = [ 'melodic'  : ['9'] ,
                                      'noetic'   : ['11'] ,
                                      'foxy'     : ['11'] ,
                                      'rolling'  : ['11']]

   static String ign2gz(String str) {
     str = str.replaceAll("ignitionrobotics","gazebosim")
     str = str.replaceAll("ign-gazebo","gz-sim")
     // This one is to workaround failures in main DSL files generating
     // gz-gazebo instead of gz-sim
     str = str.replaceAll("gz-gazebo","gz-sim")
     return str.replaceAll("ign-","gz-")
   }

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
    return [ 'focal' ]
   }

   static ArrayList get_abi_distro()
   {
     return [ 'focal' ]
   }

   static ArrayList get_ci_gpu()
   {
     return [ 'nvidia' ]
   }

   static ArrayList get_other_supported_distros()
   {
     return [  ]
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
     return [ 'foxy', 'rolling' ]
   }

   static String get_ros2_development_distro() {
     return 'rolling'
   }

   static String nontest_label(String original_label) {
    return "(${original_label}) && !test-instance"
   }
}
