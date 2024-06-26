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
   // Run nightly scheduler every 20 minutes being sure to
   // run it at 9 just before the nightly creation.
   static CRON_NIGHTLY_NODES = '*/20 9-23 * * * \n20 0-8 * * *'

   // Start the nightly generation 10 minutes after the nigthly node
   // initial generation
   static CRON_START_NIGHTLY = '10 9 * * *'

   // Only one -E regex can be passed, so make a regex that matches both
   // _ign_TEST and _gz_TEST
   static MAKETEST_SKIP_GZ = "-E _i?g[nz]_TEST"

   static gpu_by_distro  = [ bionic  : [ 'nvidia' ]]

   static ros_ci = [ 'noetic'   : ['focal'] ,
                     'foxy'     : ['focal']]

   // This should be in sync with archive_library
   static gz_version_by_rosdistro = [ 'noetic'   : ['11'] ,
                                      'foxy'     : ['11']]

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

   static String nontest_label(String original_label) {
    return "(${original_label}) && !test-instance"
   }

   static String get_canonical_package_name(String package_name) {
    return package_name.replaceAll('\\d*$', '')
  }

   static String s3_releases_dir(String package_name) {
    return get_canonical_package_name(package_name) + '/releases'
   }

   static String s3_upload_tarball_path(String package_name) {
    return 's3://osrf-distributions/' + s3_releases_dir(package_name)
   }

   static String s3_download_url_basedir(String package_name) {
    return 'https://osrf-distributions.s3.amazonaws.com/' + s3_releases_dir(package_name)
   }

   /* rest of the s3 paths need to be cumputed during job running time since
    * they depend on VERSION and it is not avialble at DSL time */
}
