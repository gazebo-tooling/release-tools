import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION COLLECTIONS
arch = 'amd64'

ignition_nightly = 'fortress'

ignition_collections = [
  [ name : 'citadel',
    distros : [ 'bionic' ],
  ],
  [ name : 'dome',
    distros : [ 'focal' ],
  ],
  [ name : 'edifice',
    distros : [ 'focal' ],
  ],
  [ name : 'fortress',
    distros : [ 'focal' ],
    // These are the branches currently targeted at the upcoming collection
    nightly_jobs: [
          'cmake'     : [ debbuild: 'ign-cmake2'      , branch: 'ign-cmake2' ],
          'common'    : [ debbuild: 'ign-common4'     , branch: 'ign-common4' ],
          'fortress'  : [ debbuild: 'ign-fortress'    , branch: 'main' ],
          'fuel-tools': [ debbuild: 'ign-fuel-tools6' , branch: 'ign-fuel-tools6' ],
          'gazebo'    : [ debbuild: 'ign-gazebo5'     , branch: 'ign-gazebo5' ],
          'gui'       : [ debbuild: 'ign-gui5'        , branch: 'ign-gui5' ],
          'launch'    : [ debbuild: 'ign-launch4'     , branch: 'ign-launch4' ],
          'math'      : [ debbuild: 'ign-math6'       , branch: 'ign-math6' ],
          'msgs'      : [ debbuild: 'ign-msgs8'       , branch: 'main' ],
          'physics'   : [ debbuild: 'ign-physics4'    , branch: 'ign-physics4' ],
          'plugin'    : [ debbuild: 'ign-plugin1'     , branch: 'ign-plugin1' ],
          'rendering' : [ debbuild: 'ign-rendering6'  , branch: 'main' ],
          'sdformat'  : [ debbuild: 'sdformat11'      , branch: 'sdf11' ],
          'sensors'   : [ debbuild: 'ign-sensors5'    , branch: 'ign-sensors5' ],
          'tools'     : [ debbuild: 'ign-tools1'      , branch: 'ign-tools1' ],
          'transport' : [ debbuild: 'ign-transport10' , branch: 'ign-transport10' ],
          'utils'     : [ debbuild: 'ign-utils1'      , branch: 'ign-utils1' ],
    ],
  ],
]

ignition_collection_jobs =
[
  'citadel' : [
        'ign_gazebo-ign-3-win',
        'ign_gui-ign-3-win',
        'ign_physics-ign-2-win',
        'ign_rendering-ign-3-win',
        'ign_sensors-ign-3-win',
        'ignition_citadel-ci-main-homebrew-amd64',
        'ignition_citadel-install-pkg-bionic-amd64',
        'ignition_citadel-install_bottle-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-bionic-amd64',
        'ignition_cmake-ci-ign-cmake2-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-windows7-amd64',
        'ignition_cmake2-install-pkg-bionic-amd64',
        'ignition_cmake2-install_bottle-homebrew-amd64',
        'ignition_common-ci-ign-common3-bionic-amd64',
        'ignition_common-ci-ign-common3-homebrew-amd64',
        'ignition_common-ci-ign-common3-windows7-amd64',
        'ignition_common3-install-pkg-bionic-amd64',
        'ignition_common3-install_bottle-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools4-bionic-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools4-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools4-windows7-amd64',
        'ignition_fuel-tools4-install-pkg-bionic-amd64',
        'ignition_fuel-tools4-install_bottle-homebrew-amd64',
        'ignition_gazebo-ci-ign-gazebo3-bionic-amd64',
        'ignition_gazebo-ci-ign-gazebo3-homebrew-amd64',
        'ignition_gazebo3-install-pkg-bionic-amd64',
        'ignition_gazebo3-install_bottle-homebrew-amd64',
        'ignition_gui-ci-ign-gui3-bionic-amd64',
        'ignition_gui-ci-ign-gui3-homebrew-amd64',
        'ignition_gui3-install-pkg-bionic-amd64',
        'ignition_gui3-install_bottle-homebrew-amd64',
        'ignition_launch-ci-ign-launch2-bionic-amd64',
        'ignition_launch-ci-ign-launch2-homebrew-amd64',
        'ignition_launch2-install-pkg-bionic-amd64',
        'ignition_launch2-install_bottle-homebrew-amd64',
        'ignition_math-ci-ign-math6-bionic-amd64',
        'ignition_math-ci-ign-math6-homebrew-amd64',
        'ignition_math-ci-ign-math6-windows7-amd64',
        'ignition_math6-install-pkg-bionic-amd64',
        'ignition_math6-install_bottle-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs5-bionic-amd64',
        'ignition_msgs-ci-ign-msgs5-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs5-windows7-amd64',
        'ignition_msgs5-install-pkg-bionic-amd64',
        'ignition_msgs5-install_bottle-homebrew-amd64',
        'ignition_physics-ci-ign-physics2-bionic-amd64',
        'ignition_physics-ci-ign-physics2-homebrew-amd64',
        'ignition_physics2-install-pkg-bionic-amd64',
        'ignition_physics2-install_bottle-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-bionic-amd64',
        'ignition_plugin-ci-ign-plugin1-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-windows7-amd64',
        'ignition_plugin-install-pkg-bionic-amd64',
        'ignition_plugin1-install_bottle-homebrew-amd64',
        'ignition_rendering-ci-ign-rendering3-bionic-amd64',
        'ignition_rendering-ci-ign-rendering3-homebrew-amd64',
        'ignition_rendering3-install-pkg-bionic-amd64',
        'ignition_rendering3-install_bottle-homebrew-amd64',
        'ignition_sensors-ci-ign-sensors3-bionic-amd64',
        'ignition_sensors-ci-ign-sensors3-homebrew-amd64',
        'ignition_sensors3-install-pkg-bionic-amd64',
        'ignition_sensors3-install_bottle-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-bionic-amd64',
        'ignition_tools-ci-ign-tools1-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-windows7-amd64',
        'ignition_tools-install-pkg-bionic-amd64',
        'ignition_tools1-install_bottle-homebrew-amd64',
        'ignition_transport-ci-ign-transport8-bionic-amd64',
        'ignition_transport-ci-ign-transport8-homebrew-amd64',
        'ign_transport-ign-8-win',
        'ignition_transport8-install-pkg-bionic-amd64',
        'ignition_transport8-install_bottle-homebrew-amd64',
        'sdformat-ci-sdformat9-bionic-amd64',
        'sdformat-ci-sdformat9-homebrew-amd64',
        'sdformat-ci-sdformat9-windows7-amd64',
        'sdformat-install-sdformat9_pkg-bionic-amd64'
  ],
  'dome' : [
        'ign_gazebo-ign-4-win',
        'ign_gui-ign-4-win',
        'ign_physics-ign-3-win',
        'ign_rendering-ign-4-win',
        'ign_sensors-ign-4-win',
        'ignition_dome-ci-main-homebrew-amd64',
        'ignition_dome-install-pkg-bionic-amd64',
        'ignition_dome-install_bottle-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-bionic-amd64',
        'ignition_cmake-ci-ign-cmake2-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-windows7-amd64',
        'ignition_cmake2-install-pkg-bionic-amd64',
        'ignition_cmake2-install_bottle-homebrew-amd64',
        'ignition_common-ci-ign-common3-bionic-amd64',
        'ignition_common-ci-ign-common3-homebrew-amd64',
        'ignition_common-ci-ign-common3-windows7-amd64',
        'ignition_common3-install-pkg-bionic-amd64',
        'ignition_common3-install_bottle-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools5-bionic-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools5-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools5-windows7-amd64',
        'ignition_fuel-tools5-install-pkg-bionic-amd64',
        'ignition_fuel-tools5-install_bottle-homebrew-amd64',
        'ignition_gazebo-ci-ign-gazebo4-bionic-amd64',
        'ignition_gazebo-ci-ign-gazebo4-homebrew-amd64',
        'ignition_gazebo4-install-pkg-bionic-amd64',
        'ignition_gazebo4-install_bottle-homebrew-amd64',
        'ignition_gui-ci-ign-gui4-bionic-amd64',
        'ignition_gui-ci-ign-gui4-homebrew-amd64',
        'ignition_gui4-install-pkg-bionic-amd64',
        'ignition_gui4-install_bottle-homebrew-amd64',
        'ignition_launch-ci-ign-launch3-bionic-amd64',
        'ignition_launch-ci-ign-launch3-homebrew-amd64',
        'ignition_launch3-install-pkg-bionic-amd64',
        'ignition_launch3-install_bottle-homebrew-amd64',
        'ignition_math-ci-ign-math6-bionic-amd64',
        'ignition_math-ci-ign-math6-homebrew-amd64',
        'ignition_math-ci-ign-math6-windows7-amd64',
        'ignition_math6-install-pkg-bionic-amd64',
        'ignition_math6-install_bottle-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs6-bionic-amd64',
        'ignition_msgs-ci-ign-msgs6-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs6-windows7-amd64',
        'ignition_msgs6-install-pkg-bionic-amd64',
        'ignition_msgs6-install_bottle-homebrew-amd64',
        'ignition_physics-ci-ign-physics3-bionic-amd64',
        'ignition_physics-ci-ign-physics3-homebrew-amd64',
        'ignition_physics3-install-pkg-bionic-amd64',
        'ignition_physics3-install_bottle-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-bionic-amd64',
        'ignition_plugin-ci-ign-plugin1-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-windows7-amd64',
        'ignition_plugin-install-pkg-bionic-amd64',
        'ignition_plugin1-install_bottle-homebrew-amd64',
        'ignition_rendering-ci-ign-rendering4-bionic-amd64',
        'ignition_rendering-ci-ign-rendering4-homebrew-amd64',
        'ignition_rendering4-install-pkg-bionic-amd64',
        'ignition_rendering4-install_bottle-homebrew-amd64',
        'ignition_sensors-ci-ign-sensors4-bionic-amd64',
        'ignition_sensors-ci-ign-sensors4-homebrew-amd64',
        'ignition_sensors4-install-pkg-bionic-amd64',
        'ignition_sensors4-install_bottle-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-bionic-amd64',
        'ignition_tools-ci-ign-tools1-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-windows7-amd64',
        'ignition_tools-install-pkg-bionic-amd64',
        'ignition_tools1-install_bottle-homebrew-amd64',
        'ignition_transport-ci-ign-transport9-bionic-amd64',
        'ignition_transport-ci-ign-transport9-homebrew-amd64',
        'ign_transport-ign-9-win',
        'ignition_transport9-install-pkg-bionic-amd64',
        'ignition_transport9-install_bottle-homebrew-amd64',
        'sdformat-ci-sdformat10-bionic-amd64',
        'sdformat-ci-sdformat10-homebrew-amd64',
        'sdformat-ci-sdformat10-windows7-amd64',
        'sdformat-install-sdformat10_pkg-bionic-amd64'
  ],
  'edifice' : [
        'ign_gazebo-ign-5-win',
        'ign_gui-ign-5-win',
        'ign_physics-ign-4-win',
        'ign_rendering-ign-5-win',
        'ign_sensors-ign-5-win',
        'ign_utils-ign-1-win',
        'ignition_edifice-ci-main-homebrew-amd64',
        'ignition_edifice-install-pkg-bionic-amd64',
        'ignition_edifice-install_bottle-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-bionic-amd64',
        'ignition_cmake-ci-ign-cmake2-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-windows7-amd64',
        'ignition_cmake2-install-pkg-bionic-amd64',
        'ignition_cmake2-install_bottle-homebrew-amd64',
        'ignition_common-ci-ign-common4-bionic-amd64',
        'ignition_common-ci-ign-common4-homebrew-amd64',
        'ignition_common-ci-ign-common4-windows7-amd64',
        'ignition_common4-install-pkg-bionic-amd64',
        'ignition_common4-install_bottle-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools6-bionic-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools6-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools6-windows7-amd64',
        'ignition_fuel-tools6-install-pkg-bionic-amd64',
        'ignition_fuel-tools6-install_bottle-homebrew-amd64',
        'ignition_gazebo-ci-ign-gazebo5-bionic-amd64',
        'ignition_gazebo-ci-ign-gazebo5-homebrew-amd64',
        'ignition_gazebo5-install-pkg-bionic-amd64',
        'ignition_gazebo5-install_bottle-homebrew-amd64',
        'ignition_gui-ci-ign-gui5-bionic-amd64',
        'ignition_gui-ci-ign-gui5-homebrew-amd64',
        'ignition_gui5-install-pkg-bionic-amd64',
        'ignition_gui5-install_bottle-homebrew-amd64',
        'ignition_launch-ci-ign-launch4-bionic-amd64',
        'ignition_launch-ci-ign-launch4-homebrew-amd64',
        'ignition_launch4-install-pkg-bionic-amd64',
        'ignition_launch4-install_bottle-homebrew-amd64',
        'ignition_math-ci-ign-math6-bionic-amd64',
        'ignition_math-ci-ign-math6-homebrew-amd64',
        'ignition_math-ci-ign-math6-windows7-amd64',
        'ignition_math6-install-pkg-bionic-amd64',
        'ignition_math6-install_bottle-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs7-bionic-amd64',
        'ignition_msgs-ci-ign-msgs7-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs7-windows7-amd64',
        'ignition_msgs7-install-pkg-bionic-amd64',
        'ignition_msgs7-install_bottle-homebrew-amd64',
        'ignition_physics-ci-ign-physics4-bionic-amd64',
        'ignition_physics-ci-ign-physics4-homebrew-amd64',
        'ignition_physics4-install-pkg-bionic-amd64',
        'ignition_physics4-install_bottle-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-bionic-amd64',
        'ignition_plugin-ci-ign-plugin1-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-windows7-amd64',
        'ignition_plugin-install-pkg-bionic-amd64',
        'ignition_plugin1-install_bottle-homebrew-amd64',
        'ignition_rendering-ci-ign-rendering5-bionic-amd64',
        'ignition_rendering-ci-ign-rendering5-homebrew-amd64',
        'ignition_rendering5-install-pkg-bionic-amd64',
        'ignition_rendering5-install_bottle-homebrew-amd64',
        'ignition_sensors-ci-ign-sensors5-bionic-amd64',
        'ignition_sensors-ci-ign-sensors5-homebrew-amd64',
        'ignition_sensors5-install-pkg-bionic-amd64',
        'ignition_sensors5-install_bottle-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-bionic-amd64',
        'ignition_tools-ci-ign-tools1-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-windows7-amd64',
        'ignition_tools-install-pkg-bionic-amd64',
        'ignition_tools1-install_bottle-homebrew-amd64',
        'ignition_transport-ci-ign-transport10-bionic-amd64',
        'ignition_transport-ci-ign-transport10-homebrew-amd64',
        'ign_transport-ci-win',
        'ignition_transport10-install-pkg-bionic-amd64',
        'ignition_transport10-install_bottle-homebrew-amd64',
        'ignition_utils-ci-ign-utils1-bionic-amd64',
        'ignition_utils-ci-ign-utils1-homebrew-amd64',
        'sdformat-ci-sdformat11-bionic-amd64',
        'sdformat-ci-sdformat11-homebrew-amd64',
        'sdformat-ci-sdformat11-windows7-amd64',
        'sdformat-install-sdformat11_pkg-bionic-amd64'
  ],
  'fortress' : [
        'ign_gazebo-ign-5-win',
        'ign_gui-ign-5-win',
        'ign_physics-ign-4-win',
        'ign_rendering-ign-5-win',
        'ign_sensors-ign-5-win',
        'ign_utils-ign-1-win',
        'ignition_fortress-ci-main-homebrew-amd64',
        'ignition_fortress-install-pkg-bionic-amd64',
        'ignition_fortress-install_bottle-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-bionic-amd64',
        'ignition_cmake-ci-ign-cmake2-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-windows7-amd64',
        'ignition_cmake2-install-pkg-bionic-amd64',
        'ignition_cmake2-install_bottle-homebrew-amd64',
        'ignition_common-ci-ign-common4-bionic-amd64',
        'ignition_common-ci-ign-common4-homebrew-amd64',
        'ignition_common-ci-ign-common4-windows7-amd64',
        'ignition_common4-install-pkg-bionic-amd64',
        'ignition_common4-install_bottle-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools6-bionic-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools6-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools6-windows7-amd64',
        'ignition_fuel-tools6-install-pkg-bionic-amd64',
        'ignition_fuel-tools6-install_bottle-homebrew-amd64',
        'ignition_gazebo-ci-ign-gazebo5-bionic-amd64',
        'ignition_gazebo-ci-ign-gazebo5-homebrew-amd64',
        'ignition_gazebo5-install-pkg-bionic-amd64',
        'ignition_gazebo5-install_bottle-homebrew-amd64',
        'ignition_gui-ci-ign-gui5-bionic-amd64',
        'ignition_gui-ci-ign-gui5-homebrew-amd64',
        'ignition_gui5-install-pkg-bionic-amd64',
        'ignition_gui5-install_bottle-homebrew-amd64',
        'ignition_launch-ci-ign-launch4-bionic-amd64',
        'ignition_launch-ci-ign-launch4-homebrew-amd64',
        'ignition_launch4-install-pkg-bionic-amd64',
        'ignition_launch4-install_bottle-homebrew-amd64',
        'ignition_math-ci-ign-math6-bionic-amd64',
        'ignition_math-ci-ign-math6-homebrew-amd64',
        'ignition_math-ci-ign-math6-windows7-amd64',
        'ignition_math6-install-pkg-bionic-amd64',
        'ignition_math6-install_bottle-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs7-bionic-amd64',
        'ignition_msgs-ci-ign-msgs7-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs7-windows7-amd64',
        'ignition_msgs7-install-pkg-bionic-amd64',
        'ignition_msgs7-install_bottle-homebrew-amd64',
        'ignition_physics-ci-ign-physics4-bionic-amd64',
        'ignition_physics-ci-ign-physics4-homebrew-amd64',
        'ignition_physics4-install-pkg-bionic-amd64',
        'ignition_physics4-install_bottle-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-bionic-amd64',
        'ignition_plugin-ci-ign-plugin1-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-windows7-amd64',
        'ignition_plugin-install-pkg-bionic-amd64',
        'ignition_plugin1-install_bottle-homebrew-amd64',
        'ignition_rendering-ci-ign-rendering5-bionic-amd64',
        'ignition_rendering-ci-ign-rendering5-homebrew-amd64',
        'ignition_rendering5-install-pkg-bionic-amd64',
        'ignition_rendering5-install_bottle-homebrew-amd64',
        'ignition_sensors-ci-ign-sensors5-bionic-amd64',
        'ignition_sensors-ci-ign-sensors5-homebrew-amd64',
        'ignition_sensors5-install-pkg-bionic-amd64',
        'ignition_sensors5-install_bottle-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-bionic-amd64',
        'ignition_tools-ci-ign-tools1-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-windows7-amd64',
        'ignition_tools-install-pkg-bionic-amd64',
        'ignition_tools1-install_bottle-homebrew-amd64',
        'ignition_transport-ci-ign-transport10-bionic-amd64',
        'ignition_transport-ci-ign-transport10-homebrew-amd64',
        'ign_transport-ci-win',
        'ignition_transport10-install-pkg-bionic-amd64',
        'ignition_transport10-install_bottle-homebrew-amd64',
        'ignition_utils-ci-ign-utils1-bionic-amd64',
        'ignition_utils-ci-ign-utils1-homebrew-amd64',
        'sdformat-ci-sdformat11-bionic-amd64',
        'sdformat-ci-sdformat11-homebrew-amd64',
        'sdformat-ci-sdformat11-windows7-amd64',
        'sdformat-install-sdformat11_pkg-bionic-amd64'
  ],
]

def DISABLE_TESTS           = false

// Testing compilation from source
ignition_collections.each { ign_collection ->
  // COLCON - Windows
  ign_collection_name = ign_collection.get('name')
  def ignition_win_ci_job = job("ign_${ign_collection_name}-ci-win")
  Globals.gazebodistro_branch = true
  OSRFWinCompilation.create(ignition_win_ci_job, false)
  ignition_win_ci_job.with
  {
      steps {
        batchFile("""\
              set IGNITION_COLLECTION=${ign_collection_name}
              call "./scripts/jenkins-scripts/lib/ign_collection-base.bat"
              """.stripIndent())
      }
  }
  Globals.gazebodistro_branch = false

  ign_collection.get('distros').each { distro ->
    // INSTALL JOBS:
    // --------------------------------------------------------------
    def install_default_job = job("ignition_${ign_collection_name}-install-pkg-${distro}-${arch}")
    OSRFLinuxInstall.create(install_default_job)

    install_default_job.with
    {
      triggers {
        cron(Globals.CRON_EVERY_THREE_DAYS)
      }

      def dev_package = "ignition-${ign_collection_name}"

      label "gpu-reliable"

      def job_name = 'ign_launch-install-test-job.bash'

      steps {
       shell("""\
             #!/bin/bash -xe

             export DISTRO=${distro}
             export ARCH=${arch}
             export INSTALL_JOB_PKG=${dev_package}
             export GZDEV_PROJECT_NAME="${dev_package}"
             /bin/bash -x ./scripts/jenkins-scripts/docker/${job_name}
             """.stripIndent())
      }
    }
  }

  // MAC Brew CI job
  // --------------------------------------------------------------
  def ignition_brew_ci_job = job("ignition_${ign_collection_name}-ci-main-homebrew-amd64")
  OSRFBrewCompilation.create(ignition_brew_ci_job, DISABLE_TESTS)
  OSRFGitHub.create(ignition_brew_ci_job,
                    "ignitionrobotics/ign-${ign_collection_name}",
                    "main",
                    "ign-${ign_collection_name}")
  ignition_brew_ci_job.with
  {
      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -xe "./scripts/jenkins-scripts/lib/project-default-devel-homebrew-amd64.bash" "ignition-${ign_collection_name}"
              """.stripIndent())
      }
  }

  // MAC Brew bottle install job
  // --------------------------------------------------------------
  def ignition_brew_install_bottle_job = job("ignition_${ign_collection_name}-install_bottle-homebrew-amd64")
  OSRFBrewInstall.create(ignition_brew_install_bottle_job)

  ignition_brew_install_bottle_job.with
  {
    triggers {
      cron('@daily')
    }

    def bottle_name = "ignition-${ign_collection_name}"

    steps {
     shell("""\
           #!/bin/bash -xe

           /bin/bash -x ./scripts/jenkins-scripts/lib/project-install-homebrew.bash ${bottle_name}
           """.stripIndent())
    }

    publishers
    {
       configure { project ->
         project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
            unstableOnWarning true
            failBuildOnError false
            parsingRulesPath('/var/lib/jenkins/logparser_warn_on_mark_unstable')
          }
       }
    }
  }

  // DEBBUILD: linux package builder
  // --------------------------------------------------------------
  def build_pkg_job = job("ign-${ign_collection_name}-debbuilder")
  OSRFLinuxBuildPkg.create(build_pkg_job)
  build_pkg_job.with
  {
      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-ignition-debbuild.bash
              """.stripIndent())
      }
   }

  // Ignition dashboards
  dashboardView("ign-${ign_collection_name}")
  {
      jobs {
          ignition_collection_jobs["${ign_collection_name}"].each { jobname ->
            name(jobname)
          }
      }

      columns {
        status()
        weather()
        name()
        testResult(0)
        lastSuccess()
        lastFailure()
        lastDuration()
        buildButton()

      }

      bottomPortlets {
        jenkinsJobsList {
            displayName('Jenkins jobs list')
        }
      }

      configure { view ->
        view / columns << "hudson.plugins.warnings.WarningsColumn" (plugin: 'warnings@5.0.1')

        def topPortlets = view / NodeBuilder.newInstance().topPortlets {}

        topPortlets << 'hudson.plugins.view.dashboard.core.UnstableJobsPortlet' {
            id createPortletId()
            name 'Failing jobs'
            showOnlyFailedJobs 'true'
            recurse 'false'
        }
      }
   }
}

// NIGHTLY GENERATION
def get_nightly_branch(collection_data, ign_package)
{
  try {
    if (collection_data.get(ign_package))
      return collection_data.get(ign_package).get('branch')
  } catch(Exception e) {
    return 'not_enabled_in_DSL'
  }
  return 'not_enabled_in_DSL'
}

collection_data = []
list_of_pkgs = ""

collection_data = ignition_collections.find { it.get('name') == ignition_nightly }
collection_data = collection_data.get('nightly_jobs')

collection_data.each { job ->
  debbuild = job.getValue().get('debbuild')
  list_of_pkgs = "${list_of_pkgs} ${debbuild}"
}

def nightly_scheduler_job = job("ignition-${ignition_nightly}-nightly-scheduler")
OSRFUNIXBase.create(nightly_scheduler_job)

nightly_scheduler_job.with
{
  label "master"

  parameters
  {
     stringParam('NIGHTLY_PACKAGES',"${list_of_pkgs}",
                 'space separated list of packages to build')

     booleanParam('DRY_RUN',false,
                  'run a testing run with no effects')
  }

  triggers {
     cron('0   9    *    *    *')
  }

  cmake_branch = get_nightly_branch(collection_data, 'cmake')
  common_branch = get_nightly_branch(collection_data, 'common')
  fuel_tools_branch = get_nightly_branch(collection_data, 'fuel-tools')
  gazebo_branch = get_nightly_branch(collection_data, 'gazebo')
  gui_branch = get_nightly_branch(collection_data, 'gui')
  launch_branch = get_nightly_branch(collection_data, 'launch')
  math_branch = get_nightly_branch(collection_data, 'math')
  msgs_branch =  get_nightly_branch(collection_data, 'msgs')
  physics_branch = get_nightly_branch(collection_data, 'physics')
  plugin_branch = get_nightly_branch(collection_data, 'plugin')
  rendering_branch = get_nightly_branch(collection_data, 'rendering')
  sensors_branch = get_nightly_branch(collection_data, 'sensors')
  sdformat_branch = get_nightly_branch(collection_data, 'sdformat')
  transport_branch = get_nightly_branch(collection_data, 'transport')
  utils_branch = get_nightly_branch(collection_data, 'utils')

  steps {
    shell("""\
          #!/bin/bash -xe
          set +x # keep password secret
          PASS=\$(cat \$HOME/build_pass)

          dry_run_str=""
          if \$DRY_RUN; then
            dry_run_str="--dry-run"
          fi

          # redirect to not display the password
          for n in \${NIGHTLY_PACKAGES}; do

              if [[ "\${n}" != "\${n/cmake/}" ]]; then
                src_branch="${cmake_branch}"
              elif [[ "\${n}" != "\${n/common/}" ]]; then
                src_branch="${common_branch}"
              elif [[ "\${n}" != "\${n/fuel-tools/}" ]]; then
                src_branch="${fuel_tools_branch}"
              elif  [[ "\${n}" != "\${n/gazebo/}" ]]; then
                src_branch="${gazebo_branch}"
              elif  [[ "\${n}" != "\${n/gui/}" ]]; then
                src_branch="${gui_branch}"
              elif [[ "\${n}" != "\${n/launch/}" ]]; then
                src_branch="${launch_branch}"
              elif [[ "\${n}" != "\${n/math/}" ]]; then
                src_branch="${math_branch}"
              elif [[ "\${n}" != "\${n/msgs/}" ]]; then
                src_branch="${msgs_branch}"
              elif [[ "\${n}" != "\${n/physics/}" ]]; then
                src_branch="${physics_branch}"
              elif [[ "\${n}" != "\${n/plugin/}" ]]; then
                src_branch="${plugin_branch}"
              elif [[ "\${n}" != "\${n/rendering/}" ]]; then
                src_branch="${rendering_branch}"
              elif [[ "\${n}" != "\${n/sensors/}" ]]; then
                src_branch="${sensors_branch}"
              elif [[ "\${n}" != "\${n/sdformat/}" ]]; then
                src_branch="${sdformat_branch}"
              elif [[ "\${n}" != "\${n/transport/}" ]]; then
                src_branch="${transport_branch}"
              elif [[ "\${n}" != "\${n/utils/}" ]]; then
                src_branch="${utils_branch}"
              else
                src_branch="main"
              fi

              echo "releasing \${n} (from branch \${src_branch}"
              python3 ./scripts/release.py \${dry_run_str} "\${n}" nightly "\${PASS}" --extra-osrf-repo prerelease --release-repo-branch main --nightly-src-branch \${src_branch} --upload-to-repo nightly > log || echo "MARK_AS_UNSTABLE"
              echo " - done"
          done

          """.stripIndent())
  }

  publishers
  {
     configure { project ->
       project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
          unstableOnWarning true
          failBuildOnError false
          parsingRulesPath('/var/lib/jenkins/logparser_warn_on_mark_unstable')
        }
     }
  }
}
