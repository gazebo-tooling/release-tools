import _configs_.*
import javaposse.jobdsl.dsl.Job
// If failed to import locally be sure of using tools/ scripts
import org.yaml.snakeyaml.Yaml

// GZ COLLECTIONS
arch = 'amd64'

script_dir = new File(".").absolutePath
gz_collections_yaml = new Yaml().load(new FileReader(script_dir + "/gz-collections.yaml"))

gz_nightly = 'harmonic'
gz_collections = [
  [ name : 'harmonic',
    distros : [ 'focal' ],
    // These are the branches currently targeted at the upcoming collection
    // They're in topological order
    nightly_jobs: [
          'cmake'     : [ debbuild: 'gz-cmake3'      , branch: 'gz-cmake3' ],
          'tools'     : [ debbuild: 'gz-tools2'      , branch: 'gz-tools2' ],
          'utils'     : [ debbuild: 'gz-utils2'      , branch: 'gz-utils2' ],
          'math'      : [ debbuild: 'gz-math7'       , branch: 'gz-math7' ],
          'plugin'    : [ debbuild: 'gz-plugin2'     , branch: 'gz-plugin2' ],
          'common'    : [ debbuild: 'gz-common5'     , branch: 'gz-common5' ],
          'msgs'      : [ debbuild: 'gz-msgs10'       , branch: 'main' ],
          'rendering' : [ debbuild: 'gz-rendering8'  , branch: 'main' ],
          'sdformat'  : [ debbuild: 'sdformat13'     , branch: 'sdf13' ],
          'fuel-tools': [ debbuild: 'gz-fuel-tools9' , branch: 'main' ],
          'transport' : [ debbuild: 'gz-transport13' , branch: 'main' ],
          'gui'       : [ debbuild: 'gz-gui8'        , branch: 'main' ],
          'sensors'   : [ debbuild: 'gz-sensors8'    , branch: 'main' ],
          'physics'   : [ debbuild: 'gz-physics6'    , branch: 'gz-physics6' ],
          'gazebo'    : [ debbuild: 'gz-sim8'        , branch: 'main' ],
          'launch'    : [ debbuild: 'gz-launch7'     , branch: 'main' ],
          'garden'    : [ debbuild: 'gz-garden'      , branch: 'main' ],
    ],
  ],
]

gz_collection_jobs =
[
  'citadel' : [
        'ign_cmake-ign-2-win',
        'ign_common-ign-3-win',
        'ign_fuel-tools-ign-4-win',
        'ign_gazebo-ign-3-win',
        'ign_gui-ign-3-win',
        'ign_math-ign-6-win',
        'ign_msgs-ign-5-win',
        'ign_physics-ign-2-win',
        'ign_plugin-ign-1-win',
        'ign_rendering-ign-3-win',
        'ign_sensors-ign-3-win',
        'ign_tools-ign-1-win',
        'ign_transport-ign-8-win',
        'ignition_citadel-ci-main-homebrew-amd64',
        'ignition_citadel-install-pkg-bionic-amd64',
        'ignition_citadel-install_bottle-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-focal-amd64',
        'ignition_cmake-ci-ign-cmake2-homebrew-amd64',
        'ignition_cmake2-install-pkg-focal-amd64',
        'ignition_cmake2-install_bottle-homebrew-amd64',
        'ignition_common-ci-ign-common3-focal-amd64',
        'ignition_common-ci-ign-common3-homebrew-amd64',
        'ignition_common3-install-pkg-focal-amd64',
        'ignition_common3-install_bottle-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools4-focal-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools4-homebrew-amd64',
        'ignition_fuel-tools4-install-pkg-focal-amd64',
        'ignition_fuel-tools4-install_bottle-homebrew-amd64',
        'ignition_gazebo-ci-ign-gazebo3-focal-amd64',
        'ignition_gazebo-ci-ign-gazebo3-homebrew-amd64',
        'ignition_gazebo3-install-pkg-focal-amd64',
        'ignition_gazebo3-install_bottle-homebrew-amd64',
        'ignition_gui-ci-ign-gui3-focal-amd64',
        'ignition_gui-ci-ign-gui3-homebrew-amd64',
        'ignition_gui3-install-pkg-focal-amd64',
        'ignition_gui3-install_bottle-homebrew-amd64',
        'ignition_launch-ci-ign-launch2-focal-amd64',
        'ignition_launch-ci-ign-launch2-homebrew-amd64',
        'ignition_launch2-install-pkg-focal-amd64',
        'ignition_launch2-install_bottle-homebrew-amd64',
        'ignition_math-ci-ign-math6-focal-amd64',
        'ignition_math-ci-ign-math6-homebrew-amd64',
        'ignition_math6-install-pkg-focal-amd64',
        'ignition_math6-install_bottle-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs5-focal-amd64',
        'ignition_msgs-ci-ign-msgs5-homebrew-amd64',
        'ignition_msgs5-install-pkg-focal-amd64',
        'ignition_msgs5-install_bottle-homebrew-amd64',
        'ignition_physics-ci-ign-physics2-focal-amd64',
        'ignition_physics-ci-ign-physics2-homebrew-amd64',
        'ignition_physics2-install-pkg-focal-amd64',
        'ignition_physics2-install_bottle-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-focal-amd64',
        'ignition_plugin-ci-ign-plugin1-homebrew-amd64',
        'ignition_plugin-install-pkg-focal-amd64',
        'ignition_plugin1-install_bottle-homebrew-amd64',
        'ignition_rendering-ci-ign-rendering3-focal-amd64',
        'ignition_rendering-ci-ign-rendering3-homebrew-amd64',
        'ignition_rendering3-install-pkg-focal-amd64',
        'ignition_rendering3-install_bottle-homebrew-amd64',
        'ignition_sensors-ci-ign-sensors3-focal-amd64',
        'ignition_sensors-ci-ign-sensors3-homebrew-amd64',
        'ignition_sensors3-install-pkg-focal-amd64',
        'ignition_sensors3-install_bottle-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-focal-amd64',
        'ignition_tools-ci-ign-tools1-homebrew-amd64',
        'ignition_tools-install-pkg-focal-amd64',
        'ignition_tools1-install_bottle-homebrew-amd64',
        'ignition_transport-ci-ign-transport8-focal-amd64',
        'ignition_transport-ci-ign-transport8-homebrew-amd64',
        'ignition_transport8-install-pkg-focal-amd64',
        'ignition_transport8-install_bottle-homebrew-amd64',
        'sdformat-ci-sdformat9-focal-amd64',
        'sdformat-ci-sdformat9-homebrew-amd64',
        'sdformat-ci-sdformat9-windows7-amd64',
        'sdformat-install-sdformat9_pkg-focal-amd64',
        'sdformat-sdf-9-win'
  ],
  'fortress' : [
        'ign_cmake-ign-2-win',
        'ign_common-ign-4-win',
        'ign_fuel-tools-ign-7-win',
        'ign_gazebo-ign-6-win',
        'ign_gui-ign-6-win',
        'ign_launch-ign-5-win',
        'ign_math-ign-6-win',
        'ign_msgs-ign-8-win',
        'ign_physics-ign-5-win',
        'ign_plugin-ign-1-win',
        'ign_rendering-ign-6-win',
        'ign_sensors-ign-6-win',
        'ign_tools-ign-1-win',
        'ign_transport-ign-11-win',
        'ign_utils-ign-1-win',
        'ignition_cmake-ci-ign-cmake2-focal-amd64',
        'ignition_cmake-ci-ign-cmake2-homebrew-amd64',
        'ignition_cmake2-install-pkg-focal-amd64',
        'ignition_cmake2-install_bottle-homebrew-amd64',
        'ignition_common-ci-ign-common4-focal-amd64',
        'ignition_common-ci-ign-common4-homebrew-amd64',
        'ignition_common4-install-pkg-focal-amd64',
        'ignition_common4-install_bottle-homebrew-amd64',
        'ignition_fortress-ci-main-homebrew-amd64',
        'ignition_fortress-install-pkg-focal-amd64',
        'ignition_fortress-install_bottle-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools7-focal-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools7-homebrew-amd64',
        'ignition_fuel-tools7-install-pkg-focal-amd64',
        'ignition_fuel-tools7-install_bottle-homebrew-amd64',
        'ignition_gazebo-ci-ign-gazebo6-focal-amd64',
        'ignition_gazebo-ci-ign-gazebo6-homebrew-amd64',
        'ignition_gazebo6-install-pkg-focal-amd64',
        'ignition_gazebo6-install_bottle-homebrew-amd64',
        'ignition_gui-ci-ign-gui6-focal-amd64',
        'ignition_gui-ci-ign-gui6-homebrew-amd64',
        'ignition_gui6-install-pkg-focal-amd64',
        'ignition_gui6-install_bottle-homebrew-amd64',
        'ignition_launch-ci-ign-launch5-focal-amd64',
        'ignition_launch-ci-ign-launch5-homebrew-amd64',
        'ignition_launch5-install-pkg-focal-amd64',
        'ignition_launch5-install_bottle-homebrew-amd64',
        'ignition_math-ci-ign-math6-focal-amd64',
        'ignition_math-ci-ign-math6-homebrew-amd64',
        'ignition_math6-install-pkg-focal-amd64',
        'ignition_math6-install_bottle-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs8-focal-amd64',
        'ignition_msgs-ci-ign-msgs8-homebrew-amd64',
        'ignition_msgs8-install-pkg-focal-amd64',
        'ignition_msgs8-install_bottle-homebrew-amd64',
        'ignition_physics-ci-ign-physics5-focal-amd64',
        'ignition_physics-ci-ign-physics5-homebrew-amd64',
        'ignition_physics5-install-pkg-focal-amd64',
        'ignition_physics5-install_bottle-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-focal-amd64',
        'ignition_plugin-ci-ign-plugin1-homebrew-amd64',
        'ignition_plugin-install-pkg-focal-amd64',
        'ignition_plugin1-install_bottle-homebrew-amd64',
        'ignition_rendering-ci-ign-rendering6-focal-amd64',
        'ignition_rendering-ci-ign-rendering6-homebrew-amd64',
        'ignition_rendering6-install-pkg-focal-amd64',
        'ignition_rendering6-install_bottle-homebrew-amd64',
        'ignition_sensors-ci-ign-sensors6-focal-amd64',
        'ignition_sensors-ci-ign-sensors6-homebrew-amd64',
        'ignition_sensors6-install-pkg-focal-amd64',
        'ignition_sensors6-install_bottle-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-focal-amd64',
        'ignition_tools-ci-ign-tools1-homebrew-amd64',
        'ignition_tools-install-pkg-focal-amd64',
        'ignition_tools1-install_bottle-homebrew-amd64',
        'ignition_transport-ci-ign-transport11-focal-amd64',
        'ignition_transport-ci-ign-transport11-homebrew-amd64',
        'ignition_transport11-install-pkg-focal-amd64',
        'ignition_transport11-install_bottle-homebrew-amd64',
        'ignition_utils-ci-ign-utils1-focal-amd64',
        'ignition_utils-ci-ign-utils1-homebrew-amd64',
        'ignition_utils-install-pkg-focal-amd64',
        'ignition_utils1-install_bottle-homebrew-amd64',
        'sdformat-ci-sdformat12-focal-amd64',
        'sdformat-ci-sdformat12-homebrew-amd64',
        'sdformat-install-sdformat12_pkg-focal-amd64',
        'sdformat-sdf-12-win.xml'
  ],
  'garden' : [
        'gz_cmake3-install-pkg-focal-amd64',
        'gz_common5-install-pkg-focal-amd64',
        'gz_fuel-tools8-install-pkg-focal-amd64',
        'gz_garden-install-pkg-focal-amd64',
        'gz_sim7-install-pkg-focal-amd64',
        'gz_gui7-install-pkg-focal-amd64',
        'gz_launch6-install-pkg-focal-amd64',
        'gz_math7-install-pkg-focal-amd64',
        'gz_msgs9-install-pkg-focal-amd64',
        'gz_physics6-install-pkg-focal-amd64',
        'gz_plugin2-install-pkg-focal-amd64',
        'gz_rendering7-install-pkg-focal-amd64',
        'gz_sensors7-install-pkg-focal-amd64',
        'gz_tools2-install-pkg-focal-amd64',
        'gz_transport12-install-pkg-focal-amd64',
        'gz_utils2-install-pkg-focal-amd64',
        'sdformat-install-sdformat13_pkg-focal-amd64',
        'ign_cmake-gz-3-win',
        'ign_common-gz-5-win',
        'ign_fuel-tools-gz-8-win',
        'ign_gazebo-gz-7-win',
        'ign_gui-gz-7-win',
        'ign_launch-gz-6-win',
        'ign_math-gz-7-win',
        'ign_msgs-gz-9-win',
        'ign_physics-gz-6-win',
        'ign_plugin-gz-2-win',
        'ign_rendering-gz-7-win',
        'ign_sensors-gz-7-win',
        'ign_tools-gz-2-win',
        'ign_transport-gz-12-win',
        'ign_utils-gz-2-win',
        'sdformat-sdf-13-win',
        'ignition_cmake-ci-gz-cmake3-focal-amd64',
        'ignition_cmake-ci-gz-cmake3-homebrew-amd64',
        'ignition_cmake3-install_bottle-homebrew-amd64',
        'ignition_common-ci-gz-common5-focal-amd64',
        'ignition_common-ci-gz-common5-homebrew-amd64',
        'ignition_common5-install_bottle-homebrew-amd64',
        'ignition_fuel-tools-ci-gz-fuel-tools8-focal-amd64',
        'ignition_fuel-tools-ci-gz-fuel-tools8-homebrew-amd64',
        'ignition_fuel-tools8-install_bottle-homebrew-amd64',
        'ignition_garden-ci-main-homebrew-amd64',
        'ignition_garden-install-pkg-focal-amd64',
        'ignition_garden-install_bottle-homebrew-amd64',
        'ignition_gazebo-ci-gz-sim7-focal-amd64',
        'ignition_gazebo-ci-gz-sim7-homebrew-amd64',
        'ignition_sim7-install_bottle-homebrew-amd64',
        'ignition_gui-ci-gz-gui7-focal-amd64',
        'ignition_gui-ci-gz-gui7-homebrew-amd64',
        'ignition_gui6-install_bottle-homebrew-amd64',
        'ignition_launch-ci-gz-launch6-focal-amd64',
        'ignition_launch-ci-gz-launch6-homebrew-amd64',
        'ignition_launch6-install_bottle-homebrew-amd64',
        'ignition_math-ci-gz-math7-focal-amd64',
        'ignition_math-ci-gz-math7-homebrew-amd64',
        'ignition_math7-install_bottle-homebrew-amd64',
        'ignition_msgs-ci-gz-msgs9-focal-amd64',
        'ignition_msgs-ci-gz-msgs9-homebrew-amd64',
        'ignition_msgs9-install_bottle-homebrew-amd64',
        'ignition_physics-ci-gz-physics6-focal-amd64',
        'ignition_physics-ci-gz-physics6-homebrew-amd64',
        'ignition_physics6-install_bottle-homebrew-amd64',
        'ignition_plugin-ci-gz-plugin2-focal-amd64',
        'ignition_plugin-ci-gz-plugin2-homebrew-amd64',
        'ignition_plugin2-install_bottle-homebrew-amd64',
        'ignition_rendering-ci-gz-rendering7-focal-amd64',
        'ignition_rendering-ci-gz-rendering7-homebrew-amd64',
        'ignition_rendering7-install_bottle-homebrew-amd64',
        'ignition_sensors-ci-gz-sensors7-focal-amd64',
        'ignition_sensors-ci-gz-sensors7-homebrew-amd64',
        'ignition_sensors7-install_bottle-homebrew-amd64',
        'ignition_tools-ci-gz-tools2-focal-amd64',
        'ignition_tools-ci-gz-tools2-homebrew-amd64',
        'ignition_tools2-install_bottle-homebrew-amd64',
        'ignition_transport-ci-gz-transport12-focal-amd64',
        'ignition_transport-ci-gz-transport12-homebrew-amd64',
        'ignition_transport12-install_bottle-homebrew-amd64',
        'ignition_utils-ci-gz-utils2-focal-amd64',
        'ignition_utils-ci-gz-utils2-homebrew-amd64',
        'ignition_utils2-install_bottle-homebrew-amd64',
        'sdformat-ci-sdformat13-focal-amd64',
        'sdformat-ci-sdformat13-homebrew-amd64'
  ],
  'harmonic' : [
        'ign_cmake-gz-3-win',
        'ign_common-gz-5-win',
        'ign_fuel-tools-gz-9-win',
        'ign_gazebo-ci-win',
        'ign_gui-ci-win',
        'ign_launch-ci-win',
        'ign_math-gz-7-win',
        'ign_msgs-gz-10-win',
        'ign_physics-gz-6-win',
        'ign_plugin-gz-2-win',
        'ign_rendering-ci-win',
        'ign_sensors-ci-win',
        'ign_tools-gz-2-win',
        'ign_transport-gz-13-win',
        'ign_utils-gz-2-win',
        'ignition_cmake-ci-gz-cmake3-focal-amd64',
        'ignition_cmake-ci-gz-cmake3-homebrew-amd64',
        'ignition_common-ci-gz-common5-focal-amd64',
        'ignition_common-ci-gz-common5-homebrew-amd64',
        'ignition_fuel-tools-ci-gz-fuel-tools9-focal-amd64',
        'ignition_fuel-tools-ci-gz-fuel-tools9-homebrew-amd64',
        'ignition_harmonic-ci-main-homebrew-amd64',
        'ignition_gazebo-ci-main-focal-amd64',
        'ignition_gazebo-ci-main-homebrew-amd64',
        'ignition_gui-ci-main-focal-amd64',
        'ignition_gui-ci-main-homebrew-amd64',
        'ignition_launch-ci-main-focal-amd64',
        'ignition_launch-ci-main-homebrew-amd64',
        'ignition_math-ci-gz-math7-focal-amd64',
        'ignition_math-ci-gz-math7-homebrew-amd64',
        'ignition_msgs-ci-gz-msgs10-focal-amd64',
        'ignition_msgs-ci-gz-msgs10-homebrew-amd64',
        'ignition_physics-ci-gz-physics6-focal-amd64',
        'ignition_physics-ci-gz-physics6-homebrew-amd64',
        'ignition_plugin-ci-gz-plugin2-focal-amd64',
        'ignition_plugin-ci-gz-plugin2-homebrew-amd64',
        'ignition_rendering-ci-main-focal-amd64',
        'ignition_rendering-ci-main-homebrew-amd64',
        'ignition_sensors-ci-main-focal-amd64',
        'ignition_sensors-ci-main-homebrew-amd64',
        'ignition_tools-ci-gz-tools2-focal-amd64',
        'ignition_tools-ci-gz-tools2-homebrew-amd64',
        'ignition_transport-ci-gz-transport13-focal-amd64',
        'ignition_transport-ci-gz-transport13-homebrew-amd64',
        'ignition_utils-ci-gz-utils2-focal-amd64',
        'ignition_utils-ci-gz-utils2-homebrew-amd64',
        'sdformat-ci-sdformat13-focal-amd64',
        'sdformat-ci-sdformat13-homebrew-amd64',
        'sdformat-sdf-13-win'
  ],
]

def DISABLE_TESTS           = false

void generate_install_job(prefix, gz_collection_name, distro, arch)
{
  def install_default_job = job("${prefix}_${gz_collection_name}-install-pkg-${distro}-${arch}")
  OSRFLinuxInstall.create(install_default_job)

  install_default_job.with
  {
    triggers {
      cron(Globals.CRON_EVERY_THREE_DAYS)
    }

    def dev_package = "${prefix}-${gz_collection_name}"
    def job_name = 'ign_launch-install-test-job.bash'

    label Globals.nontest_label("gpu-reliable")

    steps {
     shell("""\
           #!/bin/bash -xe

           export DISTRO=${distro}
           export ARCH=${arch}
           export INSTALL_JOB_PKG=${dev_package}
           export GZDEV_PROJECT_NAME="${dev_package}"
           if [[ ${gz_collection_name} == 'citadel' || ${gz_collection_name} == 'fortress' ]]; then
              export GZ_SIM_RUNTIME_TEST_USE_IGN=true
           fi
           /bin/bash -x ./scripts/jenkins-scripts/docker/${job_name}
           """.stripIndent())
    }
  }
}

// Testing compilation from source
gz_collections_yaml.collections.each { collection ->
  gz_collection_name = collection.name
  distros = collection.ci.linux.reference_distro

  // COLCON - Windows
  def gz_win_ci_job = job("ign_${gz_collection_name}-ci-win")
  Globals.gazebodistro_branch = true
  OSRFWinCompilation.create(gz_win_ci_job, false)
  gz_win_ci_job.with
  {
      steps {
        batchFile("""\
              set IGNITION_COLLECTION=${gz_collection_name}
              call "./scripts/jenkins-scripts/lib/ign_collection-base.bat"
              """.stripIndent())
      }
  }
  Globals.gazebodistro_branch = false

  distros.each { distro ->
    // INSTALL JOBS:
    // --------------------------------------------------------------
    if ((gz_collection_name == "citadel") || (gz_collection_name == "fortress")) {
      generate_install_job('ignition', gz_collection_name, distro, arch)
    }
    generate_install_job('gz', gz_collection_name, distro, arch)

    // ROS BOOTSTRAP INSTALL JOBS:
    // --------------------------------------------------------------
    def install_ros_bootstrap_job =
    job("ignition_${gz_collection_name}-install-pkg_ros_bootstrap-any-manual")
    OSRFLinuxInstall.create(install_ros_bootstrap_job)
    install_ros_bootstrap_job.with
    {
      parameters {
        stringParam("LINUX_DISTRO", 'ubuntu', "Linux distribution to build packages for")
        stringParam("DISTRO", distro, "Linux release inside LINUX_DISTRO to build packages for")
        stringParam("ARCH", arch, "Architecture to build packages for")
        labelParam('JENKINS_NODE_TAG') {
          description('Jenkins node or group to run the build on')
          defaultValue('gpu-reliable')
        }
      }

      // Designed to be run manually. No triggers.
      label Globals.nontest_label("gpu-reliable")

      steps {
        systemGroovyCommand("""\
          build.setDescription(
          '<b>' + build.buildVariableResolver.resolve('LINUX_DISTRO') + '/' +
                  build.buildVariableResolver.resolve('DISTRO') + '::' +
                  build.buildVariableResolver.resolve('ARCH') + '</b>' +
          '<br />' +
          'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent())

        shell("""\
             #!/bin/bash -xe


             export INSTALL_JOB_PKG=ignition-${gz_collection_name}
             export USE_ROS_REPO=true
             export ROS_BOOTSTRAP=true
             # needed for arm64 machines and other arch tests
             export ENABLE_GZ_SIM_RUNTIME_TEST=false
             if [[ \${JENKINS_NODE_TAG} == 'gpu-reliable' ]]; then
               export ENABLE_GZ_SIM_RUNTIME_TEST=true
             fi
             /bin/bash -x ./scripts/jenkins-scripts/docker/ign_launch-install-test-job.bash
             """.stripIndent())
      }
    }
  }

  // MAC Brew CI job
  // --------------------------------------------------------------
  def gz_brew_ci_job = job("ignition_${gz_collection_name}-ci-main-homebrew-amd64")
  OSRFBrewCompilation.create(gz_brew_ci_job, DISABLE_TESTS)
  OSRFGitHub.create(gz_brew_ci_job,
                    "gazebosim/gz-${gz_collection_name}",
                    "main",
                    "ign-${gz_collection_name}")
  gz_brew_ci_job.with
  {
      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -xe
              "./scripts/jenkins-scripts/lib/project-default-devel-homebrew-amd64.bash" "gz-${gz_collection_name}"
              """.stripIndent())
      }
  }

  // MAC Brew bottle install job
  // --------------------------------------------------------------
  def gz_brew_install_bottle_job = job("ignition_${gz_collection_name}-install_bottle-homebrew-amd64")
  OSRFBrewInstall.create(gz_brew_install_bottle_job)

  gz_brew_install_bottle_job.with
  {
    triggers {
      cron('@daily')
    }

    def bottle_name = "ignition-${gz_collection_name}"

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
  def build_pkg_job = job("gz-${gz_collection_name}-debbuilder")
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

  // Gazebo dashboards
  dashboardView("ign-${gz_collection_name}")
  {
      jobs {
          gz_collection_jobs["${gz_collection_name}"].each { jobname ->
            name(jobname)
          }
          if (gz_collection_name == gz_nightly) {
            // add nightly debbuild jobs too
            gz_collections.find { it.get('name') == gz_nightly }.get('nightly_jobs').each { job ->
              name(job.getValue().get('debbuild') + '-debbuilder')
            }
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

collection_data = gz_collections.find { it.get('name') == gz_nightly }
collection_data = collection_data.get('nightly_jobs')

collection_data.each { job ->
  debbuild = job.getValue().get('debbuild')
  list_of_pkgs = "${list_of_pkgs} ${debbuild}"
}

def nightly_scheduler_job = job("ignition-${gz_nightly}-nightly-scheduler")
OSRFUNIXBase.create(nightly_scheduler_job)

nightly_scheduler_job.with
{
  label Globals.nontest_label("master")

  parameters
  {
     stringParam('NIGHTLY_PACKAGES',"${list_of_pkgs}",
                 'space separated list of packages to build')

     booleanParam('DRY_RUN',false,
                  'run a testing run with no effects')
  }

  triggers {
     cron(Globals.CRON_START_NIGHTLY)
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
  tools_branch = get_nightly_branch(collection_data, 'tools')
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
              elif  [[ "\${n}" != "\${n/sim/}" ]]; then
                src_branch="${gazebo_branch}"
              elif [[ "\${n}" != "\${n/transport/}" ]]; then
                src_branch="${transport_branch}"
              elif [[ "\${n}" != "\${n/tools/}" ]]; then
                src_branch="${tools_branch}"
              elif [[ "\${n}" != "\${n/utils/}" ]]; then
                src_branch="${utils_branch}"
              else
                src_branch="main"
              fi

              echo "releasing \${n} (from branch \${src_branch}"
              python3 ./scripts/release.py \${dry_run_str} "\${n}" nightly "\${PASS}" --release-repo-branch main --nightly-src-branch \${src_branch} --upload-to-repo nightly > log || echo "MARK_AS_UNSTABLE"
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
