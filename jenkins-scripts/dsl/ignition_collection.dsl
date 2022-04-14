import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION COLLECTIONS
arch = 'amd64'

ignition_nightly = 'garden'

ignition_collections = [
  [ name : 'citadel',
    distros : [ 'bionic' ],
  ],
  [ name : 'fortress',
    distros : [ 'focal' ],
  ],
  [ name : 'garden',
    distros : [ 'focal' ],
    // These are the branches currently targeted at the upcoming collection
    // They're in topological order
    nightly_jobs: [
          'tools'     : [ debbuild: 'ign-tools2'      , branch: 'main' ],
          'cmake'     : [ debbuild: 'ign-cmake3'      , branch: 'main' ],
          'utils'     : [ debbuild: 'ign-utils2'      , branch: 'main' ],
          'math'      : [ debbuild: 'ign-math7'       , branch: 'main' ],
          'plugin'    : [ debbuild: 'ign-plugin2'     , branch: 'main' ],
          'common'    : [ debbuild: 'ign-common5'     , branch: 'main' ],
          'msgs'      : [ debbuild: 'ign-msgs9'       , branch: 'main' ],
          'rendering' : [ debbuild: 'ign-rendering7'  , branch: 'main' ],
          'sdformat'  : [ debbuild: 'sdformat13'      , branch: 'main' ],
          'fuel-tools': [ debbuild: 'ign-fuel-tools8' , branch: 'main' ],
          'transport' : [ debbuild: 'ign-transport12' , branch: 'main' ],
          'gui'       : [ debbuild: 'ign-gui7'        , branch: 'main' ],
          'sensors'   : [ debbuild: 'ign-sensors7'    , branch: 'main' ],
          'physics'   : [ debbuild: 'ign-physics6'    , branch: 'main' ],
          'gazebo'    : [ debbuild: 'ign-gazebo7'     , branch: 'main' ],
          'launch'    : [ debbuild: 'ign-launch6'     , branch: 'main' ],
          'garden'    : [ debbuild: 'ign-garden'      , branch: 'main' ],
    ],
  ],
]

ignition_collection_jobs =
[
  'citadel' : [
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
        'ignition_cmake-ci-ign-cmake2-windows7-amd64',
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
        'sdformat-install-sdformat9_pkg-focal-amd64'
  ],
  'fortress' : [
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
        'ignition_cmake-ci-ign-cmake2-windows7-amd64',
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
        'sdformat-ci-sdformat12-windows7-amd64',
        'sdformat-install-sdformat12_pkg-focal-amd64'
  ],
  'garden' : [
        'ign_common-ci-win',
        'ign_fuel-tools-ci-win',
        'ign_gazebo-ci-win',
        'ign_gui-ci-win',
        'ign_launch-ci-win',
        'ign_math-ci-win',
        'ign_msgs-ci-win',
        'ign_physics-ci-win',
        'ign_plugin-ign-1-win',
        'ign_rendering-ci-win',
        'ign_sensors-ci-win',
        'ign_tools-ign-1-win',
        'ign_transport-ci-win',
        'ign_utils-ign-1-win',
        'ignition_cmake-ci-ign-cmake2-focal-amd64',
        'ignition_cmake-ci-ign-cmake2-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-windows7-amd64',
        'ignition_cmake2-install-pkg-focal-amd64',
        'ignition_cmake2-install_bottle-homebrew-amd64',
        'ignition_common-ci-main-focal-amd64',
        'ignition_common-ci-main-homebrew-amd64',
        'ignition_common4-install-pkg-focal-amd64',
        'ignition_common4-install_bottle-homebrew-amd64',
        'ignition_garden-ci-main-homebrew-amd64',
        'ignition_garden-install-pkg-focal-amd64',
        'ignition_garden-install_bottle-homebrew-amd64',
        'ignition_fuel-tools-ci-main-focal-amd64',
        'ignition_fuel-tools-ci-main-homebrew-amd64',
        'ignition_fuel-tools7-install-pkg-focal-amd64',
        'ignition_fuel-tools7-install_bottle-homebrew-amd64',
        'ignition_gazebo-ci-main-focal-amd64',
        'ignition_gazebo-ci-main-homebrew-amd64',
        'ignition_gazebo6-install-pkg-focal-amd64',
        'ignition_gazebo6-install_bottle-homebrew-amd64',
        'ignition_gui-ci-main-focal-amd64',
        'ignition_gui-ci-main-homebrew-amd64',
        'ignition_gui6-install-pkg-focal-amd64',
        'ignition_gui6-install_bottle-homebrew-amd64',
        'ignition_launch-ci-main-focal-amd64',
        'ignition_launch-ci-main-homebrew-amd64',
        'ignition_launch5-install-pkg-focal-amd64',
        'ignition_launch5-install_bottle-homebrew-amd64',
        'ignition_math-ci-main-focal-amd64',
        'ignition_math-ci-main-homebrew-amd64',
        'ignition_math6-install-pkg-focal-amd64',
        'ignition_math6-install_bottle-homebrew-amd64',
        'ignition_msgs-ci-main-focal-amd64',
        'ignition_msgs-ci-main-homebrew-amd64',
        'ignition_msgs8-install-pkg-focal-amd64',
        'ignition_msgs8-install_bottle-homebrew-amd64',
        'ignition_physics-ci-main-focal-amd64',
        'ignition_physics-ci-main-homebrew-amd64',
        'ignition_physics5-install-pkg-focal-amd64',
        'ignition_physics5-install_bottle-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-focal-amd64',
        'ignition_plugin-ci-ign-plugin1-homebrew-amd64',
        'ignition_plugin-install-pkg-focal-amd64',
        'ignition_plugin1-install_bottle-homebrew-amd64',
        'ignition_rendering-ci-main-focal-amd64',
        'ignition_rendering-ci-main-homebrew-amd64',
        'ignition_rendering6-install-pkg-focal-amd64',
        'ignition_rendering6-install_bottle-homebrew-amd64',
        'ignition_sensors-ci-main-focal-amd64',
        'ignition_sensors-ci-main-homebrew-amd64',
        'ignition_sensors6-install-pkg-focal-amd64',
        'ignition_sensors6-install_bottle-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-focal-amd64',
        'ignition_tools-ci-ign-tools1-homebrew-amd64',
        'ignition_tools-install-pkg-focal-amd64',
        'ignition_tools1-install_bottle-homebrew-amd64',
        'ignition_transport-ci-main-focal-amd64',
        'ignition_transport-ci-main-homebrew-amd64',
        'ignition_transport11-install-pkg-focal-amd64',
        'ignition_transport11-install_bottle-homebrew-amd64',
        'ignition_utils-ci-ign-utils1-focal-amd64',
        'ignition_utils-ci-ign-utils1-homebrew-amd64',
        'ignition_utils-install-pkg-focal-amd64',
        'ignition_utils1-install_bottle-homebrew-amd64',
        'sdformat-ci-main-focal-amd64',
        'sdformat-ci-main-homebrew-amd64',
        'sdformat-ci-main-windows7-amd64',
        'sdformat-install-sdformat12_pkg-focal-amd64'
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
          if (ign_collection_name == ignition_nightly) {
            // add nightly debbuild jobs too
            ignition_collections.find { it.get('name') == ignition_nightly }.get('nightly_jobs').each { job ->
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
