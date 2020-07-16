import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION COLLECTIONS
arch = 'amd64'

ignition_nightly = 'dome'

ignition_collections = [
  [ name : 'blueprint',
    distros : [ 'bionic' ],
  ],
  [ name : 'citadel',
    distros : [ 'bionic' ],
  ],
  [ name : 'dome',
    distros : [ 'focal' ],
    nightly_jobs: [
          'fuel-tools': [ debbuild: 'ign-fuel-tools5', branch: 'master '],
          'gazebo'    : [ debbuild: 'ign-gazebo4'    , branch: 'master' ],
          'gui'       : [ debbuild: 'ign-gui4'       , branch: 'master' ],
          'launch'    : [ debbuild: 'ign-launch3'    , branch: 'master' ],
          'msgs'      : [ debbuild: 'ign-msgs6'      , branch: 'master' ],
          'physics'   : [ debbuild: 'ign-physics3'   , branch: 'master' ],
          'rendering' : [ debbuild: 'ign-rendering4' , branch: 'master' ],
          'sensors'   : [ debbuild: 'ign-sensors4'   , branch: 'master' ],
          'sdformat'  : [ debbuild: 'sdformat10'     , branch: 'sdf10'  ],
          'transport' : [ debbuild: 'ign-transport9' , branch: 'master' ],
    ],
  ],
]

ignition_collection_jobs =
[
  'blueprint' : [
        'ign_gazebo-ign-2-win',
        'ign_gui-ign-2-win',
        'ign_physics-ign-1-win',
        'ign_rendering-ign-2-win',
        'ign_sensors-ign-2-win',
        'ignition_blueprint-ci-default-homebrew-amd64',
        'ignition_blueprint-install-pkg-bionic-amd64',
        'ignition_blueprint-install_bottle-homebrew-amd64',
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
        'ignition_fuel-tools-ci-ign-fuel-tools3-bionic-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools3-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools3-windows7-amd64',
        'ignition_fuel-tools3-install-pkg-bionic-amd64',
        'ignition_fuel-tools3-install_bottle-homebrew-amd64',
        'ignition_gazebo-ci-ign-gazebo2-bionic-amd64',
        'ignition_gazebo-ci-ign-gazebo2-homebrew-amd64',
        'ignition_gazebo2-install-pkg-bionic-amd64',
        'ignition_gazebo2-install_bottle-homebrew-amd64',
        'ignition_gui-ci-ign-gui2-bionic-amd64',
        'ignition_gui-ci-ign-gui2-homebrew-amd64',
        'ignition_gui2-install-pkg-bionic-amd64',
        'ignition_gui2-install_bottle-homebrew-amd64',
        'ignition_launch-ci-ign-launch1-bionic-amd64',
        'ignition_launch-ci-ign-launch1-homebrew-amd64',
        'ignition_launch-install-pkg-bionic-amd64',
        'ignition_launch1-install_bottle-homebrew-amd64',
        'ignition_math-ci-ign-math6-bionic-amd64',
        'ignition_math-ci-ign-math6-homebrew-amd64',
        'ignition_math-ci-ign-math6-windows7-amd64',
        'ignition_math6-install-pkg-bionic-amd64',
        'ignition_math6-install_bottle-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs4-bionic-amd64',
        'ignition_msgs-ci-ign-msgs4-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs4-windows7-amd64',
        'ignition_msgs4-install-pkg-bionic-amd64',
        'ignition_msgs4-install_bottle-homebrew-amd64',
        'ignition_physics-ci-ign-physics1-bionic-amd64',
        'ignition_physics-ci-ign-physics1-homebrew-amd64',
        'ignition_physics-install-pkg-bionic-amd64',
        'ignition_physics1-install_bottle-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-bionic-amd64',
        'ignition_plugin-ci-ign-plugin1-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-windows7-amd64',
        'ignition_plugin-install-pkg-bionic-amd64',
        'ignition_plugin1-install_bottle-homebrew-amd64',
        'ignition_rendering-ci-ign-rendering2-bionic-amd64',
        'ignition_rendering-ci-ign-rendering2-homebrew-amd64',
        'ignition_rendering2-install-pkg-bionic-amd64',
        'ignition_rendering2-install_bottle-homebrew-amd64',
        'ignition_sensors-ci-ign-sensors2-bionic-amd64',
        'ignition_sensors-ci-ign-sensors2-homebrew-amd64',
        'ignition_sensors2-install-pkg-bionic-amd64',
        'ignition_sensors2-install_bottle-homebrew-amd64',
        'ignition_tools-ci-ign-tools0-bionic-amd64',
        'ignition_tools-ci-ign-tools0-homebrew-amd64',
        'ignition_tools-ci-ign-tools0-windows7-amd64',
        'ignition_tools-install-pkg-bionic-amd64',
        'ignition_tools0-install_bottle-homebrew-amd64',
        'ignition_transport-ci-ign-transport7-bionic-amd64',
        'ignition_transport-ci-ign-transport7-homebrew-amd64',
        'ignition_transport-ci-ign-transport7-windows7-amd64',
        'ignition_transport7-install-pkg-bionic-amd64',
        'ignition_transport7-install_bottle-homebrew-amd64',
        'sdformat-ci-sdformat8-bionic-amd64',
        'sdformat-ci-sdformat8-homebrew-amd64',
        'sdformat-ci-sdformat8-windows7-amd64',
        'sdformat-install-sdformat8_pkg-bionic-amd64'
  ],

  'citadel' : [
        'ign_gazebo-ign-3-win',
        'ign_gui-ign-3-win',
        'ign_physics-ign-2-win',
        'ign_rendering-ign-3-win',
        'ign_sensors-ign-3-win',
        'ignition_citadel-ci-default-homebrew-amd64',
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
        'ignition_transport-ci-ign-transport8-windows7-amd64',
        'ignition_transport8-install-pkg-bionic-amd64',
        'ignition_transport8-install_bottle-homebrew-amd64',
        'sdformat-ci-sdformat9-bionic-amd64',
        'sdformat-ci-sdformat9-homebrew-amd64',
        'sdformat-ci-sdformat9-windows7-amd64',
        'sdformat-install-sdformat9_pkg-bionic-amd64'
  ],
  'dome' : [
        'ign_gazebo-ci-master-win',
        'ign_gui-ci-master-win',
        'ign_physics-ci-master-win',
        'ign_rendering-ci-master-win',
        'ign_sensors-ci-master-win',
        //'ignition_dome-ci-default-homebrew-amd64',
        //'ignition_dome-install-pkg-bionic-amd64',
        //'ignition_dome-install_bottle-homebrew-amd64',
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
        'ignition_fuel-tools-ci-master-bionic-amd64',
        'ignition_fuel-tools-ci-master-homebrew-amd64',
        'ignition_fuel-tools-ci-master-windows7-amd64',
        // 'ignition_fuel-tools5-install-pkg-bionic-amd64',
        // 'ignition_fuel-tools5-install_bottle-homebrew-amd64',
        'ignition_gazebo-ci-master-bionic-amd64',
        'ignition_gazebo-ci-master-homebrew-amd64',
        // 'ignition_gazebo4-install-pkg-bionic-amd64',
        // 'ignition_gazebo4-install_bottle-homebrew-amd64',
        'ignition_gui-ci-master-bionic-amd64',
        'ignition_gui-ci-master-homebrew-amd64',
        // 'ignition_gui4-install-pkg-bionic-amd64',
        // 'ignition_gui4-install_bottle-homebrew-amd64',
        'ignition_launch-ci-master-bionic-amd64',
        'ignition_launch-ci-master-homebrew-amd64',
        // 'ignition_launch3-install-pkg-bionic-amd64',
        // 'ignition_launch3-install_bottle-homebrew-amd64',
        'ignition_math-ci-ign-math6-bionic-amd64',
        'ignition_math-ci-ign-math6-homebrew-amd64',
        'ignition_math-ci-ign-math6-windows7-amd64',
        'ignition_math6-install-pkg-bionic-amd64',
        'ignition_math6-install_bottle-homebrew-amd64',
        'ignition_msgs-ci-master-bionic-amd64',
        'ignition_msgs-ci-master-homebrew-amd64',
        'ignition_msgs-ci-master-windows7-amd64',
        // 'ignition_msgs6-install-pkg-bionic-amd64',
        // 'ignition_msgs6-install_bottle-homebrew-amd64',
        'ignition_physics-ci-master-bionic-amd64',
        'ignition_physics-ci-master-homebrew-amd64',
        // 'ignition_physics3-install-pkg-bionic-amd64',
        // 'ignition_physics3-install_bottle-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-bionic-amd64',
        'ignition_plugin-ci-ign-plugin1-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-windows7-amd64',
        'ignition_plugin-install-pkg-bionic-amd64',
        'ignition_plugin1-install_bottle-homebrew-amd64',
        'ignition_rendering-ci-master-bionic-amd64',
        'ignition_rendering-ci-master-homebrew-amd64',
        // 'ignition_rendering4-install-pkg-bionic-amd64',
        // 'ignition_rendering4-install_bottle-homebrew-amd64',
        'ignition_sensors-ci-master-bionic-amd64',
        'ignition_sensors-ci-master-homebrew-amd64',
        // 'ignition_sensors4-install-pkg-bionic-amd64',
        // 'ignition_sensors4-install_bottle-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-bionic-amd64',
        'ignition_tools-ci-ign-tools1-homebrew-amd64',
        'ignition_tools-ci-ign-tools1-windows7-amd64',
        'ignition_tools-install-pkg-bionic-amd64',
        // 'ignition_tools1-install_bottle-homebrew-amd64',
        'ignition_transport-ci-master-bionic-amd64',
        'ignition_transport-ci-master-homebrew-amd64',
        'ignition_transport-ci-master-windows7-amd64',
        // 'ignition_transport9-install-pkg-bionic-amd64',
        // 'ignition_transport9-install_bottle-homebrew-amd64',
        'sdformat-ci-sdformat10-bionic-amd64',
        'sdformat-ci-sdformat10-homebrew-amd64',
        'sdformat-ci-sdformat10-windows7-amd64',
        // 'sdformat-install-sdformat10_pkg-bionic-amd64'
  ],
]


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
        cron('@daily')
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
  def ignition_brew_ci_job = job("ignition_${ign_collection_name}-ci-default-homebrew-amd64")
  OSRFBrewCompilationAnyGitHub.create(ignition_brew_ci_job,
                                "ignitionrobotics/ign-${ign_collection_name}",
                                false)
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
OSRFLinuxBase.create(nightly_scheduler_job)

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

              # remove 0 or 1 trailing versions. Use echo + sed to avoid scaping
              # problems with <<<
              if [[ \$(echo \$n | sed -r 's:[a-z]*[A-Z]*([0-9]*):\\1:g') -lt 2 ]]; then
                n=\${n%[0-1]}
              fi

              if [[ "\${n}" == "\${n/ign/ignition}" ]]; then
                    alias=\${n}
                    ignitionrepo=""
              else
                    alias="\${n/ign/ignition}"
                    ignitionrepo="--ignition-repo"
              fi

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
              else
                src_branch="default"
              fi

              echo "releasing \${n} (as \${alias}) from branch \${src_branch} \${ignitionrepo}"
              python ./scripts/release.py \${dry_run_str} "\${n}" nightly "\${PASS}" -a \${alias} --extra-osrf-repo prerelease --nightly-src-branch \${src_branch} --upload-to-repo nightly  \${ignitionrepo} > log || echo "MARK_AS_UNSTABLE"
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

