import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION COLLECTIONS
arch = 'amd64'

ignition_nightly = 'blueprint'

ignition_collections = [
  [ name : 'acropolis',
    distros : [ 'bionic' ],
  ],
  [ name : 'blueprint',
    nightly_jobs: [
          'cmake'     : [ debbuild: 'ign-cmake2'     , branch: 'ign-cmake2'      ],
          'common'    : [ debbuild: 'ign-common3'    , branch: 'ign-common3'     ],
          'fuel-tools': [ debbuild: 'ign-fuel-tools3', branch: 'ign-fuel-tools3' ],
          'gazebo'    : [ debbuild: 'ign-gazebo2'    , branch: 'ign-gazebo2'     ],
          'gui'       : [ debbuild: 'ign-gui2'       , branch: 'ign-gui2'        ],
          'launch'    : [ debbuild: 'ign-launch'     , branch: 'ign-launch1'     ],
          'math'      : [ debbuild: 'ign-math6'      , branch: 'ign-math6'       ],
          'msgs'      : [ debbuild: 'ign-msgs4'      , branch: 'ign-msgs4'       ],
          'physics'   : [ debbuild: 'ign-physics'    , branch: 'ign-physics1'    ],
          'plugin'    : [ debbuild: 'ign-plugin'     , branch: 'ign-plugin1'     ],
          'rendering' : [ debbuild: 'ign-rendering2' , branch: 'ign-rendering2'  ],
          'sensors'   : [ debbuild: 'ign-sensors2'   , branch: 'ign-sensors2'    ],
          'sdformat'  : [ debbuild: 'sdformat8'      , branch: 'sdf8'            ],
          'transport' : [ debbuild: 'ign-transport7' , branch: 'ign-transport7'  ]
    ],
    distros : [ 'bionic' ],
  ]
]

ignition_collection_jobs =
[
  'acropolis' : [
        'ign_gazebo-ign-1-win',
        'ign_gui-ign-1-win',
        'ign_physics-ign-1-win',
        'ign_rendering-ign-1-win',
        'ign_sensors-ign-1-win',
        'ignition_acropolis-ci-default-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-bionic-amd64',
        'ignition_cmake-ci-ign-cmake2-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-windows7-amd64',
        'ignition_common-ci-ign-common3-bionic-amd64',
        'ignition_common-ci-ign-common3-homebrew-amd64',
        'ignition_common-ci-ign-common3-windows7-amd64',
        'ignition_common3-install-pkg-bionic-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools3-bionic-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools3-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools3-windows7-amd64',
        'ignition_fuel-tools3-install-pkg-bionic-amd64',
        'ignition_gazebo-ci-ign-gazebo1-bionic-amd64',
        'ignition_gazebo-ci-ign-gazebo1-homebrew-amd64',
        'ignition_gui-ci-ign-gui1-bionic-amd64',
        'ignition_gui-ci-ign-gui1-homebrew-amd64',
        'ignition_gui-install-pkg-bionic-amd64',
        'ignition_launch-ci-ign-launch1-bionic-amd64',
        'ignition_launch-ci-ign-launch1-homebrew-amd64',
        'ignition_math-ci-ign-math6-bionic-amd64',
        'ignition_math-ci-ign-math6-homebrew-amd64',
        'ignition_math-ci-ign-math6-windows7-amd64',
        'ignition_math6-install-pkg-bionic-amd64',
        'ignition_msgs-ci-ign-msgs3-bionic-amd64',
        'ignition_msgs-ci-ign-msgs3-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs3-windows7-amd64',
        'ignition_msgs3-install-pkg-bionic-amd64',
        'ignition_physics-ci-ign-physics1-bionic-amd64',
        'ignition_physics-ci-ign-physics1-homebrew-amd64',
        'ignition_physics-install-pkg-bionic-amd64',
        'ignition_plugin-ci-ign-plugin1-bionic-amd64',
        'ignition_plugin-ci-ign-plugin1-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-windows7-amd64',
        'ignition_plugin-install-pkg-bionic-amd64',
        'ignition_rendering-ci-ign-rendering1-bionic-amd64',
        'ignition_rendering-ci-ign-rendering1-homebrew-amd64',
        'ignition_rendering-install-pkg-bionic-amd64',
        'ignition_sensors-ci-ign-sensors1-bionic-amd64',
        'ignition_sensors-ci-ign-sensors1-homebrew-amd64',
        'ignition_sensors-install-pkg-bionic-amd64',
        'ignition_tools-ci-ign-tools0-bionic-amd64',
        'ignition_tools-ci-ign-tools0-homebrew-amd64',
        'ignition_tools-ci-ign-tools0-windows7-amd64',
        'ignition_tools-install-pkg-bionic-amd64',
        'ignition_transport-ci-ign-transport6-bionic-amd64',
        'ignition_transport-ci-ign-transport6-homebrew-amd64',
        'ignition_transport-ci-ign-transport6-windows7-amd64',
        'ignition_transport6-install-pkg-bionic-amd64',
        'sdformat-ci-sdformat8-bionic-amd64',
        'sdformat-ci-sdformat8-homebrew-amd64',
        'sdformat-ci-sdformat8-windows7-amd64',
        'sdformat-install-sdformat8_pkg-bionic-amd64'
  ],

  'blueprint' : [
        'ign_gazebo-ign-2-win',
        'ign_gui-ign-2-win',
        'ign_physics-ign-1-win',
        'ign_rendering-ign-2-win',
        'ign_sensors-ign-2-win',
        'ignition_blueprint-ci-default-homebrew-amd64',
        'ignition_blueprint-install-pkg-bionic-amd64',
        'ignition_cmake-ci-ign-cmake2-bionic-amd64',
        'ignition_cmake-ci-ign-cmake2-homebrew-amd64',
        'ignition_cmake-ci-ign-cmake2-windows7-amd64',
        'ignition_common-ci-ign-common3-bionic-amd64',
        'ignition_common-ci-ign-common3-homebrew-amd64',
        'ignition_common-ci-ign-common3-windows7-amd64',
        'ignition_common3-install-pkg-bionic-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools3-bionic-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools3-homebrew-amd64',
        'ignition_fuel-tools-ci-ign-fuel-tools3-windows7-amd64',
        'ignition_fuel-tools3-install-pkg-bionic-amd64',
        'ignition_gazebo-ci-ign-gazebo2-bionic-amd64',
        'ignition_gazebo-ci-ign-gazebo2-homebrew-amd64',
        'ignition_gazebo2-install-pkg-bionic-amd64',
        'ignition_gui-ci-ign-gui2-bionic-amd64',
        'ignition_gui-ci-ign-gui2-homebrew-amd64',
        'ignition_gui2-install-pkg-bionic-amd64',
        'ignition_launch-ci-ign-launch1-bionic-amd64',
        'ignition_launch-ci-ign-launch1-homebrew-amd64',
        'ignition_launch-install-pkg-bionic-amd64',
        'ignition_math-ci-ign-math6-bionic-amd64',
        'ignition_math-ci-ign-math6-homebrew-amd64',
        'ignition_math-ci-ign-math6-windows7-amd64',
        'ignition_math6-install-pkg-bionic-amd64',
        'ignition_msgs-ci-ign-msgs4-bionic-amd64',
        'ignition_msgs-ci-ign-msgs4-homebrew-amd64',
        'ignition_msgs-ci-ign-msgs4-windows7-amd64',
        'ignition_msgs4-install-pkg-bionic-amd64',
        'ignition_physics-ci-ign-physics1-bionic-amd64',
        'ignition_physics-ci-ign-physics1-homebrew-amd64',
        'ignition_physics-install-pkg-bionic-amd64',
        'ignition_plugin-ci-ign-plugin1-bionic-amd64',
        'ignition_plugin-ci-ign-plugin1-homebrew-amd64',
        'ignition_plugin-ci-ign-plugin1-windows7-amd64',
        'ignition_plugin-install-pkg-bionic-amd64',
        'ignition_rendering-ci-ign-rendering2-bionic-amd64',
        'ignition_rendering-ci-ign-rendering2-homebrew-amd64',
        'ignition_rendering2-install-pkg-bionic-amd64',
        'ignition_sensors-ci-ign-sensors2-bionic-amd64',
        'ignition_sensors-ci-ign-sensors2-homebrew-amd64',
        'ignition_sensors2-install-pkg-bionic-amd64',
        'ignition_tools-ci-ign-tools0-bionic-amd64',
        'ignition_tools-ci-ign-tools0-homebrew-amd64',
        'ignition_tools-ci-ign-tools0-windows7-amd64',
        'ignition_tools-install-pkg-bionic-amd64',
        'ignition_transport-ci-ign-transport7-bionic-amd64',
        'ignition_transport-ci-ign-transport7-homebrew-amd64',
        'ignition_transport-ci-ign-transport7-windows7-amd64',
        'ignition_transport7-install-pkg-bionic-amd64',
        'sdformat-ci-sdformat8-bionic-amd64',
        'sdformat-ci-sdformat8-homebrew-amd64',
        'sdformat-ci-sdformat8-windows7-amd64',
        'sdformat-install-sdformat8_pkg-bionic-amd64'
  ]
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
      // acropolis deb is broken because old libignition-launch-dev debs are not available
      if (ign_collection_name == 'acropolis')
        disabled()

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

  // MAC Brew
  // --------------------------------------------------------------
  def ignition_brew_ci_job = job("ignition_${ign_collection_name}-ci-default-homebrew-amd64")
  OSRFBrewCompilationAny.create(ignition_brew_ci_job,
                                "https://bitbucket.org/ignitionrobotics/ign-${ign_collection_name}",
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

  cmake_branch = collection_data.get('cmake').get('branch')
  common_branch = collection_data.get('common').get('branch')
  fuel_tools_branch = collection_data.get('fuel-tools').get('branch')
  gazebo_branch = collection_data.get('gazebo').get('branch')
  gui_branch = collection_data.get('gui').get('branch')
  launch_branch = collection_data.get('launch').get('branch')
  math_branch = collection_data.get('math').get('branch')
  msgs_branch = collection_data.get('msgs').get('branch')
  physics_branch = collection_data.get('physics').get('branch')
  plugin_branch = collection_data.get('plugin').get('branch')
  rendering_branch = collection_data.get('rendering').get('branch')
  sensors_branch = collection_data.get('sensors').get('branch')
  sdformat_branch = collection_data.get('sdformat').get('branch')
  transport_branch = collection_data.get('transport').get('branch')

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

              # remove 0 or 1 trailing versions
              n=\${n%[0-1]}

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

