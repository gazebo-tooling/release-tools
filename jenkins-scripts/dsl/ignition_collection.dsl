import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION COLLECTIONS
ignition_collections = [ 'acropolis' ]

ignition_linux_distros = [ 'acropolis' : [ 'bionic' ] ]
arch = 'amd64'

ignition_collection_jobs = [ 'acropolis' : [
        'ign_gui-ign-1-win',
        'ign_physics-ign-1-win',
        'ign_rendering-ign-1-win',
        'ign_sensors-ign-1-win',
        'ignition_acropolis-ci-default-homebrew-amd64',
        'ignition_acropolis-install-pkg-bionic-amd64',
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
        'ignition_gazebo-ci-default-bionic-amd64',
        'ignition_gazebo-ci-default-homebrew-amd64',
        'ignition_gazebo-ci-default-windows7-amd64',
        'ignition_gui-ci-ign-gui1-bionic-amd64',
        'ignition_gui-ci-ign-gui1-homebrew-amd64',
        'ignition_gui-install-pkg-bionic-amd64',
        'ignition_launch-ci-default-bionic-amd64',
        'ignition_launch-ci-default-homebrew-amd64',
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
        'ignition_tools-ci-default-bionic-amd64',
        'ignition_tools-ci-default-homebrew-amd64',
        'ignition_tools-ci-default-windows7-amd64',
        'ignition_tools-install-pkg-bionic-amd64',
        'ignition_transport-ci-ign-transport6-bionic-amd64',
        'ignition_transport-ci-ign-transport6-homebrew-amd64',
        'ignition_transport-ci-ign-transport6-windows7-amd64',
        'ignition_transport6-install-pkg-bionic-amd64',
        'sdformat-ci-sdformat8-bionic-amd64',
        'sdformat-ci-sdformat8-homebrew-amd64',
        'sdformat-ci-sdformat8-windows7-amd64',
        'sdformat-install-sdformat8_pkg-bionic-amd64']]

// Testing compilation from source
ignition_collections.each { ign_collection ->
  // COLCON - Windows
  def ignition_win_ci_job = job("ign_${ign_collection}-ci-win")
  Globals.gazebodistro_branch = true
  OSRFWinCompilation.create(ignition_win_ci_job, false)
  ignition_win_ci_job.with
  {
      steps {
        batchFile("""\
              set IGNITION_COLLECTION=${ign_collection}
              call "./scripts/jenkins-scripts/lib/ign_collection-base.bat"
              """.stripIndent())
      }
  }

  ignition_linux_distros["${ign_collection}"].each { distro ->
    // INSTALL JOBS:
    // --------------------------------------------------------------
    def install_default_job = job("ignition_${ign_collection}-install-pkg-${distro}-${arch}")
    OSRFLinuxInstall.create(install_default_job)

    install_default_job.with
    {
      triggers {
        cron('@daily')
      }

      def dev_package = "ignition-${ign_collection}"

      steps {
       shell("""\
             #!/bin/bash -xe

             export DISTRO=${distro}
             export ARCH=${arch}
             export INSTALL_JOB_PKG=${dev_package}
             export INSTALL_JOB_REPOS="stable"
             /bin/bash -x ./scripts/jenkins-scripts/docker/generic-install-test-job.bash
             """.stripIndent())
      }
    }
  }

  // MAC Brew
  // --------------------------------------------------------------
  def ignition_brew_ci_job = job("ignition_${ign_collection}-ci-default-homebrew-amd64")
  OSRFBrewCompilation.create(ignition_brew_ci_job, false)
  ignition_brew_ci_job.with
  {
      steps {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -xe "./scripts/jenkins-scripts/lib/project-default-devel-homebrew-amd64.bash" "ignition-${ign_collection}"
              fi
              """.stripIndent())
      }
  }

  // DEBBUILD: linux package builder
  // --------------------------------------------------------------
  def build_pkg_job = job("ign-${ign_collection}-debbuilder")
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
  dashboardView("DSLign-${ign_collection}")
  {
      jobs {
          ignition_collection_jobs["${ign_collection}"].each { jobname ->
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
