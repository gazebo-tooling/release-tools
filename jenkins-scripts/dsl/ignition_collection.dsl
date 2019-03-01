import _configs_.*
import javaposse.jobdsl.dsl.Job

// IGNITION COLLECTIONS
ignition_collections = [ 'acropolis' ]

ignition_linux_distros = [ 'acropolis' : [ 'bionic' ] ]
arch = 'amd64'

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
              call "./scripts/jenkins-scripts/lib/ign_collection-default-devel-windows-amd64.bat"
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
}
