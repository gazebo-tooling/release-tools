import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org"

packages = [:]
packages['science-team'] = ['console-bridge',
                            'gazebo',
                            'fcl',
                            'ignition-cmake',
                            'ignition-common',
                            'ignition-math2',
                            'ignition-math4',
                            'ignition-msgs',
                            'ignition-tools',
                            'ignition-transport', // version 4
                            'dart',
                            'libccd',
                            'octomap',
                            'sdformat', // version 6
                            'simbody',
                            'urdfdom',
                            'urdfdom-headers' ]

packages['jrivero-guest'] = ['empy']


packages.each { repo_name, pkgs ->
 pkgs.each { pkg ->

  // --------------------------------------------------------------
  // 1. Create the job that tries to install packages
  def install_job = job("${pkg}-install-pkg-debian_sid-amd64")
  OSRFLinuxInstall.create(install_job)
  install_job.with
  {
     triggers {
       cron('@weekly')
     }

    // No accepted in Sid yet
    if ((pkg == 'sdformat6') || (pkg == 'ignition-transport4'))
    {
      disabled()
    }

     steps {
      shell("""\
            #!/bin/bash -xe

            export LINUX_DISTRO=debian
            export DISTRO=sid
            export ARCH=amd64
            # Hack to select the latest -dev of series
            export INSTALL_JOB_PKG="\\\$(apt-cache search ${pkg} | grep '${pkg}[0-9]-dev -' | tail -1 | awk '{print \\\$1}')"
            /bin/bash -x ./scripts/jenkins-scripts/docker/generic-install-test-job.bash
            """.stripIndent())
    }
  }

// ratt package to help during transition
def ratt_pkg_job = job("debian-ratt-builder")
OSRFLinuxBase.create(ratt_pkg_job)
ratt_pkg_job.with
{
  // use only the most powerful nodes
  label "large-memory"

  parameters
  {
     stringParam('DEB_PACKAGE','master',
                 'package to run ratt against (check lib transition)')
  }

  logRotator {
    artifactNumToKeep(10)
  }

  concurrentBuild(true)

  throttleConcurrentBuilds {
    maxPerNode(1)
    maxTotal(5)
  }

  steps {
    shell("""\
          #!/bin/bash -xe

          /bin/bash -xe ./scripts/jenkins-scripts/docker/debian-ratt-builder.bash
          """.stripIndent())
  }
}




