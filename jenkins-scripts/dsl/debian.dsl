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
                            'kido',
                            'libccd',
                            'octomap',
                            'sdformat', // version 6
                            'simbody',
                            'urdfdom',
                            'urdfdom-headers' ]

packages['collab-maint']   = ['peak-linux-driver',
                              'peak-pcan-basic']

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

  // --------------------------------------------------------------
  // 2. Create the job that tries to build the package and run lintian

  if (repo_name == 'science-team') {
    git_repo = "https://salsa.debian.org/${repo_name}/${pkg}.git"
  } else {
    git_repo = "git://anonscm.debian.org/${repo_name}/${pkg}.git"
  }

  def ci_job = job("${pkg}-pkg_builder-master-debian_sid-amd64")
  OSRFLinuxBuildPkgBase.create(ci_job)
  ci_job.with
  {
     scm {
        git {
          remote {
            url("${git_repo}")
          }
          extensions {
            cleanBeforeCheckout()
            relativeTargetDirectory('repo')
          }

          branch('refs/heads/master')
        }
      }

      triggers {
        scm('@daily')
      }

      properties {
        priority 350
      }

      parameters {
        textParam("RELEASE_VERSION", null, "osrfX, OSRF postix version")
        textParam("RELEASE_ARCH_VERSION", null, "~ARCH-X, release version")
      }

      steps {
        shell("""\
              #!/bin/bash -xe

              export LINUX_DISTRO=debian
              export DISTRO=sid

              /bin/bash -xe ./scripts/jenkins-scripts/docker/debian-git-debbuild.bash
              """.stripIndent())
      }

      publishers
      {
        postBuildScripts {
          steps {
            shell("""\
              #!/bin/bash -xe

              [[ -d \${WORKSPACE}/repo ]] && sudo chown -R jenkins \${WORKSPACE}/repo
              """.stripIndent())
          }

          onlyIfBuildSucceeds(false)
          onlyIfBuildFails(false)
        }


         // Added the lintian parser
         configure { project ->
           project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
              unstableOnWarning true
              failBuildOnError false
              parsingRulesPath('/var/lib/jenkins/logparser_lintian')
           }
         }
      }
    }
  }
}
