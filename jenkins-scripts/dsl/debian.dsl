import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org"

packages = [:]
packages['debian-science'] = ['console-bridge',
                              'gazebo',
                              'fcl',
                              'ignition-math2',
                              'ignition-transport',
                              'kido',
                              'libccd',
                              'robot-player',
                              'sdformat',
                              'simbody',
                              'urdfdom',
                              'urdfdom-headers' ]

packages['collab-maint']   = ['peak-linux-driver']

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

  if (repo_name == 'debian-science') {
    git_repo = "git://anonscm.debian.org/${repo_name}/packages/${pkg}.git"
  } else {
    git_repo = "git://anonscm.debian.org/${repo_name}/${pkg}.git"
  }

  def ci_job = job("${pkg}-pkg_builder-master-debian_sid-amd64")
  OSRFLinuxBase.create(ci_job)
  ci_job.with
  {
      scm {
	git("${git_repo}") {
	  branch('master')
	  subdirectory("${pkg}")
	}
      }

      triggers {
        scm('@daily')
      }

      priority 300

      logRotator {
        artifactNumToKeep(10)
      }

      concurrentBuild(true)

      throttleConcurrentBuilds {
	maxPerNode(1)
	maxTotal(5)
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
              export GIT_REPOSITORY="${git_repo}"

              /bin/bash -xe ./scripts/jenkins-scripts/docker/debian-git-debbuild.bash
              """.stripIndent())
      }

      publishers
      {
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
