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
  // 2. Create the job that tries to build the package and run lintian
  def ci_job = job("${pkg}-pkg_builder-master-debian_sid-amd64")
  OSRFLinuxBuildPkgBase.create(ci_job)
  ci_job.with
  {
     scm {
        git {
          remote {
            url("https://salsa.debian.org/${repo_name}/${pkg}.git")
          }
          extensions {
            cleanBeforeCheckout()
            relativeTargetDirectory('repo')
          }

          branch('refs/heads/master')
        }
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

// ratt package to help during transition
def ratt_pkg_job = job("debian-ratt-builder")
OSRFLinuxBase.create(ratt_pkg_job)
ratt_pkg_job.with
{
  // use only the most powerful nodes
  label Globals.nontest_label("large-memory")

  parameters
  {
     stringParam('DEB_PACKAGE','master',
                 'package to run ratt against (check lib transition)')
     booleanParam('USE_UNSTABLE', true,
                 'use unstable instead of experimental to test packages')
     stringParam('RATT_INCLUDE','',
                 'Regexp for package inclusion: ^(hwloc|fltk1.3|starpu)$. See https://github.com/j-rivero/ratt/blob/master/README.md')
     stringParam('RATT_EXCLUDE','',
                 'Regexp for package exclusion: ^(gcc-9|gcc-8)$. See https://github.com/j-rivero/ratt/blob/master/README.md')
  }

  logRotator {
    artifactNumToKeep(10)
    numToKeep(75)
  }

  concurrentBuild(true)

  throttleConcurrentBuilds {
    maxPerNode(1)
    maxTotal(5)
  }

  publishers
  {
    // Added the checker result parser (UNSTABLE if not success)
    configure { project ->
      project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
        unstableOnWarning true
        failBuildOnError false
        parsingRulesPath('/var/lib/jenkins/logparser_warn_on_mark_unstable')
      }
    }

    archiveArtifacts('logs/buildlogs/*')
  }


  steps {
    shell("""\
          #!/bin/bash -xe

          /bin/bash -xe ./scripts/jenkins-scripts/docker/debian-ratt-builder.bash
          """.stripIndent())
  }
}
