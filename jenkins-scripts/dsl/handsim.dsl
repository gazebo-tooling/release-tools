import _configs_.*
import javaposse.jobdsl.dsl.Job

def ci_distro = 'trusty'

def supported_distros = [ 'trusty' ]
def supported_arches = [ 'amd64' ]

def supported_arches_windows = [ 'amd64', 'i386' ]

def handsim_packages = [ 'handsim', 'haptix-comm' ]

// --------------------------------------------------------------
// 1. Create the bundler job
def bundler_job = job("handsim-offline_bundler-builder")
OSRFLinuxBase.create(bundler_job)

bundler_job.with
{
   // Script made to run in the same machine that package repo
   label "master"

   wrappers {
        preBuildCleanup()
   }

   logRotator {
        artifactNumToKeep(2)
   }

   steps {
    shell("""\
          #!/bin/bash -xe

          /bin/bash -x ./scripts/jenkins-scripts/handsim-bundler.bash
          """.stripIndent())
   }
}

// LINUX
handsim_packages.each { pkg ->

  def pkg_name = "${pkg}"

  if ("${pkg_name}" == "haptix-comm")
  {
    pkg_name = "haptix_comm"
  }

  // --------------------------------------------------------------
  // debbuilder jobs
  // debbuilder does not use the underscore name so pkg instead of pkg_name
  def build_pkg_job = job("${pkg}-debbuilder")
  OSRFLinuxBuildPkg.create(build_pkg_job)
  build_pkg_job.with
  {
    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-no-ros-debbuild.bash
            """.stripIndent())
    }

    publishers 
    {
      downstreamParameterized {
        trigger("${pkg_name}-install-pkg-${ci_distro}-amd64") {
          condition('SUCCESS')
          parameters {
            currentBuild()
          }
        }
      }
    }
  }

  supported_distros.each { distro ->
    supported_arches.each { arch ->
      // --------------------------------------------------------------
      // 1. Create the default ci jobs
      def handsim_ci_job = job("${pkg_name}-ci-default-${distro}-${arch}")
      OSRFLinuxCompilation.create(handsim_ci_job)

      handsim_ci_job.with
      {
          if ("${pkg}" == 'handsim')
          {
            label "gpu-reliable-${distro}"
          }

          scm {
            hg("http://bitbucket.org/osrf/${pkg}") {
              branch('default')
              subdirectory("${pkg}")
            }
          }

          triggers {
            scm('*/5 * * * *')
          }

          steps {
            shell("""\
                  #!/bin/bash -xe

                  export DISTRO=${distro}
                  export ARCH=${arch}

                  /bin/bash -xe ./scripts/jenkins-scripts/docker/${pkg}-compilation.bash
                  """.stripIndent())
          }
      }
   
      // --------------------------------------------------------------
      // 2. Create the ANY job
      def handsim_ci_any_job = job("${pkg_name}-ci-pr_any-${distro}-${arch}")
      OSRFLinuxCompilationAny.create(handsim_ci_any_job,
                                    "http://bitbucket.org/osrf/${pkg}")
      handsim_ci_any_job.with
      {
          if ("${pkg}" == 'handsim')
          {
            label "gpu-reliable-${distro}"
          }

          steps 
          {
            shell("""\
                  export DISTRO=${distro}
                  export ARCH=${arch}

                  /bin/bash -xe ./scripts/jenkins-scripts/docker/${pkg}-compilation.bash
                  """.stripIndent())
          }
      }
    }
  }
}

// LINUX (only handsim) 
supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 1. Testing online installation
    def install_default_job = job("handsim-install-pkg-${distro}-${arch}")

    // Use the linux install as base
    OSRFLinuxInstall.create(install_default_job)

    install_default_job.with
    {
       label "gpu-reliable-${distro}"

       triggers {
          cron('@weekly')
       }

        steps {
          shell("""\
                #!/bin/bash -xe

                export INSTALL_JOB_PKG=handsim
                export INSTALL_JOB_REPOS=stable
                /bin/bash -x ./scripts/jenkins-scripts/docker/handsim-install-test-job.bash
                """.stripIndent())
       }
    }

    // --------------------------------------------------------------
    // 2. multiany test  
    def handsim_multiany_job = job("handsim-ci-ign_any+haptix_any-${distro}-${arch}")
    OSRFLinuxCompilation.create(handsim_multiany_job)

    handsim_multiany_job.with
    {
      label "gpu-reliable-${distro}"

      parameters {
        stringParam('HANDSIM_BRANCH','default','handsim branch to use')
        stringParam('IGN_BRANCH','default','ignition transport branch to use')
        stringParam('HAPTIX_BRANCH','default','haptix branch to use')
      }

      steps
      {
         systemGroovyCommand("""\
            job_description = 
                   'handsim: ' + 
                     build.buildVariableResolver.resolve('HANDSIM_BRANCH') + '</b>' +
                   'haptix-comm: ' + 
                     build.buildVariableResolver.resolve('HAPTIX_BRANCH') + '<br />' +
                   'ign-transport: ' + 
                     build.buildVariableResolver.resolve('IGN_BRANCH') + '<br /><br />' +
                   'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH')
            build.setDescription(job_description)
          """.stripIndent()
         )

         shell("""#!/bin/bash -xe

              /bin/bash -x ./scripts/jenkins-scripts/docker/handsim-multiany-devel-trusty-amd64.bash
              """.stripIndent())
      }
    }

    // --------------------------------------------------------------
    // 3. Offline tester
    def unbundler_job = job("handsim-install-offline_bundler-${distro}-${arch}")

    // Use the linux install as base
    OSRFLinuxInstall.create(unbundler_job)

    unbundler_job.with
    {
      parameters
      {
        stringParam('INSTALLED_BUNDLE','',
          'Bundle zip filename to be installed in the system. It is used as base to simulate an update on top of it')
        stringParam('UPDATE_BUNDLE','',
          'Bundle zip filename which will update INSTALLED_BUNDLE in the system')
      }

      steps
      {
        shell("""\
              #!/bin/bash -xe

              /bin/bash -x ./scripts/jenkins-scripts/docker/handsim-install_offline_bundle-test-job.bash
              """.stripIndent())
      }
    }
  }
}

// --------------------------------------------------------------
// WINDOWS

// --------------------------------------------------------------
// 1. Windows any for haptix
def haptix_win_ci_any_job = job("haptix_comm-ci-pr_any-windows7-amd64")
OSRFWinCompilationAny.create(haptix_win_ci_any_job,
                              "http://bitbucket.org/osrf/haptix-comm")
haptix_win_ci_any_job.with
{
    steps {
      batchFile("""\
            call "./scripts/jenkins-scripts/haptix_comm-default-devel-windows-amd64.bat"
            """.stripIndent())
    }
}

// --------------------------------------------------------------
// 2. Windows default for haptix
def haptix_win_ci_job = job("haptix_comm-ci-default-windows7-amd64")
OSRFWinCompilation.create(haptix_win_ci_job)

haptix_win_ci_job.with
{
    scm {
      hg("http://bitbucket.org/osrf/haptix-comm") {
        branch('default')
        // in win use ign-math to match OSRFWinCompilationAny mechanism
        subdirectory("haptix-comm")
      }
    }

    triggers {
      scm('@weekly')
    }

    steps {
      batchFile("""\
            call "./scripts/jenkins-scripts/haptix_comm-default-devel-windows-amd64.bat"
            """.stripIndent())
    }
}

class OSRFWinHaptixSDK
{
  static void create(Job job, String arch = 'amd64')
  {
    OSRFWinBase.create(job)

    job.with
    {
      steps 
      {
        batchFile("""\
            call "./scripts/jenkins-scripts/haptix_comm-sdk-debbuilder-${arch}.bat"
            """.stripIndent())      
      }
   
      publishers
      {
        archiveArtifacts('pkgs/*.zip')

        // Added the lintian parser
        configure { project ->
           project / publishers << 'hudson.plugins.logparser.LogParserPublisher' {
              unstableOnWarning true
              failBuildOnError true
              parsingRulesPath('/var/lib/jenkins/logparser_warn_on_windows_errors')
           }
        }
      }
    }
  }
}

supported_arches_windows.each { arch ->
  // --------------------------------------------------------------
  // 3. Haptix-comm SDK builder
  def haptix_sdk_builder = job("haptix_comm-sdk-builder-windows7-${arch}")
  OSRFWinHaptixSDK.create(haptix_sdk_builder, "${arch}")

  haptix_sdk_builder.with
  {
    steps {
        systemGroovyCommand("""\
            build.setDescription(
             'sdk ${arch} windows7 </b></b>' +
             'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
            """.stripIndent()
          )
    }

    publishers
    {
      downstreamParameterized {
        trigger('repository_uploader_ng') {
          condition('SUCCESS')
          parameters {
            currentBuild()
            predefinedProp("PROJECT_NAME_TO_COPY_ARTIFACTS", "\${JOB_NAME}")
            predefinedProp("S3_UPLOAD_PATH", "haptix/")
            predefinedProp("S3_UPLOAD_CANONICAL_PATH", "false")
            predefinedProp("UPLOAD_TO_REPO", "stable")
            predefinedProp("PACKAGE_ALIAS" , "handsim-sdk")
            predefinedProp("DISTRO",         "win7")
            predefinedProp("ARCH",           "${arch}")
          }
        }
      }
    }
  }
}


// 3. Haptix-comm ANY SDK builder
def haptix_any_sdk_builder = job("haptix_comm-sdk+ign_any+haptix_any-builder-windows7-amd64")
OSRFWinHaptixSDK.create(haptix_any_sdk_builder)

haptix_any_sdk_builder.with
{
  parameters
  {
    stringParam('IGN_TRANSPORT_BRANCH', 'default', 'ignition transport branch to use')
    stringParam('HAPTIX_COMM_BRANCH', 'default', 'ignition transport branch to use')
  }

  steps 
  {
      systemGroovyCommand("""\
          build.setDescription(
           '<b>ign_transport:</b> ' +
              build.buildVariableResolver.resolve('IGN_TRANSPORT_BRANCH') +'<br/>' + 
           '<b>haptix-comm:</b> ' +
              build.buildVariableResolver.resolve('HAPTIX_COMM_BRANCH') + '<br /><br />' +
           'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));
          """.stripIndent()
        )
  }
}
