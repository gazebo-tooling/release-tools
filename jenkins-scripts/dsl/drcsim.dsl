import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'trusty' ]
def supported_arches = [ 'amd64' ]

def drcsim_packages = [ 'drcsim', 'drcsim5', 'drcsim7' ]

// LINUX
drcsim_packages.each { pkg ->

  if ("${pkg}" == "drcsim")
  {
     gazebo_version = "4"
  }
  else if ("${pkg}" == "drcsim5")
  {
     gazebo_version = "5"
  }
  else if ("${pkg}" == "drcsim7")
  {
     gazebo_version = "7"
  }

  supported_distros.each { distro ->
    supported_arches.each { arch ->
      // --------------------------------------------------------------
      // 1. Create the default ci jobs
      def drcsim_ci_job = job("${pkg}-ci-default-${distro}-${arch}")
      // not testing, no cppcheck
      OSRFLinuxCompilation.create(drcsim_ci_job, false, false)

      drcsim_ci_job.with
      {
          label "gpu-reliable-${distro}"

          scm {
            hg("https://bitbucket.org/osrf/drcsim") {
              branch('default')
              subdirectory("drcsim")
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
                  export GAZEBO_VERSION_FOR_ROS=${gazebo_version}

                  /bin/bash -xe ./scripts/jenkins-scripts/docker/drcsim-compilation.bash
                  """.stripIndent())
          }
      }
   
      // --------------------------------------------------------------
      // 2. Create the ANY job
      def drcsim_ci_any_job = job("${pkg}-ci_any-default-${distro}-${arch}")
      OSRFLinuxCompilationAny.create(drcsim_ci_any_job,
                                    "https://bitbucket.org/osrf/drcsim", false)
      drcsim_ci_any_job.with
      {
          if ("${pkg}" == 'drcsim')
          {
            label "gpu-reliable-${distro}"
          }

          steps 
          {
            shell("""\
                  export DISTRO=${distro}
                  export ARCH=${arch}
                  export GAZEBO_VERSION_FOR_ROS=${gazebo_version}

                  /bin/bash -xe ./scripts/jenkins-scripts/docker/drcsim-compilation.bash
                  """.stripIndent())
          }
      }

      // --------------------------------------------------------------
      // 3. Testing online installation
      def install_default_job = job("${pkg}-install-pkg-${distro}-${arch}")
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

                  export INSTALL_JOB_PKG=${pkg}
                  export INSTALL_JOB_REPOS=stable
                  /bin/bash -x ./scripts/jenkins-scripts/docker/drcsim-install-test-job.bash
                  """.stripIndent())
         }
      }
    }
  }
}
