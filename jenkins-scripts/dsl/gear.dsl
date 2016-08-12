import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'trusty' ]
def supported_arches = [ 'amd64' ]

// LINUX
// --------------------------------------------------------------
// 1. Create the deb build job
def build_pkg_job = job("gear-debbuilder")
OSRFLinuxBuildPkg.create(build_pkg_job)

build_pkg_job.with
{
  scm {
    hg('http://bitbucket.org/osrf/gear') {
      branch('default')
      subdirectory('gear')
    }
  }

  steps {
    steps {
      shell("""\
            #!/bin/bash -xe

            /bin/bash -x ./scripts/jenkins-scripts/docker/multidistribution-debbuild.bash
            """.stripIndent())
    }
  }
}

supported_distros.each { distro ->
  supported_arches.each { arch ->
    // --------------------------------------------------------------
    // 2. Create the install test job
    def install_default_job = job("gear-install-pkg-${distro}-${arch}")

    // Use the linux install as base
    OSRFLinuxInstall.create(install_default_job)

    install_default_job.with
    {
        triggers {
          cron('@weekly')
        }

        steps {
          shell("""#!/bin/bash -xe

                export LINUX_DISTRO=ubuntu
                export ARCH=${arch}
                export DISTRO=${distro}

                /bin/bash -x ./scripts/jenkins-scripts/docker/gear-install-test-job.bash
                """.stripIndent())
       }
    }
  }
}
