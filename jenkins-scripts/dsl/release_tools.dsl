import _configs_.*
import javaposse.jobdsl.dsl.Job

Globals.default_emails = "jrivero@osrfoundation.org"

def supported_distros = [ 'bionic' ]
def supported_arches = [ 'amd64' ]

void include_default_params(Job job, String distro, String arch)
{
  job.with
  {
    steps {
      shell("""\
	    #!/bin/bash -xe

	    export DISTRO=${distro}
	    export ARCH=${arch}

	    /bin/bash -xe ./scripts/jenkins-scripts/docker/release-tools-checker.bash
	    """.stripIndent())
    }

    publishers {
      checkstyle('**/shellcheck_results/*') {
         defaultEncoding('UTF-8')
         thresholds(
           unstableTotal: [all: 1,  high: 0, normal: 300, low: 805],
           unstableNew:   [all: 0,  high: 0, normal: 0, low: 0],
           failedNew:     [all: 0,  high: 0, normal: 0, low: 0])
      }
    }
  }
}

supported_distros.each { distro ->
  supported_arches.each { arch ->
    // 1. ci default
    def ci_job = job("release-tools-ci-default-${distro}-${arch}")
    OSRFLinuxCompilation.create(ci_job)
    include_default_params(ci_job, distro, arch)
    ci_job.with
    {
      scm {
	hg("https://bitbucket.org/osrf/release-tools") {
	  branch('default')
	  subdirectory("release-tools")
	}
      }

      triggers {
	scm('*/5 * * * *')
      }
    }

    // 2. ci-pr-any-job
    def rtools_any_job = job("release-tools-ci-pr_any-${distro}-${arch}")
    OSRFLinuxCompilationAny.create(rtools_any_job,
                                    "https://bitbucket.org/osrf/release-tools", false, false)
    include_default_params(rtools_any_job, distro, arch)
  }
}
