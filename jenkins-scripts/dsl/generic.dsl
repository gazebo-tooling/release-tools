import _configs_.*
import javaposse.jobdsl.dsl.Job

def build_pkg_job = job("generic_backport-debbuilder")
OSRFLinuxBuildPkg.create(build_pkg_job)

build_pkg_job.with
{
    steps {
      shell("""\
            #!/bin/bash -xe

            for ARCH in \${ARCHES}; do
              /bin/bash -x ./scripts/jenkins-scripts/docker/lib/debbuild-backport.bash
            done
            """.stripIndent())
    }
}
