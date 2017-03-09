import _configs_.*
import javaposse.jobdsl.dsl.Job

def build_pkg_job = job("generic_backport-debbuilder")
OSRFLinuxBackportPkg.create(build_pkg_job)

build_pkg_job.with
{
    steps {
      shell("""\
            #!/bin/bash -xe

            for ARCH in \${ARCHES}; do
              /bin/bash -x ./scripts/jenkins-scripts/docker/generic_backport-debbuild.bash
            done
            """.stripIndent())
    }
}
