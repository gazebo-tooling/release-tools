import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'trusty' ]
def supported_arches = [ 'amd64' ]

// TODO: remove after development
Globals.default_emails = ""

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

// --------------------------------------------------------------
// 2. Offline tester

def unbundler_job = job("handsim-install-offline_bundler-trusty-amd64")

// Use the linux install as base
OSRFLinuxInstall.create(unbundler_job)
unbundler_job.with
{
    steps {
      shell("""#!/bin/bash -xe

            /bin/bash -x ./scripts/jenkins-scripts/handsim-install_offline_bundle-test-job.bash
            """.stripIndent())
   }
}
