import _configs_.*
import javaposse.jobdsl.dsl.Job

def supported_distros = [ 'trusty' ]
def supported_arches = [ 'amd64' ]

// TODO: remove after development
Globals.default_emails = ""

// --------------------------------------------------------------
// 1. Create the bundler job
def bundler_job = job("handsim-offline_bundler-builder")
OSRFBase.create(bundler_job)

bundler_job.with
{
   // Script made to run in the same machine that package repo
   label "master"

   steps {
    shell("""\
          #!/bin/bash -xe

          /bin/bash -x ./scripts/jenkins-scripts/handsim-bundler.bash
          """.stripIndent())
   }
}
