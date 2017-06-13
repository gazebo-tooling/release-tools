package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFBase

  Implements:
     - bash: RTOOLS checkout
*/
class OSRFUNIXBase extends OSRFBase
{
  static void create(Job job)
  {
    OSRFBase.create(job)

    job.with
    {
      steps
      {
        shell("""\
             #!/bin/bash -xe

             [[ -d ./scripts ]] &&  rm -fr ./scripts
             mkdir ./scripts
             curl https://bitbucket.org/osrf/release-tools/get/\${RTOOLS_BRANCH}.tar.gz > scripts.tar.gz
             tar xf scripts.tar.gz --strip 1 -C ./scripts/
             """.stripIndent())
      }
    }
  }
}
