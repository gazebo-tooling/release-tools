package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxCompilation
  -> GenericAnyJob

  Implements:
   - DEST_BRANCH parameter
*/
class OSRFLinuxCompilationAny
{
  static void create(Job job, String repo)
  {
    OSRFLinuxCompilation.create(job)

    /* Properties from generic any */
    GenericAnyJob.create(job, repo)

    job.with
    {
      steps
      {
        shell("""\
        #!/bin/bash -xe

        export CONFIG_FILE_PATH="\$WORKSPACE/config_pybitbucket.yml"
        export BITBUCKET_USER_PASS_FILE="/var/lib/jenkins/remote_token"
        export REPO_SHORT_NAME=`echo \${SRC_REPO} | sed s:.*.org/::`

        echo "Generating config file ..."
        cat > \$CONFIG_FILE_PATH << DELIM_CONFIG
        bitbucket_origin:
          repository_name: \${REPO_SHORT_NAME}
          sha: \${MERCURIAL_REVISION_SHORT}
        jenkins_job:
          name: \${JOB_NAME}
          url: \${BUILD_URL}
        DELIM_CONFIG
        cat config.yml

        echo "Running the set_status_from_file in hidden mode"
        set +x # keep password secret
        BITBUCKET_USER_PASS=`cat \$BITBUCKET_USER_PASS_FILE`
        \${WORKSPACE}/scripts/jenkins-scripts/python-bitbucket/set_status_from_file.py \
             --user osrf_jenkins \
             --pass \${BITBUCKET_USER_PASS} \
             --status inprogress \
             --load_from_file \${CONFIG_FILE_PATH} >& pybitbucket.log
        set -x # back to debug
        cat pybitbucket.log
        """.stripIndent())
      }

      parameters
      {
        stringParam('DEST_BRANCH','default',
                    'Destination branch where the pull request will be merged.' +
                    'Mostly used to decide if calling to ABI checker')
      }
    }
  }
}
