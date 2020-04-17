package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFCIWorkFlow

  Implements:
     - parameters
     - definition (pipeline plugin)
*/

class OSRFCIWorkFlowMultiAnyGitHub
{
  static void create(Job job, String build_any_job_name)
  {
    OSRFCIWorkFlowMultiAny.create(job, [build_any_job_name])
  }

  static String build_stage_name(String ci_job_name)
  {
    try
    {
      return ci_job_name.split("-ci-pr_any-")[1]
    }
    catch (java.lang.ArrayIndexOutOfBoundsException e)
    {
      return ci_job_name
    }
  }

  static void create(Job job, ArrayList any_job_name_list)
  {
    OSRFCIWorkFlow.create(job)

    String build_jobs_with_status = "";

    any_job_name_list.eachWithIndex { build_any_job_name, index ->
      // Handle job parallelization
      String stage_name = build_stage_name(build_any_job_name)

      String parallel_init = ""
      if (index == 0) {
        parallel_init = "def jobs = [ \n"
      }
      parallel_init = build_jobs_with_status + parallel_init + "'${stage_name}' : {"

      String parallel_end = "}"
      if (index == any_job_name_list.size() - 1) {
        String end_of_map_and_call = """\
                                     ]

                                    stage "Executing testing jobs"
                                    parallel jobs
                                    """
        parallel_end = parallel_end + end_of_map_and_call
      } else {
        // not the last element in map
        parallel_end = parallel_end + ",\n"
      }

      build_jobs_with_status = parallel_init +
      parallel_end
    }

    job.with
    {
      // TODO: share parameters with ci-py_any- jobs
      parameters {
        stringParam('RTOOLS_BRANCH','master','release-tools branch to send to jobs')
        stringParam('SRC_REPO','','URL pointing to repository')
        stringParam('SRC_BRANCH','default','Branch of SRC_REPO to test')
        stringParam('JOB_DESCRIPTION','','Description of the job in course. For information proposes.')
        stringParam('DEST_BRANCH','default','Branch to merge in')
      }

      definition
      {
        cps
        {
          // Disable sandbox until the bug/issue is resolved
          // https://issues.jenkins-ci.org/browse/JENKINS-28178
          sandbox(false)
          script(
           (OSRFCIWorkFlow.script_code_init_hook() +
           build_jobs_with_status +
           OSRFCIWorkFlow.script_code_end_hook()).stripIndent()
          )
        } // end of cps
      } // end of definition
    } // end of job
  } // end of create
}
