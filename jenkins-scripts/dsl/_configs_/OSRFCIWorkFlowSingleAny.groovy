package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFCIWorkFlow

  Implements:
     - parameters
     - definition (pipeline plugin)
*/

class OSRFCIWorkFlowSingleAny
{
   static String script_code_build_any(String job_name)
   {
      return """\
         stage 'compiling + QA'
          def result_URL = env.JENKINS_URL + '/job/${job_name}/'
          def bitbucket_publish_job_result[$job_name : 'ok']
          def jenkins_pipeline_job_result[$job_name : 'SUCCESS']

          compilation_job = null

          node("lightweight-linux")
          {
            compilation_job = build job: '${job_name}',
                propagate: false, wait: true,
                parameters:
                 [[\$class: 'StringParameterValue',  name: 'RTOOLS_BRANCH',   value: "\$RTOOLS_BRANCH"],
                  [\$class: 'BooleanParameterValue', name: 'NO_MAILS',        value: false],
                  [\$class: 'StringParameterValue',  name: 'SRC_REPO',        value: "\$SRC_REPO"],
                  [\$class: 'StringParameterValue',  name: 'SRC_BRANCH',      value: "\$SRC_BRANCH"],
                  [\$class: 'StringParameterValue',  name: 'JOB_DESCRIPTION', value: "\$JOB_DESCRIPTION"],
                  [\$class: 'StringParameterValue',  name: 'DEST_BRANCH',     value: "\$DEST_BRANCH"]]
          }

          result_URL = result_URL + compilation_job.getNumber()

          if (compilation_job.getResult() != 'SUCCESS')
          {
            // any non success is a failure in bitbucket status
            bitbucket_publish_job_result[$job_name : 'failed']
            jenkins_pipeline_job_result[$job_name : compilation_job.getResult()]
          }
      """
   }

   static void create(Job job, String build_any_job_name)
   {
      OSRFCIWorkFlowSingleAny.create(job, [build_any_job_name])
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

          String parallel_init = "parallel ${stage_name} : {"
          if (index > 0)
          {
            parallel_init = build_jobs_with_status + "${stage_name} : {"
          }

          String parallel_end = "}"
          if (index != any_job_name_list.size() - 1)
          {
            parallel_end = parallel_end + ","
          }
          parallel_end = parallel_end + "\n"

          build_jobs_with_status = parallel_init +
          OSRFCIWorkFlow.script_code_set_code(build_status : '"inprogress"',
                                               build_desc   : '"Testing in progress"',
                                               build_name   : "'${build_any_job_name}'") + // different order of quotes!
          OSRFCIWorkFlowSingleAny.script_code_build_any(build_any_job_name) +
          OSRFCIWorkFlow.script_code_set_code(build_status : '"$bitbucket_publish_final_result "',
                                               build_desc  : '"Testing is finished"',
                                               build_name  : "'${build_any_job_name}'", // different order of quotes!
                                               build_url   : '"$result_URL"') +
          parallel_end
      }

      job.with
      {
        // TODO: share parameters with ci-py_any- jobs
        parameters {
          stringParam('RTOOLS_BRANCH','default','release-tools branch to send to jobs')
          stringParam('SRC_REPO','','URL pointing to repository')
          stringParam('SRC_BRANCH','default','Branch of SRC_REPO to test')
          stringParam('JOB_DESCRIPTION','','Description of the job in course. For information proposes.')
          stringParam('DEST_BRANCH','default','Branch to merge in')
        }

        definition
        {
          cps
          {
            // run script in sandbox groovy
            sandbox()
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
