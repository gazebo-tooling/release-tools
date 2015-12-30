package _configs_

import javaposse.jobdsl.dsl.Job

/*
   Implements:
     - parameters: SRC_REPO, SRC_BRANCH, JOB_DESCRIPTION
     - job.Description
     - scm check with SRC_REPO + SRC_BRANCH
*/


class GenericAnyJob
{
   static void create(Job job, String repo)
   {
     // setup special mail subject
     GenericMail.update_field(job, 'defaultSubject',
                    '$PROJECT_NAME - Branch: $SRC_BRANCH (#$BUILD_NUMBER) - $BUILD_STATUS!')
     GenericMail.update_field(job, 'defaultContent', 
                    '$JOB_DESCRIPTION \n' + GenericCompilation.get_compilation_mail_content())

     String subdirectoy = repo.tokenize('/').last()

     job.with
     {
        parameters { 
	  stringParam('SRC_REPO', repo,'URL pointing to repository')
	  stringParam('SRC_BRANCH','default','Branch of SRC_REPO to test')
	  stringParam('JOB_DESCRIPTION','','Description of the job in course. For information proposes.')
	}

        steps
        {
           systemGroovyCommand("""\
                job_description = build.buildVariableResolver.resolve('JOB_DESCRIPTION')

                if (job_description == "")
                {
                  job_description = 'branch: <b>' + build.buildVariableResolver.resolve('SRC_BRANCH') + '</b><br />' +
                                    'repo: ' + build.buildVariableResolver.resolve('SRC_REPO') + '<br />' +
                                    'RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH')
                }

                build.setDescription(job_description)
                """.stripIndent()
           )
        }

        scm {
          hg('${SRC_REPO}') {
            branch('${SRC_BRANCH}')
            subdirectory(subdirectoy)
          }
        }
      }
   }
}
