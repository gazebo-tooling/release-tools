package _configs_

import javaposse.jobdsl.dsl.Job

/*
   Implements:
     - parameters: SRC_REPO, SRC_BRANCH, JOB_DESCRIPTION
     - job.Description
     - scm check
*/


class GenericAnyJob
{
   static void create(Job job, String repo)
   {
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
           systemGroovyCommand("")
        }

        scm {
          hg('${SRC_REPO}') {
            branch('${SRC_BRANCH}')
            subdirectory(subdirectoy)
          }
        }

        triggers {
          scm('*/5 * * * *')
        }
      }
   }
}
