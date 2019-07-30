package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> GenericMail

  Implements:
     - description
     - RTOOLS parame + groovy to set jobDescription
     - base mail for Failures and Unstables
*/

class OSRFBase
{
   static void create(Job job)
   {
     GenericMail.include_mail(job)

     job.with
     {
     	description 'Automatic generated job by DSL jenkins. Please do not edit manually'

        parameters {
          stringParam('RTOOLS_BRANCH','default','release-tool branch to use')
          if (Globals.gazebodistro_branch)
          {
            stringParam('GAZEBODISTRO_BRANCH','default','gazebodistro branch to use')
          }
          booleanParam('NO_MAILS',false,'do not send any notification by mail')
        }

        if (Globals.rtools_description)
        {
          steps
          {
             systemGroovyCommand("build.setDescription('RTOOLS_BRANCH: ' + build.buildVariableResolver.resolve('RTOOLS_BRANCH'));")
          }
        }
      }
    }
}
