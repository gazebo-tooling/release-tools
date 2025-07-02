package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> GenericMail

  Implements:
     - description
     - keep only 75 builds
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

        logRotator {
          numToKeep(75)
        }

        parameters {
          stringParam('RTOOLS_BRANCH','master','release-tools branch to use')
          if (Globals.gazebodistro_branch)
          {
            stringParam('GAZEBODISTRO_BRANCH','master','gazebodistro branch to use')
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

        // Create the naginator retry tags
        HelperRetryFailures.create(job, [
          regexpForRerun: "java.nio.channels.ClosedChannelException",
          checkRegexp: true,
          maxSchedule: 1
        ])

        HelperRetryFailures.create(job, [
          regexpForRerun: "Error: Another `brew update` process is already running.",
          checkRegexp: true,
          maxSchedule: 1
        ])
      }
    }
}
