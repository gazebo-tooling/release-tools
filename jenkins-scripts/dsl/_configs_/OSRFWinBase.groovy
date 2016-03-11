package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
  - run on win
  - checkout release-tools on windows
  - Retry once on internal error in MSVC
*/
class OSRFWinBase extends OSRFBase
{
   static void create(Job job)
   {
     OSRFBase.create(job)
     job.with
     {
        label "win"

        steps
        {
          batchFile("""\
                IF exist scripts ( rmdir scripts /s /q )
                hg clone http://bitbucket.org/osrf/release-tools scripts -b %RTOOLS_BRANCH%
                """.stripIndent())
        }

        /* retry plugin exists in DSL but does not support regexpForRerun */
        configure { project ->
           project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' {
             regexpForRerun '/INTERNAL COMPILER ERROR/'
             rerunIfUnstable false
             rerunMatrixPart false
             checkRegexp false
             maxSchedule 2
           }
        }
     }
   }
}
