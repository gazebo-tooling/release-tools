package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
  - run on win
  - checkout release-tools on windows
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
                git clone https://github.com/scpeters-test/release-tools_auto_import scripts -b %RTOOLS_BRANCH%
                """.stripIndent())
        }
     }
   }
}
