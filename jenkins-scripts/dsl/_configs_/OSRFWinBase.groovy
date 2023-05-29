package _configs_

import javaposse.jobdsl.dsl.Job
import _configs_.Globals

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
        label Globals.nontest_label("win")

        steps
        {
          batchFile("""\
                IF exist scripts ( rmdir scripts /s /q )
                git clone https://github.com/gazebo-tooling/release-tools scripts -b %RTOOLS_BRANCH%
                """.stripIndent())
        }

        publishers {
          consoleParsing {
            projectRules('scripts/jenkins-scripts/parser_rules/opengl_problem.parser')
            failBuildOnError(true)
          }
        }
     }
   }
}
