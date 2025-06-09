package _configs_

import javaposse.jobdsl.dsl.Job

class OSRFDslBase extends OSRFUnixBase {
  static void create(Job job, String target_dsl_scripts)
  {
    OSRFUNixBase.create(job)

    job.with
    {
      label(Globals.nontest_label("built-in"))

      steps {
        jobDsl {
          targets(target_dsl_scripts)
          removedJobAction('DISABLE')
          removedViewAction('DELETE')
          removedConfigFilesAction('IGNORE')
        }
      }
    }
  }
}
