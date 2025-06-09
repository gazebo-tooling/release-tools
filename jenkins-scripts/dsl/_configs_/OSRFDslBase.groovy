package _configs_

import javaposse.jobdsl.dsl.Job

class OSRFDslBase extends OSRFUNIXBase
{
  static void create(Job job, String target_dsl_scripts)
  {
    OSRFUNIXBase.create(job)

    job.with
    {
      label("built-in")

      steps
      {
        dsl {
          external(target_dsl_scripts)
          removeAction('DISABLE')
          removeViewAction('DELETE')
        }
      }
    }
  }
}
