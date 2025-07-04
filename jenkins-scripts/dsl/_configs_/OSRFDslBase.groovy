package _configs_

import javaposse.jobdsl.dsl.Job

class OSRFDslBase extends OSRFBase {
  static void create(Job job, String target_dsl_scripts) {
    OSRFBase.create(job)
    OSRFGitHub.create(job,
                     'gazebo-tooling/release-tools',
                     '${RTOOLS_BRANCH}',
                     'scripts')

    job.with
    {
      label(Globals.nontest_label("built-in"))

      steps {
        dsl {
          text(target_dsl_scripts)
          removeAction('DISABLE')
          removeViewAction('DELETE')
        }
      }
    }
  }
}
