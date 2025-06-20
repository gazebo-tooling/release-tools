package _configs_

import javaposse.jobdsl.dsl.Job

class OSRFDslBase extends OSRFBase {
  static void create(Job job, String target_dsl_scripts) {
    OSRFBase.create(job)

    job.with {
      label(Globals.nontest_label("built-in"))

      scm {
        git {
          remote {
            github("gazebo-tooling/release-tools")
          }
          branch('${RTOOLS_BRANCH}')
          extensions {
            cloneOption {
              shallow(true)
              noTags(false)
              reference('')
              timeout(2)
            }
            relativeTargetDirectory('scripts')
          }
        }
      }

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
