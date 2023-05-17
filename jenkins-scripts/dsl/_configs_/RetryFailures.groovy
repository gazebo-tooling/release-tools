package _configs_

import javaposse.jobdsl.dsl.Job

class RetryFailures {
   static void update_retry_parameters(Job job, String new_retry_regex) {

    job.with {
      publishers {
        configure { project ->
          def current_retry_regex = (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / regexpForRerun).text()
          (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / regexpForRerun).value = current_retry_regex + "|" + new_retry_regex
        }
      }
    }
    }
}
