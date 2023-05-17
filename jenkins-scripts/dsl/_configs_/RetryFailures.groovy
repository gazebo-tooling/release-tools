package _configs_

import javaposse.jobdsl.dsl.Job

class RetryFailures {
   static void update_retry_parameters(Job job, String new_regexpForRerun, int new_delay = 0, boolean new_checkRegexp = true, int new_maxSchedule = 1) {

    job.with {
      publishers {
        configure { project ->
          def new_retry_regex = (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / regexpForRerun).text() + "|" + new_regexpForRerun
  
          (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / regexpForRerun).value = new_retry_regex
          (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / checkRegexp).value = new_checkRegexp
          (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / maxSchedule).value = new_maxSchedule
          
          if (new_delay) {
            (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / delay(class: "com.chikli.hudson.plugin.naginator.FixedDelay") / delay).value = new_delay
          }
        }
      }
    }
    }
}
