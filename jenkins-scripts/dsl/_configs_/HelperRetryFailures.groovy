package _configs_

import javaposse.jobdsl.dsl.Job

class HelperRetryFailures {

  /**@
    * Create method for Nagiator Publisher retry plugin
    * @param job - Job to add the plugin to
    * @param args - Map of arguments to pass to the plugin
    * @param args.regexpForRerun - Regular expression to match against the console output
    * @param args.checkRegexp - Whether to check the regular expression
    * @param args.maxSchedule - Maximum number of times to retry the build
    * @param args.delay - Delay in seconds to wait before retrying the build
    */
  static void create(Job job, Map args) {
    job.with {
      publishers {
        configure { project ->
          def old_regexpForRerun = (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / regexpForRerun).text()
          def old_checkRegexp = (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / checkRegexp).text()
          def old_maxSchedule = (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / maxSchedule).text()
          def old_delay = (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / delay(class: 'com.chikli.hudson.plugin.naginator.FixedDelay') / delay).text()

          // Convert the values to the correct type
          old_checkRegexp = old_checkRegexp ? old_checkRegexp.toBoolean() : null
          old_maxSchedule = old_maxSchedule ? old_maxSchedule.toInteger() : 0
          old_delay = old_delay ? old_delay.toInteger() : 0


          if (args.regexpForRerun) {
            // If the old value exists, append the new value to it
            def updated_regexpForRerun = old_regexpForRerun ? old_regexpForRerun + '|' + args.regexpForRerun : args.regexpForRerun
            (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / regexpForRerun).value = updated_regexpForRerun
          }

          if (args.checkRegexp) {
            // Get the previous value if it exists (that means it only has to be set once)
            def updated_checkRegexp = old_checkRegexp ? old_checkRegexp : args.checkRegexp
            (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / checkRegexp).value = updated_checkRegexp
          }

          if (args.maxSchedule) {
            // Use the highest maxSchedule
            def updated_maxSchedule = old_maxSchedule > args.maxSchedule ? old_maxSchedule : args.maxSchedule
            (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / maxSchedule).value = updated_maxSchedule
          }

          if (args.delay) {
            // Use the highest delay
            def updated_delay = old_delay > args.delay ? old_delay : args.delay
            (project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher' / delay(class: 'com.chikli.hudson.plugin.naginator.FixedDelay') / delay).value = updated_delay
          }
        }
      }
    }
  }
}
