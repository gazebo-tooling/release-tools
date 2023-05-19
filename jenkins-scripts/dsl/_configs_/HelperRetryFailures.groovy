package _configs_

import javaposse.jobdsl.dsl.Job

class HelperRetryFailures {

  /**@
    * Creates a Naginator Publisher retry plugin for a given job.
    * 
    * @param job - The job to add the plugin to.
    * @param args - A map of arguments to pass to the plugin:
    *     - args.regexpForRerun: Regular expression to match against the console output (new value is appended to the old value if it exists)
    *     - args.checkRegexp: Whether to check the regular expression (true/false)
    *     - args.maxSchedule: Maximum number of times to retry the build
    *     - args.delay: Delay in seconds to wait before retrying the build (highest value between old and new is used)
    * @throws Exception if there checkRegexp or maxSchedule are already set and different
    */
  static void create(Job job, Map args) {
    job.with {
      publishers {
        configure { project ->
          def naginator_root = project / publishers / 'com.chikli.hudson.plugin.naginator.NaginatorPublisher'

          def old_regexpForRerun = (naginator_root / regexpForRerun).text()
          def old_checkRegexp = (naginator_root / checkRegexp).text()
          def old_maxSchedule = (naginator_root / maxSchedule).text()
          def old_delay = (naginator_root / delay(class: 'com.chikli.hudson.plugin.naginator.FixedDelay') / delay).text()

          // Convert the values to the correct type
          old_checkRegexp = old_checkRegexp ? old_checkRegexp.toBoolean() : null
          old_maxSchedule = old_maxSchedule ? old_maxSchedule.toInteger() : 0
          old_delay = old_delay ? old_delay.toInteger() : 0


          if (args.regexpForRerun) {
            // If the old value exists, append the new value to it
            def updated_regexpForRerun = old_regexpForRerun ? old_regexpForRerun + '|' + args.regexpForRerun : args.regexpForRerun
            (naginator_root / regexpForRerun).value = updated_regexpForRerun
          }

          if (args.checkRegexp) {
            // Fail if old and new checkRegexp are both set and different
            if (old_checkRegexp && old_checkRegexp != args.checkRegexp) {
              throw new Exception("checkRegexp is already set to ${old_checkRegexp} and cannot be changed to ${args.checkRegexp}")
            }
            (naginator_root / checkRegexp).value = args.checkRegexp
          }

          if (args.maxSchedule) {
            // Fail if old and new maxSchedule are both set and different
            if (old_maxSchedule && old_maxSchedule != args.maxSchedule) {
              throw new Exception("maxSchedule is already set to ${old_maxSchedule} and cannot be changed to ${args.maxSchedule}")
            }
            (naginator_root / maxSchedule).value = args.maxSchedule
          }

          if (args.delay) {
            // Use the highest delay
            def updated_delay = old_delay > args.delay ? old_delay : args.delay
            (naginator_root / delay(class: 'com.chikli.hudson.plugin.naginator.FixedDelay') / delay).value = updated_delay
          }
        }
      }
    }
  }
}
