package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFLinuxBase

  Implements:
    - priority 400
    - logrotator
    - performance plugin
*/
 
class OSRFLinuxPerformance
{
  static void create(Job job)
  {
    OSRFLinuxBase.create(job)

    job.with
    {
      properties {
        priority 400
      }

      logRotator {
        artifactNumToKeep(10)
      }

      publishers
      {
        configure { project ->
          project / publishers / 'hudson.plugins.performance.PerformancePublisher' {
            errorFailedThreshold 0
            errorUnstableThreshold 0
            errorUnstableResponseTimeThreshold ''
            relativeFailedThresholdPositive '20.0'
            relativeFailedThresholdNegative '-100.0'
            relativeUnstableThresholdPositive '10.0'
            relativeUnstableThresholdNegative '-50.0'
            nthBuildNumber 0
            modeRelativeThresholds false
            configType ART
            modeOfThreshold true
            compareBuildPrevious true
            modePerformancePerTestCase true              
            
            parsers {
              'hudson.plugins.performance.JUnitParser' {
                glob 'build/test_results/PERFORMANCE_*.xml'
              }
            } // end of parsers
          } // end of project
        } // end of congfigure
      } // end of publishers
    } // end of with
  } // end of create
}
