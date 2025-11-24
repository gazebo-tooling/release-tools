package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFWinBase
  -> GenericCompilation

  Implements:
    - compiler warning
*/
class OSRFWinCompilation extends OSRFWinBase
{
  static void create(Job job, enable_testing = true, enable_cmake_warnings = false)
  {
    OSRFWinBase.create(job)

    /* Properties from generic compilations */
    GenericCompilation.create(job, enable_testing)

    job.with
    {
      parameters {
	  stringParam('BUILD_TYPE', 'Release','Release|Debug compilation type for MSVC')
      }

      publishers
      {
        configure { project ->
          project / publishers / 'io.jenkins.plugins.analysis.core.steps.IssuesRecorder' {
            analysisTools {
              'io.jenkins.plugins.analysis.warnings.MsBuild' {
                id()
                name()
                pattern()
                reportEncoding()
                skipSymbolicLinks(false)
              }
              if (enable_cmake_warnings) {
                'io.jenkins.plugins.analysis.warnings.Cmake' {
                    id()
                    name()
                    pattern()
                    reportEncoding()
                    skipSymbolicLinks(false)
                }
              }
            }

            sourceCodeEncoding()
            ignoreQualityGate(false)
            ignoreFailedBuilds(true)
            referenceJobName()
            healthy(0)
            unhealthy(0)
            minimumSeverity {
              name('LOW')
            }
            filters { }
            isEnabledForFailure(false)
            isAggregatingResults(false)
            isBlameDisabled(false)

            qualityGates {
              'io.jenkins.plugins.analysis.core.util.WarningsQualityGate' {
                threshold(1.0)
                criticality(UNSTABLE)
                type('TOTAL')
              }
            }
          }
        }
      }
    } // end of job
  } // end of method createJob
} // end of class
