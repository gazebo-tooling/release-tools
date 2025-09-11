package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFOsXBase
  -> GenericCompilation

  Implements:
    - compiler warning
*/
class OSRFBrewCompilation extends OSRFOsXBase
{
  static void create(Job job, String arch, enable_testing = true, enable_cmake_warnings = false)
  {
    OSRFOsXBase.create(job, arch)

    /* Properties from generic compilations */
    GenericCompilation.create(job, enable_testing)

    job.with
    {
      parameters
      {
        stringParam("PULL_REQUEST_URL", '',
                    'Pull request URL (osrf/homebrew-simulation) pointing to a pull request.')
      }

      publishers
      {
        configure { project ->
          project / publishers / 'io.jenkins.plugins.analysis.core.steps.IssuesRecorder' {
            analysisTools {
              'io.jenkins.plugins.analysis.warnings.Clang' {
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
              'io.jenkins.plugins.analysis.core.util.QualityGate' {
                threshold(1)
                type('TOTAL')
                status('WARNING')
              }
            }
          }
        }
      }
    } // end of job
  } // end of method createJob
} // end of class
