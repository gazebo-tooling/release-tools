package _configs_

import javaposse.jobdsl.dsl.Job

/*
  Implements:
    - Clang compiler warnings, and CMake warnings when enabled, in the
      warnings-ng IssuesRecorder: any warning makes the build UNSTABLE
*/
class OSRFClangWarnings
{
  static void create(Job job, boolean enable_cmake_warnings = false)
  {
    job.with
    {
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
  } // end of method create
} // end of class
