package _configs_

import javaposse.jobdsl.dsl.Job
import _configs_.Globals

/*
  -> OSRFUNIXBase
  -> GenericCompilation
  -> OSRFClangWarnings

  Implements:
    - run on the macOS arm64 agents, building with pixi (no brew setup)
*/
class OSRFOsXCondaCompilation extends OSRFUNIXBase
{
  static void create(Job job, enable_testing = true, enable_cmake_warnings = false)
  {
    OSRFUNIXBase.create(job)

    /* Properties from generic compilations */
    GenericCompilation.create(job, enable_testing)

    job.with
    {
      label Globals.nontest_label("osx && arm64")

      parameters {
        stringParam('BUILD_TYPE', 'RelWithDebInfo', 'RelWithDebInfo|Release|Debug CMake build type')
      }
    }

    OSRFClangWarnings.create(job, enable_cmake_warnings)
  } // end of method create
} // end of class
