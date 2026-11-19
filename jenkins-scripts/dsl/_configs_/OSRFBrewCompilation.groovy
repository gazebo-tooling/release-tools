package _configs_

import javaposse.jobdsl.dsl.Job

/*
  -> OSRFOsXBase
  -> GenericCompilation

  Implements:
    - compiler warnings (OSRFClangWarnings)
*/
class OSRFBrewCompilation extends OSRFOsXBase
{
  static void create(Job job, String arch, enable_testing = true, enable_cmake_warnings = false)
  {
    OSRFOsXBase.create(job, arch)

    /* Properties from generic compilations */
    GenericCompilation.create(job, enable_testing)

    OSRFClangWarnings.create(job, enable_cmake_warnings)
  } // end of method createJob
} // end of class
