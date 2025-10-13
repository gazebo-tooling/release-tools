package _configs_

import javaposse.jobdsl.dsl.Job
import _configs_.Globals

/*
  -> OSRFOsXBase

  Implements:
    - run on osx
*/
class OSRFOsXBase
{
  static void create(Job job, String arch)
  {
    // UNIX Base
    OSRFUNIXBase.create(job)

    job.with
    {
      label Globals.nontest_label("osx && ${arch}")

      parameters {
        booleanParam('CLEAR_BREW_CACHE',false,'remove cached brew downloads')
      }
    }
  }
}
