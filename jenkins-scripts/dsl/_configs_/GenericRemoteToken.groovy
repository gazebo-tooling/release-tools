package _configs_

import javaposse.jobdsl.dsl.Job

class GenericRemoteToken
{
  // FIXME getEnvVars can not be called in a static scope. Hardcoded by now.
  // static File token_file = new File(build.getEnvVars()['HOME'] + '/remote_token')
  static File token_file = new File('/var/lib/jenkins/remote_token')

  static void create(Job job)
  {
    if (! token_file.exists()) {
      println("!!! token file was not found for setting the remote password")
      println("check your filesystem in the jenkins node for: ")
      println(token_file)
      // We can not use exit here, DSL job hangs
      buildUnstable()
    }

    job.with
    {
      // remote calls don't have DSL implementation
      configure { project ->
        project / authToken(token_file.text.replaceAll("[\n\r]", ""))
      }
    }
  }
}
