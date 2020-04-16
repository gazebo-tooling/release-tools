package _configs_

import javaposse.jobdsl.dsl.Job

class OSRFGitHub
{
  static void create(Job job, String repo,
                              String rev    = 'master',
                              String subdir = 'NOT-DEFINED-USE-DEFAULT',
                              String installName = 'Default')
  {
    String software_name = repo.tokenize('/').last()

    if (subdir == 'NOT-DEFINED-USE-DEFAULT')
      subdir = software_name

    job.with
    {
      scm {
        github(repo) {
          branch(rev)
          installation(installName)
          subdirectory(subdir)
        }
      }
    }
  }
}
