package _configs_

import javaposse.jobdsl.dsl.Job

class OSRFBitbucketHg
{
  static void create(Job job, String repo,
                              String rev    = 'default',
                              String subdir = 'NOT-DEFINED-USE-DEFAULT')
  {
    String software_name = repo.tokenize('/').last()

    if (subdir == 'NOT-DEFINED-USE-DEFAULT')
      subdir = software_name

    // browser needs it for correct display
    if (repo[-1] != '/')
      repo = repo + "/"

    job.with
    {
      scm {
        hg(repo) {
          branch(rev)
          subdirectory(subdir)
        configure { project ->
           project / browser(class: 'hudson.plugins.mercurial.browser.BitBucket') / "url" << repo
          }
        }
      }
    }
  }
}
