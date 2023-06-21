import java.util.Date;

long eightDaysAgoMillis = System.currentTimeMillis() - 8 * 24 * 60 * 60 * 1000; // 8 days ago in milis
Date eightDaysAgoDate = new Date(eightDaysAgoMillis);

// Filter jobs that have not been updated in the last 8 days
def jenkinsJobs = hudson.model.Hudson.instance
jenkinsJobs.getItems(hudson.model.Project).each {project ->
  if (project.lastBuild != null) {
    if (project.lastBuild.getTime().before(eightDaysAgoDate)) {
      println('JOB: ' + project.displayName + ' - Last time: ' + project.lastBuild.getTime());
      println('JOB HAS NOT BEEN UPDATED RECENTLY');
    }
  }
}