import jenkins.*;
import jenkins.model.*;
import hudson.model.JobPropertyDescriptor;
import hudson.model.Job;

def jenkinsJobs = hudson.model.Hudson.instance
def build = Thread.currentThread().executable
jenkinsJobs.getItems(hudson.model.Project).each {project ->
  if (project.displayName == 'pre-seed') {
    if (project.lastBuild.result != hudson.model.Result.SUCCESS) {
      println('JOB: pre-seed - BUILD STATUS: ' + project.lastBuild.result)
      build.@result = hudson.model.Result.UNSTABLE
	  return
      } else {
        println('JOB: pre-seed - BUILD STATUS: ' + project.lastBuild.result)
      }
    } else if (project.displayName == 'pre-unittest') {
    if (project.lastBuild.result != hudson.model.Result.SUCCESS) {
      println('JOB: pre-unittest - BUILD STATUS: ' + project.lastBuild.result)
      build.@result = hudson.model.Result.UNSTABLE
	  return
    } else {
        println('JOB: pre-unittest - BUILD STATUS: ' + project.lastBuild.result)
      }
    }
}print("JobPropertyDescriptor.all(): ")
println(JobPropertyDescriptor.all())

