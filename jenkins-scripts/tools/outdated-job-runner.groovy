/*
 * This script is used to run a job that checks if there are any
 * outdated jobs in the buildfarm. It also runs the jobs
 * if there is any node available.
 */ 

import hudson.model.*;
import jenkins.model.Jenkins;
import java.util.Date;

/*
 * Schedule a job to run immediately
 * Will return true or false depending on if the job was scheduled
 */
def runJob(String jobName) {
    def job = Jenkins.instance.getItemByFullName(jobName)
    return job.scheduleBuild(0)
}

/*
 * Find available nodes to run jobs per OS using labels
 * win: windows
 * osx: macos
 * linux: docker
 */
def findAvailableNodes() {
    def availableNodes = [osx: [], win: [], docker: []]

    Jenkins.instance.nodes.each { node ->
        if (node.computer.online && node.computer.countIdle() > 0) {
            if (node.getLabelString().contains("win")) {
                availableNodes.win << node.name
            } else if (node.getLabelString().contains("osx")) {
                availableNodes.osx << node.name
            } else if (node.getLabelString().contains("docker")) {
                availableNodes.docker << node.name
            }
        }
    }

    return availableNodes
}

/*
 * For each node in each OS, run a job if there are jobs to run
 */
def runJobsInAvailableNodes(LinkedHashMap outdatedJobs) {
    def availableNodes = findAvailableNodes()

    availableNodes.each { os -> 
        os.value.each { node ->
                if (outdatedJobs[os.key].size() > 0) {
                    try {
                        def jobName = outdatedJobs[os.key].remove(0)
                        println("Running job ${jobName}")
                        if (!runJob(jobName)) {
                            println("Error scheduling job ${jobName}. Project disabled?")
                            println("MARK_AS_UNSTABLE")
                        }
                    } catch (NullPointerException e) {
                        println("Error running job ${jobName}. Project exist?")
                        throw e
                    }
                }
            }
        }
}

def JOBS_URL = 'https://raw.githubusercontent.com/gazebo-tooling/release-tools/master/jenkins-scripts/dsl/logs/generated_jobs.txt'

def trackedJobs = new LinkedHashSet();

def get = new URL(JOBS_URL).openConnection();
def getRC = get.getResponseCode();
println(getRC);
if(getRC.equals(200)) {
    get.getInputStream().getText().split('\n').each {
  	    line -> trackedJobs << (line  =~ /.+? .+? (.+)/)[0][1];
    };
} else {
    throw new Exception("Error getting jobs list from ${JOBS_URL}");
}

def jenkinsJobs = Hudson.instance

def jobsToRun = [osx: [], win: [], docker: []]

long eightDaysAgoMillis = System.currentTimeMillis() - 4 * 24 * 60 * 60 * 1000; // 4 days ago in milis
Date eightDaysAgoDate = new Date(eightDaysAgoMillis);

jenkinsJobs.getItems(Project).each { project ->
    // Filter jobs that have not been updated in the last 8 days
    if (!project.disabled && trackedJobs.contains(project.displayName) && project.lastBuild.getTime().before(eightDaysAgoDate)) {
        if (project.displayName.contains('homebrew')) {
            jobsToRun.osx << project.displayName
        } else if (project.displayName.contains('win')) {
            jobsToRun.win << project.displayName
        } else {
            jobsToRun.docker << project.displayName
        }
    }
}

println('\n--- Jobs to run: ---')
println('OSX: ' + jobsToRun.osx)
println('WIN: ' + jobsToRun.win)
println('docker: ' + jobsToRun.docker)

println('\n--- Running jobs: ---')
runJobsInAvailableNodes(jobsToRun)

