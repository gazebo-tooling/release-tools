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
 */
def runJob(String jobName) {
    def job = Jenkins.instance.getItemByFullName(jobName)
    job.scheduleBuild(0)
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
                        runJob(jobName)
                    } catch (Exception e) {
                        println("Error running job ${jobName}")
                        println(e)
                        println("MARK_AS_UNSTABLE")
                    }
                }
            }
        }
}

// TODO (Cristobal): This is a hardcoded list of jobs that we're tracking. We should get this list dynamically
// Currently is the result of https://github.com/osrf/buildfarmer/blob/main/common.py#L51 minus debbuilders
def trackedJobsList = [
    'ign_cmake-gz-3-win',
    'ign_cmake-ign-2-win',
    'ign_common-gz-5-win',
    'ign_common-ign-3-win',
    'ign_common-ign-4-win',
    'ign_fuel-tools-ci-win',
    'ign_fuel-tools-gz-8-win',
    'ign_fuel-tools-ign-4-win',
    'ign_fuel-tools-ign-7-win',
    'ign_gazebo-ci-win',
    'ign_gazebo-gz-7-win',
    'ign_gazebo-ign-3-win',
    'ign_gazebo-ign-6-win',
    'ign_gui-ci-win',
    'ign_gui-gz-7-win',
    'ign_gui-ign-3-win',
    'ign_gui-ign-6-win',
    'ign_launch-ci-win',
    'ign_launch-gz-6-win',
    'ign_launch-ign-2-win',
    'ign_launch-ign-5-win',
    'ign_math-gz-7-win',
    'ign_math-ign-6-win',
    'ign_msgs-ci-win',
    'ign_msgs-gz-9-win',
    'ign_msgs-ign-5-win',
    'ign_msgs-ign-8-win',
    'ign_physics-gz-6-win',
    'ign_physics-ign-2-win',
    'ign_physics-ign-5-win',
    'ign_plugin-gz-2-win',
    'ign_plugin-ign-1-win',
    'ign_rendering-ci-win',
    'ign_rendering-gz-7-win',
    'ign_rendering-ign-3-win',
    'ign_rendering-ign-6-win',
    'ign_sensors-ci-win',
    'ign_sensors-gz-7-win',
    'ign_sensors-ign-3-win',
    'ign_sensors-ign-6-win',
    'ign_tools-gz-2-win',
    'ign_tools-ign-1-win',
    'ign_transport-ci-win',
    'ign_transport-gz-12-win',
    'ign_transport-ign-11-win',
    'ign_transport-ign-8-win',
    'ign_utils-gz-2-win',
    'ign_utils-ign-1-win',
    'ignition_cmake-ci-gz-cmake3-focal-amd64',
    'ignition_cmake-ci-gz-cmake3-homebrew-amd64',
    'ignition_cmake-ci-ign-cmake2-focal-amd64',
    'ignition_cmake-ci-ign-cmake2-homebrew-amd64',
    'ignition_common-ci-gz-common5-focal-amd64',
    'ignition_common-ci-gz-common5-homebrew-amd64',
    'ignition_common-ci-ign-common3-focal-amd64',
    'ignition_common-ci-ign-common3-homebrew-amd64',
    'ignition_common-ci-ign-common4-focal-amd64',
    'ignition_common-ci-ign-common4-homebrew-amd64',
    'ignition_fuel-tools-ci-gz-fuel-tools8-focal-amd64',
    'ignition_fuel-tools-ci-gz-fuel-tools8-homebrew-amd64',
    'ignition_fuel-tools-ci-ign-fuel-tools4-focal-amd64',
    'ignition_fuel-tools-ci-ign-fuel-tools4-homebrew-amd64',
    'ignition_fuel-tools-ci-ign-fuel-tools7-focal-amd64',
    'ignition_fuel-tools-ci-ign-fuel-tools7-homebrew-amd64',
    'ignition_fuel-tools-ci-main-focal-amd64',
    'ignition_fuel-tools-ci-main-homebrew-amd64',
    'ignition_gazebo-ci-gz-sim7-focal-amd64',
    'ignition_gazebo-ci-gz-sim7-homebrew-amd64',
    'ignition_gazebo-ci-ign-gazebo3-focal-amd64',
    'ignition_gazebo-ci-ign-gazebo3-homebrew-amd64',
    'ignition_gazebo-ci-ign-gazebo6-focal-amd64',
    'ignition_gazebo-ci-ign-gazebo6-homebrew-amd64',
    'ignition_gazebo-ci-main-focal-amd64',
    'ignition_gazebo-ci-main-homebrew-amd64',
    'ignition_gui-ci-gz-gui7-focal-amd64',
    'ignition_gui-ci-gz-gui7-homebrew-amd64',
    'ignition_gui-ci-ign-gui3-focal-amd64',
    'ignition_gui-ci-ign-gui3-homebrew-amd64',
    'ignition_gui-ci-ign-gui6-focal-amd64',
    'ignition_gui-ci-ign-gui6-homebrew-amd64',
    'ignition_gui-ci-main-focal-amd64',
    'ignition_gui-ci-main-homebrew-amd64',
    'ignition_launch-ci-gz-launch6-focal-amd64',
    'ignition_launch-ci-gz-launch6-homebrew-amd64',
    'ignition_launch-ci-ign-launch2-focal-amd64',
    'ignition_launch-ci-ign-launch2-homebrew-amd64',
    'ignition_launch-ci-ign-launch5-focal-amd64',
    'ignition_launch-ci-ign-launch5-homebrew-amd64',
    'ignition_launch-ci-main-focal-amd64',
    'ignition_launch-ci-main-homebrew-amd64',
    'ignition_math-ci-gz-math7-focal-amd64',
    'ignition_math-ci-gz-math7-homebrew-amd64',
    'ignition_math-ci-ign-math6-focal-amd64',
    'ignition_math-ci-ign-math6-homebrew-amd64',
    'ignition_msgs-ci-gz-msgs9-focal-amd64',
    'ignition_msgs-ci-gz-msgs9-homebrew-amd64',
    'ignition_msgs-ci-ign-msgs5-focal-amd64',
    'ignition_msgs-ci-ign-msgs5-homebrew-amd64',
    'ignition_msgs-ci-ign-msgs8-focal-amd64',
    'ignition_msgs-ci-ign-msgs8-homebrew-amd64',
    'ignition_msgs-ci-main-focal-amd64',
    'ignition_msgs-ci-main-homebrew-amd64',
    'ignition_physics-ci-gz-physics6-focal-amd64',
    'ignition_physics-ci-gz-physics6-homebrew-amd64',
    'ignition_physics-ci-ign-physics2-focal-amd64',
    'ignition_physics-ci-ign-physics2-homebrew-amd64',
    'ignition_physics-ci-ign-physics5-focal-amd64',
    'ignition_physics-ci-ign-physics5-homebrew-amd64',
    'ignition_plugin-ci-gz-plugin2-focal-amd64',
    'ignition_plugin-ci-gz-plugin2-homebrew-amd64',
    'ignition_plugin-ci-ign-plugin1-focal-amd64',
    'ignition_plugin-ci-ign-plugin1-homebrew-amd64',
    'ignition_rendering-ci-gz-rendering7-focal-amd64',
    'ignition_rendering-ci-gz-rendering7-homebrew-amd64',
    'ignition_rendering-ci-ign-rendering3-focal-amd64',
    'ignition_rendering-ci-ign-rendering3-homebrew-amd64',
    'ignition_rendering-ci-ign-rendering6-focal-amd64',
    'ignition_rendering-ci-ign-rendering6-homebrew-amd64',
    'ignition_rendering-ci-main-focal-amd64',
    'ignition_rendering-ci-main-homebrew-amd64',
    'ignition_sensors-ci-gz-sensors7-focal-amd64',
    'ignition_sensors-ci-gz-sensors7-homebrew-amd64',
    'ignition_sensors-ci-ign-sensors3-focal-amd64',
    'ignition_sensors-ci-ign-sensors3-homebrew-amd64',
    'ignition_sensors-ci-ign-sensors6-focal-amd64',
    'ignition_sensors-ci-ign-sensors6-homebrew-amd64',
    'ignition_sensors-ci-main-focal-amd64',
    'ignition_sensors-ci-main-homebrew-amd64',
    'ignition_tools-ci-gz-tools2-focal-amd64',
    'ignition_tools-ci-gz-tools2-homebrew-amd64',
    'ignition_tools-ci-ign-tools1-focal-amd64',
    'ignition_tools-ci-ign-tools1-homebrew-amd64',
    'ignition_transport-ci-gz-transport12-focal-amd64',
    'ignition_transport-ci-gz-transport12-homebrew-amd64',
    'ignition_transport-ci-ign-transport11-focal-amd64',
    'ignition_transport-ci-ign-transport11-homebrew-amd64',
    'ignition_transport-ci-ign-transport8-focal-amd64',
    'ignition_transport-ci-ign-transport8-homebrew-amd64',
    'ignition_transport-ci-main-focal-amd64',
    'ignition_transport-ci-main-homebrew-amd64',
    'ignition_utils-ci-gz-utils2-focal-amd64',
    'ignition_utils-ci-gz-utils2-homebrew-amd64',
    'ignition_utils-ci-ign-utils1-focal-amd64',
    'ignition_utils-ci-ign-utils1-homebrew-amd64',
    'sdformat-ci-sdformat12-focal-amd64',
    'sdformat-ci-sdformat12-homebrew-amd64',
    'sdformat-ci-sdformat13-focal-amd64',
    'sdformat-ci-sdformat13-homebrew-amd64',
    'sdformat-ci-sdformat9-focal-amd64',
    'sdformat-ci-sdformat9-homebrew-amd64',
    'sdformat-sdf-12-win',
    'sdformat-sdf-13-win',
    'sdformat-sdf-9-win',
];
def trackedJobs = new LinkedHashSet(trackedJobsList);

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

