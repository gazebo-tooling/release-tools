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
// Currently is the result of https://github.com/osrf/buildfarmer/blob/main/scripts/gazebo_non_nightly_jobs.py
def trackedJobsList = [
'gz_cmake-ci-ign-cmake2-focal-amd64',
'gz_cmake-ci-ign-cmake2-homebrew-amd64',
'gz_cmake-ign-cmake2-win',
'gz_cmake-ci-ign-cmake2-focal-amd64',
'gz_cmake-ign-cmake2-win',
'gz_cmake-ci-ign-cmake2-homebrew-amd64',
'gz_cmake-ci-gz-cmake3-focal-amd64',
'gz_cmake-ci-gz-cmake3-homebrew-amd64',
'gz_cmake-3-win',
'gz_cmake-ci-gz-cmake3-homebrew-amd64',
'gz_cmake-3-win',
'gz_cmake-ci-gz-cmake3-jammy-amd64',
'gz_cmake-ci-main-homebrew-amd64',
'gz_cmake-main-win',
'gz_cmake-ci-main-jammy-amd64',
'gz_tools-ci-ign-tools1-focal-amd64',
'gz_tools-ci-ign-tools1-homebrew-amd64',
'gz_tools-ci-ign-tools1-homebrew-amd64',
'gz_tools-ci-ign-tools1-focal-amd64',
'gz_tools-ci-gz-tools2-focal-amd64',
'gz_tools-ci-gz-tools2-homebrew-amd64',
'gz_tools-ci-gz-tools2-homebrew-amd64',
'gz_tools-ci-gz-tools2-homebrew-amd64',
'gz_tools-ign-tools1-win',
'gz_tools-ign-tools1-win',
'gz_tools-2-win',
'gz_tools-2-win',
'gz_tools-2-win',
'gz_tools-ci-gz-tools2-jammy-amd64',
'gz_tools-ci-gz-tools2-jammy-amd64',
'gz_math-ci-ign-math6-focal-amd64',
'gz_math-ci-ign-math6-focal-amd64',
'gz_math-ci-gz-math7-focal-amd64',
'gz_math-ci-ign-math6-homebrew-amd64',
'gz_math-ci-ign-math6-homebrew-amd64',
'gz_math-ci-gz-math7-homebrew-amd64',
'gz_math-ci-gz-math7-homebrew-amd64',
'gz_math-ci-main-homebrew-amd64',
'gz_math-ign-math6-win',
'gz_math-ign-math6-win',
'gz_math-7-win',
'gz_math-7-win',
'gz_math-main-win',
'gz_math-ci-gz-math7-jammy-amd64',
'gz_math-ci-main-jammy-amd64',
'gz_plugin-ci-ign-plugin1-focal-amd64',
'gz_plugin-ci-ign-plugin1-focal-amd64',
'gz_plugin-ci-gz-plugin2-focal-amd64',
'gz_plugin-ci-ign-plugin1-homebrew-amd64',
'gz_plugin-ci-ign-plugin1-homebrew-amd64',
'gz_plugin-ci-gz-plugin2-homebrew-amd64',
'gz_plugin-ci-gz-plugin2-homebrew-amd64',
'gz_plugin-ci-main-homebrew-amd64',
'gz_plugin-ign-plugin1-win',
'gz_plugin-ign-plugin1-win',
'gz_plugin-2-win',
'gz_plugin-2-win',
'gz_plugin-main-win',
'gz_plugin-ci-gz-plugin2-jammy-amd64',
'gz_plugin-ci-main-jammy-amd64',
'gz_common-ci-ign-common3-focal-amd64',
'gz_common-ci-ign-common4-focal-amd64',
'gz_common-ci-gz-common5-focal-amd64',
'gz_common-ci-ign-common3-homebrew-amd64',
'gz_common-ci-ign-common4-homebrew-amd64',
'gz_common-ci-gz-common5-homebrew-amd64',
'gz_common-ci-gz-common5-homebrew-amd64',
'gz_common-ci-main-homebrew-amd64',
'gz_common-ign-common3-win',
'gz_common-ign-common4-win',
'gz_common-5-win',
'gz_common-5-win',
'gz_common-main-win',
'gz_common-ci-gz-common5-jammy-amd64',
'gz_common-ci-main-jammy-amd64',
'gz_msgs-ci-ign-msgs5-focal-amd64',
'gz_msgs-ci-ign-msgs8-focal-amd64',
'gz_msgs-ci-gz-msgs9-focal-amd64',
'gz_msgs-ci-ign-msgs5-homebrew-amd64',
'gz_msgs-ci-ign-msgs8-homebrew-amd64',
'gz_msgs-ci-gz-msgs9-homebrew-amd64',
'gz_msgs-ci-gz-msgs10-homebrew-amd64',
'gz_msgs-ci-main-homebrew-amd64',
'gz_msgs-ign-msgs5-win',
'gz_msgs-ign-msgs8-win',
'gz_msgs-9-win',
'gz_msgs-10-win',
'gz_msgs-main-win',
'gz_msgs-ci-gz-msgs10-jammy-amd64',
'gz_msgs-ci-main-jammy-amd64',
'gz_rendering-ci-ign-rendering3-focal-amd64',
'gz_rendering-ci-ign-rendering6-focal-amd64',
'gz_rendering-ci-gz-rendering7-focal-amd64',
'gz_rendering-ci-ign-rendering3-homebrew-amd64',
'gz_rendering-ci-ign-rendering6-homebrew-amd64',
'gz_rendering-ci-gz-rendering7-homebrew-amd64',
'gz_rendering-ci-gz-rendering8-homebrew-amd64',
'gz_rendering-ci-main-homebrew-amd64',
'gz_rendering-ign-rendering3-win',
'gz_rendering-ign-rendering6-win',
'gz_rendering-7-win',
'gz_rendering-8-win',
'gz_rendering-main-win',
'gz_rendering-ci-gz-rendering8-jammy-amd64',
'gz_rendering-ci-main-jammy-amd64',
'sdformat-ci-sdf9-focal-amd64',
'sdformat-ci-sdf12-focal-amd64',
'sdformat-ci-sdf13-focal-amd64',
'sdformat-ci-sdf9-homebrew-amd64',
'sdformat-ci-sdf12-homebrew-amd64',
'sdformat-ci-sdf13-homebrew-amd64',
'sdformat-ci-sdf14-homebrew-amd64',
'sdformat-ci-main-homebrew-amd64',
'sdformat-sdf9-win',
'sdformat-sdf12-win',
'sdformat-sdf13-win',
'sdformat-sdf14-win',
'sdformat-main-win',
'sdformat-ci-sdf14-jammy-amd64',
'sdformat-ci-main-jammy-amd64',
'gz_fuel_tools-ci-ign-fuel-tools4-focal-amd64',
'gz_fuel_tools-ci-ign-fuel-tools7-focal-amd64',
'gz_fuel_tools-ci-gz-fuel-tools8-focal-amd64',
'gz_fuel_tools-ci-ign-fuel-tools4-homebrew-amd64',
'gz_fuel_tools-ci-ign-fuel-tools7-homebrew-amd64',
'gz_fuel_tools-ci-gz-fuel-tools8-homebrew-amd64',
'gz_fuel_tools-ci-gz-fuel-tools9-homebrew-amd64',
'gz_fuel_tools-ci-main-homebrew-amd64',
'gz_fuel_tools-ign-fuel-tools4-win',
'gz_fuel_tools-ign-fuel-tools7-win',
'gz_fuel_tools-8-win',
'gz_fuel_tools-9-win',
'gz_fuel_tools-main-win',
'gz_fuel_tools-ci-gz-fuel-tools9-jammy-amd64',
'gz_fuel_tools-ci-main-jammy-amd64',
'gz_transport-ci-ign-transport8-focal-amd64',
'gz_transport-ci-ign-transport11-focal-amd64',
'gz_transport-ci-gz-transport12-focal-amd64',
'gz_transport-ci-ign-transport8-homebrew-amd64',
'gz_transport-ci-ign-transport11-homebrew-amd64',
'gz_transport-ci-gz-transport12-homebrew-amd64',
'gz_transport-ci-gz-transport13-homebrew-amd64',
'gz_transport-ci-main-homebrew-amd64',
'gz_transport-ign-transport8-win',
'gz_transport-ign-transport11-win',
'gz_transport-12-win',
'gz_transport-13-win',
'gz_transport-main-win',
'gz_transport-ci-gz-transport13-jammy-amd64',
'gz_transport-ci-main-jammy-amd64',
'gz_gui-ci-ign-gui3-focal-amd64',
'gz_gui-ci-ign-gui6-focal-amd64',
'gz_gui-ci-gz-gui7-focal-amd64',
'gz_gui-ci-ign-gui3-homebrew-amd64',
'gz_gui-ci-ign-gui6-homebrew-amd64',
'gz_gui-ci-gz-gui7-homebrew-amd64',
'gz_gui-ci-gz-gui8-homebrew-amd64',
'gz_gui-ci-main-homebrew-amd64',
'gz_gui-ign-gui3-win',
'gz_gui-ign-gui6-win',
'gz_gui-7-win',
'gz_gui-8-win',
'gz_gui-main-win',
'gz_gui-ci-gz-gui8-jammy-amd64',
'gz_gui-ci-main-jammy-amd64',
'gz_sensors-ci-ign-sensors3-focal-amd64',
'gz_sensors-ci-ign-sensors6-focal-amd64',
'gz_sensors-ci-gz-sensors7-focal-amd64',
'gz_sensors-ci-ign-sensors3-homebrew-amd64',
'gz_sensors-ci-ign-sensors6-homebrew-amd64',
'gz_sensors-ci-gz-sensors7-homebrew-amd64',
'gz_sensors-ci-gz-sensors8-homebrew-amd64',
'gz_sensors-ci-main-homebrew-amd64',
'gz_sensors-ign-sensors3-win',
'gz_sensors-ign-sensors6-win',
'gz_sensors-7-win',
'gz_sensors-8-win',
'gz_sensors-main-win',
'gz_sensors-ci-gz-sensors8-jammy-amd64',
'gz_sensors-ci-main-jammy-amd64',
'gz_physics-ci-ign-physics2-focal-amd64',
'gz_physics-ci-ign-physics5-focal-amd64',
'gz_physics-ci-gz-physics6-focal-amd64',
'gz_physics-ci-ign-physics2-homebrew-amd64',
'gz_physics-ci-ign-physics5-homebrew-amd64',
'gz_physics-ci-gz-physics6-homebrew-amd64',
'gz_physics-ci-gz-physics7-homebrew-amd64',
'gz_physics-ci-main-homebrew-amd64',
'gz_physics-ign-physics2-win',
'gz_physics-ign-physics5-win',
'gz_physics-6-win',
'gz_physics-7-win',
'gz_physics-main-win',
'gz_physics-ci-gz-physics7-jammy-amd64',
'gz_physics-ci-main-jammy-amd64',
'gz_sim-ci-ign-gazebo3-focal-amd64',
'gz_sim-ci-ign-gazebo6-focal-amd64',
'gz_sim-ci-gz-sim7-focal-amd64',
'gz_sim-ci-ign-gazebo3-homebrew-amd64',
'gz_sim-ci-ign-gazebo6-homebrew-amd64',
'gz_sim-ci-gz-sim7-homebrew-amd64',
'gz_sim-ci-gz-sim8-homebrew-amd64',
'gz_sim-ci-main-homebrew-amd64',
'gz_sim-ign-gazebo3-win',
'gz_sim-ign-gazebo6-win',
'gz_sim-7-win',
'gz_sim-8-win',
'gz_sim-main-win',
'gz_sim-ci-gz-sim8-jammy-amd64',
'gz_sim-ci-main-jammy-amd64',
'gz_launch-ci-ign-launch2-focal-amd64',
'gz_launch-ci-ign-launch5-focal-amd64',
'gz_launch-ci-gz-launch6-focal-amd64',
'gz_launch-ci-ign-launch2-homebrew-amd64',
'gz_launch-ci-ign-launch5-homebrew-amd64',
'gz_launch-ci-gz-launch6-homebrew-amd64',
'gz_launch-ci-gz-launch7-homebrew-amd64',
'gz_launch-ci-main-homebrew-amd64',
'gz_launch-ign-launch2-win',
'gz_launch-ign-launch5-win',
'gz_launch-6-win',
'gz_launch-7-win',
'gz_launch-main-win',
'gz_launch-ci-gz-launch7-jammy-amd64',
'gz_launch-ci-main-jammy-amd64',
'gz_utils-ci-ign-utils1-focal-amd64',
'gz_utils-ci-gz-utils2-focal-amd64',
'gz_utils-ci-ign-utils1-homebrew-amd64',
'gz_utils-ci-gz-utils2-homebrew-amd64',
'gz_utils-ci-gz-utils2-homebrew-amd64',
'gz_utils-ci-main-homebrew-amd64',
'gz_utils-ign-utils1-win',
'gz_utils-2-win',
'gz_utils-2-win',
'gz_utils-main-win',
'gz_utils-ci-gz-utils2-jammy-amd64',
'gz_utils-ci-main-jammy-amd64'
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

