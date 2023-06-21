import hudson.model.*;
import jenkins.model.Jenkins;
import java.util.Date;

long eightDaysAgoMillis = System.currentTimeMillis() - 8 * 24 * 60 * 60 * 1000; // 8 days ago in milis
Date eightDaysAgoDate = new Date(eightDaysAgoMillis);

def runJob(String jobName) {
    def job = Jenkins.instance.getItemByFullName(jobName)

    // Workaround for not having the default parameters:
    // https://issues.jenkins.io/browse/JENKINS-13768?focusedCommentId=290659&page=com.atlassian.jira.plugin.system.issuetabpanels%3Acomment-tabpanel#comment-290659
    def params = []
    for (ParameterDefinition paramDefinition : job.getProperty(ParametersDefinitionProperty.class).getParameterDefinitions()) {
        params += paramDefinition.getDefaultParameterValue();
    }
    def paramsAction = new ParametersAction(params)

    def currentBuild = Thread.currentThread().executable
    def causeAction = new CauseAction( new hudson.model.Cause.UpstreamCause(currentBuild) )
    Hudson.instance.queue.schedule(job, 0, causeAction, paramsAction);
}

// TODO: This is a hardcoded list of jobs that we're tracking. I'd like
// to find a way to get this list dynamically
def tracked_jobs_list = ['ignition_launch-ci-ign-launch5-focal-amd64', 'ign_physics-ign-2-win', 'ign_fuel-tools-ign-4-win', 'ignition_rendering-ci-ign-rendering3-homebrew-amd64', 'ignition_rendering-ci-gz-rendering7-homebrew-amd64', 'ignition_transport-ci-ign-transport8-focal-amd64', 'ignition_plugin-ci-gz-plugin2-focal-amd64', 'ignition_gui-ci-ign-gui3-focal-amd64', 'ignition_msgs-ci-ign-msgs5-focal-amd64', 'ignition_gui-ci-gz-gui7-focal-amd64', 'ign_launch-ign-2-win', 'ignition_sensors-ci-gz-sensors7-homebrew-amd64', 'ignition_plugin-ci-gz-plugin2-homebrew-amd64', 'ignition_common-ci-ign-common3-focal-amd64', 'ign_rendering-ign-3-win', 'ignition_physics-ci-gz-physics6-focal-amd64', 'ignition_rendering-ci-gz-rendering7-focal-amd64', 'ignition_msgs-ci-gz-msgs9-homebrew-amd64', 'ignition_msgs-ci-ign-msgs8-homebrew-amd64', 'ign_gui-ign-3-win', 'ign_launch-ign-5-win', 'ign_tools-ign-1-win', 'ignition_tools-ci-ign-tools1-focal-amd64', 'ignition_transport-ci-ign-transport8-homebrew-amd64', 'ign_utils-gz-2-win', 'ignition_physics-ci-ign-physics5-homebrew-amd64', 'ignition_sensors-ci-ign-sensors6-focal-amd64', 'ignition_gui-ci-ign-gui6-focal-amd64', 'ignition_common-ci-ign-common3-homebrew-amd64', 'ign_msgs-ign-5-win', 'ignition_physics-ci-ign-physics5-focal-amd64', 'ignition_utils-ci-ign-utils1-homebrew-amd64', 'ign_math-gz-7-win', 'ignition_common-ci-ign-common4-focal-amd64', 'sdformat-ci-sdformat12-focal-amd64', 'sdformat-ci-sdformat9-homebrew-amd64', 'ignition_utils-ci-ign-utils1-focal-amd64', 'ignition_rendering-ci-ign-rendering3-focal-amd64', 'ign_common-ign-4-win', 'ignition_transport-ci-gz-transport12-focal-amd64', 'ign_rendering-gz-7-win', 'ignition_math-ci-ign-math6-focal-amd64', 'ignition_launch-ci-ign-launch2-homebrew-amd64', 'ignition_rendering-ci-ign-rendering6-focal-amd64', 'ign_physics-ign-5-win', 'ignition_transport-ci-ign-transport11-focal-amd64', 'ignition_physics-ci-ign-physics2-homebrew-amd64', 'ign_common-gz-5-win', 'ign_fuel-tools-ign-7-win', 'ign_transport-gz-12-win', 'ign_transport-ign-8-win', 'ignition_gui-ci-ign-gui3-homebrew-amd64', 'ignition_fuel-tools-ci-ign-fuel-tools7-focal-amd64', 'ign_math-ign-6-win', 'ign_msgs-gz-9-win', 'ign_gazebo-ign-3-win', 'ignition_physics-ci-gz-physics6-homebrew-amd64', 'ignition_msgs-ci-ign-msgs5-homebrew-amd64', 'ignition_tools-ci-gz-tools2-homebrew-amd64', 'sdformat-sdf-12-win', 'ignition_cmake-ci-ign-cmake2-homebrew-amd64', 'ignition_sensors-ci-ign-sensors3-homebrew-amd64', 'ign_rendering-ign-6-win', 'ignition_gazebo-ci-ign-gazebo3-homebrew-amd64', 'ign_plugin-ign-1-win', 'sdformat-ci-sdformat9-focal-amd64', 'ign_cmake-ign-2-win', 'ign_utils-ign-1-win', 'ignition_fuel-tools-ci-ign-fuel-tools4-homebrew-amd64', 'ignition_utils-ci-gz-utils2-focal-amd64', 'ignition_plugin-ci-ign-plugin1-homebrew-amd64', 'ignition_tools-ci-gz-tools2-focal-amd64', 'ignition_launch-ci-ign-launch2-focal-amd64', 'ign_launch-gz-6-win', 'ignition_fuel-tools-ci-gz-fuel-tools8-homebrew-amd64', 'ignition_utils-ci-gz-utils2-homebrew-amd64', 'ignition_math-ci-ign-math6-homebrew-amd64', 'ignition_gazebo-ci-gz-sim7-focal-amd64', 'ign_fuel-tools-gz-8-win', 'ignition_sensors-ci-gz-sensors7-focal-amd64', 'sdformat-ci-sdformat12-homebrew-amd64', 'ignition_msgs-ci-ign-msgs8-focal-amd64', 'ignition_plugin-ci-ign-plugin1-focal-amd64', 'ignition_physics-ci-ign-physics2-focal-amd64', 'ignition_cmake-ci-gz-cmake3-homebrew-amd64', 'ign_plugin-gz-2-win', 'ignition_launch-ci-ign-launch5-homebrew-amd64', 'ignition_common-ci-gz-common5-focal-amd64', 'ign_sensors-ign-3-win', 'ign_physics-gz-6-win', 'ign_common-ign-3-win', 'ign_tools-gz-2-win', 'ignition_launch-ci-gz-launch6-homebrew-amd64', 'sdformat-sdf-13-win', 'ignition_common-ci-ign-common4-homebrew-amd64', 'ign_gui-ign-6-win', 'ignition_gui-ci-ign-gui6-homebrew-amd64', 'ignition_tools-ci-ign-tools1-homebrew-amd64', 'ignition_gazebo-ci-ign-gazebo3-focal-amd64', 'ignition_cmake-ci-ign-cmake2-focal-amd64', 'ign_sensors-gz-7-win', 'ignition_fuel-tools-ci-gz-fuel-tools8-focal-amd64', 'ignition_rendering-ci-ign-rendering6-homebrew-amd64', 'ign_msgs-ign-8-win', 'ignition_msgs-ci-gz-msgs9-focal-amd64', 'ignition_gazebo-ci-ign-gazebo6-homebrew-amd64', 'ign_gui-gz-7-win', 'sdformat-sdf-9-win', 'ignition_transport-ci-gz-transport12-homebrew-amd64', 'ignition_gazebo-ci-ign-gazebo6-focal-amd64', 'ignition_launch-ci-gz-launch6-focal-amd64', 'ignition_fuel-tools-ci-ign-fuel-tools4-focal-amd64', 'ign_cmake-gz-3-win', 'ign_gazebo-ign-6-win', 'ignition_sensors-ci-ign-sensors3-focal-amd64', 'sdformat-ci-sdformat13-homebrew-amd64', 'sdformat-ci-sdformat13-focal-amd64', 'ignition_common-ci-gz-common5-homebrew-amd64', 'ignition_cmake-ci-gz-cmake3-focal-amd64', 'ign_transport-ign-11-win', 'ign_gazebo-gz-7-win', 'ign_sensors-ign-6-win', 'ignition_sensors-ci-ign-sensors6-homebrew-amd64', 'ignition_gazebo-ci-gz-sim7-homebrew-amd64', 'ignition_gui-ci-gz-gui7-homebrew-amd64', 'ignition_math-ci-gz-math7-focal-amd64', 'ignition_transport-ci-ign-transport11-homebrew-amd64', 'ignition_math-ci-gz-math7-homebrew-amd64', 'ignition_fuel-tools-ci-ign-fuel-tools7-homebrew-amd64'];
def tracked_jobs = new LinkedHashSet(tracked_jobs_list);

// Take max 7 jobs per OS
def jenkinsJobs = Hudson.instance


def linux_jobs_to_run = [];
def homebrew_jobs_to_run = [];
def windows_jobs_to_run = [];
def spare_jobs_to_run = [];

jenkinsJobs.getItems(Project).each {project ->
    // Filter jobs that have not been updated in the last 8 days
  if (!project.disabled && tracked_jobs.contains(project.displayName) && project.lastBuild.getTime().before(eightDaysAgoDate)) {
    if (project.displayName.contains('focal') || project.displayName.contains('jammy')) {
        linux_jobs_to_run.add(project.displayName);
    } else if (project.displayName.contains('homebrew')) {
        homebrew_jobs_to_run.add(project.displayName);
    } else if (project.displayName.contains('win')) {
        windows_jobs_to_run.add(project.displayName);
    } else {
        spare_jobs_to_run.add(project.displayName);
    }
    
    // println('Running: ' + project.displayName + ' - Last time: ' + project.lastBuild.getTime());
    // runJob(project.displayName);
  }
}

println('Linux jobs to run: ' + linux_jobs_to_run);
println('Homebrew jobs to run: ' + homebrew_jobs_to_run);
println('Windows jobs to run: ' + windows_jobs_to_run);
println('Spare jobs to run: ' + spare_jobs_to_run);

