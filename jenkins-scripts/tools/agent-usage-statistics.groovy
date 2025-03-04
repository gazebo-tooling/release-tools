import hudson.model.*
import hudson.model.queue.*
import hudson.model.labels.LabelAtom
import java.text.SimpleDateFormat

def timestampFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss")
def timestamp = timestampFormat.format(new Date())

def agentFilePath = "agent_data.csv"
def queueFilePath = "queue_data.csv"

def queue = Jenkins.instance.queue
def queueItems = queue.items

if (!new File(agentFilePath).exists()) {
    new File(agentFilePath).withWriter { writer ->
        writer.writeLine("Timestamp,Node,Label,Job")
    }
}

if (!new File(queueFilePath).exists()) {
    new File(queueFilePath).withWriter { writer ->
        writer.writeLine("Timestamp,Job,Blocked Reason,Waiting For")
    }
}

Jenkins.instance.nodes.each { node ->
    def nodeName = node.getDisplayName()
    def nodeLabels = node.getAssignedLabels().collect { it.name }.join(", ") ?: "None"
    nodeLabels = "\"${nodeLabels}\""
    def currentBuild = node.toComputer().getExecutors().find { it.isBusy() }?.currentExecutable
    def currentJob = currentBuild?.fullDisplayName ?: "None"
    new File(agentFilePath).append(
        "${timestamp},${nodeName},${nodeLabels},${currentJob}\n")
}

queueItems.each { item ->
    def blockedReason = item.getCauseOfBlockage()?.getShortDescription() ?: "Not blocked"
    def waitingForNodes = item.assignedLabel?.getExpression() ?: "Any available node"
    def jobName = item.task.fullDisplayName
    new File(queueFilePath).append(
        "${timestamp},${jobName},${blockedReason},${waitingForNodes}\n")
}

println "Agent and queue data have been written to CSV files."