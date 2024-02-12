/*
 * The script is designed to assign the nightly labels
 * for nodes dynamically, Only one node should build
 * nightlies for each distribution in order to ensure
 * the correct orchestration of the Gazebo libraries
 * interdependencies.
 */

import jenkins.*;
import jenkins.model.*;
import hudson.model.Label;

/*
  Each label is in a 2-tuple with a label tag that indicates its "pool"
  of potential assignees. The fields must be a single label (no spaces).
*/
def nightly_label_prefix = "linux-nightly"
def exactly_one_labels = [
  ["${nightly_label_prefix}-focal", "large-memory"],
  ["${nightly_label_prefix}-jammy", "large-memory"],
  ["${nightly_label_prefix}-noble", "large-memory"],
]

for (tup in exactly_one_labels) {
  nightly_label = tup[0]
  pool_label = tup[1]

  label_nodes = Label.get(nightly_label).getNodes().findAll { it.toComputer().isOnline() }

  if (label_nodes.size() == 1) {
    println("${nightly_label} is already assigned to " + label_nodes[0].name)
    continue
  }

  // return unstable if the labels are not set correctly before running the
  // process to assign the new nightly labels.
  println("MARK_AS_UNSTABLE")

  if (label_nodes.size() > 1) {
    println("WARNING: Too many online nodes with the label ${nightly_label}")
    label_nodes.each { node ->
      println("  " + node.name)
    }
    continue
  }

  if (label_nodes.size() < 1) {
    println("No online host currently has the label ${nightly_label}")
    println("Appointing a node from the configured pool matching '${pool_label}'")

    def node_pool = Jenkins.instance.nodes.findAll { node ->
      node.computer.online &&
      node.getLabelString().contains(pool_label) &&
      !node.getLabelString().contains(nightly_label_prefix) &&
      !node.getLabelString().contains("test-instance")
    }

    if (node_pool.size() <= 0) {
      println("The Pool of '${pool_label}' machines for ${nightly_label} is empty. Reusing a node:")
      node_pool = Jenkins.instance.nodes.findAll { node ->
        node.computer.online &&
        node.getLabelString().contains(pool_label) &&
        !node.getLabelString().contains("test-instance")
      }
    }

    // Sort the node pool based on the count of labels containing nightly_label_prefix
    node_pool.sort { node ->
      node.getLabelString().split(' ').count { it.contains(nightly_label_prefix) }
    }

    // Pick the node with the least count of labels containing nightly_label_prefix
    appointed_node = node_pool.first()

    new_label_string = appointed_node.getAssignedLabels().join(" ")
    new_label_string = nightly_label + " " + new_label_string
    appointed_node.setLabelString(new_label_string)
    appointed_node.save()
    println("Added label to " + appointed_node.name)
    continue
  }
}
