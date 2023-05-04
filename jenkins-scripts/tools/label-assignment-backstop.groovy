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
  Each label is in a 2-tuple with a label expression that indicates its "pool"
  of potential assignees.
  The first field must be a single label (no spaces) but the second may be an
  arbitrarily complex label expression like "(docker && linux) && !armhf"
*/
def exactly_one_labels = [
  ["linux-nightly-focal", "docker"],
  ["linux-nightly-jammy", "docker"],
]

for (tup in exactly_one_labels) {
  label_nodes = Label.get(tup[0]).getNodes().findAll { it.toComputer().isOnline() }

  if (label_nodes.size() == 1) {
    println(tup[0] + " is currently applied to " + label_nodes[0].name)
    continue
  }

  // Mark build as unstable there are problems with labels or if the
  println("MARK_AS_UNSTABLE")

  if (label_nodes.size() > 1) {
    println("WARNING: Too many online nodes with the label " + tup[0])
    for (node in label_nodes) {
      println("  " + node.name)
    }
    continue
  }

  if (label_nodes.size() < 1) {
    println("No online host currently has the label " + tup[0])
    println("Appointing a node from the configured pool matching '" + tup[1] + "'")
    node_pool = Label.get(tup[1]).getNodes().findAll { it.toComputer().isOnline() }
    if (node_pool.size() <= 0) {
      println("WARNING: Pool of '" + tup[1] + "' machines for " + tup[0] + " is empty!")
    }
    // Pick a random node from the pool to receive the label.
    appointed_node = node_pool[(new Random()).nextInt(node_pool.size())]
    new_label_string = appointed_node.getAssignedLabels().join(" ")
    new_label_string = tup[0] + " " + new_label_string
    appointed_node.setLabelString(new_label_string)
    appointed_node.save()
    println("Added label to " + appointed_node.name)
    continue
  }
}
