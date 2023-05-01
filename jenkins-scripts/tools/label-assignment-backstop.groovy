import jenkins.*;
import jenkins.model.*;
import hudson.model.Label;

def exactly_one_labels = [
  ["linux-nightly-focal", "docker"],
  ["linux-nightly-jammy", "docker"],
]

for (tup in exactly_one_labels) {
  label_nodes = Label.get(tup[0]).getNodes()
  if (label_nodes.size() == 1) {
    println(tup[0] + " is currently applied to " + label_nodes[0].name)
    continue
  }

  if (label_nodes.size() > 1) {
    println("WARNING: Too many nodes with the label " + tup[0])
    for (node in label_nodes) {
      println("  " + node.name)
    }
    continue
  }

  if (label_nodes.size() < 1) {
    println("No host currently has the label " + tup[0])
    println("Appointing a node from the configured pool matching '" + tup[1] + "'")
    node_set = Label.get(tup[1]).getNodes()
    if (node_set.size() <= 0) {
      println("WARNING: Pool of '" + tup[1] + "' machines for " + tup[0] + " is empty!")
    }
    node_pool = []
    for (node in node_set) { 
      node_pool.add(node)
    }
    Collections.shuffle(node_pool)
    appointed_node = node_pool[0]
    new_label_string = appointed_node.getAssignedLabels().join(" ")
    new_label_string = tup[0] + " " + new_label_string
    appointed_node.setLabelString(new_label_string)
    appointed_node.save()
    println("Added label to " + appointed_node.name)
    continue
  }
}
