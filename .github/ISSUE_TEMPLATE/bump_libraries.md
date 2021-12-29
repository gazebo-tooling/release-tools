---
name: Bump libraries
about: Checklist when bumping major versions in a collection
title: "Bump in [COLLECTION]: [LIBRARIES]"
---

<!--

Use this template to keep track of everything that needs to be done to
bump the major version of a library within a collection.

For example, Ignition-X is currently under development, and it's using
ign-math42. This checklist includes all that's needed to use ign-math43
instead.

This script can be used to do most of the work:

https://github.com/ignition-tooling/release-tools/blob/master/release-repo-scripts/bump_dependency.bash

When opening PRs, add a link back to this issue for easier tracking.

-->

Libraries being bumped:

<!-- Be sure to include all downstream libraries that will need to be bumped too -->

* <LIBRARY NAME AND VERSION>: <!-- Explain why bump is needed -->
    - [ ] Source code of all downstream libraries ([example](https://github.com/ignitionrobotics/ign-transport/pull/149))
    - [ ] Release repositories of all downstream libraries ([example](https://github.com/ignition-release/ign-fuel-tools7-release/pull/1))
    - [ ] Use `main` branch on `gazebodistro` ([example](https://github.com/ignition-tooling/gazebodistro/pull/42))
    - [ ] Build nightlies from the `main` branch ([example](https://github.com/ignition-tooling/release-tools/pull/437))
    - [ ] homebrew-simulation: create formula and update dependencies ([example](https://github.com/osrf/homebrew-simulation/pull/14230))
    - [ ] docs (the collection’s page) ([example](https://github.com/ignitionrobotics/docs/pull/175))
* <!-- Add more libraries here and copy the checklist for each of them -->
