---
name: New collection
about: Checklist when creating a new collection
title: "New collection: <COLLECTION>"
---

<!--

Use this template to keep track of everything that needs to be done when
starting a new collection.

When opening PRs, add a link back to this issue for easier tracking.

-->

- [ ] Create `gazebodistro` file for new collection ([example](https://github.com/gazebo-tooling/gazebodistro/pull/181))
- [ ] Create `gz-<collection>` and `gz-<collection>-release` repositories ([example source](https://github.com/gazebosim/gz-jetty), [example release](https://github.com/gazebo-release/gz-jetty-release))
- [ ] Create homebrew-simulation formula for new collection ([example](https://github.com/osrf/homebrew-simulation/pull/2832))
- [ ] Create collection view on release-tools and build nightlies ([example](https://github.com/gazebo-tooling/release-tools/pull/1199))
- [ ] Add docs for new collection ([example](https://github.com/gazebosim/docs/pull/524))
- [ ] Add new collection label:
    - [ ] Add to pr-collection-labeler ([example](https://github.com/gazebo-tooling/pr-collection-labeler/pull/36))
    - [ ] Add to all libraries
      Example script:`cat collection-kura.yaml | sed -n "s#.*https://github.com/\(.*\)#\1#p" | xargs -L1 gh label create "🏯 kura" -R` 
