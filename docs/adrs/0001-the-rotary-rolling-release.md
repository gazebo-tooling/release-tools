---
status: accepted
implementation: in-progress
date: 2026-02-14
category: releasing, packaging
impact: high
---

# 0001 — The Rotary Rolling Release

## Context

Gazebo needs a strategy for releasing packages from `main` branches between
versioned collection releases. This is supplemental to the strategy for major
version bumps discussed in [issue #1403]. The current approach of creating new
versioned release repos each cycle has maintenance overhead, and there was no
mechanism for continuously releasing from `main` into a rolling distribution.

## Decision

We will create `gz-rotary-{package_designation}-release` repos for each package in a Gazebo
collection. These repos contain debian metadata for unversioned Gazebo packages
(e.g. `libgz-math-dev`) and `rotary`-labeled aliases (e.g.
`libgz-rotary-math-dev`). Source repo `packages.apt` files use the rotary
aliases.

### Naming Convention

The alias naming rule is to inject the collection name after `gz-`:

| Platform | Original                | Rotary alias                    |
|----------|-------------------------|---------------------------------|
| Ubuntu   | `libgz-cmake-dev`      | `libgz-rotary-cmake-dev`       |
| Ubuntu   | `gz-plugin-cli`        | `gz-rotary-plugin-cli`         |
| Ubuntu   | `python3-gz-math`      | `python3-gz-rotary-math`       |
| Brew     | `gz-mathN`             | `gz-rotary-math`               |

Special case for sdformat (no `gz-` in its package name):
`libsdformat-dev` -> `libgz-rotary-sdformat-dev` (Ubuntu),
`gz-rotary-sdformat` (Brew).

### Implementation Phases

**Initial implementation:**

- Remove versions from control files (packages and Gazebo dependencies become
  unversioned).
- Add rotary aliases pointing to the unversioned packages.
- Remove versions from `rules` (including versioned install path directories).
- Make nightly releases from `rotary` by modifying `ignition_collection.dsl`.
- Bump `main` source branches to new major versions (can happen in parallel
  with the next step).
- Update `packages.apt` files to use `rotary` aliases packages.
- The `__upcoming__` collection is replaced by the new `rotary` collection
  in PR integration and the Jenkins views
  ([PR #1461](https://github.com/gazebo-tooling/release-tools/pull/1461)).

**At branch-off (when creating a new Gazebo Stable Release):**

- Fork each `{package}-rotary-release` repo and modify control files to include
  the major version number.
- Set explicit versions in `find_package` calls using
  `set_explicit_find_package_versions.bash`.
- As stable branches are created, update `packages.apt` with the rotary ->
  versioned package mapping.
- Update `gz-collection.yaml`, `gazebodistro`, and homebrew so the new
  collection points to stable branches with versioned package dependencies.

**Next release cycle:**

- No changes needed to `{package}-rotary-release` repos.
- Bump major version numbers in `main`; no change in `packages.apt`.
- Follow the same branch-off steps as above.

## Consequences

### Positive

- The `rotary` label follows a similar pattern to numbered versions, so
  existing tooling handles it with little or no modification.
- At branch-off, forking `*-rotary-release` repos makes creating versioned
  release repos straightforward.
- No annual updates needed for the rotary release repos between cycles.
- Provides a continuous integration path for packages built from `main`.

### Negative

- Requires initial effort to create and populate all `{package}-rotary-release`
  repos.
- The sdformat package requires special-case handling due to its non-standard
  naming (no `gz-` prefix).
- Some `debian/rules` files need non-trivial changes to remove versioned
  install paths.
- Nightly build infrastructure (`ignition_collection.dsl`) needs modifications
  since debbuilder names are computed as `{package}{major_version}-debbuilder`.

### Neutral

- Existing Jetty collection aliases use a different format (`gz-jetty-math`
  rather than `libgz-jetty-math-dev`). Jetty aliases will be duplicated to
  match the new convention.
- The `gzdev` tool requires updates to support the rotary workflow
  ([gzdev#103]).

## Alternatives Considered

**Single `gz-rotary-release` repo with aliases to versioned packages:** Instead
of per-package rotary release repos, create a single `gz-rotary-release` repo
containing aliases that point to all the versioned packages from `main`
branches. This was rejected because the alias repo would need to be updated
every year when version numbers change, adding recurring maintenance overhead
that the per-package approach avoids.

## References

- [Issue #1446 — proposal: Rotary release repos][issue #1446]
- [Issue #1403 — Strategy for major version bumps in new collections][issue #1403]
- [PR #1449 — Tool for adding collection-specific aliases](https://github.com/gazebo-tooling/release-tools/pull/1449)
- [PR #1430 — set_explicit_find_package_versions.bash](https://github.com/gazebo-tooling/release-tools/pull/1430)
- [Prototype: gz-rotary-cmake-release](https://github.com/scpeters/gz-rotary-cmake-release/pull/1)
- [Prototype: gz-rotary-utils-release](https://github.com/scpeters/gz-rotary-utils-release/pull/1)
- [Prototype: homebrew-simulation rotary formulae](https://github.com/osrf/homebrew-simulation/pull/3287)
- [Full Jetty alias naming convention gist](https://gist.github.com/j-rivero/2f6f910db5f38fe36d9c54d516d9e0a9)
- [Support source metadata that defines the GZDEV_PROJECT_NAME (technical debt)][gzdev#103]

[issue #1446]: https://github.com/gazebo-tooling/release-tools/issues/1446
[issue #1403]: https://github.com/gazebo-tooling/release-tools/issues/1403
[gzdev#103]: https://github.com/gazebo-tooling/gzdev/pull/103
