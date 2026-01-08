# Source repositories scripts

The directory hosts the scripts designed to work with Gazebo source
repositories, real code repositories. Mostly hosted under
https://github.com/gazebosim/ GitHub organization.

Some of them could also do another kind of operations outside of
this repositories (like updated -release repositories info).

## Scripts available

### set_explicit_find_package_versions.bash

The script sets explicit `find_package` versions in each library of a given Gazebo
collection. It automates the process of ensuring all `find_package` and `gz_find_package`
calls specify the correct major version number for packages in the collection.

The script modifies the following `CMakeLists.txt` files in each repository:

- Root `CMakeLists.txt`
- `python/CMakeLists.txt`
- `src/python_pybind11/CMakeLists.txt`

For each repository with changes, the script will create a branch, commit, and
open a pull request.

#### Requisites

Requires the following tools to be installed:

- `gh` CLI (GitHub CLI)
- `xmllint` (for parsing `package.xml` files)
- `python-vcstool` (for importing repositories from collection yaml)

#### Usage

```bash
# A dry-run execution can be done to preview changes without creating PRs
DRY_RUN=true ./set_explicit_find_package_versions.bash <collection> <issue_reference>

# Real execution
./set_explicit_find_package_versions.bash <collection> <issue_reference>
```

The script clones all necessary repositories under `/tmp/set_explicit_find_package_versions`.

Before committing to each repository, the script asks for confirmation. Navigate to
the repository and verify the diff looks reasonable before saying yes.

#### Example

To set explicit versions in all `find_package` calls for the Jetty collection:

```bash
./set_explicit_find_package_versions.bash jetty gazebosim/gz-jetty#138
```

### bump_dependency.bash

> [!NOTE]
> The script has been migrated to work with non-versioned packages as
> described in https://github.com/gazebo-tooling/release-tools/issues/1244

The script will bump the major version of a library for a given collection taking
care of most of the operations needed through different repositories ***including the
reverse dependencies of the libraries to update***. Changes will be done in the following
repositories:

* gazebosim/gz-* repositories: modifying build system code, docs, etc.
* gazebo-release/gz-*-release repositories: modifying versions to match new version bumps
* release-tools: update nightlies in ignition_gazebo.dsl
* docs: bumping lib versions under `$distro/install.md` files from X to X+1
* homebrew-simulation: creating a new Formula X+1 from the latest version X
* gazebodistro: update reverse dependencies yaml files to new branches

> [!IMPORTANT]
> The script does not yet implement the necessary changes to jenkins/dsl/gz-collections.yaml in
> release-tools, please do them manually.

### Requisites

Requires the 'gh' CLI to be installed.

```bash
# A dry-run execution can be done to see how many different commits and pushes will be done
# without executing changes in the official repositories
DRY_RUN=true bump_dependency.bash <collection> <library>;<library> <version>;<version> <issue_number> <prev_collection> [<docs_branch>]

# real execution
bump_dependency.bash <collection> <library>;<library> <version>;<version> <issue_number> <prev_collection> [<docs_branch>]
```

The `docs_branch` parameter is optional and defaults to `master` if not specified.

The script clones all the necessary repositories under /tmp/bump_dependency.

#### Example

To bump to `gz-rendering6` and all its dependencies, as well as `sdf12` and
its dependencies on fortress using the `chapulina/fortress` branch for `docs`:

```bash
./bump_dependency.bash fortress "gz-rendering;sdformat" "6;12" 428 edifice
"chapulina/fortress"
```

### merge_forward_pull_request.bash

The script will open a pull request for a forward port after the user has
already created the branch, run `git merge`, resolved conflicts, and committed
the result.

#### Usage

Requires the 'gh' CLI to be installed.
```bash
./merge_forward_pull_request.bash <from_branch> <to_branch>
```

#### Example

To merge `ign-rendering6` to `main`:

```bash
cd ign-rendering
git fetch origin
git checkout main
git reset --hard origin/main
git checkout -b merge_6_to_main
git merge origin/ign-rendering6
# manually resolve conflicts if necessary
git merge --continue
/path/to/merge_forward_pull_request.bash ign-rendering6 main
```

### source_changelog.bash

The script will generate updates for `Changelog.md` files useful when preparing a
new release.

#### Usage

Go to the source directory containing a local checkout of a Gazebo repository:

```
source_changelog.bash <last_version>
```

The script will parse git log entries and generate Markdown as console output
ready to be injected to `Changelog.md` files.

### Example

To update the Changelog for `gz-math` new version 6.11.0:

```
cd gz-math
~/release-tools/source-repo-scripts/source_changelog.bash 6.10.0
```

### Release summary

Print a markdown summary of a release (not `Changelog.md` entries), with its
changelog and contributors. The script is designed to publish release summaries
from the internal Open Robotics team to the Community (usually in the public forum).

#### Usage

Run the script, then copy the end of the output into the announcement body.

```bash
cd <path_to_source_code>
bash release_summary.bash <prev release version> <new release version>
```

The release versions can be checked on `Changelog.md`. They don't need to be
consecutive (useful in cases when multiple releases happen between announcements).

#### Example

To announce the changes between Sensors 6.2.0 and 6.3.0:

```bash
cd <path_to_source_code>
bash release_summary.bash 6.2.0 6.3.0
