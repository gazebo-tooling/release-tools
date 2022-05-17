# Source repositories scripts

The directory hosts the scripts designed to work with Gazebo source
repositories, real code repositories. Mostly hosted under
https://github.com/gazebosim/ GitHub organization.

Some of them could also do another kind of operations outside of
this repositories (like updated -release repositories info).

## Scripts available

### bump_dependency.bash

The script will bump the major version of a library for a given collection:

#### Usage

Requires the 'gh' CLI to be installed.
```bash
bump_dependency.bash <collection> <library>;<library> <version>;<version> <issue_number> <prev_collection> [<docs_branch>]
```

The `docs_branch` parameter is optional and defaults to `master` if not specified.

The script clones all the necessary repositories under /tmp/bump_dependency.

Before committing to each repository, the script asks "Commit <repository
name>?".  Before saying yes, navigate to the repository and check if the diff
looks reasonable.  When you say yes, the changes will be committed and pushed.
Click on the link printed by GitHub to open the pull request.

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
