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

To bump to `ign-rendering6` and all its dependencies, as well as `sdf12` and
its dependencies on fortress using the `chapulina/fortress` branch for `docs`:

```bash
./bump_dependency.bash fortress "ign-rendering;sdformat" "6;12" 428 edifice
"chapulina/fortress"
`
