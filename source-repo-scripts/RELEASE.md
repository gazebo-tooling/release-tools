# Source repositories scripts

The directory hosts the scripts designed to work with Gazebo source
repositories, the repositories hosting source code, hosted under
https://github.com/gazebosim/.

## Scripts available

### source_changelog.bash

Generates a changelog entry for unreleased changes in a release branch.

#### Usage

```bash
cd <path to source repo>
source_changelog.bash <previous release version>
```

The script prints the changelog entry to the console. The user must copy that into
`Changelog.md` and make any necessary adjustments before commiting.

#### Example

TODO after https://github.com/ignition-tooling/release-tools/pull/541

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
```

