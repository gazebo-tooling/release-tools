# Stable Branch CI Workflow

This document explains the continuous integration (CI) process for stable branches, which are the main development branches for libraries within a given Gazebo collection. These jobs are designed to run on a regular schedule and after any new commits are merged to ensure the branch is always in a healthy state.

## 1. Job Definition and Triggering

The jobs for stable branches are defined and generated using the same core infrastructure as the pull request (PR) jobs, but with important differences in naming and triggering. The entire process is orchestrated by `jenkins-scripts/dsl/gazebo_libs.dsl`, which reads its configuration from `jenkins-scripts/dsl/gz-collections.yaml`.

### Job Generation

Within the `gazebo_libs.dsl` script, a specific block of code is responsible for creating stable branch jobs. It checks if the `stable_branches` category is enabled for a given CI configuration:

```groovy
// In jenkins-scripts/dsl/gazebo_libs.dsl
...
if (categories_enabled.contains('stable_branches')) {
  // Logic to generate a stable branch job
}
```

If the condition is met, the script generates a job with a name that explicitly includes the stable branch name. Using `gz-math` from the **Jetty** collection (which uses the `gz-math9` branch) as an example, the job names are:

-   **Linux:** `gz_math-ci-gz-math9-noble-amd64`
-   **macOS:** `gz_math-ci-gz-math9-homebrew-amd64`
-   **Windows:** `gz_math-9-cnlwin`

Unlike PR jobs, these jobs are configured to check out one specific, hardcoded branch (e.g., `gz-math9`).

### The Trigger: Scheduled and On-Commit

The trigger mechanism is the primary difference from the PR workflow. Instead of listening for GitHub PR events, these jobs use a standard Jenkins SCM (Source Code Management) trigger.

The DSL script adds the following `triggers` block to each generated job:
```groovy
// In jenkins-scripts/dsl/gazebo_libs.dsl
gz_ci_job.with
{
  triggers {
    scm('@daily')
  }
}
```

This configuration enables two trigger conditions simultaneously:
1.  **On-Commit:** The `scm` trigger configures Jenkins to poll the Git repository. When it detects a new commit on the configured branch (for example, after a PR is merged into `gz-math9`), it automatically starts a new build.
2.  **Scheduled:** The `@daily` cron expression tells Jenkins to run the job once a day, even if no new commits have been pushed. This ensures the code is continuously tested against its dependencies on a regular basis.

## 2. Dependency Resolution (Identical to PR Workflow)

Once a stable branch job is triggered, the process for building the code and resolving dependencies is **exactly the same** as the process for a pull request job. The generated job calls the same platform-specific build scripts, which handle dependency resolution as follows.

### Linux: Dependencies Pre-installed in a Dynamic Dockerfile
1.  **Script Chain:** A chain of scripts (`...-compilation.bash` -> `generic-building-base.bash` -> `docker_generate_dockerfile.bash`) assembles a master list of required `.deb` packages.
2.  **Dependency Sources:** This list is aggregated from multiple sources:
    - Library-specific lists defined in `jenkins-scripts/lib/dependencies_archive.sh` (e.g., `GZ_MATH_DEPENDENCIES`).
    - Common packages defined in `BASE_DEPENDENCIES`.
    - Packages listed in `packages.apt` files within the library's own source code.
3.  **Dockerfile Generation:** The `docker_generate_dockerfile.bash` script writes a new `Dockerfile`, injecting the master package list into an `apt-get install` command. The job then builds this image and runs the compilation inside the container, where all dependencies are already present.

### Windows: Building Dependencies from Source
1.  **Baseline via `pixi`/`conda`:** The `colcon-default-devel-windows.bat` script first installs large, third-party libraries (Boost, OGRE, etc.) as pre-built binaries from `conda`.
2.  **Checkout from `gazebodistro`:** It then uses `vcs import` to read the appropriate `.yaml` file from the `gazebodistro` repository (e.g., `gz-math9.yaml`). This clones the source code for **all Gazebo dependencies** into the workspace.
3.  **Build Workspace:** `colcon build` is used to compile the entire workspace from source.

### macOS: Pre-built Homebrew Packages ("Bottles")
1.  **Formula-based:** The `project-default-devel-homebrew-amd64.bash` script relies on Homebrew formulas from the `osrf/simulation` tap.
2.  **`brew install`:** It runs `brew install gz-math9 --only-dependencies`. Homebrew then reads the formula for `gz-math9` and installs all its listed dependencies, downloading pre-compiled "bottles" when available. `gazebodistro` is not used.
