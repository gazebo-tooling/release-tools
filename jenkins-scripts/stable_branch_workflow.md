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

Once a stable branch job is triggered, the process for building the code and resolving dependencies is **exactly the same** as the process for a pull request job. See the Dependency Resolution in the [pr_job_workflow.md](pr_job_workflow.md) document.
