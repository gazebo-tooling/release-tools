# README

This directory contains tools and data to create milestones on all Gazebo
libraries as it is not currently possible to share a milestone between different
Github repositories. Instead, we create a milestone with the same name on all
the repos and use queries like
[`milestone:"Jetty Release"`](https://github.com/search?q=org%3Agazebosim+milestone%3A%22Jetty+Release%22&type=pullrequests)
to filter pull request/issues. This is typically done during preparation for the
release of a collection (e.g. Gazebo Jetty).

Usage:

1. Install and configure the Github CLI tool [`gh`](https://cli.github.com/).
   Ensure you have write permission on all the Gazebo library repos.

1. Create a `json` file with the following format replacing `<collection>` and
   `<due date>`.

```json
{
  "title": "<collection> Release",
  "state": "open",
  "description": "Track pull requests that are meant to go into the release. Note that this also includes pull requests that get merged into older branches and get forward merged.",
  "due_on": "<due date>"
}
```

1. Run `create_milestones.bash` with the file created above as an argument.
