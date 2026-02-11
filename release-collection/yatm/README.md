# Collection release management using Github Project

This directory contains data and tools used to create github issues for managing
a Gazebo collection release in a Github Project.

## Process for creating github issues

The YATMv2 tool is used to create github issues from requirement files. The
requirement files are where the actual issues are kept.

- We assign labels to the issues to simplify the process of assigning
  milestones. This is mainly because the YATM tool does not support assigning
  milestones.

- When creating the issues for a new release for the first time, you need to
  first create the labels defined in `config.yaml`.
  - Run `yatm_v2 github utils create-labels`

- Preview the issues using `yatm_v2 github preview`. This will generate a file in the `generated_files` directory containing the preview markdown.

- Create the issues using `yatm_v2 github upload`

- Once the issues are created, filter by label and assign milestones manually.
