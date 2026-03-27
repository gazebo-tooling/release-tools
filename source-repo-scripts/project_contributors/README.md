# GitHub Contributor Generator

This project provides a set of tools to generate a list of contributors from
merged pull requests in a GitHub organization.

## Installation

It is recommended to use a virtual environment to manage dependencies:

```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -e .
```

## Prerequisites

This project requires the [GitHub CLI (`gh`)](https://cli.github.com/) to be
installed and authenticated. Please ensure you have it set up before running
the scripts.

## Usage

### 1. Get Merged Pull Requests

The `get-merged-prs` script fetches all merged pull requests for a given
GitHub organization and date range. It handles pagination and bypasses the
GitHub API's 1000-item search limit.

**Example:**

To get all merged pull requests for the `gazebosim` organization from
October 1, 2024, to September 23, 2025 (corresponding to the Jetty Release),
run the following command:

```bash
get-merged-prs gazebosim 2024-10-01 2025-09-23 > gazebosim-prs.json
```

This will create a `gazebosim-prs.json` file containing the merged pull
requests.

### 2. Generate Contributors List and Collage

The `generate-contributors` script can be used to generate a Markdown file
with a list of contributors or a collage of their avatars.

#### Generate Markdown List

To generate a Markdown file with the list of contributors from the
`gazebosim-prs.json` file, run:

```bash
generate-contributors md gazebosim-prs.json contributors.md
```

This will create a `contributors.md` file.

#### Generate Avatar Collage

To generate a collage of contributor avatars, you can use either the JSON
file or a list of usernames.

**From JSON file:**

```bash
generate-contributors collage --input-json gazebosim-prs.json \
  media/contributors.png --columns 15
```

**From a list of usernames:**

```bash
generate-contributors collage --usernames user1 user2 user3 \
  media/contributors.png --rows 5
```

This will create a `media/contributors.png` file with the collage of
avatars. You can use the `--rows` or `--columns` flags to specify the
layout of the collage.

### Example Collage

![Contributors Collage](media/contributors.png)
