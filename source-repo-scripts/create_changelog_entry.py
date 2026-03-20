#!/usr/bin/env python3
"""
Create and optionally apply a validated changelog section from .changelog/*.md.

The script validates entries, discovers related PR metadata, auto-calculates the
next version, and asks for confirmation before mutating Changelog.md. If
confirmed, it optionally removes processed .changelog/ files via ``git rm`` and
commits all changes.
"""

import glob
import json
import os
import re
import subprocess
import sys
from datetime import datetime

CONVENTIONAL_COMMIT_HEADER_RE = re.compile(
    r"^(?P<type>[a-z]+)(\([^\r\n()]+\))?(?P<breaking>!)?: (?P<description>\S.*)$")
CHANGELOG_RELEASE_HEADER_RE = re.compile(
    r"^## (?P<project>.+) (?P<version>\d+\.\d+\.\d+) "
    r"\((?P<date>\d{4}-\d{2}-\d{2})\)$")
CHANGELOG_PLACEHOLDER_RE = re.compile(
    r"^## (?P<project>.+) (?P<major>\d+)\.[xX]$")


def first_non_blank_line(text):
    """Return the first non-blank line from *text*, stripped, or ``""``."""
    for line in text.split('\n'):
        stripped = line.strip()
        if stripped:
            return stripped
    return ""


def run_subprocess(cmd, timeout=10, context=""):
    """
    Run a subprocess and return its CompletedProcess, or None on error.

    On failure an appropriate warning is printed to stderr.

    Args:
        cmd (list): Command and arguments.
        timeout (int): Timeout in seconds.
        context (str): Human-readable label used in warning messages.

    Returns:
        subprocess.CompletedProcess or None: The result, or None when the
        command could not be executed at all.
    """
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired as e:
        print(f"Warning: {context} timed out: {e}", file=sys.stderr)
    except FileNotFoundError as e:
        print(f"Warning: {context} command not found: {e}", file=sys.stderr)
    except OSError as e:
        print(f"Warning: {context} failed: {e}", file=sys.stderr)
    return None


def parse_changelog_header(changelog_path):
    """
    Parse project name and current version from the first release header.

    Args:
        changelog_path (str): Path to Changelog.md.

    Returns:
        tuple[str, str]: (project_name, current_version)

    Raises:
        RuntimeError: If the changelog file cannot be read.
        ValueError: If no valid release header is found.
    """
    try:
        with open(changelog_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except OSError as e:
        raise RuntimeError(
            f"Could not read changelog file {changelog_path}: {e}") from e

    for line in lines:
        match = CHANGELOG_RELEASE_HEADER_RE.match(line.strip())
        if match:
            return match.group("project"), match.group("version")

    # No dated release found — fall back to placeholder header to extract
    # project name and major version, using <major>.0.0 as the base.
    for line in lines:
        match = CHANGELOG_PLACEHOLDER_RE.match(line.strip())
        if match:
            major = match.group("major")
            return match.group("project"), f"{major}.0.0"

    raise ValueError(
        f"Could not determine current version from {changelog_path}. "
        "Expected a release header like "
        "'## <project> <major>.<minor>.<patch> (YYYY-MM-DD)' "
        "or a placeholder like '## <project> <major>.x'")


def validate_changelog_entry(entry_content, filename):
    """
    Validate that the changelog entry follows Conventional Commits format.

    Args:
        entry_content (str): Changelog entry content without template comments.
        filename (str): File name used for reporting validation errors.

    Returns:
        str or None: None when valid, otherwise a validation error message.
    """
    first_line = first_non_blank_line(entry_content)

    if not first_line:
        return f"{filename}: entry is empty after removing template comments"

    if not CONVENTIONAL_COMMIT_HEADER_RE.match(first_line):
        return (
            f"{filename}: first non-comment line must follow Conventional Commits "
            f"format '<type>(<optional-scope>)!: <description>' "
            f"(both scope and '!' are optional), got '{first_line}'")

    return None


def read_changelog_entries(changelog_dir):
    """
    Read all markdown files in the changelog directory.

    Args:
        changelog_dir (str): Path to the changelog directory.

    Returns:
        tuple[list[str], list[str]]: A pair of (entries, errors). Each entry
        is the validated changelog text, potentially with an appended PR-link
        or commit-reference line. Errors contains human-readable validation
        failure messages.
    """
    entries = []
    errors = []

    pattern = os.path.join(changelog_dir, '*.md')
    files = sorted(glob.glob(pattern))

    for file_path in files:
        filename = os.path.basename(file_path)

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read().strip()
        except OSError as e:
            errors.append(f"{filename}: could not be read ({e})")
            continue

        # Remove template comment lines (lines whose first non-whitespace
        # character is '#').
        lines = content.split('\n')
        entry_lines = [line for line in lines if not line.strip().startswith('#')]
        entry_content = '\n'.join(entry_lines).strip()

        validation_error = validate_changelog_entry(entry_content, filename)
        if validation_error is not None:
            errors.append(validation_error)
            continue

        pr_info = get_pr_info(file_path)
        if pr_info:
            entries.append(f"{entry_content}\n{pr_info}")
        else:
            entries.append(entry_content)

    return entries, errors


def calculate_next_version(entries, current_version):
    """
    Calculate the next version using changelog entries and the current version.

    Args:
        entries (list[str]): Validated changelog entry texts.
        current_version (str): Current semver version string (``X.Y.Z``).

    Returns:
        str: The calculated next semver version.

    Raises:
        ValueError: If *current_version* is not valid semver, if an entry
            header cannot be parsed, or if any entry uses the breaking
            marker (``!``).

    Rules:
      - Entries with breaking marker '!' are unsupported and fail fast.
      - If any changelog entry is a feature (feat), bump minor and reset patch.
      - Otherwise, bump patch.
    """
    try:
        major, minor, patch = map(int, current_version.split('.'))
    except ValueError as e:
        raise ValueError(
            f"Current version '{current_version}' is not valid semver") from e

    parsed_headers = []
    for entry in entries:
        first_line = first_non_blank_line(entry)
        match = CONVENTIONAL_COMMIT_HEADER_RE.match(first_line)
        if not match:
            raise ValueError(
                f"Could not parse changelog entry header: '{first_line}'")
        parsed_headers.append(match)

    if any(match.group("breaking") for match in parsed_headers):
        raise ValueError(
            "Entries with breaking marker '!' are not supported by automatic "
            "versioning. Please remove '!' entries or handle versioning manually.")

    has_feature = any(match.group("type") == "feat" for match in parsed_headers)
    if has_feature:
        minor += 1
        patch = 0
    else:
        patch += 1

    return f"{major}.{minor}.{patch}"


def format_markdown_list_item(entry):
    """
    Format a changelog entry as a markdown bullet preserving multiline structure.

    Args:
        entry (str): Changelog entry text, potentially multiline.

    Returns:
        str: Markdown list item with indented continuation lines.
    """
    lines = entry.split('\n')
    formatted_lines = [f"* {lines[0]}"]
    for line in lines[1:]:
        formatted_lines.append(f"  {line}" if line else "")
    return '\n'.join(formatted_lines)


def generate_changelog_entry(entries, project_name, version):
    """
    Generate a changelog entry from entries.

    Args:
        entries (list): List of changelog entries with optional PR metadata.
        project_name (str): Changelog project name.
        version (str): Semver version for the release.

    Returns:
        str: Formatted changelog entry.
    """
    today = datetime.now().strftime("%Y-%m-%d")

    changelog_lines = [
        f"## {project_name} {version} ({today})",
        ""
    ]

    for entry in entries:
        changelog_lines.append(format_markdown_list_item(entry))

    return '\n'.join(changelog_lines)


def update_changelog(changelog_path, new_entry):
    """
    Update the changelog file with the new entry.

    Args:
        changelog_path (str): Path to the changelog file.
        new_entry (str): New changelog entry to add.

    Raises:
        RuntimeError: If file I/O fails while reading or writing changelog.
        ValueError: If no valid insertion anchor can be found.
    """
    try:
        with open(changelog_path, 'r', encoding='utf-8') as f:
            current_content = f.read()
    except OSError as e:
        raise RuntimeError(f"Could not read {changelog_path}: {e}") from e

    lines = current_content.split('\n')
    insert_index = None

    # Prefer inserting after the top placeholder header (e.g. "## Gazebo Math 8.x").
    for i, line in enumerate(lines):
        if CHANGELOG_PLACEHOLDER_RE.match(line.strip()):
            insert_index = i + 1
            while insert_index < len(lines) and lines[insert_index].strip() == '':
                insert_index += 1
            break

    # Otherwise insert before the first dated release header.
    if insert_index is None:
        for i, line in enumerate(lines):
            if CHANGELOG_RELEASE_HEADER_RE.match(line.strip()):
                insert_index = i
                break

    if insert_index is None:
        raise ValueError(
            f"Could not find insertion point in {changelog_path}. "
            "Expected either a '## ... N.x' placeholder header or a dated release header.")

    lines.insert(insert_index, new_entry)
    lines.insert(insert_index + 1, "")

    try:
        with open(changelog_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))
    except OSError as e:
        raise RuntimeError(f"Could not write updated changelog to {changelog_path}: {e}") from e

    print(f"Successfully updated {changelog_path}")


def get_pr_info(file_path):
    """
    Get PR information for a changelog file using GitHub CLI.

    Args:
        file_path (str): Full path to the .changelog/ file. Used to identify
            the git commit that introduced the file, which is then used to
            locate the associated merged PR.

    Returns:
        str: Formatted PR link (``    * [Pull request #N](url)``) when found,
            commit SHA reference (``    * Commit: {sha}``) as fallback when
            the commit is known but no PR is found, or empty string when the
            originating commit cannot be determined.
    """
    filename = os.path.basename(file_path)

    # Get the SHA of the commit that created the file using git log.
    git_cmd = ["git", "log", "--follow", "--diff-filter=A", "--format=%H", "--", file_path]
    git_result = run_subprocess(git_cmd, context=f"git log for {filename}")
    if git_result is None:
        return ""

    if git_result.returncode != 0:
        stderr = git_result.stderr.strip()
        print(
            f"Warning: git log returned exit code {git_result.returncode} "
            f"for {filename}{': ' + stderr if stderr else ''}",
            file=sys.stderr)
        return ""

    if not git_result.stdout.strip():
        print(
            f"Info: no git history found for {filename} "
            "(file may not be committed yet)",
            file=sys.stderr)
        return ""

    commit_sha = git_result.stdout.strip().split('\n')[-1]
    print(f"Found commit SHA: {commit_sha} for file {filename}")

    # Use gh CLI to search for merged PRs containing the commit SHA.
    cmd = ["gh", "pr", "list", "--state=merged", "--search", commit_sha, "--json", "number,url"]
    result = run_subprocess(cmd, context=f"gh pr list for {commit_sha}")
    if result is None:
        return f"    * Commit: {commit_sha}"

    if result.returncode != 0:
        stderr = result.stderr.strip()
        print(
            f"Warning: gh pr list returned exit code {result.returncode} "
            f"for {commit_sha}{': ' + stderr if stderr else ''}",
            file=sys.stderr)
        return f"    * Commit: {commit_sha}"

    if not result.stdout.strip():
        return f"    * Commit: {commit_sha}"

    try:
        prs = json.loads(result.stdout)
        if prs:
            pr = prs[0]
            pr_number = pr["number"]
            pr_url = pr["url"]
            print(f"Found PR #{pr_number} for commit {commit_sha}")
            return f"    * [Pull request #{pr_number}]({pr_url})"
    except json.JSONDecodeError as e:
        print(
            f"Warning: gh returned invalid JSON for {commit_sha}: {e}",
            file=sys.stderr)
    except (KeyError, TypeError) as e:
        print(
            f"Warning: unexpected PR metadata structure for {commit_sha} "
            f"({type(e).__name__}: {e})",
            file=sys.stderr)

    return f"    * Commit: {commit_sha}"


def main():
    """Main function to create changelog entry."""
    changelog_dir = os.path.abspath('.changelog')
    changelog_path = os.path.abspath('Changelog.md')

    # Verify we are inside a git repository.
    try:
        subprocess.run(
            ["git", "rev-parse", "--git-dir"],
            capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("Error: this script must be run inside a git repository",
              file=sys.stderr)
        return 1

    if not os.path.exists(changelog_dir):
        print(f"Error: Changelog directory {changelog_dir} not found",
              file=sys.stderr)
        return 1

    if not os.path.exists(changelog_path):
        print(f"Error: Changelog file {changelog_path} not found",
              file=sys.stderr)
        return 1

    try:
        project_name, current_version = parse_changelog_header(changelog_path)
    except (RuntimeError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

    print(f"Reading changelog entries from {changelog_dir}...")
    entries, entry_errors = read_changelog_entries(changelog_dir)

    if entry_errors:
        print("Error: invalid changelog entries detected:", file=sys.stderr)
        for entry_error in entry_errors:
            print(f"  - {entry_error}", file=sys.stderr)
        return 1

    if not entries:
        print("No changelog entries found!", file=sys.stderr)
        return 1

    print("Generating changelog entry...")
    try:
        new_version = calculate_next_version(entries, current_version)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    print(f"Calculated next version: {new_version}")
    new_entry = generate_changelog_entry(entries, project_name, new_version)

    print("Generated changelog entry:")
    print("-" * 50)
    print(new_entry)
    print("-" * 50)

    response = input("Update Changelog.md with this entry? (y/N): ")
    if response.lower() not in ['y', 'yes']:
        print("Changelog update cancelled.")
        return 0

    try:
        update_changelog(changelog_path, new_entry)
    except (RuntimeError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

    try:
        subprocess.run(["git", "add", changelog_path], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: could not stage {changelog_path}: {e}",
              file=sys.stderr)
        return 1

    cleanup_response = input("Remove processed changelog entry files? (y/N): ")
    if cleanup_response.lower() in ['y', 'yes']:
        files = glob.glob(os.path.join(changelog_dir, '*.md'))
        for file_path in files:
            try:
                subprocess.run(["git", "rm", file_path], check=True)
                print(f"Removed {file_path}")
            except subprocess.CalledProcessError as e:
                print(f"Error: could not remove {file_path}: {e}",
                      file=sys.stderr)
                return 1

    try:
        subprocess.run(
            ["git", "commit", "-m",
             f"Generate changelog entry for version {new_version}"],
            check=True)
        print("Committed changelog updates")
    except subprocess.CalledProcessError as e:
        print(f"Error: could not commit changelog updates: {e}",
              file=sys.stderr)
        return 1

    return 0


if __name__ == '__main__':
    exit(main())
