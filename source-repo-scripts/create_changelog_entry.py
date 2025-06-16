#!/usr/bin/env python3
"""
Create and optionally apply a validated changelog section from .changelog/*.md.

The script validates entries, discovers related PR metadata, auto-calculates the
next version, and asks for confirmation before mutating Changelog.md.
"""

import os
import glob
import subprocess
import json
import re
import sys
from datetime import datetime

CONVENTIONAL_COMMIT_HEADER_RE = re.compile(
    r"^(?P<type>[a-z]+)(\([^\r\n()]+\))?(?P<breaking>!)?: (?P<description>\S.*)$")
CHANGELOG_RELEASE_HEADER_RE = re.compile(
    r"^## (?P<project>.+) (?P<version>\d+\.\d+\.\d+) "
    r"\((?P<date>\d{4}-\d{2}-\d{2})\)$")


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
            changelog_content = f.read()
    except OSError as e:
        raise RuntimeError(
            f"Could not read changelog file {changelog_path}: {e}") from e

    for line in changelog_content.split('\n'):
        match = CHANGELOG_RELEASE_HEADER_RE.match(line.strip())
        if match:
            return match.group("project"), match.group("version")

    raise ValueError(
        f"Could not determine current version from {changelog_path}. "
        "Expected a header like "
        "'## <project> <major>.<minor>.<patch> (YYYY-MM-DD)'")


def validate_changelog_entry(entry_content, filename):
    """
    Validate that the changelog entry follows Conventional Commits format.

    Args:
        entry_content (str): Changelog entry content without template comments
        filename (str): File name used for reporting validation errors

    Returns:
        str: Empty string when valid, otherwise a validation error message
    """
    first_line = ""
    for line in entry_content.split('\n'):
        if line.strip():
            first_line = line.strip()
            break

    if not first_line:
        return f"{filename}: entry is empty after removing template comments"

    if not CONVENTIONAL_COMMIT_HEADER_RE.match(first_line):
        return (
            f"{filename}: first non-comment line must follow Conventional Commits "
            f"format '<type>(<optional-scope>)!: <description>' "
            f"(both scope and '!' are optional), got '{first_line}'")

    return ""


def read_changelog_entries(changelog_dir):
    """
    Read all markdown files in the changelog directory.

    Args:
        changelog_dir (str): Path to the changelog directory

    Returns:
        tuple: (list of entries with PR metadata, list of validation errors)
    """
    entries = []
    errors = []

    # Find all .md files in the changelog directory
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

        # Skip template comment lines (lines starting with '#')
        lines = content.split('\n')
        entry_lines = [line for line in lines if not line.strip().startswith('#')]
        entry_content = '\n'.join(entry_lines).strip()

        validation_error = validate_changelog_entry(entry_content, filename)
        if validation_error:
            errors.append(validation_error)
            continue

        pr_info = get_pr_info(filename)
        if pr_info:
            entries.append(f"{entry_content}\n{pr_info}")
        else:
            entries.append(entry_content)

    return entries, errors


def calculate_next_version(entries, current_version):
    """
    Calculate the next version using changelog entries and the current version.

    Rules:
      - Entries with breaking marker '!' are currently unsupported and fail fast.
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
        first_line = entry.split('\n', 1)[0].strip()
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
        formatted_lines.append(f"  {line}" if line else "  ")
    return '\n'.join(formatted_lines)


def generate_changelog_entry(entries, project_name, version):
    """
    Generate a changelog entry from entries.

    Args:
        entries (list): List of changelog entries with optional PR metadata
        project_name (str): Changelog project name
        version (str): Semver version for the release

    Returns:
        str: Formatted changelog entry
    """
    today = datetime.now().strftime("%Y-%m-%d")

    changelog_lines = [
        f"## {project_name} {version} ({today})",
        ""
    ]

    # Add all entries
    for entry in entries:
        changelog_lines.append(format_markdown_list_item(entry))

    if entries:
        changelog_lines.append("")

    return '\n'.join(changelog_lines)


def update_changelog(changelog_path, new_entry):
    """
    Update the changelog file with the new entry.

    Args:
        changelog_path (str): Path to the changelog file
        new_entry (str): New changelog entry to add

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

    # Prefer inserting after the top placeholder header (e.g. "## Project 0.x").
    for i, line in enumerate(lines):
        if line.startswith('## ') and '0.x' in line and '(' not in line:
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
            "Expected either a '## ... 0.x' placeholder header or a dated release header.")

    lines.insert(insert_index, new_entry)
    lines.insert(insert_index + 1, "")

    try:
        with open(changelog_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))
    except OSError as e:
        raise RuntimeError(f"Could not write updated changelog to {changelog_path}: {e}") from e

    print(f"Successfully updated {changelog_path}")


def get_pr_info(filename):
    """
    Get PR information for a changelog file using GitHub CLI.

    Args:
        filename (str): The filename to search for in merged PRs

    Returns:
        str: PR information string or empty string if not found
    """
    # Get the SHA of the commit that created the file using git log.
    file_path = os.path.join('.changelog', filename)
    git_cmd = ["git", "log", "--follow", "--diff-filter=A", "--format=%H", "--", file_path]
    try:
        git_result = subprocess.run(git_cmd, capture_output=True, text=True, timeout=10)
    except subprocess.TimeoutExpired as e:
        print(f"Warning: git log timed out while resolving {filename}: {e}", file=sys.stderr)
        return ""
    except FileNotFoundError as e:
        print(f"Warning: git command not found while resolving {filename}: {e}", file=sys.stderr)
        return ""
    except OSError as e:
        print(f"Warning: git log failed while resolving {filename}: {e}", file=sys.stderr)
        return ""

    if git_result.returncode != 0:
        stderr = git_result.stderr.strip()
        if stderr:
            print(
                f"Warning: git log failed for {filename}: {stderr}",
                file=sys.stderr)
        return ""

    if not git_result.stdout.strip():
        return ""

    commit_sha = git_result.stdout.strip().split('\n')[-1]
    print(f"Found commit SHA: {commit_sha} for file {filename}")

    # Use gh CLI to search for merged PRs containing the commit SHA.
    cmd = ["gh", "pr", "list", "--state=merged", "--search", commit_sha, "--json", "number,url"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    except subprocess.TimeoutExpired as e:
        print(f"Warning: gh pr list timed out for {commit_sha}: {e}", file=sys.stderr)
        return f"    * Commit: {commit_sha}"
    except FileNotFoundError as e:
        print(f"Warning: gh command not found for {commit_sha}: {e}", file=sys.stderr)
        return f"    * Commit: {commit_sha}"
    except OSError as e:
        print(f"Warning: gh pr list failed for {commit_sha}: {e}", file=sys.stderr)
        return f"    * Commit: {commit_sha}"

    if result.returncode != 0:
        stderr = result.stderr.strip()
        if stderr:
            print(
                f"Warning: gh pr list failed for {commit_sha}: {stderr}",
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
    except (json.JSONDecodeError, KeyError, TypeError) as e:
        print(
            f"Warning: could not parse PR metadata for {commit_sha}: {e}",
            file=sys.stderr)

    return f"    * Commit: {commit_sha}"


def main():
    """Main function to create changelog entry."""
    changelog_dir = os.path.abspath('.changelog')
    changelog_path = os.path.abspath('Changelog.md')

    if not os.path.exists(changelog_dir):
        print(f"Error: Changelog directory {changelog_dir} not found")
        return 1

    if not os.path.exists(changelog_path):
        print(f"Error: Changelog file {changelog_path} not found")
        return 1

    try:
        project_name, current_version = parse_changelog_header(changelog_path)
    except (RuntimeError, ValueError) as e:
        print(f"Error: {e}")
        return 1

    # Read changelog entries
    print(f"Reading changelog entries from {changelog_dir}...")
    entries, entry_errors = read_changelog_entries(changelog_dir)

    if entry_errors:
        print("Error: invalid changelog entries detected:")
        for entry_error in entry_errors:
            print(f"  - {entry_error}")
        return 1

    if not entries:
        print("No changelog entries found!")
        return 1

    # Generate new changelog entry
    print("Generating changelog entry...")
    try:
        new_version = calculate_next_version(entries, current_version)
    except ValueError as e:
        print(f"Error: {e}")
        return 1
    print(f"Calculated next version: {new_version}")
    new_entry = generate_changelog_entry(entries, project_name, new_version)

    print("Generated changelog entry:")
    print("-" * 50)
    print(new_entry)
    print("-" * 50)

    # Ask for confirmation
    response = input("Update Changelog.md with this entry? (y/N): ")
    if response.lower() in ['y', 'yes']:
        try:
            update_changelog(changelog_path, new_entry)
        except (RuntimeError, ValueError) as e:
            print(f"Error: {e}")
            return 1

        # Ask if user wants to clean up the changelog entry files
        cleanup_response = input("Remove processed changelog entry files? (y/N): ")
        if cleanup_response.lower() in ['y', 'yes']:
            files = glob.glob(os.path.join(changelog_dir, '*.md'))
            for file_path in files:
                try:
                    subprocess.run(["git", "rm", file_path], check=True)
                    print(f"Removed {file_path}")
                except Exception as e:
                    print(f"Warning: Could not remove {file_path}: {e}")
    else:
        print("Changelog not updated.")

    return 0


if __name__ == '__main__':
    exit(main())
