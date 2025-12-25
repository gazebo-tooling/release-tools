#!/usr/bin/env python3
"""
Script to create a new changelog entry from files in .changelog directory.
"""

import os
import glob
import subprocess
import json
from datetime import datetime


def read_changelog_entries(changelog_dir):
    """
    Read all markdown files in the changelog directory.

    Args:
        changelog_dir (str): Path to the changelog directory

    Returns:
        list: List of all entries with PR information
    """
    entries = []

    # Find all .md files in the changelog directory
    pattern = os.path.join(changelog_dir, '*.md')
    files = glob.glob(pattern)

    for file_path in files:
        filename = os.path.basename(file_path)

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read().strip()

                # Skip Markdown comments
                lines = content.split('\n')
                entry_lines = [line for line in lines if not line.strip().startswith('#')]
                entry_content = '\n'.join(entry_lines).strip()

                if entry_content:
                    pr_info = get_pr_info(filename)

                    if pr_info:
                        formatted_entry = f"{entry_content}\n{pr_info}"
                    else:
                        formatted_entry = entry_content

                    entries.append(formatted_entry)

        except Exception as e:
            print(f"Warning: Could not read {file_path}: {e}")

    return entries


def generate_changelog_entry(entries, changelog_path, version="0.x.x"):
    """
    Generate a changelog entry from entries.

    Args:
        entries (list): List of entries
        changelog_path (str): Path to the changelog file to extract project name
        version (str): Version number for the release

    Returns:
        str: Formatted changelog entry
    """
    today = datetime.now().strftime("%Y-%m-%d")

    project_name = ""
    try:
        with open(changelog_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line.startswith('## ') and '(' in line:
                    # Extract project name from header like "## Gazebo Test 0.x.x (2025-06-16)"
                    header_part = line[3:]  # Remove "## "
                    if '(' in header_part:
                        project_part = header_part.split('(')[0].strip()
                        # Remove version number from the end
                        parts = project_part.split()
                        if len(parts) >= 2:
                            # Join all parts except the last one (which should be version)
                            project_name = ' '.join(parts[:-1])
                    break
    except Exception as e:
        print(f"Warning: Could not extract project name from {changelog_path}: {e}")

    changelog_lines = [
        f"## {project_name} {version} ({today})",
        ""
    ]

    # Add all entries
    for entry in entries:
        changelog_lines.append(f"* {entry}")

    if entries:
        changelog_lines.append("")

    return '\n'.join(changelog_lines)


def update_changelog(changelog_path, new_entry):
    """
    Update the changelog file with the new entry.

    Args:
        changelog_path (str): Path to the changelog file
        new_entry (str): New changelog entry to add
    """
    try:
        # Read the current changelog
        with open(changelog_path, 'r', encoding='utf-8') as f:
            current_content = f.read()

        # Find where to insert the new entry (after the first header)
        lines = current_content.split('\n')
        insert_index = len(lines)  # Default to end of file

        # Find the first header (like "## Gazebo Foo 0.x") to insert after it
        for i, line in enumerate(lines):
            if line.startswith('## ') and '0.x' in line and '(' not in line:
                # Found the main header without date, insert after it
                insert_index = i + 1
                # Skip any empty lines after the header
                while insert_index < len(lines) and lines[insert_index].strip() == '':
                    insert_index += 1
                break

        # Insert the new entry
        lines.insert(insert_index, new_entry)
        lines.insert(insert_index + 1, "")  # Add blank line

        # Write back to file
        updated_content = '\n'.join(lines)
        with open(changelog_path, 'w', encoding='utf-8') as f:
            f.write(updated_content)

        print(f"Successfully updated {changelog_path}")

    except Exception as e:
        print(f"Error updating changelog: {e}")


def get_pr_info(filename):
    """
    Get PR information for a changelog file using GitHub CLI.

    Args:
        filename (str): The filename to search for in merged PRs

    Returns:
        str: PR information string or empty string if not found
    """
    try:
        # Use gh CLI to search for merged PRs containing the filename
        # Get the SHA of the commit that created the file
        file_path = os.path.join('.changelog', filename)
        git_cmd = ["git", "log", "--follow", "--diff-filter=A", "--format=%H", "--", file_path]
        git_result = subprocess.run(git_cmd, capture_output=True, text=True, timeout=10)

        if git_result.returncode != 0 or not git_result.stdout.strip():
            return ""

        commit_sha = git_result.stdout.strip().split('\n')[-1]  # Get the last (oldest) commit
        print(f"Found commit SHA: {commit_sha} for file {filename}")

        # Use gh CLI to search for merged PRs containing the commit SHA
        cmd = ["gh", "pr", "list", "--state=merged", "--search", commit_sha, "--json", "number,url"]
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)

        if result.returncode == 0 and result.stdout.strip():
            prs = json.loads(result.stdout)
            if prs:
                # Get the first (most recent) PR
                pr = prs[0]
                pr_number = pr["number"]
                pr_url = pr["url"]
                print(f"Found PR #{pr_number} for commit {commit_sha}")
                return f"    * [Pull request #{pr_number}]({pr_url})"

    except (json.JSONDecodeError, KeyError):
        # Handle any errors by returning empty string (omit PR info)
        return ""

    return f"    * [Commit #{commit_sha}]({commit_sha})"


def main():
    """Main function to create changelog entry."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    changelog_dir = os.path.join(script_dir, '.changelog')
    changelog_path = os.path.join(script_dir, 'Changelog.md')

    if not os.path.exists(changelog_dir):
        print(f"Error: Changelog directory {changelog_dir} not found")
        return 1

    if not os.path.exists(changelog_path):
        print(f"Error: Changelog file {changelog_path} not found")
        return 1

    # Read changelog entries
    print(f"Reading changelog entries from {changelog_dir}...")
    entries = read_changelog_entries(changelog_dir)

    if not entries:
        print("No changelog entries found!")
        return 1

    # Generate new changelog entry
    print("Generating changelog entry...")
    new_entry = generate_changelog_entry(entries, changelog_path)

    print("Generated changelog entry:")
    print("-" * 50)
    print(new_entry)
    print("-" * 50)

    # Ask for confirmation
    response = input("Update Changelog.md with this entry? (y/N): ")
    if response.lower() in ['y', 'yes']:
        update_changelog(changelog_path, new_entry)

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
