#!/usr/bin/env python3
"""
Create and optionally apply a validated changelog section from .changelog/.

The script validates entries, discovers related PR metadata, auto-calculates the
next version, and asks for confirmation before mutating Changelog.md. If
confirmed, it stages Changelog.md, removes the processed .changelog/ files via
``git rm`` (so the next release does not publish them again), and creates a
signed-off commit containing only those paths. The git index, Changelog.md and
the .changelog/ entry files must be clean (committed) before running it.
"""

import json
import os
import re
import subprocess
import sys
from datetime import date

CONVENTIONAL_COMMIT_HEADER_RE = re.compile(
    r"^(?P<type>[a-z]+)(\([^\r\n()]+\))?(?P<breaking>!)?: (?P<description>\S.*)$")
# Gazebo changelogs group releases under a "## <project> N.x" placeholder
# with "### <project> X.Y.Z (YYYY-MM-DD)" release headers. Older files (and
# some repos) use "##" for releases or "###" for the placeholder, so accept
# both heading levels. Unreleased sections use dates like "20XX-XX-XX".
CHANGELOG_RELEASE_HEADER_RE = re.compile(
    r"^(?P<level>#{2,3}) (?P<project>.+) (?P<version>\d+\.\d+\.\d+) "
    r"\((?P<date>[^)]*)\)$")
CHANGELOG_PLACEHOLDER_RE = re.compile(
    r"^(?P<level>#{2,3}) (?P<project>.+) (?P<major>\d+)\.[xX]$")
RELEASE_DATE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}$")
# GitHub appends "(#N)" to squash-merge subjects; rebase and merge-commit
# merges keep the subject of the original commit, and a backport cherry-pick
# keeps the subject of the commit merged on the source branch.
SUBJECT_PR_REF_RE = re.compile(r"\(#(?P<number>\d+)\)\s*$")
CHERRY_PICK_TRAILER_RE = re.compile(
    r"^\(cherry picked from commit (?P<sha>[0-9a-f]{7,40})\)$", re.M)
# Template comments are "# ..." lines or a lone "#"; body text such as
# "#123 is fixed too" is kept.
TEMPLATE_COMMENT_RE = re.compile(r"^\s*#(\s|$)")


def first_non_blank_line(text):
    """Return the first non-blank line from *text*, stripped, or ``""``."""
    for line in text.split('\n'):
        stripped = line.strip()
        if stripped:
            return stripped
    return ""


def run_subprocess(cmd, timeout=10, context="", quiet=False):
    """
    Run a subprocess and return its CompletedProcess, or None on error.

    On failure an appropriate warning is printed to stderr.

    Args:
        cmd (list): Command and arguments.
        timeout (int): Timeout in seconds.
        context (str): Human-readable label used in warning messages.
        quiet (bool): Do not warn when the command exits non-zero.

    Returns:
        subprocess.CompletedProcess or None: The result, or None when the
        command could not be executed or exited non-zero.
    """
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired as e:
        print(f"Warning: {context} timed out: {e}", file=sys.stderr)
        return None
    except FileNotFoundError as e:
        print(f"Warning: {context} command not found: {e}", file=sys.stderr)
        return None
    except OSError as e:
        print(f"Warning: {context} failed: {e}", file=sys.stderr)
        return None

    if result.returncode != 0:
        if not quiet:
            stderr = result.stderr.strip()
            print(
                f"Warning: {context} returned exit code {result.returncode}"
                f"{': ' + stderr if stderr else ''}",
                file=sys.stderr)
        return None
    return result


def read_changelog(changelog_path):
    """
    Return the lines of *changelog_path*, without line endings.

    Raises:
        RuntimeError: If the changelog file cannot be read.
    """
    try:
        with open(changelog_path, 'r', encoding='utf-8') as f:
            return f.read().split('\n')
    except OSError as e:
        raise RuntimeError(
            f"Could not read changelog file {changelog_path}: {e}") from e


def parse_changelog_header(changelog_lines, changelog_path):
    """
    Parse project name and current version from Changelog.md.

    The current version is the first release header inside the top
    "<project> N.x" placeholder section (up to the next placeholder). Its
    major version must match N. If the file has no placeholder at all, the
    first release header in the file is used.

    Args:
        changelog_lines (list[str]): Lines of Changelog.md (see
            read_changelog).
        changelog_path (str): Path to Changelog.md, used in error messages.

    Returns:
        tuple[str, str, str, int]: (project_name, current_version, heading,
            index) where heading is the markdown prefix used by that release
            header (e.g. ``###``), so new releases can be written at the same
            level, and index is the position of that header in
            *changelog_lines*, where the new release is inserted.

    Raises:
        ValueError: If the current version cannot be determined reliably:
            no release header, a pending (undated) release, a release whose
            major does not match its section, or a section with no release
            yet (first release of a new major must be written manually).
    """
    lines = [line.strip() for line in changelog_lines]

    placeholder_index = None
    for i, line in enumerate(lines):
        if CHANGELOG_PLACEHOLDER_RE.match(line):
            placeholder_index = i
            break

    if placeholder_index is None:
        # Legacy layout without "N.x" sections.
        start = 0
        major = None
    else:
        placeholder = CHANGELOG_PLACEHOLDER_RE.match(lines[placeholder_index])
        major = placeholder.group("major")
        start = placeholder_index + 1

    for i, line in enumerate(lines[start:], start):
        if CHANGELOG_PLACEHOLDER_RE.match(line):
            break
        match = CHANGELOG_RELEASE_HEADER_RE.match(line)
        if not match:
            continue
        version = match.group("version")
        if not RELEASE_DATE_RE.match(match.group("date")):
            raise ValueError(
                f"Top release '{line}' in {changelog_path} is not released yet "
                f"(date '{match.group('date')}'); finish that release manually")
        if major is not None and version.split('.')[0] != major:
            raise ValueError(
                f"Release {version} in {changelog_path} does not match its "
                f"section '{lines[placeholder_index]}'")
        return match.group("project"), version, match.group("level"), i

    if major is not None:
        raise ValueError(
            f"No release found under '{lines[placeholder_index]}' in "
            f"{changelog_path}; the first {major}.x release must be written "
            "manually")

    raise ValueError(
        f"Could not determine current version from {changelog_path}. "
        "Expected a placeholder like '## <project> <major>.x' followed by a "
        "release header like '### <project> <major>.<minor>.<patch> (YYYY-MM-DD)'")


def parse_changelog_entry(entry_content, filename):
    """
    Parse the Conventional Commits header of a changelog entry.

    Args:
        entry_content (str): Changelog entry content without template comments.
        filename (str): File name used for reporting validation errors.

    Returns:
        re.Match: CONVENTIONAL_COMMIT_HEADER_RE match of the first line.

    Raises:
        ValueError: If the entry is empty, its first line does not follow
            Conventional Commits, or it uses the breaking marker ('!'), which
            automatic versioning does not support.
    """
    first_line = first_non_blank_line(entry_content)

    if not first_line:
        raise ValueError(
            f"{filename}: entry is empty after removing template comments")

    match = CONVENTIONAL_COMMIT_HEADER_RE.match(first_line)
    if not match:
        raise ValueError(
            f"{filename}: first non-comment line must follow Conventional Commits "
            f"format '<type>(<optional-scope>): <description>' "
            f"(scope is optional), got '{first_line}'")

    if match.group("breaking"):
        raise ValueError(
            f"{filename}: breaking marker '!' is not supported by automatic "
            "versioning; handle this release manually")

    return match


def list_changelog_files(changelog_dir):
    """
    Return the sorted paths of the entry files in *changelog_dir*.

    pr-data-action accepts any file under .changelog/, subdirectories
    included, so every file is an entry except hidden ones (e.g. .gitkeep
    keeping the directory) and those inside hidden directories.
    """
    files = []
    for root, dirs, names in os.walk(changelog_dir):
        dirs[:] = [d for d in dirs if not d.startswith('.')]
        files.extend(
            os.path.join(root, name) for name in names
            if not name.startswith('.'))
    return sorted(files)


def uncommitted_changelog_files(files):
    """
    Return ``git status --porcelain`` lines for entry *files* that differ from HEAD.

    Processed entries are removed with ``git rm``, which fails on untracked,
    ignored or locally modified files after Changelog.md is already staged.
    Returns None when git cannot answer.
    """
    if not files:
        return []
    result = run_subprocess(
        ["git", "status", "--porcelain", "--untracked-files=all", "--ignored",
         "--", *files],
        context="git status for .changelog/")
    if result is None:
        return None
    return result.stdout.splitlines()


def read_changelog_entries(files):
    """
    Read the changelog entry *files*.

    Args:
        files (list[str]): Paths of the entry files (see list_changelog_files).

    Returns:
        tuple[list[dict], list[str]]: A tuple of (entries, errors). Each entry
        has the file ``path``, the parsed Conventional Commits ``header``
        match and the ``body`` lines that follow the header; main() adds its
        PR ``reference`` (see get_pr_info) once every entry is valid. Errors
        contains human-readable validation failure messages.
    """
    entries = []
    errors = []

    for file_path in files:
        filename = os.path.relpath(file_path)

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read().strip()
        except OSError as e:
            errors.append(f"{filename}: could not be read ({e})")
            continue

        entry_content = '\n'.join(
            line for line in content.split('\n')
            if not TEMPLATE_COMMENT_RE.match(line)).strip()

        try:
            header = parse_changelog_entry(entry_content, filename)
        except ValueError as e:
            errors.append(str(e))
            continue

        entries.append({
            "path": file_path,
            "header": header,
            "body": entry_content.split('\n')[1:],
        })

    return entries, errors


def calculate_next_version(entries, current_version):
    """
    Calculate the next version using changelog entries and the current version.

    Args:
        entries (list[dict]): Validated entries from read_changelog_entries.
        current_version (str): Current semver version string (``X.Y.Z``).

    Returns:
        str: The calculated next semver version.

    Raises:
        ValueError: If *current_version* is not valid semver.

    Rules:
      - If any changelog entry is a feature (feat), bump minor and reset patch.
      - Otherwise, bump patch.
      Entries with the breaking marker '!' never get here: they are rejected
      by parse_changelog_entry.
    """
    try:
        major, minor, patch = map(int, current_version.split('.'))
    except ValueError as e:
        raise ValueError(
            f"Current version '{current_version}' is not valid semver") from e

    if any(entry["header"].group("type") == "feat" for entry in entries):
        minor += 1
        patch = 0
    else:
        patch += 1

    return f"{major}.{minor}.{patch}"


def format_markdown_list_item(entry):
    """
    Format a changelog entry as a numbered Changelog.md item.

    The item text is the entry description, without the Conventional Commits
    type and scope. The entry body and a ``* <reference>`` sub-bullet follow,
    indented by 4 spaces so they stay inside the item, as in the existing
    Changelog.md entries::

        1. Add Centroid() to the simple shape classes
            * [Pull request #793](https://github.com/gazebosim/gz-math/pull/793)

    Args:
        entry (dict): Changelog entry with its parsed ``header``, its ``body``
            lines and its PR ``reference`` (see get_pr_info): a PR link,
            ``Commit: <sha>``, or empty for no sub-bullet.

    Returns:
        str: Markdown list item.
    """
    sub_lines = list(entry["body"])
    if entry["reference"]:
        sub_lines.append(f"* {entry['reference']}")
    return '\n'.join(
        [f"1. {entry['header'].group('description')}"] +
        [f"    {line}" if line else "" for line in sub_lines])


def generate_changelog_entry(entries, project_name, version, heading="###"):
    """
    Generate the Changelog.md section of a new release.

    Args:
        entries (list[dict]): Entries from read_changelog_entries, each with
            its ``reference`` set (see format_markdown_list_item).
        project_name (str): Changelog project name.
        version (str): Semver version for the release.
        heading (str): Markdown heading prefix for the release header, matching
            the existing release headers in Changelog.md.

    Returns:
        str: The release header followed by one numbered item per entry,
            separated by blank lines, without a trailing newline.
    """
    today = date.today().isoformat()

    # Changelog.md separates the release header and every item by a blank line.
    return '\n\n'.join(
        [f"{heading} {project_name} {version} ({today})"] +
        [format_markdown_list_item(entry) for entry in entries])


def update_changelog(changelog_path, changelog_lines, insert_index, new_entry):
    """
    Write the changelog file with the new entry above the current release.

    Args:
        changelog_path (str): Path to the changelog file.
        changelog_lines (list[str]): Current lines of the changelog (see
            read_changelog).
        insert_index (int): Index of the current release header in
            *changelog_lines* (see parse_changelog_header).
        new_entry (str): New changelog entry to add.

    Raises:
        RuntimeError: If writing the changelog fails.
    """
    lines = (changelog_lines[:insert_index] + [new_entry, ""] +
             changelog_lines[insert_index:])

    try:
        with open(changelog_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines))
    except OSError as e:
        raise RuntimeError(f"Could not write updated changelog to {changelog_path}: {e}") from e

    print(f"Successfully updated {changelog_path}")


def format_pr_link(pr):
    """Return the Changelog.md link for a PR dict with number and url."""
    return f"[Pull request #{pr['number']}]({pr['url']})"


def resolve_github_repo():
    """
    Return the ``owner/name`` GitHub repository for PR lookups, or None.

    gh picks the repository from the git remotes of the checkout. In a fork
    checkout that can be the fork, where the upstream PRs do not exist, so
    the parent repository is used when the picked one is a fork.
    """
    result = run_subprocess(
        ["gh", "repo", "view", "--json", "nameWithOwner,parent"],
        context="gh repo view")
    if result is None:
        return None
    try:
        repo = json.loads(result.stdout)
        parent = repo.get("parent")
        if parent:
            return f"{parent['owner']['login']}/{parent['name']}"
        return repo["nameWithOwner"]
    except (json.JSONDecodeError, KeyError, TypeError, AttributeError) as e:
        print(
            f"Warning: unexpected gh repo view output "
            f"({type(e).__name__}: {e})",
            file=sys.stderr)
    return None


def get_merged_pr_by_number(pr_number, repo):
    """
    Return ``{"number", "url"}`` for merged PR *pr_number* of *repo*, or None.

    None is returned when the number is not a merged PR (e.g. an issue) or
    gh cannot answer.
    """
    cmd = ["gh", "pr", "view", pr_number, "--repo", repo,
           "--json", "number,url,state"]
    result = run_subprocess(cmd, context=f"gh pr view {pr_number}", quiet=True)
    if result is None:
        return None
    try:
        pr = json.loads(result.stdout)
        if pr["state"] == "MERGED":
            return {"number": pr["number"], "url": pr["url"]}
    except (json.JSONDecodeError, KeyError, TypeError) as e:
        print(
            f"Warning: unexpected gh pr view output for #{pr_number} "
            f"({type(e).__name__}: {e})",
            file=sys.stderr)
    return None


def search_merged_pr_by_commit(commit_sha, repo):
    """Return ``{"number", "url"}`` of the merged PR of *repo* containing *commit_sha*, or None."""
    # Use gh CLI to search for merged PRs containing the commit SHA.
    cmd = ["gh", "pr", "list", "--repo", repo, "--state=merged",
           "--search", commit_sha, "--json", "number,url"]
    result = run_subprocess(cmd, context=f"gh pr list for {commit_sha}")
    if result is None or not result.stdout.strip():
        return None

    try:
        prs = json.loads(result.stdout)
        if prs:
            return {"number": prs[0]["number"], "url": prs[0]["url"]}
    except json.JSONDecodeError as e:
        print(
            f"Warning: gh returned invalid JSON for {commit_sha}: {e}",
            file=sys.stderr)
    except (KeyError, TypeError) as e:
        print(
            f"Warning: unexpected PR metadata structure for {commit_sha} "
            f"({type(e).__name__}: {e})",
            file=sys.stderr)
    return None


def find_pr_for_commit(commit_sha, message, repo):
    """
    Return the PR of *repo* to credit for the commit that added an entry.

    Backports reach a stable branch as a cherry-pick of the commit merged on
    the source branch, inside a backport PR. The merge method of the backport
    PR decides which PR is credited:

    * squash-merge: the squash commit is new work of the backport PR and its
      subject ends with ``(#<backport PR>)``, so the backport PR is linked.
    * rebase-merge (or merge commit): the cherry-pick lands unchanged, with the
      original subject ``... (#<original PR>)``, so the original PR is linked.
      When the original PR was not squash-merged either, its subject carries
      no ``(#N)`` and the ``(cherry picked from commit X)`` trailer is used.
      A backport of a backport carries one trailer per hop, oldest first, so
      the first trailer names the commit merged by the original PR.

    Anything else falls back to searching the PR that contains the commit.
    """
    lines = message.split('\n', 1)
    subject = lines[0]
    body = lines[1] if len(lines) > 1 else ""

    match = SUBJECT_PR_REF_RE.search(subject)
    if match:
        pr = get_merged_pr_by_number(match.group("number"), repo)
        if pr:
            return pr
        print(
            f"Info: #{match.group('number')} in the subject of {commit_sha} "
            "is not a merged PR, searching by commit",
            file=sys.stderr)
    else:
        trailers = CHERRY_PICK_TRAILER_RE.findall(body)
        if trailers:
            pr = search_merged_pr_by_commit(trailers[0], repo)
            if pr:
                return pr

    return search_merged_pr_by_commit(commit_sha, repo)


def get_pr_info(file_path, repo):
    """
    Get PR information for a changelog file using GitHub CLI.

    Args:
        file_path (str): Full path to the .changelog/ file. Used to identify
            the git commit that introduced the file, which is then used to
            locate the associated merged PR (see find_pr_for_commit).
        repo (str): ``owner/name`` GitHub repository of the PRs (see
            resolve_github_repo).

    Returns:
        str: Formatted PR link (``[Pull request #N](url)``) when found,
            commit SHA reference (``Commit: {sha}``) as fallback when the
            commit is known but no PR is found, or empty string when the
            originating commit cannot be determined.
    """
    filename = os.path.relpath(file_path)

    # Get the SHA and message of the most recent commit that added the file.
    # Entry files are removed at every release, so a reused name (e.g.
    # fix.md) has older adds from previous cycles. --follow is avoided because
    # its rename detection can jump to an unrelated small file with similar
    # content.
    git_cmd = ["git", "log", "-n1", "--diff-filter=A", "--format=%H%n%B", "--", file_path]
    git_result = run_subprocess(git_cmd, context=f"git log for {filename}")
    if git_result is None:
        return ""

    if not git_result.stdout.strip():
        print(
            f"Info: no git history found for {filename} "
            "(file may not be committed yet)",
            file=sys.stderr)
        return ""

    commit_sha, _, message = git_result.stdout.strip().partition('\n')
    print(f"Found commit SHA: {commit_sha} for file {filename}")

    pr = find_pr_for_commit(commit_sha, message, repo)
    if pr:
        print(f"Found PR #{pr['number']} for commit {commit_sha}")
        return format_pr_link(pr)
    return f"Commit: {commit_sha}"


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

    # The script commits on the user's behalf; refuse to sweep unrelated
    # staged changes or local Changelog.md edits into that commit.
    if subprocess.run(["git", "diff", "--cached", "--quiet"]).returncode != 0:
        print("Error: the git index has staged changes; commit or unstage "
              "them before running this script", file=sys.stderr)
        return 1

    if subprocess.run(
            ["git", "diff", "--quiet", "--", changelog_path]).returncode != 0:
        print(f"Error: {changelog_path} has uncommitted changes; commit or "
              "discard them before running this script", file=sys.stderr)
        return 1

    # Entries come from merged PRs; a file that is not committed as-is would
    # make the final "git rm" fail after Changelog.md has been staged.
    entry_files = list_changelog_files(changelog_dir)
    uncommitted = uncommitted_changelog_files(entry_files)
    if uncommitted is None:
        print("Error: could not check the git status of .changelog/",
              file=sys.stderr)
        return 1
    if uncommitted:
        print("Error: changelog entry files are not committed; commit or "
              "remove them before running this script:", file=sys.stderr)
        for line in uncommitted:
            print(f"  {line}", file=sys.stderr)
        return 1

    try:
        changelog_lines = read_changelog(changelog_path)
        project_name, current_version, heading, insert_index = \
            parse_changelog_header(changelog_lines, changelog_path)
    except (RuntimeError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

    print(f"Reading changelog entries from {changelog_dir}...")
    entries, entry_errors = read_changelog_entries(entry_files)

    if entry_errors:
        print("Error: invalid changelog entries detected:", file=sys.stderr)
        for entry_error in entry_errors:
            print(f"  - {entry_error}", file=sys.stderr)
        return 1

    if not entries:
        print("No changelog entries found!", file=sys.stderr)
        return 1

    github_repo = resolve_github_repo()
    if github_repo is None:
        print("Error: gh could not tell the GitHub repository of this "
              "checkout; check that gh is installed and authenticated, or "
              "pick one with 'gh repo set-default'", file=sys.stderr)
        return 1
    print(f"Looking up pull requests in {github_repo}")

    # Only now that every entry is valid: format_markdown_list_item needs
    # the reference of each one.
    for entry in entries:
        entry["reference"] = get_pr_info(entry["path"], github_repo)

    print("Generating changelog entry...")
    try:
        new_version = calculate_next_version(entries, current_version)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1
    print(f"Calculated next version: {new_version}")
    new_entry = generate_changelog_entry(
        entries, project_name, new_version, heading)

    print("Generated changelog entry:")
    print("-" * 50)
    print(new_entry)
    print("-" * 50)

    print("Processed entry files that will be removed:")
    for file_path in entry_files:
        print(f"  {os.path.relpath(file_path)}")

    try:
        response = input(
            "Update Changelog.md with this entry and remove the processed "
            "entry files? (y/N): ")
    except EOFError:
        # No answer available (stdin closed or piped): treat it as "no".
        print()
        response = ""
    if response.lower() not in ['y', 'yes']:
        print("Changelog update cancelled.")
        return 0

    # From here on a failure leaves Changelog.md and .changelog/ half
    # updated. Restore only those paths: "git reset --hard" would also drop
    # unrelated local changes, which the checks above allow.
    undo_hint = (
        "To undo the partial update run: git restore --source=HEAD "
        f"--staged --worktree -- {os.path.relpath(changelog_path)} "
        f"{os.path.relpath(changelog_dir)}")

    try:
        update_changelog(
            changelog_path, changelog_lines, insert_index, new_entry)
    except RuntimeError as e:
        print(f"Error: {e}\n{undo_hint}", file=sys.stderr)
        return 1

    # Not run_subprocess: git output stays visible, and commit hooks or
    # signing (e.g. waiting for a key touch) can outlast its timeout.
    try:
        subprocess.run(["git", "add", changelog_path], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: could not stage {changelog_path}: {e}\n{undo_hint}",
              file=sys.stderr)
        return 1

    try:
        subprocess.run(["git", "rm", "--", *entry_files], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: could not remove the processed entry files: {e}\n"
              f"{undo_hint}", file=sys.stderr)
        return 1

    try:
        subprocess.run(
            ["git", "commit", "--signoff", "-m",
             f"Generate changelog entry for version {new_version}",
             "--", changelog_path, *entry_files],
            check=True)
        print("Committed changelog updates")
    except subprocess.CalledProcessError as e:
        print(f"Error: could not commit changelog updates: {e}\n"
              f"{undo_hint}", file=sys.stderr)
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
