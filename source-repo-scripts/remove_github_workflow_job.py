#!/usr/bin/env python3
import sys
import re
from pathlib import Path

def remove_job_from_workflow(workflow_path: Path, job_name: str = "noble-ci"):
    """
    Removes a specified job block from a GitHub Actions workflow YAML file.
    """
    if not workflow_path.is_file():
        print(f"Error: File '{workflow_path}' not found.", file=sys.stderr)
        sys.exit(1)

    content = workflow_path.read_text()

    # Regex breakdown:
    # (?m)                      - Multiline mode: '^' matches start of any line.
    # ^  {re.escape(job_name)}: - Matches 2-space indented job key (e.g. '  noble-ci:').
    # \n                        - Trailing newline after the job key header.
    # (?:                       - Non-capturing group to repeat line-by-line matching:
    #   (?!^  [a-zA-Z0-9_-]+:)  - Negative Lookahead: ensure line does NOT start with another 2-space job key.
    #   .*\n                    - Consume all remaining characters on the line including newline.
    # )*                        - Match lines until the next job block or end of file.
    pattern = rf'(?m)^  {re.escape(job_name)}:\n(?:(?!^  [a-zA-Z0-9_-]+:).*\n)*'

    new_content, count = re.subn(pattern, '', content)

    if count == 0:
        print(f"Job '{job_name}' was not found in '{workflow_path}'.")
        return False

    workflow_path.write_text(new_content)
    print(f"Successfully removed job '{job_name}' from '{workflow_path}'.")
    return True

if __name__ == "__main__":
    target_file = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("./.github/workflows/ci.yml")
    job_to_remove = sys.argv[2] if len(sys.argv) > 2 else "noble-ci"
    remove_job_from_workflow(target_file, job_to_remove)
