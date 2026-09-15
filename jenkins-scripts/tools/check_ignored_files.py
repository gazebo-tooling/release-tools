#!/usr/bin/env python3
"""
Check if a list of files matches a set of regex patterns defined in a pattern file.

Usage:
  python3 check_ignored_files.py <pattern_file> [file1 file2 ...]
  git diff --name-only | python3 check_ignored_files.py <pattern_file>

Exit Codes:
  0 - ALL files match an ignored pattern in pattern_file.
  1 - At least one file does NOT match any ignored pattern.
  2 - Invalid arguments or pattern file missing.
"""

import os
import re
import sys

def load_patterns(pattern_file_path):
    """Load regex patterns from file, ignoring comments and blank lines."""
    patterns = []
    try:
        with open(pattern_file_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    patterns.append(re.compile(line))
    except FileNotFoundError:
        print(f"Error: Pattern file '{pattern_file_path}' not found.", file=sys.stderr)
        sys.exit(2)
    except Exception as e:
        print(f"Error reading pattern file '{pattern_file_path}': {e}", file=sys.stderr)
        sys.exit(2)
    return patterns

def is_ignored(filename, compiled_patterns):
    """Check if a filename matches any of the compiled regex patterns."""
    for pattern in compiled_patterns:
        if pattern.search(filename):
            return True
    return False

def main():
    if len(sys.argv) < 2:
        print("Usage: check_ignored_files.py <pattern_file> [file1 file2 ...]", file=sys.stderr)
        print("       or pipe file list to stdin: cat files.txt | check_ignored_files.py <pattern_file>", file=sys.stderr)
        sys.exit(2)

    pattern_file = sys.argv[1]
    compiled_patterns = load_patterns(pattern_file)

    # Gather file list from positional arguments or stdin
    if len(sys.argv) > 2:
        input_files = sys.argv[2:]
    else:
        # Read from stdin if stdin is not a TTY (i.e. piped or redirected)
        if not sys.stdin.isatty():
            input_files = [line.strip() for line in sys.stdin if line.strip()]
        else:
            input_files = []

    if not input_files:
        print("No files provided to check.")
        sys.exit(0)

    if not compiled_patterns:
        print("Warning: Pattern file is empty or contains no valid patterns.", file=sys.stderr)
        non_ignored = input_files
    else:
        non_ignored = [f for f in input_files if not is_ignored(f, compiled_patterns)]

    if non_ignored:
        print("The following files do NOT match the ignored set:")
        for f in non_ignored:
            print(f)
        sys.exit(1)
    else:
        print("All files match the ignored set.")
        sys.exit(0)

if __name__ == '__main__':
    main()
