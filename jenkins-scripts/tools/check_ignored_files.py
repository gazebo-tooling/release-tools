#!/usr/bin/env python3
"""
Check if a list of files matches a set of regex patterns defined in pattern files.

Usage:
  python3 check_ignored_files.py -i <pattern_file1> [-i <pattern_file2> ...] [file1 file2 ...]
  git diff --name-only | python3 check_ignored_files.py -i <pattern_file1> [-i <pattern_file2> ...]

Exit Codes:
  0 - ALL files match an ignored pattern in pattern_file.
  1 - At least one file does NOT match any ignored pattern.
  2 - Invalid arguments or pattern file missing.
"""

import argparse
import re
import sys

def load_patterns(pattern_file_path):
    """Load regex patterns from a single file, ignoring comments and blank lines."""
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

def load_patterns_from_files(pattern_file_paths):
    """Load and combine regex patterns from multiple pattern files."""
    all_patterns = []
    for path in pattern_file_paths:
        all_patterns.extend(load_patterns(path))
    return all_patterns

def is_ignored(filename, compiled_patterns):
    """Check if a filename matches any of the compiled regex patterns."""
    for pattern in compiled_patterns:
        if pattern.search(filename):
            return True
    return False

def main():
    parser = argparse.ArgumentParser(
        description="Check if a list of files matches a set of regex patterns defined in pattern files.",
        usage="%(prog)s -i PATTERN_FILE [-i PATTERN_FILE ...] [FILE1 FILE2 ...]"
    )
    parser.add_argument(
        '-i', '--ignore-file', action='append', dest='pattern_files', required=True,
        help='Path to an ignore pattern file (can be specified multiple times)'
    )
    parser.add_argument(
        'input_files', nargs='*',
        help='Files to check (if not provided via stdin)'
    )

    args = parser.parse_args()
    pattern_files = args.pattern_files
    input_files = args.input_files

    compiled_patterns = load_patterns_from_files(pattern_files)

    # Gather file list from stdin if not provided as positional args and stdin is redirected
    if not input_files and not sys.stdin.isatty():
        input_files = [line.strip() for line in sys.stdin if line.strip()]

    if not input_files:
        print("No files provided to check (treating as all files ignored).")
        sys.exit(0)

    print("Analyzing changed files against ignore patterns:")
    print("  Pattern file(s):")
    for pf in pattern_files:
        print(f"    - {pf}")
    print("  Patterns loaded:")
    for p in compiled_patterns:
        print(f"    - {p.pattern}")
    print("  Files analyzed:")
    for f in input_files:
        print(f"    - {f}")

    if not compiled_patterns:
        print("Warning: Pattern files are empty or contain no valid patterns.", file=sys.stderr)
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
