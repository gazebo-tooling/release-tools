#!/usr/bin/env python3

# Copyright (C) 2024 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import argparse
import datetime
from typing import Optional
import subprocess
import re
from pathlib import Path

REPO_NAMES = {
    "gz-cmake": "Gazebo CMake",
    "gz-utils": "Gazebo Utils",
    "gz-tools": "Gazebo Tools",
    "gz-math": "Gazebo Math",
    "gz-plugin": "Gazebo Plugin",
    "gz-common": "Gazebo Common",
    "gz-msgs": "Gazebo Msgs",
    "sdformat": "libsdformat",
    "gz-fuel-tools": "Gazebo Fuel Tools",
    "gz-transport": "Gazebo Transport",
    "gz-physics": "Gazebo Physics",
    "gz-rendering": "Gazebo Rendering",
    "gz-sensors": "Gazebo Sensors",
    "gz-gui": "Gazebo GUI",
    "gz-sim": "Gazebo Sim",
    "gz-launch": "Gazebo Launch",
}


def ext_run(cmd: list):
    # print(f"Running: {' '.join(cmd)}")
    po = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = po.communicate()
    if po.returncode != 0:
        print("Error running command (%s)." % (" ".join(cmd)))
        print("stdout: %s" % (out.decode()))
        print("stderr: %s" % (err.decode()))
        raise Exception("subprocess call failed")
    return out.decode()

CMAKE_PROJECT_VERSION_PATTERN = r"^project\W*\(\W*([a-z0-9_-]*)\W*VERSION\W*([0-9.]*)"

def match_cmake_project_version(file_content: str):
    return re.search(
        CMAKE_PROJECT_VERSION_PATTERN ,
        file_content,
        re.MULTILINE,
    )

def get_project_and_version_from_cmake(cmake_file: str):
    with open(cmake_file) as f:
        m = match_cmake_project_version(f.read())
    if m:
        return m.groups()
    else:
        raise RuntimeError("Could not parse project and version from CMakeLists.txt")

def get_repo_from_project(project: str):
    repo = project.replace("ignition", "gz").replace("gazebo", "sim")
    repo = repo.replace('_','-')
    # remove the version
    m = re.search("[a-z-_]*", repo)
    print(m)
    if not m:
        raise RuntimeError("Could not determine repo from project name")
    else:
        repo = m.group(0)
        return repo, REPO_NAMES[repo]


def calculate_new_version(bump: str, previous_version: str):

    new_version = [int(v) for v in previous_version.split(".")]
    if bump == "major":
        new_version[0] += 1
    elif bump == "minor":
        new_version[1] += 1
    elif bump == "patch":
        new_version[2] += 1
    return ".".join(map(str, new_version))


def extract_title_and_pr(title_with_pr: str):
    matcher = re.compile(r"(.*)\(.*#(\d*)\)$")
    m = re.match(matcher, title_with_pr)
    if not m:
        raise RuntimeError("Could not parse title and PR")
    else:
        title, pr = m.groups()
        title = title.strip()
        # Sometimes, the title has two PR numbers. The first number is the backport,
        # so we skip that.
        m2 = re.match(matcher, title)
        if m2:
            title = m2.group(0)
        return title.strip(), pr

def project_has_package_xml(project: str, version: str):
    # TODO(azeey) Check if the project is in Harmonic or later to see if it "should"
    # have a package.xml instead of just checking if the file exists.
    return Path("package.xml").exists()

def generate_changelog(prev_tag, repo)-> list[str]:
    commits = ext_run(
        ["git", "log", f"HEAD...{prev_tag}", "--no-merges", "--pretty=format:%h"]
    )
    commits_list = commits.split()
    # print("commits_list:", commits_list)

    changelog = []
    for commit in commits_list:
        # print("commit:", commit)
        title_with_pr = ext_run(["git", "log", "--format=%s", "-n", "1", commit]).strip()
        # Extract title and PR number
        try:
            title, pr = extract_title_and_pr(title_with_pr)
            entry = f"""\
1. {title}
    * [Pull request #{pr}](https://github.com/gazebosim/{repo}/pull/{pr})
"""
            changelog.append(entry)
        except RuntimeError:
            pass
    return changelog


def update_changelog(changelog_path: Path, title: str, content: str):
    # Find the first line that starts with a `##` and add the contents right after
    with open(changelog_path, "r+") as f:
        old_changelog = f.read()
        match = re.search(r"^## .*\n", old_changelog)
        # Look for the first newline starting from the match
        if match:
            f.seek(0)
            f.write(old_changelog[:match.end()])
            f.write("\n" + title + "\n\n")
            f.write(content + "\n")
            f.write(old_changelog[match.end():])

def update_package_xml(package_xml_path: Path, new_version):
    with open(package_xml_path, "r+") as f:
        old_package_xml = f.read()
        match = re.search(r"<version>(.*)</version>", old_package_xml)
        if match:
            f.seek(0)
            f.write(old_package_xml[:match.start(1)])
            f.write(new_version)
            f.write(old_package_xml[match.end(1):])

def update_cmakelists(cmakelists_path: Path, new_version):
    with open(cmakelists_path, "r+") as f:
        old_cmakelists = f.read()
        match = match_cmake_project_version(old_cmakelists)
        if match:
            f.seek(0)
            f.write(old_cmakelists[:match.start(2)])
            f.write(new_version)
            f.write(old_cmakelists[match.end(2):])

def bump_version(bump: str, previous_version_input: Optional[str]):

    project, previous_version = get_project_and_version_from_cmake("CMakeLists.txt")
    repo, repo_name = get_repo_from_project(project)
    print("repo:", repo)
    if previous_version_input is not None:
        previous_version = previous_version_input

    print("previous version:", previous_version)

    new_version = calculate_new_version(bump, previous_version)
    print("new version:", new_version)

    if previous_version == new_version:
        print(
            f"Previous version {previous_version} and new version {new_version} should be different"
        )
        return

    prev_tag = f"{project.replace('_','-')}_{previous_version}"
    print("prev_tag:", prev_tag)
    changelog = generate_changelog(prev_tag, repo)
    changelog_str = ("\n".join(changelog)).strip()
    date = datetime.date.today()
    changelog_title = f"### {repo_name} {new_version} ({date})"
    print(f"{changelog_title}\n\n{changelog_str}")

    update_changelog(Path("Changelog.md"), changelog_title, changelog_str)
    if project_has_package_xml(project, previous_version):
        update_package_xml(Path("package.xml"), new_version)
    update_cmakelists(Path("CMakeLists.txt"), new_version)


def main():
    parser = argparse.ArgumentParser(
        description="Bump the version of a package in all the necessary places \
                     (package.xml, CMakeLists.txt) to prepare it for a release.\
                     This also updates the Changelog file based on git logs.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--bump",
        choices=("major", "minor", "patch"),
        default="patch",
        help="The type of version bump",
    )
    parser.add_argument(
        "--previous",
        help="Previous version (e.g., 3.0.0). If left empty, the last tag found \
              using 'git describe --tags' will be used",
    )

    args = parser.parse_args()
    bump_version(args.bump, args.previous)


if __name__ == "__main__":
    main()
