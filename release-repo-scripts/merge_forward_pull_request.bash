#!/usr/bin/env bash
# Copyright (C) 2022 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# The script will merge forward and open a pull request.
#
# Requires the 'gh' CLI to be installed.
#
# Usage:
# $ ./merge_forward_pull_request.bash <from_branch> <to_branch>
#
# For example, to merge `ign-rendering6` forward to `main`:
#
# ./merge_forward_pull_request.bash ign-rendering6 main

FROM_BRANCH=${1}
TO_BRANCH=${2}

set -e

ORIGIN_URL=$(git remote get-url origin)
ORIGIN_ORG_REPO=$(echo ${ORIGIN_URL} | sed -e 's@.*github\.com.@@')

TITLE="Merge ${FROM_BRANCH} -> ${TO_BRANCH}"

BODY="# ➡️  Forward port

Port \`${FROM_BRANCH} \` to \`${TO_BRANCH}\`

Branch comparision: https://github.com/${ORIGIN_ORG_REPO}/compare/${TO_BRANCH}...${FROM_BRANCH}"

if [[ $# -lt 2 ]]; then
  echo "./merge_forward_pull_request.bash <from_branch> <to_branch>"
  exit 1
fi

gh pr create \
    --title "$TITLE" \
    --repo "$ORIGIN_ORG_REPO" \
    --base "$TO_BRANCH" \
    --body "$BODY"
