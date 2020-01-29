#!/bin/bash

HG_TMP=$(mktemp -d -t hg_delete_closed_XXXXXXXXXX)
HG_REPO=$1
echo =========== Start Cloning HG Repository
echo cloning to $HG_TMP
hg clone $HG_REPO $HG_TMP
echo =========== Done Cloning HG Repository
echo
echo =========== Identify Closed HG Branches
CLOSED_BRANCHES=$(\
  hg -R $HG_TMP log -r "closed()" --template '{branch}\n' | sort)
for b in $CLOSED_BRANCHES
do
  git show remotes/origin/$b &> /dev/null || \
    { echo remote git branch not found: $b && continue; }
  echo remote git branch found: $b
  git push --delete origin $b
done


