#!/bin/bash

HG_TMP=$(mktemp -d -t hg_delete_closed_XXXXXXXXXX)
HG_REPO=$1
echo =========== Start Cloning HG Repository
hg clone $HG_REPO $HG_TMP
echo =========== Done Cloning HG Repository
echo
echo =========== Identify Closed HG Branches
hg -R $HG_TMP log -r "closed()" --template '{branch}\n' | sort


