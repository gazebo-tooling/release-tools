#!/bin/bash
# source: https://coderwall.com/p/0ypmka
for branch in `git branch -a | grep remotes | grep -v HEAD | grep -v master `; do
       git branch --track ${branch#remotes/origin/} $branch
done
