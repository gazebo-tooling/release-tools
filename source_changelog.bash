#!/bin/bash
set -e

current_origin_remote=$(git config --get remote.origin.url)

if [[ -z ${current_origin_remote} ]]; then
  echo "Script is unable to detect current remote.origin.url in this directory"
  exit 1
fi

if [[ -f "Changelog.md" ]]; then
  software_name=$(head -1 Changelog*)
  printf "%s (%s)\n\n" "${software_name}" "$(date +%Y-%m-%d)"
fi

current_tag=$(git symbolic-ref HEAD)
start_ref="HEAD"

# Find the previous release on the same branch, skipping prereleases if the
# current tag is a full release
previous_tag=""
while [[ -z $previous_tag || ( $previous_tag == *-* && $current_tag != *-* ) ]]; do
  previous_tag="$(git describe --tags "$start_ref"^ --abbrev=0)"
  start_ref="$previous_tag"
done

git log "$previous_tag".. --reverse --first-parent --pretty=format:"%h|%s|%ae|%an"  | \
  while IFS='|' read -r sha title author_email author || [[ -n ${sha} ]]; do
    pr_num="$(grep -o '#[[:digit:]]\+' <<<"$title")"
    pr_num="${pr_num:1}"
    if [[ $title == "Merge pull request #"* ]]; then
      pr_desc="$(git show -s --format=%b "$sha" | sed -n '1,/^$/p' | tr $'\n' ' ')"
    else
      pr_desc=${title/\ (#[[:digit:]]*)}
    fi
    printf "1. %s\n" "$pr_desc"
    printf "    * [Pull request #%s](%s/pull/%s)\n" "$pr_num" "$current_origin_remote" "$pr_num"
    if [[ "${author_email/@openrobotics.org}" == "${author_email}" ]]; then
      printf "    * A contribution from: %s <%s>\n" "$author" "$author_email"
    fi
    echo
  done
