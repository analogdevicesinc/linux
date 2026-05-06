#!/bin/bash
# Collect cherry-pick original SHA from commit messages line:
#  cherry picked from commit ...
# The commit must have been created with `git cherry-pick -x <sha>`

gh-compare()
{
  curl -fL \
    -H "Accept: application/vnd.github+json" \
    -H "Authorization: Bearer $1" \
    -H "X-GitHub-Api-Version: 2026-03-10" \
    "https://api.github.com/repos/$2/compare/$3...$4?per_page=250&page=$5"
}

get_cherry_picks()
{
  local github_token="$1"
  local org_repository="$2"
  local base_sha="$3"
  local head_sha="$4"
  local output_file="$5"
  local tag json total_commits pages page shas

  json=$(gh-compare "$github_token" "$org_repository" "$base_sha" "$head_sha" 1)

  status=$(echo "$json" | jq '.status // empty')
  [ "$status" == "404" ] && { echo "Range $base_sha...$head_sha not found (404), won't collect cherry-picks" ; return 0 ; } || :
  total_commits=$(echo "$json" | jq '.total_commits // empty')
  [ -n "$total_commits" ] || { echo "$json" ; return 1 ; }
  [ "$total_commits" -eq 10000 ] && { echo "More than 10000 commits, won't collect cherry-picks" ; return 0 ; } || :

  shas=$(echo "$json" | jq -r '.commits[].commit.message | scan("cherry picked from commit ([0-9a-f]{40})") | .[0]')

  pages=$(( (total_commits + 249) / 250 ))
  for page in $(seq 2 "$pages"); do
    json=$(gh-compare "$github_token" "$org_repository" "$base_sha" "$head_sha" "$page")
    shas=$(printf '%s\n%s' "$shas" "$(echo "$json" | jq -r '.commits[].commit.message | scan("cherry picked from commit ([0-9a-f]{40})") | .[0]')")
  done

  shas=$(printf '%s' "$shas" | sort -u | grep -v '^$')

  [ -n "$output_file" ] && echo "$shas" > "$output_file" || echo "$shas"
}
