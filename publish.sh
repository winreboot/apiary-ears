#!/bin/sh
# Push this project to GitHub.
#
#   sh publish.sh <github-user> <token> [repo-name]
#
# The token is a fine-grained personal access token with, for this repository:
#   Contents: Read and write
# Create one at: github.com -> Settings -> Developer settings -> Personal access tokens
#
# If the repository does not exist yet the script will try to create it, but note
# that fine-grained tokens usually CANNOT create repositories. If that fails it
# will tell you to create it by hand, which takes about fifteen seconds, and then
# this script works normally.
set -e
USER="$1"
TOKEN="$2"
REPO="${3:-apiary-ears}"
OWNER="${OWNER:-$USER}"
[ -n "$USER" ] && [ -n "$TOKEN" ] || { echo "usage: sh publish.sh <github-user> <token> [repo-name]"; exit 1; }

api() { curl -s -o /tmp/gh.json -w "%{http_code}" -H "Authorization: Bearer $TOKEN" \
        -H "Accept: application/vnd.github+json" "$@"; }

echo "Looking for $OWNER/$REPO ..."
CODE=$(api "https://api.github.com/repos/$OWNER/$REPO")
if [ "$CODE" = "200" ]; then
  echo "  found it."
else
  echo "  not visible to this token (HTTP $CODE). Trying to create it ..."
  CODE=$(api -X POST https://api.github.com/user/repos \
    -d "{\"name\":\"$REPO\",\"private\":false,\"has_issues\":true}")
  if [ "$CODE" = "201" ]; then
    echo "  created."
  else
    echo
    echo "  Could not create it (HTTP $CODE):"
    sed -n 's/.*"message": *"\([^"]*\)".*/    \1/p' /tmp/gh.json
    echo
    echo "  Fine-grained tokens normally cannot create repositories. Do this instead:"
    echo "    1. github.com -> New repository -> owner $OWNER, name $REPO, Public,"
    echo "       and leave every 'initialize with' box UNCHECKED"
    echo "    2. Edit your token -> Repository access -> add $REPO,"
    echo "       Permissions -> Contents: Read and write"
    echo "    3. run this script again"
    exit 1
  fi
fi

if [ ! -d .git ]; then
  git init -b main
  git add .
  git -c user.name="$USER" -c user.email="$USER@users.noreply.github.com" \
      commit -m "Apiary Ears: wiring, firmware, scoring and a self-hosted page"
fi
git remote remove origin 2>/dev/null || true
git remote add origin "https://github.com/$OWNER/$REPO.git"

echo "Pushing ..."
git push -u "https://$USER:$TOKEN@github.com/$OWNER/$REPO.git" main

echo
echo "Done: https://github.com/$OWNER/$REPO"
echo "In the repo's Settings: Features -> tick Discussions, so people can ask questions."
