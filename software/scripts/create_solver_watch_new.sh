#!/bin/bash

# Arguments:
# d: folders to watch, seperated by :
# s: script to execute
# u: whether to update the hash

OPTSTRING="d:us:"
UPDATE_HASH=0
while getopts "$OPTSTRING" arg; do
  case $arg in
  d)
    SOLVER_FOLDERS=${OPTARG}
    ;;
  u)
    UPDATE_HASH=1
    ;;
  s)
    SCRIPT_ARG=${OPTARG}
    ;;
  esac
done

if [ -z "$SCRIPT_ARG" ]; then
  echo "Missing script arg -s"
  exit 1
fi

echo watched folders: $SOLVER_FOLDERS
echo script: $SCRIPT_ARG

cd "$SOLVER_FOLDER" || exit

MARKER_FILE="lib/last_run_hash"

# Create the marker file if necessary
if [ ! -f "$MARKER_FILE" ]; then
  mkdir -p lib
  touch "$MARKER_FILE"
fi

# Get the last hash from the marker file
LAST_RUN_HASH=$(cat "$MARKER_FILE")

# Initialize the latest hash
LATEST_HASH=""

# Iterate over the watched folders
IFS=":"
for WATCHED_FOLDER in $SOLVER_FOLDERS; do
  # Verify that the watched folder exists
  if [ ! -d $WATCHED_FOLDER ]; then
    echo "Error: Folder $WATCHED_FOLDER not found"
    exit 1
  fi

  # Calculate a new hash from the watched folder
  LATEST_HASH+=$(find "$WATCHED_FOLDER" -type f -exec sha256sum {} + | sha256sum | cut -d' ' -f1)
done

# Also encode the generating cmd in the hash
LATEST_HASH+=$(echo "$SCRIPT_ARG" | sha256sum | cut -d' ' -f1)

if [ "$LATEST_HASH" == "$LAST_RUN_HASH" ]; then
  echo "No changes detected"
  exit
fi

echo "Running solver generation..."

# Run the solver generation
eval "$SCRIPT_ARG"
if [ $? -ne 0 ]; then
  echo "Generation Script failed, not updating hash"
  exit 1
fi

# Save the hash
if [ "$UPDATE_HASH" -eq 1 ]; then
  echo -n "$LATEST_HASH" >"$MARKER_FILE"
fi
