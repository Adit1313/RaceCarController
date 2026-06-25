#!/bin/bash

# Arguments:
# $1: Path to the solver folder
# $2: Optional flag to update the marker file

SOLVER_FOLDER=$1

cd "$SOLVER_FOLDER" || exit

WATCHED_FOLDERS=("script" "config")
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
for WATCHED_FOLDER in "${WATCHED_FOLDERS[@]}"; do

  # Verify that the watched folder exists
  if [ ! -d $WATCHED_FOLDER ]; then
    echo "Error: Folder $WATCHED_FOLDER not found"
    exit 1
  fi

  # Calculate a new hash from the watched folder
  LATEST_HASH+=$(find "$WATCHED_FOLDER" -type f -exec sha256sum {} + | sha256sum | cut -d' ' -f1)
done

if [ "$LATEST_HASH" != "$LAST_RUN_HASH" ]; then
  echo "Running solver generation..."

  # Run the solver generation
  ./script/create_solver.sh

  # Save the hash
  if [ "$2" == "update" ]; then
    echo -n "$LATEST_HASH" > "$MARKER_FILE"
  fi
  else
  echo "No changes detected"
fi

