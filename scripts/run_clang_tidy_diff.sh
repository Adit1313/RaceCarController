#!/bin/bash

# check if clang-tidy is installed
if ! command -v clang-tidy-diff &>/dev/null; then
  echo "clang-tidy is not installed. Please install it with your system package manager."
  exit 1
fi

# cd to root of the project
cd $(catkin locate --workspace $(pwd))

# Create compile_commands.json
eval scripts/create_compile_commands.sh
if [ ! -f "build/compile_commands.json" ]; then
  echo "compile_commands.json not found - please execute the build first."
  exit 1
fi

git diff origin/main | clang-tidy-diff -path build -p1 -j $(nproc --ignore=1)
