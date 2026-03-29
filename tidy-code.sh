#!/bin/bash

# This script will run clang-format and clang-tidy on all C++ files in the repository.

# Exit on error
set -e

# Set the style for clang-format (adjust this as per your coding standards)
CLANG_FORMAT_STYLE="Google"

# Run clang-format on all .cpp and .h files
echo "Running clang-format..."
find . \( -name "*.cpp" -o -name "*.h" \) -print0 | xargs -0 clang-format -i --style=$CLANG_FORMAT_STYLE

# Run clang-tidy on all .cpp files
source /opt/ros/jazzy/setup.bash
source ~/ws/install/setup.bash

echo "Running clang-tidy..."

package_name=$(grep -oP '(?<=<name>)[^<]+' package.xml)
echo "Current package: $package_name"

if [ ! -f ./compile_commands.json ]; then
    echo "Creating copy compile_commands.json..."
    cp "$HOME/ws/build/$package_name/compile_commands.json" ./compile_commands.json
fi
ls -l ./compile_commands.json
find . -name "*.cpp" -print0 | xargs -0 clang-tidy -p="$HOME/ws/build/$package_name" --checks='boost-*,bugprone-*,performance-*,readability-*,portability-*,modernize-*,clang-analyzer-*,cppcoreguidelines-* '
echo "Code formatting and tidying completed, All Done!"
